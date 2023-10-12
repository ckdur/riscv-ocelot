package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import hardfloat._

class VConfigBundle(implicit p: Parameters) extends CoreBundle()(p) {
  val rocc = new RoCCCommand
  val setvli = Bool()
  val setivli = Bool()
  val setvl = Bool()
}
class OviAndVConfigRouter(implicit p: Parameters) extends CoreModule()(p) {
  val io = new Bundle {
    val in = Flipped(Decoupled(new RoCCCommand))
    val out = Decoupled(new RoCCCommand)
    val cfgout = Decoupled(new VConfigBundle)
  }
  val cmd = Queue(io.in)

  // Opcode Match with any SET[I]VL[I]
  val opmatch = cmd.bits.inst.opcode === "b1010111".U
  val funct3 = cmd.bits.inst.xd ## cmd.bits.inst.xs1 ## cmd.bits.inst.xs2
  val setvli = opmatch && cmd.bits.inst.funct(6) === false.B && funct3 === "b111".U
  val setivli = opmatch && cmd.bits.inst.funct(6, 5) === "b11".U && funct3 === "b111".U
  val setvl = opmatch && cmd.bits.inst.funct === "b1000000".U && funct3 === "b111".U
  val me = setvli || setivli || setvl

  // Connection of the routing process
  val cmdReadys = Wire(Vec(2, Bool()))

  io.out.valid := cmd.valid && !me
  io.out.bits := cmd.bits
  cmdReadys(0) := io.out.ready && !me

  io.cfgout.valid := cmd.valid && me
  io.cfgout.bits.rocc := cmd.bits
  io.cfgout.bits.setvli := setvli
  io.cfgout.bits.setivli := setivli
  io.cfgout.bits.setvl := setvl
  cmdReadys(1) := io.cfgout.ready && me

  cmd.ready := cmdReadys.reduce(_ || _)
}

class OviRoccWrapper(implicit p: Parameters) extends CoreModule with HasCoreParameters {
  val io = IO(new RoCCIO(0)) // NOTE: Is zero unless you need PTW

  // Ovi and Vconfig Router
  val router = Module(new OviAndVConfigRouter)
  router.io.in <> io.cmd
  val cmd = Queue(router.io.out)
  val cfgcmd = Queue(router.io.cfgout)

  val respArb = Module(new RRArbiter(new RoCCResponse()(p), 2))
  io.resp <> respArb.io.out
  respArb.io.in(0).valid := false.B
  respArb.io.in(1).valid := false.B
  respArb.io.in(0).bits := DontCare
  respArb.io.in(1).bits := DontCare

  // **************** Vconfig Phase ************************
  def ZImmGen(setivli: Bool, inst: UInt): UInt = {
    val other = 0.U((32 - 11).W)
    val i10 = Mux(setivli, inst(30), 0.U)
    val i9_0 = inst(29, 20)
    other ## i10 ## i9_0
  }

  val avl_imm = 0.U((32 - 5).W) ## cfgcmd.bits.rocc.inst.rs2
  val vtype_imm = ZImmGen(cfgcmd.bits.setivli, cfgcmd.bits.rocc.inst.asUInt)

  val vtype_raw = Wire(UInt())
  val avl = Wire(UInt(xLen.W))
  avl.suggestName("avl")
  when(cfgcmd.bits.setivli) {
    vtype_raw := vtype_imm
    avl := avl_imm
  }.elsewhen(cfgcmd.bits.setvli) {
    vtype_raw := vtype_imm
    avl := cfgcmd.bits.rocc.rs1
  }.otherwise {
    vtype_raw := cfgcmd.bits.rocc.rs2
    avl := cfgcmd.bits.rocc.rs1
  }

  val vl = Wire(UInt())
  val vtype = VType.fromUInt(vtype_raw, false)
  // vsetvl(i) with rd=x0 and rs1=x0 doesn't update vl CSR
  val set_vl = Wire(Bool())
  set_vl :=
    cfgcmd.bits.setivli ||
      cfgcmd.bits.rocc.inst.rd =/= 0.U ||
      cfgcmd.bits.rocc.inst.rs1 =/= 0.U
  // vsetvl(i) with rd!=x0 and rs1=x0 sets vl to VLMAX
  val set_vlmax = Wire(Bool())
  set_vlmax :=
    !cfgcmd.bits.setivli ||
      cfgcmd.bits.rocc.inst.rd =/= 0.U ||
      cfgcmd.bits.rocc.inst.rs1 === 0.U
  when(set_vlmax) {
    vl := VType.computeVL(avl, vtype_raw, 0.U, 0.B, 1.B, 0.B)
  }.otherwise {
    vl := VType.computeVL(avl, vtype_raw, 0.U, 0.B, 0.B, 0.B)
  }
  set_vl.suggestName("set_vl")
  set_vlmax.suggestName("set_vlmax")

  cfgcmd.ready := true.B
  respArb.io.in(1).valid := cfgcmd.fire
  respArb.io.in(1).bits.data := vl
  respArb.io.in(1).bits.rd := cfgcmd.bits.rocc.inst.rd

  val vector = io.vector.get
  vector.set_vtype.valid := respArb.io.in(1).valid
  vector.set_vtype.bits := vtype

  vector.set_vl.valid := set_vl && respArb.io.in(1).valid
  vector.set_vl.bits := vl

  vector.set_vstart := DontCare // TODO: This vstart is not set at all

  // ****************** VPU ******************
  val vpu = Module(new tt_vpu_ovi(vLen))
  vpu.io := DontCare
  vpu.io.clk := clock
  vpu.io.reset_n := ~reset.asBool
  vpu.io.issue_valid := cmd.fire
  vpu.io.issue_inst := cmd.bits.inst.asUInt

  val sb_valid = RegInit(VecInit.fill(32)(false.B))
  val next_sb_id = PriorityEncoder(sb_valid.map(!_))
  when(cmd.fire) {
    sb_valid(next_sb_id) := true.B
  }
  when(vpu.io.completed_valid) {
    sb_valid(vpu.io.completed_sb_id) := false.B
  }

  val MAX_ISSUE_CREDIT = 16
  val issue_credit_cnt = RegInit(MAX_ISSUE_CREDIT.U)
  issue_credit_cnt := issue_credit_cnt + vpu.io.issue_credit - vpu.io.issue_valid
  val vpu_ready = issue_credit_cnt =/= 0.U
  vpu.io.issue_sb_id := next_sb_id
  vpu.io.issue_scalar_opnd := cmd.bits.rs1
  vpu.io.issue_vcsr := Cat(
    0.U(1.W), // vill
    vector.vconfig.vtype.vsew, // vsew
    vector.vconfig.vtype.vlmul_mag, // vlmul
    0.U(3.W), // frm, or fcsr_rm (TODO: Connect from the FCSR)
    vector.vxrm, // vxrm
    Cat(0.U((15 - log2Ceil(vLen + 1)).W), vector.vconfig.vl), // vl
    0.U(14.W) // vstart TODO: Is not set for vstart
  )
  vpu.io.issue_vcsr_lmulb2 := vector.vconfig.vtype.vlmul_sign
  vpu.io.dispatch_sb_id := next_sb_id
  vpu.io.dispatch_next_senior := cmd.fire
  vpu.io.dispatch_kill := 0.B

  io.mem.req.valid := vpu.io.memop_sync_start
  io.mem.req.bits.cmd := M_XRD

  //vpu.io.memop_sync_end := MemSyncEnd
  //vpu.io.memop_sb_id := MemSbId
  // vpu.io.mem_vstart := MEMVstart
  vpu.io.load_valid := io.mem.resp.fire
  vpu.io.load_seq_id := io.mem.resp.bits.tag
  vpu.io.load_data := io.mem.resp.bits.data
  //vpu.io.load_mask_valid := MemReturnMaskValid
  //vpu.io.load_mask := MemReturnMask
  //vpu.io.store_credit := MemStoreCredit
  //vpu.io.mask_idx_credit := vAGen.io.release
  cmd.ready := vpu_ready
}

class OviRocc(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new OviRoccModuleImp(this)
}

class OviRoccModuleImp(outer: OviRocc)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
  with HasCoreParameters {
  val wrap = Module(new OviRoccWrapper)
  io <> wrap.io
}

object VecOpcodeSet {
  def all = new OpcodeSet(Seq("b1010111".U, "b0100111".U, "b0000111".U))
}

class WithVecRoccExample extends Config((site, here, up) => {
  case BuildRoCC => List(
    (p: Parameters) => {
      val vec = LazyModule(new OviRocc(VecOpcodeSet.all)(p))
      vec
    })
  case TilesLocated(InSubsystem) => up(TilesLocated(InSubsystem), site) map {
    case tp: RocketTileAttachParams => tp.copy(tileParams = tp.tileParams.copy(
      /*dcache = tp.tileParams.dcache.map(_.copy(
        rowBits = 64
      )),*/
      core = tp.tileParams.core.copy(
        enableVector = true
      )))
    case t => t
  }
})
