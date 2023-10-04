package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.diplomacy.LazyModule
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import hardfloat._

class OviRocc(opcodes: OpcodeSet)(implicit p: Parameters) extends LazyRoCC(opcodes) {
  override lazy val module = new OviRoccModuleImp(this)
}

class OviRoccModuleImp(outer: OviRocc)(implicit p: Parameters) extends LazyRoCCModuleImp(outer)
  with HasCoreParameters {
  val cmd = Queue(io.cmd)

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
  /*vpu.io.issue_vcsr := Cat(
    0.U(1.W), // vill
    io.vconfig.vtype.vsew, // vsew
    io.vconfig.vtype.vlmul_mag, // vlmul
    io.fcsr_rm, // frm
    io.vxrm, // vxrm
    Cat(0.U((15 - log2Ceil(vLen + 1)).W), io.vconfig.vl), // vl
    0.U(14.W) // vstart
  )*/
  vpu.io.issue_vcsr_lmulb2 := false.B // NOTE: From VCSR
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
