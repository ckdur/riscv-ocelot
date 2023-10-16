package boom.exu

import chisel3._
import chisel3.util._
import chisel3.experimental._
import freechips.rocketchip.diplomacy.{LazyModule, LazyModuleImp}
import org.chipsalliance.cde.config.{Config, Parameters}
import freechips.rocketchip.rocket._
import freechips.rocketchip.tile._
import freechips.rocketchip.subsystem._
import freechips.rocketchip.rocket.VectorInstructions._
import freechips.rocketchip.util._
import hardfloat._

class VConfigBundle(implicit p: Parameters) extends CoreBundle()(p) {
  val rocc = new RoCCCommand
  val setvli = Bool()
  val setivli = Bool()
  val setvl = Bool()
}

class VConfigWrapper(implicit p: Parameters) extends CoreModule()(p) {
  val io = IO(new Bundle{
    val cmd = Flipped(Decoupled(new VConfigBundle))
    val resp = Decoupled(new RoCCResponse)
    val vector = Flipped(new CSRVectorIO)
  })
  val cfgcmd = io.cmd
  val vector = io.vector

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
  io.resp.valid := cfgcmd.fire
  io.resp.bits.data := vl
  io.resp.bits.rd := cfgcmd.bits.rocc.inst.rd

  vector.set_vtype.valid := io.resp.valid
  vector.set_vtype.bits := vtype

  vector.set_vl.valid := set_vl && io.resp.valid
  vector.set_vl.bits := vl

  vector.set_vstart := DontCare // TODO: This vstart is not set at all
}

class VMemLSQBundle(implicit p: Parameters) extends CoreBundle()(p) {
  val MemSyncStart = Input(Bool())
  val MemStoreValid = Input(Bool())
  val MemStoreData = Input(UInt(512.W))
  val MemMaskValid = Input(Bool())
  val MemMaskId = Input(UInt(66.W))

  val MemSyncEnd = Output(Bool())
  val MemSbId = Output(UInt(5.W))
  val MemLoadValid = Output(Bool())
  val MemSeqId = Output(UInt(34.W))
  val MemLoadData = Output(UInt(512.W))
  val MemReturnMaskValid = Output(Bool())
  val MemReturnMask = Output(UInt(64.W))
  val MemStoreCredit = Output(Bool())
  val MemMaskCredit = Output(Bool())
  val MemMaskIdxCredit = Output(Bool())
}

class VecCtrlSigs(implicit p: Parameters) extends CoreBundle()(p) {
  // This is a bundle to decode some additional traits from the instruction that do not communicate the rocket/rocc
  val mem_cmd  = UInt(freechips.rocketchip.rocket.M_SZ.W)
  val mem_size = if (usingVector) UInt(3.W) else UInt(2.W)
  def is_load = mem_cmd === M_XRD

  def is_store = mem_cmd === M_XWR

  def default: List[BitPat] = List(M_X)

  def decode(inst: UInt, table: Iterable[(BitPat, List[BitPat])]) = {
    val decoder = DecodeLogic(inst, default, table)
    val sigs = Seq(mem_cmd)
    sigs zip decoder foreach { case (s, d) => s := d }
    mem_size := inst(13,12)
    this
  }
}

class EnhancedCmdReq(implicit p: Parameters) extends Bundle {
  val vconfig = new VConfig()
  val vxrm = UInt(2.W)
  val fcsr_rm = UInt(3.W)
  val req = new RoCCCommand
  val ctrl = new VecCtrlSigs
}

class VecCtrlDecode(implicit val p: Parameters) extends DecodeConstants {
   val table: Array[(BitPat, List[BitPat])] = Array(
    // Load/Store Opcodes
    V_VL8->             List(M_XRD),
    V_VL16->            List(M_XRD),
    V_VL32->            List(M_XRD),
    V_VL64->            List(M_XRD),
    V_VLX8->            List(M_XRD),
    V_VLX16->           List(M_XRD),
    V_VLX32->           List(M_XRD),
    V_VLX64->           List(M_XRD),
    V_VLS8->            List(M_XRD),
    V_VLS16->           List(M_XRD),
    V_VLS32->           List(M_XRD),
    V_VLS64->           List(M_XRD),
    V_VS8->             List(M_XWR),
    V_VS16->            List(M_XWR),
    V_VS32->            List(M_XWR),
    V_VS64->            List(M_XWR),
    V_VSX8->            List(M_XWR),
    V_VSX16->           List(M_XWR),
    V_VSX32->           List(M_XWR),
    V_VSX64->           List(M_XWR),
    V_VSS8->            List(M_XWR),
    V_VSS16->           List(M_XWR),
    V_VSS32->           List(M_XWR),
    V_VSS64->           List(M_XWR)
  )
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

class OviLsuWrapper(implicit p: Parameters) extends CoreModule with VMemLSQConsts {
  val io = IO(new Bundle{
    val cache = new HellaCacheIO
    val mem = new VMemLSQBundle
    val cmd = Input(Decoupled(new RoCCCommand)) // NOTE: We forcing the input to read all
    val vconfig = Input(new VConfig())
    val vxrm = Input(UInt(2.W))
    val next_sb_id = Input(UInt(5.W)) // TODO: This is not consolidated
  })
  val mem = io.mem
  val cmd = io.cmd

  val seqSbId = WireInit(0.U(5.W)) // 5
  val seqElCount = WireInit(1.U(7.W)) // 7
  val seqElOff = WireInit(0.U(6.W)) // 6
  val seqElId = WireInit(0.U(11.W)) // 11
  val seqVreg = WireInit(0.U(5.W)) // 5

  // vLSIQ start
  // in the middle of handling vector load store
  val inMiddle = RegInit(false.B)
  // trying to dequeue VLSIQ
  val tryDeqVLSIQ = RegInit(false.B)
  // this chunk is checking the number of outstanding mem_sync_start
  val outStandingReq = RegInit(0.U(log2Ceil(outStandingLSCount).W))
  val canStartAnother = WireInit(false.B)
  val vOSud = Cat(mem.MemSyncStart, canStartAnother)
  when(vOSud === 1.U) {
    outStandingReq := outStandingReq - 1.U
  }.elsewhen(vOSud === 2.U) {
    outStandingReq := outStandingReq + 1.U
  }

  val vLSIQueue = Module(new Queue(new EnhancedCmdReq, vlsiQDepth))
  val sbIdQueue = Module(new Queue(UInt(5.W), vlsiQDepth))

  val req_ctrl = Wire(new VecCtrlSigs()).decode(cmd.bits.inst.asUInt, (new VecCtrlDecode).table)
  when(cmd.fire && (req_ctrl.is_store || req_ctrl.is_load)) {
    vLSIQueue.io.enq.valid := true.B
    vLSIQueue.io.enq.bits.req := cmd.bits
    vLSIQueue.io.enq.bits.ctrl := req_ctrl
    vLSIQueue.io.enq.bits.vconfig := io.vconfig
    vLSIQueue.io.enq.bits.vxrm := io.vxrm
    vLSIQueue.io.enq.bits.fcsr_rm := 0.U // TODO: get the FCSR

    sbIdQueue.io.enq.enq(io.next_sb_id)
  }.otherwise {
    vLSIQueue.io.enq.noenq()
    sbIdQueue.io.enq.noenq()
  }

  // can only dequeue vLSIQueue when it is not in the middle of handling vector load store and
  // 1. memSyncStart 2. pop out outStanding 3. previously tried
  vLSIQueue.io.deq.ready := !inMiddle && (outStandingReq =/= 0.U || mem.MemSyncStart || tryDeqVLSIQ)
  sbIdQueue.io.deq.ready := vLSIQueue.io.deq.ready

  when(!inMiddle) {
    // success transaction
    when(vLSIQueue.io.deq.fire) {
      inMiddle := true.B
      // active pop failed
    }.elsewhen((outStandingReq =/= 0.U || mem.MemSyncStart) && !vLSIQueue.io.deq.valid) {
      tryDeqVLSIQ := true.B
      inMiddle := true.B
      // passive pop successful
    }.elsewhen(tryDeqVLSIQ && vLSIQueue.io.deq.valid) {
      tryDeqVLSIQ := false.B
      inMiddle := true.B
    }
    // finish one vector load stroe
  }.elsewhen(canStartAnother) {
    inMiddle := false.B
  }

  // a new set has dequeued from vLSIQueue
  val newVGenConfig = vLSIQueue.io.deq.fire

  //v-Helper Start
  val vAGen = Module(new VAgen(lsuDmemWidth, 66, vAGenDepth, vpuVlen, oviWidth))

  vAGen.io.configValid := false.B
  vAGen.io.maskData := mem.MemMaskId
  vAGen.io.maskValid := mem.MemMaskValid
  vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1
  vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2
  vAGen.io.isUnit := false.B
  vAGen.io.isStride := false.B
  vAGen.io.isIndex := false.B
  vAGen.io.isMask := false.B
  vAGen.io.isLoad := false.B
  vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
  vAGen.io.pop := false.B
  vAGen.io.initialSliceSize := 0.U
  vAGen.io.memSize := 0.U
  mem.MemMaskCredit := vAGen.io.release

  // VDB stuff
  val vdb = Module(new VDB(oviWidth, lsuDmemWidth, vpuVlen, vdbDepth))
  vdb.io.writeValid := false.B
  vdb.io.pop := false.B
  vdb.io.last := false.B
  vdb.io.configValid := false.B

  vdb.io.writeValid := mem.MemStoreValid
  vdb.io.writeData := mem.MemStoreData
  vdb.io.sliceSize := vAGen.io.sliceSizeOut
  vdb.io.packOveride := vAGen.io.spackOveride
  vdb.io.packSkipVDB := vAGen.io.packSkipVDB
  vdb.io.packId := vAGen.io.packVDBId
  mem.MemStoreCredit := vdb.io.release

  // make sure that we have enough data
  val vDBcount = RegInit(0.U(log2Ceil(vdbDepth + 1).W))
  val vDBud = Cat(mem.MemStoreValid, mem.MemStoreCredit)
  when(vDBud === 1.U) {
    vDBcount := vDBcount - 1.U
  }.elsewhen(vDBud === 2.U) {
    vDBcount := vDBcount + 1.U
  }
  assert(vDBcount <= (vdbDepth).U)

  val vIdGen = Module(new VIdGen(byteVreg, byteDmem))
  vIdGen.io.configValid := false.B
  vIdGen.io.startID := 0.U
  vIdGen.io.startVD := 0.U
  vIdGen.io.pop := false.B
  vIdGen.io.sliceSize := vAGen.io.sliceSizeOut
  vIdGen.io.packOveride := vAGen.io.packOveride
  vIdGen.io.packSkipVreg := vAGen.io.packSkipVreg
  vIdGen.io.packId := vAGen.io.packId


  // extra decoder for whole register / mask
  val vwhls = Module(new VWhLSDecoder(vpuVlen))
  vwhls.io.nf := 0.U
  vwhls.io.wth := 0.U

  // Decode start
  io.cache.req.valid := false.B
  io.cache.req.bits := DontCare

  // need to send vGenIO request
  val vGenEnable = RegInit(false.B)
  // holding the microOp from vLSIQueue
  val vGenHold = Reg(new EnhancedCmdReq)

  // holding the sbId from sbIdQueue(vLSIQueue)
  val sbIdHold = RegInit(0.U)
  // holding wheter it is store or load
  val s0l1 = RegInit(false.B)
  // holding the direction of the stride, only useful for strided
  val strideDirHold = RegInit(true.B)
  val vlIsZero = RegInit(false.B)

  // Instruction Decoding
  val vLSIQueueinst = vLSIQueue.io.deq.bits.req.inst.asUInt
  val instNf = vLSIQueueinst(31, 29)
  val instMop = vLSIQueueinst(27, 26)
  val instMaskEnable = !vLSIQueueinst(25) // 0: enable, 1 disable
  val instUMop = vLSIQueueinst(24, 20)
  val instElemSize = vLSIQueueinst(14, 12)
  val instVldDest = vLSIQueueinst(11, 7)
  val instOP = vLSIQueueinst(6, 0)
  val isWholeStore = instOP === 39.U && instUMop === 8.U && instMop === 0.U
  val isWholeLoad = instOP === 7.U && instUMop === 8.U && instMop === 0.U
  val isStoreMask = instOP === 39.U && instUMop === 11.U && instMop === 0.U
  val isLoadMask = instOP === 7.U && instUMop === 11.U && instMop === 0.U
  val isIndex = instMop === 1.U || instMop === 3.U
  val isUnit = instMop === 0.U

  // start of a new round of vector load store
  when(newVGenConfig && !vGenEnable) {
    vGenEnable := true.B
    vGenHold.req := vLSIQueue.io.deq.bits.req
    vGenHold.ctrl := vLSIQueue.io.deq.bits.ctrl
    vGenHold.ctrl.mem_size := Mux(isIndex, vLSIQueue.io.deq.bits.vconfig.vtype.vsew,
      vLSIQueue.io.deq.bits.ctrl.mem_size)
    sbIdHold := sbIdQueue.io.deq.bits
    vlIsZero := vLSIQueue.io.deq.bits.vconfig.vl === 0.U &&
      !isWholeLoad &&
      !isWholeStore
    s0l1 := vLSIQueue.io.deq.bits.ctrl.is_load
    // only use vdb when it is store
    vdb.io.configValid := vLSIQueue.io.deq.bits.ctrl.is_store
    // only use vID when it is load
    vIdGen.io.configValid := vLSIQueue.io.deq.bits.ctrl.is_load
    // vstart = 0 for now
    vIdGen.io.startID := 0.U
    // write back V dest
    vIdGen.io.startVD := instVldDest
    // vAGen always on
    vAGen.io.configValid := true.B
    // default vl
    vAGen.io.vl := vLSIQueue.io.deq.bits.vconfig.vl
    // default base address and stride
    vAGen.io.startAddr := vLSIQueue.io.deq.bits.req.rs1
    vAGen.io.stride := vLSIQueue.io.deq.bits.req.rs2
    vAGen.io.isMask := instMaskEnable
    vAGen.io.memSize := vLSIQueue.io.deq.bits.vconfig.vtype.vsew
    vAGen.io.memSize := Mux(isIndex, vLSIQueue.io.deq.bits.vconfig.vtype.vsew,
          vLSIQueue.io.deq.bits.ctrl.mem_size)

    vAGen.io.isLoad := vLSIQueue.io.deq.bits.ctrl.is_load
    // special case of whole load and whole store
    when(isWholeLoad || isWholeStore) {
      vwhls.io.nf := instNf
      vwhls.io.wth := instElemSize
      vAGen.io.vl := vwhls.io.overVl
    }.elsewhen(isStoreMask || isLoadMask) {
      vAGen.io.vl := (vLSIQueue.io.deq.bits.vconfig.vl + 7.U) >> 3
    }
    // initial SliceSize
    when(isWholeStore || isStoreMask || isLoadMask) {
      vAGen.io.initialSliceSize := 1.U
    }.elsewhen(isIndex) {
      vAGen.io.initialSliceSize := 1.U << vLSIQueue.io.deq.bits.vconfig.vtype.vsew
    }.otherwise {
      when(instElemSize === 0.U) {
        vAGen.io.initialSliceSize := 1.U
      }.elsewhen(instElemSize === 5.U) {
        vAGen.io.initialSliceSize := 2.U
      }.elsewhen(instElemSize === 6.U) {
        vAGen.io.initialSliceSize := 4.U
      }.otherwise {
        vAGen.io.initialSliceSize := 8.U
      }
    }
    // check unit stride / strided / index
    strideDirHold := 0.U
    when(instMop === 0.U) {
      vAGen.io.isStride := false.B
      vAGen.io.isUnit := true.B
    }.elsewhen(instMop === 2.U) {
      vAGen.io.isStride := true.B
      strideDirHold := vLSIQueue.io.deq.bits.req.rs2(31)
    }.otherwise {
      vAGen.io.isIndex := true.B
    }
  }

  // Fake load response for masked-off elements
  val fakeLoadReturnQueue = Module(new Queue(UInt(34.W), fakeLoadDepth))
  fakeLoadReturnQueue.io.enq.valid := s0l1 && (vAGen.io.popForce || vAGen.io.popForceLast)
  fakeLoadReturnQueue.io.enq.bits := Cat(sbIdHold, vAGen.io.elemCount, vAGen.io.elemOffset, 0.U((11 - log2Ceil(byteVreg + 1)).W), vIdGen.io.outID, vIdGen.io.outVD)
  fakeLoadReturnQueue.io.deq.ready := false.B

  // Output to LSU
  io.cache.req.valid := vGenEnable && ((!s0l1 && ((!vlIsZero && vDBcount =/= 0.U) || vlIsZero)) || s0l1) && vAGen.io.canPop

  io.cache.req.bits.addr := Mux(s0l1, Cat(vAGen.io.outAddr(39, addrBreak), 0.U(addrBreak.W)), vAGen.io.outAddr)
  // io.cache.req.bits.idx
  io.cache.req.bits.tag := sbIdHold
  io.cache.req.bits.cmd := vGenHold.ctrl.mem_cmd
  io.cache.req.bits.size := Mux(s0l1, addrBreak.U, vAGen.io.memSizeOut) // TODO: Possibly wrong
  io.cache.req.bits.signed := false.B
  io.cache.req.bits.dprv := vGenHold.req.status.dprv
  io.cache.req.bits.dv := vGenHold.req.status.dv
  io.cache.req.bits.data := Mux(s0l1, 0.U, vdb.io.outData)
  io.cache.req.bits.mask := vAGen.io.currentMaskOut  // TODO: This is highly wrong
  io.cache.req.bits.phys := false.B
  // io.cache.req.bits.no_alloc
  // io.cache.req.bits.no_xcpt

  // TODO: Not connected bits are here
  //io.vGenIO.req.bits.last := vAGen.io.last
  //io.vGenIO.reqHelp.bits.elemID := vIdGen.io.outID
  //io.vGenIO.reqHelp.bits.elemOffset := vAGen.io.elemOffset
  //io.vGenIO.reqHelp.bits.elemCount := vAGen.io.elemCount
  //io.vGenIO.reqHelp.bits.vRegID := vIdGen.io.outVD
  //io.vGenIO.reqHelp.bits.strideDir := strideDirHold
  //io.vGenIO.reqHelp.bits.isMask := vAGen.io.isMaskOut
  //io.vGenIO.reqHelp.bits.isFake := vAGen.io.isFake

  // FSM for V-helper
  when(io.cache.req.fire || vAGen.io.popForce) {
    vdb.io.pop := io.cache.req.bits.cmd === M_XWR && !vlIsZero
    vAGen.io.pop := true.B
    vIdGen.io.pop := io.cache.req.bits.cmd === M_XRD && !vlIsZero
    when(vAGen.io.last) {
      vGenEnable := false.B
      vdb.io.last := io.cache.req.bits.cmd === M_XWR && !vlIsZero
      canStartAnother := true.B
    }
  }


  // Parse LSU response
  val LSUReturnLoadValid = WireInit(false.B)
  LSUReturnLoadValid := io.cache.resp.valid && io.cache.resp.bits.cmd === M_XRD // needs fixing later if we are overlapping
  val LSUReturnData = WireInit(0.U(oviWidth.W))
  when(strideDirHold) { // negative // TODO: This is assigned to the hold version of strideDir, but this is wrong
    LSUReturnData := Cat(io.cache.resp.bits.data((lsuDmemWidth - 1), 0), 0.U((oviWidth - lsuDmemWidth).W))
  }.otherwise {
    LSUReturnData := Cat(0.U, io.cache.resp.bits.data((lsuDmemWidth - 1), 0))
  }

  // Data back to VPU
  mem.MemSbId := 0.U
  val MemSb = RegInit(0.U(32.W))
  val vectorDone = WireInit(0.U(2.W))
  val MemSbResidue = WireInit(false.B)
  /*vectorDone := Cat(io.cache.resp.bits.vectorDoneLd, io.cache.resp.bits.vectorDoneSt)
  when(vectorDone === 1.U) {
    mem.MemSbId := io.cache.resp.bits.sbIdDoneSt
  }.elsewhen(vectorDone === 2.U) {
    mem.MemSbId := io.cache.resp.bits.sbIdDoneLd
  }.elsewhen(vectorDone === 3.U) {
    mem.MemSbId := io.cache.resp.bits.sbIdDoneLd
    MemSb := MemSb.bitSet(io.cache.resp.bits.sbIdDoneSt, true.B)
  }.elsewhen(MemSb =/= 0.U) {*/
    MemSbResidue := true.B
    mem.MemSbId := PriorityEncoder(MemSb)
    MemSb := MemSb.bitSet(mem.MemSbId, false.B)
  //}
  mem.MemSyncEnd := (/*io.cache.resp.bits.vectorDone &&*/ io.cache.resp.valid) || MemSbResidue


  mem.MemLoadValid := LSUReturnLoadValid || fakeLoadReturnQueue.io.deq.valid
  mem.MemSeqId := Cat(seqSbId, seqElCount, seqElOff, seqElId, seqVreg)

  when(LSUReturnLoadValid) {
    mem.MemLoadData := LSUReturnData
    seqSbId := io.cache.resp.bits.tag
    seqElCount := 0.U // TODO: io.vGenIO.resp.bits.elemCount
    seqElOff := 0.U // TODO: io.vGenIO.resp.bits.elemOffset
    seqElId := 0.U // TODO: Cat(0.U(3.W), io.vGenIO.resp.bits.elemID)
    seqVreg := 0.U // TODO: io.vGenIO.resp.bits.vRegID

    mem.MemReturnMaskValid := io.cache.resp.bits.mask.orR // io.cache.resp.bits.isMask
    mem.MemReturnMask := io.cache.resp.bits.mask
  }.elsewhen(fakeLoadReturnQueue.io.deq.valid) {
    mem.MemLoadData := 0.U
    seqSbId := fakeLoadReturnQueue.io.deq.bits(33, 29)
    seqElCount := fakeLoadReturnQueue.io.deq.bits(28, 22)
    seqElOff := fakeLoadReturnQueue.io.deq.bits(21, 16)
    seqElId := fakeLoadReturnQueue.io.deq.bits(15, 5)
    seqVreg := fakeLoadReturnQueue.io.deq.bits(4, 0)
    mem.MemReturnMaskValid := true.B
    mem.MemReturnMask := false.B
    fakeLoadReturnQueue.io.deq.ready := true.B
  }

  mem.MemMaskIdxCredit := vAGen.io.release
}

class OviRoccWrapper(implicit p: Parameters) extends CoreModule with VMemLSQConsts {
  val io = IO(new RoCCIO(0)) // NOTE: Is zero unless you need PTW

  // Ovi and Vconfig Router
  val router = Module(new OviAndVConfigRouter)
  router.io.in <> io.cmd
  val cmd = Queue(router.io.out)
  val cfgcmd = Queue(router.io.cfgout)
  val vector = io.vector.get

  val respArb = Module(new RRArbiter(new RoCCResponse()(p), 2))
  io.resp <> respArb.io.out
  respArb.io.in(0).valid := false.B
  respArb.io.in(1).valid := false.B
  respArb.io.in(0).bits := DontCare
  respArb.io.in(1).bits := DontCare

  // **************** Vconfig Phase ************************
  val vcfg = Module(new VConfigWrapper)
  vcfg.io.cmd <> cfgcmd
  respArb.io.in(1) <> vcfg.io.resp
  vector <> vcfg.io.vector

  // ****************** VPU ******************
  // Credit system
  val sb_valid = RegInit(VecInit.fill(32)(false.B))
  val sb_cmd = Reg(Vec(32, new RoCCCommand))
  val next_sb_id = PriorityEncoder(sb_valid.map(!_))
  when(cmd.fire) {
    sb_cmd(next_sb_id) := cmd.bits
    sb_valid(next_sb_id) := true.B
  }
  val resp_valid = Wire(Bool())
  val completed_sb_id = Wire(UInt())
  val completed_dest_reg = Wire(UInt())
  val resp_cmd = sb_cmd(completed_sb_id)
  when(resp_valid) {
    sb_valid(completed_sb_id) := false.B
  }
  val MAX_ISSUE_CREDIT = 16
  val issue_credit_cnt = RegInit(MAX_ISSUE_CREDIT.U)
  val issue_credit = Wire(UInt())
  val issue_valid = Wire(UInt())
  issue_credit_cnt := issue_credit_cnt + issue_credit - issue_valid
  val vpu_ready = issue_credit_cnt =/= 0.U
  cmd.ready := vpu_ready

  // Response to the writeback
  // TODO: This function should't come from here. This should come from the decoder in Rocket
  def IsVinstRd(inst: RoCCInstruction): Bool = {
    // V_VWXUNARY0= BitPat("b010000???????????010?????1010111")
    inst.funct(6,1) === "b010000".U && !inst.xd && inst.xs1 && !inst.xs2 && inst.opcode === "b1010111".U
  }
  respArb.io.in(0).valid := resp_valid && IsVinstRd(resp_cmd.inst)
  respArb.io.in(0).bits.rd := resp_cmd.inst.rd
  respArb.io.in(0).bits.data := completed_dest_reg
  // TODO: The completed_fflags are not used in this context

  // Load Store Unit
  val lsu = Module(new OviLsuWrapper)
  io.mem <> lsu.io.cache
  lsu.io.cmd := cmd
  lsu.io.vconfig := vector.vconfig
  lsu.io.vxrm := vector.vxrm
  lsu.io.next_sb_id := next_sb_id

  // VPU connections
  val vpu = Module(new tt_vpu_ovi(vLen))
  vpu.io := DontCare
  vpu.io.clk := clock
  vpu.io.reset_n := ~reset.asBool
  vpu.io.issue_valid := cmd.fire
  vpu.io.issue_inst := cmd.bits.inst.asUInt
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
  resp_valid := vpu.io.completed_valid
  completed_sb_id := vpu.io.completed_sb_id
  issue_credit := vpu.io.issue_credit
  issue_valid := vpu.io.issue_valid
  completed_dest_reg := vpu.io.completed_dest_reg

  // VPU Mem connections
  lsu.io.mem.MemSyncStart := vpu.io.memop_sync_start
  lsu.io.mem.MemStoreValid := vpu.io.store_valid
  lsu.io.mem.MemStoreData := vpu.io.store_data
  lsu.io.mem.MemMaskValid := vpu.io.mask_idx_valid
  lsu.io.mem.MemMaskId := Cat(vpu.io.mask_idx_last_idx, vpu.io.mask_idx_item)
  vpu.io.memop_sync_end := lsu.io.mem.MemSyncEnd
  vpu.io.memop_sb_id := lsu.io.mem.MemSbId
  // vpu.io.mem_vstart := MEMVstart
  vpu.io.load_valid := lsu.io.mem.MemLoadValid
  vpu.io.load_seq_id := lsu.io.mem.MemSeqId
  vpu.io.load_data := lsu.io.mem.MemLoadData
  vpu.io.load_mask_valid := lsu.io.mem.MemReturnMaskValid
  vpu.io.load_mask := lsu.io.mem.MemReturnMask
  vpu.io.store_credit := lsu.io.mem.MemStoreCredit
  vpu.io.mask_idx_credit := lsu.io.mem.MemMaskIdxCredit
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
