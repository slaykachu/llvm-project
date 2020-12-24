//===- llvm/lib/Target/Z80/Z80CallLowering.cpp - Call lowering ------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
/// \file
/// This file implements the lowering of LLVM calls to machine code calls for
/// GlobalISel.
//
//===----------------------------------------------------------------------===//

#include "Z80CallLowering.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "Z80CallingConv.h"
#include "Z80ISelLowering.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80Subtarget.h"
#include "llvm/CodeGen/Analysis.h"
#include "llvm/CodeGen/GlobalISel/MachineIRBuilder.h"
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetMachine.h"
using namespace llvm;

#define DEBUG_TYPE "z80-call-lowering"

Z80CallLowering::Z80CallLowering(const Z80TargetLowering &TLI)
    : CallLowering(&TLI) {}

namespace {

struct Z80OutgoingValueHandler : public CallLowering::OutgoingValueHandler {
  Z80OutgoingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI, MachineInstrBuilder &MIB,
                          CCAssignFn *AssignFn)
      : OutgoingValueHandler(MIRBuilder, MRI, AssignFn), MIB(MIB),
        DL(MIRBuilder.getMF().getDataLayout()),
        STI(MIRBuilder.getMF().getSubtarget<Z80Subtarget>()) {
    LLT PtrTy = LLT::pointer(0, DL.getPointerSizeInBits(0));
    Register SPReg = STI.getRegisterInfo()->getStackRegister();
    SPRegCopy = MIRBuilder.buildCopy(PtrTy, SPReg).getReg(0);
  }

  Register getStackAddress(uint64_t Size, int64_t Off,
                           MachinePointerInfo &MPO) override {
    LLT PtrTy = LLT::pointer(0, DL.getPointerSizeInBits(0));
    LLT OffTy = LLT::scalar(DL.getIndexSizeInBits(0));
    MPO = MachinePointerInfo::getStack(MIRBuilder.getMF(), Off);
    auto OffI = MIRBuilder.buildConstant(OffTy, Off);
    return MIRBuilder.buildPtrAdd(PtrTy, SPRegCopy, OffI).getReg(0);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    MIB.addUse(PhysReg, RegState::Implicit);
    MIRBuilder.buildCopy(PhysReg, extendRegister(ValVReg, VA));
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    auto MMO = MIRBuilder.getMF().getMachineMemOperand(
        MPO, MachineMemOperand::MOStore, VA.getLocVT().getStoreSize(),
        Align());
    MIRBuilder.buildStore(extendRegister(ValVReg, VA), Addr, *MMO);
  }

  bool finalize(CCState &State) override {
    if (State.getCallingConv() == CallingConv::Z80_TIFlags) {
      bool Is24Bit = STI.is24Bit();
      MVT VT = Is24Bit ? MVT::i24 : MVT::i16;
      Register FlagsReg =
          MIRBuilder
              .buildConstant(LLT(VT), STI.hasEZ80Ops() ? 0xD00080 : 0x89F0)
              .getReg(0);
      CCValAssign VA = CCValAssign::getReg(~0, VT, Is24Bit ? Z80::UIY : Z80::IY,
                                           VT, CCValAssign::Full);
      assignValueToReg(FlagsReg, VA.getLocReg(), VA);
    }
    if (MRI.use_empty(SPRegCopy))
      MRI.getVRegDef(SPRegCopy)->eraseFromParent();
    return ValueHandler::finalize(State);
  }

protected:
  MachineInstrBuilder &MIB;
  const DataLayout &DL;
  const Z80Subtarget &STI;
  Register SPRegCopy;
};

struct TailCallArgHandler : public Z80OutgoingValueHandler {
  TailCallArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                     MachineInstrBuilder &MIB, CCAssignFn *AssignFn, int FPDiff)
      : Z80OutgoingValueHandler(MIRBuilder, MRI, MIB, AssignFn),
        FPDiff(FPDiff) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    MachineFunction &MF = MIRBuilder.getMF();
    int FI = MF.getFrameInfo().CreateFixedObject(Size, FPDiff + Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MF, FI);
    return MIRBuilder
        .buildFrameIndex(LLT::pointer(0, DL.getPointerSizeInBits(0)), FI)
        .getReg(0);
  }

private:
  int FPDiff;
};

struct CallArgHandler : public Z80OutgoingValueHandler {
  CallArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                 MachineInstrBuilder &MIB, CCAssignFn *AssignFn)
      : Z80OutgoingValueHandler(MIRBuilder, MRI, MIB, AssignFn),
        After(MIRBuilder.getInsertPt()) {}

  bool prepareArg(CCValAssign &VA) override {
    if (!VA.getValNo())
      Before = std::prev(MIRBuilder.getInsertPt());
    MIRBuilder.setInsertPt(MIRBuilder.getMBB(), std::next(Before));
    return Z80OutgoingValueHandler::prepareArg(VA);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    MIRBuilder.setInsertPt(MIRBuilder.getMBB(), After);
    Z80OutgoingValueHandler::assignValueToReg(ValVReg, PhysReg, VA);
  }

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    MIRBuilder.setInsertPt(MIRBuilder.getMBB(), std::next(Before));
    return Z80OutgoingValueHandler::getStackAddress(Size, Offset, MPO);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    if (MachineInstr *AddrMI = MRI.getVRegDef(Addr)) {
      unsigned Size = STI.is24Bit() ? 3 : 2;
      if (Size == VA.getLocVT().getStoreSize() &&
          AddrMI->getOpcode() == TargetOpcode::G_PTR_ADD) {
        if (MachineInstr *BaseMI =
                getDefIgnoringCopies(AddrMI->getOperand(1).getReg(), MRI)) {
          if (auto OffConst = getConstantVRegValWithLookThrough(
                  AddrMI->getOperand(2).getReg(), MRI)) {
            if (BaseMI->getOpcode() == TargetOpcode::COPY &&
                BaseMI->getOperand(1).getReg() ==
                    STI.getRegisterInfo()->getStackRegister() &&
                OffConst->Value == SetupFrameAdjustment) {
              MIRBuilder.setInsertPt(MIRBuilder.getMBB(), std::next(Before));
              MIRBuilder.buildInstr(Size == 3 ? Z80::PUSH24r : Z80::PUSH16r, {},
                                    {extendRegister(ValVReg, VA)});
              SetupFrameAdjustment += Size;
              return;
            }
          }
        }
      }
    }
    LLT PtrTy = LLT::pointer(0, DL.getPointerSizeInBits(0));
    LLT OffTy = LLT::scalar(DL.getIndexSizeInBits(0));
    auto OffI = MIRBuilder.buildConstant(OffTy, -SetupFrameAdjustment);
    Addr = MIRBuilder.buildPtrAdd(PtrTy, Addr, OffI).getReg(0);
    return Z80OutgoingValueHandler::assignValueToAddress(ValVReg, Addr, Size,
                                                         MPO, VA);
  }

  bool finalize(CCState &State) override {
    FrameSize = State.getNextStackOffset();
    MIRBuilder.setInsertPt(MIRBuilder.getMBB(), After);
    return Z80OutgoingValueHandler::finalize(State);
  }

  unsigned getPreFrameAdjustment() const {
    return 0;
  }

  unsigned getFrameSize() const {
    return FrameSize;
  }

  unsigned getFrameTotalSize() const {
    return getPreFrameAdjustment() + getFrameSize();
  }

  unsigned getSetupFrameAdjustment() const {
    return SetupFrameAdjustment;
  }

  unsigned getDestroyFrameAdjustment() const {
    return 0;
  }

protected:
  MachineBasicBlock::iterator After, Before;
  unsigned FrameSize, SetupFrameAdjustment = 0;
};

struct Z80IncomingValueHandler : public CallLowering::IncomingValueHandler {
  Z80IncomingValueHandler(MachineIRBuilder &MIRBuilder,
                          MachineRegisterInfo &MRI, CCAssignFn *AssignFn)
      : IncomingValueHandler(MIRBuilder, MRI, AssignFn),
        DL(MIRBuilder.getMF().getDataLayout()) {}

  Register getStackAddress(uint64_t Size, int64_t Offset,
                           MachinePointerInfo &MPO) override {
    auto &MFI = MIRBuilder.getMF().getFrameInfo();
    int FI = MFI.CreateFixedObject(Size, Offset, true);
    MPO = MachinePointerInfo::getFixedStack(MIRBuilder.getMF(), FI);
    LLT p0 = LLT::pointer(0, DL.getPointerSizeInBits(0));
    return MIRBuilder.buildFrameIndex(p0, FI).getReg(0);
  }

  void assignValueToAddress(Register ValVReg, Register Addr, uint64_t Size,
                            MachinePointerInfo &MPO, CCValAssign &VA) override {
    auto MMO = MIRBuilder.getMF().getMachineMemOperand(
        MPO, MachineMemOperand::MOLoad | MachineMemOperand::MOInvariant, Size,
        Align());
    MIRBuilder.buildLoad(ValVReg, Addr, *MMO);
  }

  void assignValueToReg(Register ValVReg, Register PhysReg,
                        CCValAssign &VA) override {
    markPhysRegUsed(PhysReg);

    switch (VA.getLocInfo()) {
    default: {
      // If we are copying the value from a physical register with the
      // size larger than the size of the value itself - build the copy
      // of the phys reg first and then build the truncation of that copy.
      // The example of that would be copying from xmm0 to s32, for which
      // case ValVT == LocVT == MVT::f32. If LocSize and ValSize are not equal
      // we expect this to be handled in SExt/ZExt/AExt case.
      unsigned PhysRegSize =
          MRI.getTargetRegisterInfo()->getRegSizeInBits(PhysReg, MRI);
      unsigned ValSize = VA.getValVT().getSizeInBits();
      unsigned LocSize = VA.getLocVT().getSizeInBits();
      if (PhysRegSize > ValSize && LocSize == ValSize) {
        auto Copy = MIRBuilder.buildCopy(LLT::scalar(PhysRegSize), PhysReg);
        MIRBuilder.buildTrunc(ValVReg, Copy);
        return;
      }

      MIRBuilder.buildCopy(ValVReg, PhysReg);
      break;
    }
    case CCValAssign::LocInfo::SExt:
    case CCValAssign::LocInfo::ZExt:
    case CCValAssign::LocInfo::AExt: {
      auto Copy = MIRBuilder.buildCopy(LLT{VA.getLocVT()}, PhysReg);
      MIRBuilder.buildTrunc(ValVReg, Copy);
      break;
    }
    }
  }

  /// How the physical register gets marked varies between formal
  /// parameters (it's a basic-block live-in), and a call instruction
  /// (it's an implicit-def of the BL).
  virtual void markPhysRegUsed(unsigned PhysReg) = 0;

protected:
  const DataLayout &DL;
};

struct FormalArgHandler : public Z80IncomingValueHandler {
  FormalArgHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                   CCAssignFn *AssignFn)
      : Z80IncomingValueHandler(MIRBuilder, MRI, AssignFn) {}

  void markPhysRegUsed(unsigned PhysReg) override {
    MIRBuilder.getMRI()->addLiveIn(PhysReg);
    MIRBuilder.getMBB().addLiveIn(PhysReg);
  }

  bool finalize(CCState &State) override {
    MachineFunction &MF = MIRBuilder.getMF();
    auto &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();
    FuncInfo.setArgFrameSize(State.getNextStackOffset());
    if (State.isVarArg()) {
      int FrameIdx = MF.getFrameInfo().CreateFixedObject(
          1, State.getNextStackOffset(), true);
      FuncInfo.setVarArgsFrameIndex(FrameIdx);
    }
    return true;
  }
};

struct CallReturnHandler : public Z80IncomingValueHandler {
  CallReturnHandler(MachineIRBuilder &MIRBuilder, MachineRegisterInfo &MRI,
                    CCAssignFn *AssignFn, MachineInstrBuilder &MIB)
      : Z80IncomingValueHandler(MIRBuilder, MRI, AssignFn), MIB(MIB) {}

  void markPhysRegUsed(unsigned PhysReg) override {
    MIB.addDef(PhysReg, RegState::Implicit);
  }

protected:
  MachineInstrBuilder &MIB;
};

} // end anonymous namespace

void Z80CallLowering::splitToValueTypes(const ArgInfo &OrigArg,
                                        SmallVectorImpl<ArgInfo> &SplitArgs,
                                        const DataLayout &DL,
                                        MachineRegisterInfo &MRI) const {
  const Z80TargetLowering &TLI = *getTLI<Z80TargetLowering>();
  LLVMContext &Ctx = OrigArg.Ty->getContext();

  if (OrigArg.Ty->isVoidTy())
    return;

  SmallVector<EVT, 4> SplitVTs;
  SmallVector<uint64_t, 4> Offsets;
  ComputeValueVTs(TLI, DL, OrigArg.Ty, SplitVTs, &Offsets, 0);

  for (unsigned I = 0, E = SplitVTs.size(); I != E; ++I) {
    Type *SplitTy = SplitVTs[I].getTypeForEVT(Ctx);
    SplitArgs.emplace_back(OrigArg.Regs[I], SplitTy, OrigArg.Flags[0],
                           OrigArg.IsFixed);
  }
}

/// Return true if the calling convention is one that we can guarantee TCO for.
static bool canGuaranteeTCO(CallingConv::ID CC) {
  return CC == CallingConv::Fast;
}

/// Return true if we might ever do TCO for calls with this calling convention.
static bool mayTailCallThisCC(CallingConv::ID CC) {
  switch (CC) {
  case CallingConv::C:
  case CallingConv::PreserveMost:
  case CallingConv::Z80_LibCall:
  case CallingConv::Z80_LibCall_AB:
  case CallingConv::Z80_LibCall_AC:
  case CallingConv::Z80_LibCall_BC:
  case CallingConv::Z80_LibCall_L:
  case CallingConv::Z80_LibCall_F:
  case CallingConv::Z80_TIFlags:
    return true;
  default:
    return canGuaranteeTCO(CC);
  }
}

bool Z80CallLowering::doCallerAndCalleePassArgsTheSameWay(
    CallLoweringInfo &Info, MachineFunction &MF,
    SmallVectorImpl<ArgInfo> &InArgs) const {
  const Function &CallerF = MF.getFunction();
  CallingConv::ID CalleeCC = Info.CallConv;
  CallingConv::ID CallerCC = CallerF.getCallingConv();

  // If the calling conventions match, then everything must be the same.
  if (CalleeCC == CallerCC)
    return true;

  // Check if the caller and callee will handle arguments in the same way.
  if (!resultsCompatible(Info, MF, InArgs, CC_Z80, CC_Z80, CC_Z80, CC_Z80))
    return false;

  // Make sure that the caller and callee preserve all of the same registers.
  const auto &TRI = *MF.getSubtarget<Z80Subtarget>().getRegisterInfo();
  const uint32_t *CallerPreserved = TRI.getCallPreservedMask(MF, CallerCC);
  const uint32_t *CalleePreserved = TRI.getCallPreservedMask(MF, CalleeCC);

  return TRI.regmaskSubsetEqual(CallerPreserved, CalleePreserved);
}

bool Z80CallLowering::areCalleeOutgoingArgsTailCallable(
    CallLoweringInfo &Info, MachineFunction &MF,
    SmallVectorImpl<ArgInfo> &OutArgs) const {
  // If there are no outgoing arguments, then we are done.
  if (OutArgs.empty())
    return true;

  const Function &CallerF = MF.getFunction();
  CallingConv::ID CalleeCC = Info.CallConv;
  CallingConv::ID CallerCC = CallerF.getCallingConv();

  // We have outgoing arguments. Make sure that we can tail call with them.
  SmallVector<CCValAssign, 16> OutLocs;
  CCState OutInfo(CalleeCC, false, MF, OutLocs, CallerF.getContext());

  if (!analyzeArgInfo(OutInfo, OutArgs, CC_Z80, CC_Z80)) {
    LLVM_DEBUG(dbgs() << "... Could not analyze call operands.\n");
    return false;
  }

  // Make sure that they can fit on the caller's stack.
  const auto &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();
  if (OutInfo.getNextStackOffset() > FuncInfo.getArgFrameSize()) {
    LLVM_DEBUG(dbgs() << "... Cannot fit call operands on caller's stack.\n");
    return false;
  }

  // Verify that the parameters in callee-saved registers match.
  // TODO: Port this over to CallLowering as general code once swiftself is
  // supported.
  const auto &TRI = *MF.getSubtarget<Z80Subtarget>().getRegisterInfo();
  const uint32_t *CallerPreservedMask = TRI.getCallPreservedMask(MF, CallerCC);
  MachineRegisterInfo &MRI = MF.getRegInfo();

  for (unsigned i = 0; i < OutLocs.size(); ++i) {
    auto &ArgLoc = OutLocs[i];
    // If it's not a register, it's fine.
    if (!ArgLoc.isRegLoc()) {
      if (Info.IsVarArg) {
        // Be conservative and disallow variadic memory operands to match SDAG's
        // behaviour.
        // FIXME: If the caller's calling convention is C, then we can
        // potentially use its argument area. However, for cases like fastcc,
        // we can't do anything.
        LLVM_DEBUG(
            dbgs()
            << "... Cannot tail call vararg function with stack arguments\n");
        return false;
      }
      continue;
    }

    Register Reg = ArgLoc.getLocReg();

    // Only look at callee-saved registers.
    if (MachineOperand::clobbersPhysReg(CallerPreservedMask, Reg))
      continue;

    LLVM_DEBUG(
        dbgs()
        << "... Call has an argument passed in a callee-saved register.\n");

    // Check if it was copied from.
    ArgInfo &OutInfo = OutArgs[i];

    if (OutInfo.Regs.size() > 1) {
      LLVM_DEBUG(
          dbgs() << "... Cannot handle arguments in multiple registers.\n");
      return false;
    }

    // Check if we copy the register, walking through copies from virtual
    // registers. Note that getDefIgnoringCopies does not ignore copies from
    // physical registers.
    MachineInstr *RegDef = getDefIgnoringCopies(OutInfo.Regs[0], MRI);
    if (!RegDef || RegDef->getOpcode() != TargetOpcode::COPY) {
      LLVM_DEBUG(
          dbgs()
          << "... Parameter was not copied into a VReg, cannot tail call.\n");
      return false;
    }

    // Got a copy. Verify that it's the same as the register we want.
    Register CopyRHS = RegDef->getOperand(1).getReg();
    if (CopyRHS != Reg) {
      LLVM_DEBUG(dbgs() << "... Callee-saved register was not copied into "
                           "VReg, cannot tail call.\n");
      return false;
    }
  }

  return true;
}

bool Z80CallLowering::isEligibleForTailCallOptimization(
    MachineIRBuilder &MIRBuilder, CallLoweringInfo &Info,
    SmallVectorImpl<ArgInfo> &InArgs, SmallVectorImpl<ArgInfo> &OutArgs) const {

  // Must pass all target-independent checks in order to tail call optimize.
  if (!Info.IsTailCall)
    return false;

  CallingConv::ID CalleeCC = Info.CallConv;
  MachineFunction &MF = MIRBuilder.getMF();
  const Function &CallerF = MF.getFunction();

  LLVM_DEBUG(dbgs() << "Attempting to lower call as tail call\n");

  if (Info.SwiftErrorVReg) {
    // TODO: We should handle this.
    // Note that this is also handled by the check for no outgoing arguments.
    // Proactively disabling this though, because the swifterror handling in
    // lowerCall inserts a COPY *after* the location of the call.
    LLVM_DEBUG(dbgs() << "... Cannot handle tail calls with swifterror yet.\n");
    return false;
  }

  if (!mayTailCallThisCC(CalleeCC)) {
    LLVM_DEBUG(dbgs() << "... Calling convention cannot be tail called.\n");
    return false;
  }

  // Byval parameters hand the function a pointer directly into the stack area
  // we want to reuse during a tail call. Working around this *is* possible (see
  // X86).
  //
  // FIXME: In Z80ISelLowering, this isn't worked around. Can/should we try it?
  //
  // FIXME: Check whether the callee also has an "inreg" argument.
  //
  // When the caller has a swifterror argument, we don't want to tail call
  // because would have to move into the swifterror register before the
  // tail call.
  if (any_of(CallerF.args(), [](const Argument &A) {
        return A.hasByValAttr() || A.hasInRegAttr() || A.hasSwiftErrorAttr();
      })) {
    LLVM_DEBUG(dbgs() << "... Cannot tail call from callers with byval, "
                         "inreg, or swifterror arguments\n");
    return false;
  }

  // If we have -tailcallopt, then we're done.
  if (MF.getTarget().Options.GuaranteedTailCallOpt)
    return canGuaranteeTCO(CalleeCC) && CalleeCC == CallerF.getCallingConv();

  // We don't have -tailcallopt, so we're allowed to change the ABI (sibcall).
  // Try to find cases where we can do that.

  // I want anyone implementing a new calling convention to think long and hard
  // about this assert.
  assert((!Info.IsVarArg || CalleeCC == CallingConv::C) &&
         "Unexpected variadic calling convention");

  // Verify that the incoming and outgoing arguments from the callee are
  // safe to tail call.
  if (!doCallerAndCalleePassArgsTheSameWay(Info, MF, InArgs)) {
    LLVM_DEBUG(
        dbgs()
        << "... Caller and callee have incompatible calling conventions.\n");
    return false;
  }

  if (!areCalleeOutgoingArgsTailCallable(Info, MF, OutArgs))
    return false;

  LLVM_DEBUG(dbgs() << "... Call is eligible for tail call optimization.\n");
  return true;
}

bool Z80CallLowering::lowerTailCall(MachineIRBuilder &MIRBuilder,
                                    CallLoweringInfo &Info,
                                    SmallVectorImpl<ArgInfo> &OutArgs) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const Function &F = MF.getFunction();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const auto &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();
  const Z80RegisterInfo &TRI = *STI.getRegisterInfo();
  const auto &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();

  // True when we're tail calling, but without -tailcallopt.
  bool IsSibCall = !MF.getTarget().Options.GuaranteedTailCallOpt;

  // TODO: Right now, regbankselect doesn't know how to handle the rtcGPR64
  // register class. Until we can do that, we should fall back here.
  if (F.hasFnAttribute("branch-target-enforcement")) {
    LLVM_DEBUG(
        dbgs() << "Cannot lower indirect tail calls with BTI enabled yet.\n");
    return false;
  }

  MachineInstrBuilder CallSeqStart;
  if (!IsSibCall)
    CallSeqStart = MIRBuilder.buildInstr(TII.getCallFrameSetupOpcode());

  bool Is24Bit = STI.is24Bit();
  unsigned TCRetOpc = Info.Callee.isReg()
                          ? Is24Bit ? Z80::TCRETURN24r : Z80::TCRETURN16r
                          : Is24Bit ? Z80::TCRETURN24 : Z80::TCRETURN16;
  auto MIB = MIRBuilder.buildInstrNoInsert(TCRetOpc).add(Info.Callee)
                 .addRegMask(TRI.getCallPreservedMask(MF, Info.CallConv));

  // FPDiff is the byte offset of the call's argument area from the callee's.
  // Stores to callee stack arguments will be placed in FixedStackSlots offset
  // by this amount for a tail call. In a sibling call it must be 0 because the
  // caller will deallocate the entire stack and the callee still expects its
  // arguments to begin at SP+0.
  int FPDiff = 0;

  // This will be 0 for sibcalls, potentially nonzero for tail calls produced
  // by -tailcallopt. For sibcalls, the memory operands for the call are
  // already available in the caller's incoming argument space.
  unsigned NumBytes = 0;
  if (!IsSibCall) {
    // We aren't sibcalling, so we need to compute FPDiff. We need to do this
    // before handling assignments, because FPDiff must be known for memory
    // arguments.
    unsigned NumReusableBytes = FuncInfo.getArgFrameSize();
    SmallVector<CCValAssign, 16> OutLocs;
    CCState OutInfo(Info.CallConv, false, MF, OutLocs, F.getContext());
    analyzeArgInfo(OutInfo, OutArgs, CC_Z80, CC_Z80);

    // FPDiff will be negative if this tail call requires more space than we
    // would automatically have in our incoming argument space. Positive if we
    // actually shrink the stack.
    FPDiff = NumReusableBytes - NumBytes;
  }

  // Do the actual argument marshalling.
  SmallVector<unsigned, 8> PhysRegs;
  TailCallArgHandler Handler(MIRBuilder, MRI, MIB, CC_Z80, FPDiff);
  if (!handleAssignments(Info.CallConv, Info.IsVarArg, MIRBuilder, OutArgs,
                         Handler))
    return false;

  // If we have -tailcallopt, we need to adjust the stack. We'll do the call
  // sequence start and end here.
  if (!IsSibCall) {
    MIB->getOperand(1).setImm(FPDiff);
    CallSeqStart.addImm(NumBytes).addImm(0);
    // End the call sequence *before* emitting the call. Normally, we would
    // tidy the frame up after the call. However, here, we've laid out the
    // parameters so that when SP is reset, they will be in the correct
    // location.
    MIRBuilder.buildInstr(TII.getCallFrameDestroyOpcode())
        .addImm(NumBytes).addImm(0);
  }

  // Now we can add the actual call instruction to the correct basic block.
  MIRBuilder.insertInstr(MIB);

  // If Callee is a reg, since it is used by a target specific instruction,
  // it must have a register class matching the constraint of that instruction.
  if (Info.Callee.isReg())
    MIB->getOperand(0).setReg(constrainOperandRegClass(
        MF, TRI, MRI, TII, *MF.getSubtarget().getRegBankInfo(), *MIB,
        Info.Callee, 0));

  MF.getFrameInfo().setHasTailCall();
  Info.LoweredTailCall = true;
  return true;
}

bool Z80CallLowering::lowerCall(MachineIRBuilder &MIRBuilder,
                                CallLoweringInfo &Info) const {
  MachineFunction &MF = MIRBuilder.getMF();
  const Function &F = MF.getFunction();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const DataLayout &DL = F.getParent()->getDataLayout();
  const auto &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();
  const Z80FrameLowering &TFI = *STI.getFrameLowering();
  const Z80RegisterInfo &TRI = *STI.getRegisterInfo();

  // Look through bitcasts of the callee.
  while (Info.Callee.isReg()) {
    if (MachineInstr *MI = MRI.getVRegDef(Info.Callee.getReg())) {
      switch (MI->getOpcode()) {
      case TargetOpcode::COPY:
      case TargetOpcode::G_GLOBAL_VALUE:
      case TargetOpcode::G_INTTOPTR:
      case TargetOpcode::G_CONSTANT:
        Info.Callee = MI->getOperand(1);
        continue;
      }
    }
    break;
  }

  SmallVector<ArgInfo, 8> OutArgs;
  for (const auto &OrigArg : Info.OrigArgs) {
    if (OrigArg.Regs.size() > 1)
      return false;
    splitToValueTypes(OrigArg, OutArgs, DL, MRI);
  }

  SmallVector<ArgInfo, 8> InArgs;
  if (!Info.OrigRet.Ty->isVoidTy()) {
    if (Info.OrigRet.Regs.size() > 1)
      return false;
    splitToValueTypes(Info.OrigRet, InArgs, DL, MRI);
  }

  bool CanTailCallOpt =
      isEligibleForTailCallOptimization(MIRBuilder, Info, InArgs, OutArgs);

  // We must emit a tail call if we have musttail.
  if (Info.IsMustTailCall && !CanTailCallOpt) {
    // There are types of incoming/outgoing arguments we can't handle yet, so
    // it doesn't make sense to actually die here like in ISelLowering. Instead,
    // fall back to SelectionDAG and let it try to handle this.
    LLVM_DEBUG(dbgs() << "Failed to lower musttail call as tail call\n");
    return false;
  }

  if (CanTailCallOpt)
    return lowerTailCall(MIRBuilder, Info, OutArgs);

  auto CallSeqStart = MIRBuilder.buildInstr(TII.getCallFrameSetupOpcode());

  // Create a temporarily-floating call instruction so we can add the implicit
  // uses of arg registers.
  bool Is24Bit = STI.is24Bit();
  unsigned CallOpc = Info.Callee.isReg()
                         ? Is24Bit ? Z80::CALL24r : Z80::CALL16r
                         : Is24Bit ? Z80::CALL24 : Z80::CALL16;

  auto MIB = MIRBuilder.buildInstrNoInsert(CallOpc)
                 .add(Info.Callee)
                 .addRegMask(TRI.getCallPreservedMask(MF, Info.CallConv));

  // Do the actual argument marshalling.
  CallArgHandler Handler(MIRBuilder, MRI, MIB, CC_Z80);
  if (!handleAssignments(Info.CallConv, Info.IsVarArg, MIRBuilder, OutArgs,
                         Handler))
    return false;

  // Now we can add the actual call instruction to the correct basic block.
  MIRBuilder.insertInstr(MIB);

  // If Callee is a reg, since it is used by a target specific
  // instruction, it must have a register class matching the
  // constraint of that instruction.
  if (Info.Callee.isReg())
    MIB->getOperand(0).setReg(constrainOperandRegClass(
        MF, TRI, MRI, TII, *MF.getSubtarget().getRegBankInfo(), *MIB,
        Info.Callee, 0));

  // Finally we can copy the returned value back into its virtual-register. In
  // symmetry with the arguments, the physical register must be an
  // implicit-define of the call instruction.

  if (!InArgs.empty()) {
    SmallVector<Register, 8> NewRegs;

    CallReturnHandler Handler(MIRBuilder, MRI, RetCC_Z80, MIB);
    if (!handleAssignments(Info.CallConv, Info.IsVarArg, MIRBuilder, InArgs,
                           Handler))
      return false;

    if (!NewRegs.empty()) {
      SmallVector<uint64_t, 8> Indices;
      uint64_t Index = 0;
      for (Register Reg : NewRegs) {
        Indices.push_back(Index);
        Index += MRI.getType(Reg).getSizeInBits();
      }
      MIRBuilder.buildSequence(Info.OrigRet.Regs[0], NewRegs, Indices);
    }
  }

  CallSeqStart.addImm(Handler.getFrameSize())
      .addImm(Handler.getPreFrameAdjustment())
      .addImm(Handler.getSetupFrameAdjustment());

  auto CallSeqEnd = MIRBuilder.buildInstr(TII.getCallFrameDestroyOpcode())
                        .addImm(Handler.getFrameTotalSize())
                        .addImm(Handler.getDestroyFrameAdjustment());

  // It is too early to know exactly which method will be used, however
  // sometimes a better method can be guaranteed and we can adjust the operands
  // accordingly.
  for (auto CallSeq : {CallSeqStart, CallSeqEnd}) {
    const TargetRegisterClass *ScratchRC = nullptr;
    switch (TFI.getOptimalStackAdjustmentMethod(
        MF, TII.getFrameAdjustment(*CallSeq))) {
    case Z80FrameLowering::SAM_None:
    case Z80FrameLowering::SAM_Tiny:
    case Z80FrameLowering::SAM_All:
      // These methods do not need anything.
      break;
    case Z80FrameLowering::SAM_Small:
      // This method clobbers an R register.
      ScratchRC = Is24Bit ? &Z80::R24RegClass : &Z80::R16RegClass;
      break;
    case Z80FrameLowering::SAM_Large:
      // This method also clobbers flags.
      CallSeq.addDef(Z80::F, RegState::Implicit | RegState::Dead);
      LLVM_FALLTHROUGH;
    case Z80FrameLowering::SAM_Medium:
      // These methods clobber an A register.
      ScratchRC = Is24Bit ? &Z80::A24RegClass : &Z80::A16RegClass;
      break;
    }
    if (ScratchRC)
      CallSeq.addDef(MRI.createVirtualRegister(ScratchRC),
                     RegState::Implicit | RegState::Dead);
  }

  return true;
}

bool Z80CallLowering::lowerFormalArguments(
    MachineIRBuilder &MIRBuilder, const Function &F,
    ArrayRef<ArrayRef<Register>> VRegs) const {
  MachineFunction &MF = MIRBuilder.getMF();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const DataLayout &DL = MF.getDataLayout();
  auto &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();

  SmallVector<ArgInfo, 8> SplitArgs;
  unsigned Idx = 0;
  for (auto &Arg : F.args()) {
    if (!DL.getTypeStoreSize(Arg.getType()))
      continue;

    // TODO: handle not simple cases.
    if (Arg.hasAttribute(Attribute::InReg) ||
        Arg.hasAttribute(Attribute::SwiftSelf) ||
        Arg.hasAttribute(Attribute::SwiftError) ||
        Arg.hasAttribute(Attribute::Nest) || VRegs[Idx].size() > 1)
      return false;

    if (Arg.hasAttribute(Attribute::StructRet))
      FuncInfo.setSRetReturnReg(VRegs[Idx][0]);

    ArgInfo OrigArg(VRegs[Idx], Arg.getType());
    setArgFlags(OrigArg, Idx + AttributeList::FirstArgIndex, DL, F);
    splitToValueTypes(OrigArg, SplitArgs, DL, MRI);
    Idx++;
  }

  MachineBasicBlock &MBB = MIRBuilder.getMBB();
  if (!MBB.empty())
    MIRBuilder.setInstr(*MBB.begin());

  FormalArgHandler Handler(MIRBuilder, MRI, CC_Z80);
  if (!handleAssignments(F.getCallingConv(), F.isVarArg(), MIRBuilder,
                         SplitArgs, Handler))
    return false;

  // Move back to the end of the basic block.
  MIRBuilder.setMBB(MBB);

  return true;
}

bool Z80CallLowering::lowerReturn(MachineIRBuilder &MIRBuilder,
                                  const Value *Val,
                                  ArrayRef<Register> VRegs) const {
  assert(!Val == VRegs.empty() && "Return value without a vreg");
  MachineFunction &MF = MIRBuilder.getMF();
  LLVMContext &Ctx = MF.getFunction().getContext();
  auto &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();
  const auto &STI = MF.getSubtarget<Z80Subtarget>();
  auto MIB =
      MIRBuilder.buildInstrNoInsert(STI.is24Bit() ? Z80::RET24 : Z80::RET16);

  Register SRetReturnReg = FuncInfo.getSRetReturnReg();
  assert((!SRetReturnReg || VRegs.empty()) &&
         "Struct ret should have void return");
  Type *RetTy = nullptr;
  if (SRetReturnReg) {
    VRegs = SRetReturnReg;
    RetTy = Type::getInt8PtrTy(Ctx);
  } else if (!VRegs.empty())
    RetTy = Val->getType();

  if (!VRegs.empty()) {
    const Function &F = MF.getFunction();
    MachineRegisterInfo &MRI = MF.getRegInfo();
    const DataLayout &DL = MF.getDataLayout();
    const Z80TargetLowering &TLI = *getTLI<Z80TargetLowering>();

    SmallVector<EVT, 4> SplitEVTs;
    ComputeValueVTs(TLI, DL, RetTy, SplitEVTs);
    assert(VRegs.size() == SplitEVTs.size() &&
           "For each split Type there should be exactly one VReg.");

    SmallVector<ArgInfo, 8> SplitArgs;
    for (unsigned I = 0; I < SplitEVTs.size(); ++I) {
      ArgInfo CurArgInfo = ArgInfo{VRegs[I], SplitEVTs[I].getTypeForEVT(Ctx)};
      setArgFlags(CurArgInfo, AttributeList::ReturnIndex, DL, F);
      splitToValueTypes(CurArgInfo, SplitArgs, DL, MRI);
    }

    Z80OutgoingValueHandler Handler(MIRBuilder, MRI, MIB, RetCC_Z80);
    if (!handleAssignments(F.getCallingConv(), F.isVarArg(), MIRBuilder,
                           SplitArgs, Handler))
      return false;
  }

  MIRBuilder.insertInstr(MIB);
  return true;
}
