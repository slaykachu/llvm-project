//===- Z80LegalizerInfo.cpp --------------------------------------*- C++ -*-==//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
/// \file
/// This file implements the targeting of the Machinelegalizer class for Z80.
/// \todo This should be generated by TableGen.
//===----------------------------------------------------------------------===//

#include "Z80LegalizerInfo.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80Subtarget.h"
#include "Z80TargetMachine.h"
#include "llvm/CodeGen/GlobalISel/LegalizerHelper.h"
#include <initializer_list>
using namespace llvm;
using namespace TargetOpcode;
using namespace LegalizeActions;

Z80LegalizerInfo::Z80LegalizerInfo(const Z80Subtarget &STI,
                                   const Z80TargetMachine &TM)
    : Subtarget(STI), TM(TM) {
  bool Is24Bit = Subtarget.is24Bit();
  LLT p0 = LLT::pointer(0, TM.getPointerSizeInBits(0));
  LLT s1 = LLT::scalar(1);
  LLT s8 = LLT::scalar(8);
  LLT s16 = LLT::scalar(16);
  LLT s24 = LLT::scalar(24);
  LLT s32 = LLT::scalar(32);
  LLT s64 = LLT::scalar(64);
  LLT sMax = Is24Bit ? s24 : s16;
  auto LegalTypes24 = {p0, s8, s16, s24}, LegalTypes16 = {p0, s8, s16};
  auto LegalTypes = Is24Bit ? LegalTypes24 : LegalTypes16;
  auto LegalScalars24 = {s8, s16, s24}, LegalScalars16 = {s8, s16};
  auto LegalScalars = Is24Bit ? LegalScalars24 : LegalScalars16;
  auto LegalLibcallScalars24 = {s8, s16, s24, s32, s64};
  auto LegalLibcallScalars16 = {s8, s16, s32, s64};
  auto LegalLibcallScalars =
      Is24Bit ? LegalLibcallScalars24 : LegalLibcallScalars16;
  auto NotMax24 = {s8, s16}, NotMax16 = {s8};
  auto NotMax = Is24Bit ? NotMax24 : NotMax16;
  auto NotMin24 = {s16, s24}, NotMin16 = {s16};
  auto NotMin = Is24Bit ? NotMin24 : NotMin16;
  auto NotMaxWithOne24 = {s1, s8, s16}, NotMaxWithOne16 = {s1, s8};
  auto NotMaxWithOne = Is24Bit ? NotMaxWithOne24 : NotMaxWithOne16;

  getActionDefinitionsBuilder(G_MERGE_VALUES)
      .legalForCartesianProduct(NotMin, NotMax)
      .clampScalar(0, *NotMin.begin(), *std::prev(NotMin.end()))
      .clampScalar(1, *NotMax.begin(), *std::prev(NotMax.end()));

  getActionDefinitionsBuilder(G_UNMERGE_VALUES)
      .legalForCartesianProduct(NotMax, NotMin)
      .clampScalar(1, *NotMin.begin(), *std::prev(NotMin.end()))
      .clampScalar(0, *NotMax.begin(), *std::prev(NotMax.end()));

  getActionDefinitionsBuilder(G_INSERT)
      .customForCartesianProduct(NotMin, {s8})
      .unsupported();

  getActionDefinitionsBuilder(G_EXTRACT)
      .customForCartesianProduct({s8}, NotMin)
      .unsupported();

  getActionDefinitionsBuilder({G_ZEXT, G_ANYEXT})
      .legalForCartesianProduct(LegalScalars, NotMaxWithOne)
      .clampScalar(0, *LegalScalars.begin(), *std::prev(LegalScalars.end()))
      .clampScalar(1, *NotMaxWithOne.begin(), *std::prev(NotMaxWithOne.end()));

  getActionDefinitionsBuilder(G_SEXT)
      .legalForCartesianProduct(LegalScalars, {s1})
      .maxScalar(0, sMax)
      .maxScalar(0, s8)
      .maxScalar(1, s8);

  getActionDefinitionsBuilder(G_TRUNC)
      .legalForCartesianProduct(NotMaxWithOne, LegalScalars)
      .clampScalar(1, *LegalScalars.begin(), *std::prev(LegalScalars.end()))
      .clampScalar(0, *NotMaxWithOne.begin(), *std::prev(NotMaxWithOne.end()));

  getActionDefinitionsBuilder({G_FREEZE, G_IMPLICIT_DEF, G_PHI, G_CONSTANT})
      .legalFor(LegalTypes)
      .clampScalar(0, s8, sMax);

  getActionDefinitionsBuilder(G_FCONSTANT)
      .customFor({s32, s64});

  getActionDefinitionsBuilder(G_INTTOPTR)
      .legalFor({{p0, sMax}})
      .clampScalar(1, sMax, sMax);

  getActionDefinitionsBuilder(G_PTRTOINT)
      .legalFor({{sMax, p0}})
      .clampScalar(0, sMax, sMax);

  getActionDefinitionsBuilder(G_PTR_ADD)
      .legalForCartesianProduct({p0}, LegalScalars)
      .clampScalar(1, s8, sMax);

  getActionDefinitionsBuilder({G_ADD, G_SUB})
      .legalFor(LegalScalars)
      .libcallFor({s32, s64})
      .clampScalar(0, s8, sMax);

  getActionDefinitionsBuilder({G_UADDO, G_UADDE, G_USUBO, G_USUBE,
                               G_SADDO, G_SADDE, G_SSUBO, G_SSUBE})
      .legalForCartesianProduct(LegalScalars, {s1})
      .clampScalar(0, s8, sMax);

  {
    auto &&Mul = getActionDefinitionsBuilder(G_MUL);
    if (Subtarget.hasZ180Ops())
      Mul.legalFor({s8});
    Mul.libcallFor(LegalLibcallScalars)
        .clampScalar(0, s8, s32);
  }

  getActionDefinitionsBuilder({G_SDIV, G_UDIV, G_SREM, G_UREM})
      .libcallFor(LegalLibcallScalars)
      .clampScalar(0, s8, s32);

  getActionDefinitionsBuilder({G_AND, G_OR, G_XOR})
      .legalFor({s8})
      .customFor(LegalLibcallScalars)
      .clampScalar(0, s8, s32);

  getActionDefinitionsBuilder({G_SHL, G_LSHR, G_ASHR})
      .customForCartesianProduct(LegalLibcallScalars, {s8})
      .clampScalar(1, s8, s8)
      .clampScalar(0, s8, s64);

  getActionDefinitionsBuilder({G_FSHL, G_FSHR, G_MEMCPY, G_MEMMOVE, G_MEMSET})
      .custom();

  getActionDefinitionsBuilder({G_FADD, G_FSUB, G_FMUL, G_FDIV, G_FREM, G_FNEG,
                               G_FABS, G_INTRINSIC_TRUNC, G_INTRINSIC_ROUND,
                               G_FCEIL, G_FCOS, G_FSIN, G_FSQRT, G_FFLOOR,
                               G_FRINT, G_FNEARBYINT})
      .libcallFor({s32, s64});

  getActionDefinitionsBuilder(G_FCOPYSIGN)
      .libcallFor({{s32, s32}, {s64, s64}});

  getActionDefinitionsBuilder(G_FPTRUNC)
      .libcallFor({{s32, s64}});

  getActionDefinitionsBuilder(G_FPEXT)
      .libcallFor({{s64, s32}});

  getActionDefinitionsBuilder({G_FPTOSI, G_FPTOUI})
      .libcallForCartesianProduct({s32, s64}, {s32, s64})
      .clampScalar(0, s32, s64);

  getActionDefinitionsBuilder({G_SITOFP, G_UITOFP})
      .libcallForCartesianProduct({s32, s64}, {s32, s64})
      .clampScalar(1, s32, s64);

  getActionDefinitionsBuilder({G_LOAD, G_STORE})
      .legalForCartesianProduct(LegalTypes, {p0})
      .clampScalar(0, s8, sMax);
  for (unsigned MemOp : {G_LOAD, G_STORE})
    setLegalizeScalarToDifferentSizeStrategy(MemOp, 0,
                                             narrowToSmallerAndWidenToSmallest);

  getActionDefinitionsBuilder(
      {G_FRAME_INDEX, G_GLOBAL_VALUE, G_BRINDIRECT, G_JUMP_TABLE})
      .legalFor({p0});

  getActionDefinitionsBuilder(G_VASTART)
      .customFor({p0});

  getActionDefinitionsBuilder(G_ICMP)
      .legalForCartesianProduct({s1}, LegalTypes)
      .customForCartesianProduct({s1}, {s32, s64})
      .clampScalar(1, s8, s32);

  getActionDefinitionsBuilder(G_FCMP)
      .customForCartesianProduct({s1}, {s32, s64});

  getActionDefinitionsBuilder(G_BRCOND)
      .legalFor({s1});

  getActionDefinitionsBuilder(G_BRJT)
      .legalForCartesianProduct({p0}, LegalScalars)
      .clampScalar(1, s8, sMax);

  getActionDefinitionsBuilder(G_SELECT)
      .legalForCartesianProduct(LegalTypes, {s1})
      .clampScalar(0, s8, sMax);

  getActionDefinitionsBuilder(
      {G_ABS, G_DYN_STACKALLOC, G_SEXT_INREG, G_CTLZ_ZERO_UNDEF,
       G_CTTZ_ZERO_UNDEF, G_CTLZ, G_CTTZ, G_BSWAP, G_SMULO, G_UMULO, G_SMULH,
       G_UMULH, G_UADDSAT, G_SADDSAT, G_USUBSAT, G_SSUBSAT})
      .lower();

  getActionDefinitionsBuilder(G_CTPOP)
      .libcallForCartesianProduct({s8}, LegalLibcallScalars)
      .clampScalar(0, s8, s8);

  getActionDefinitionsBuilder(G_BITREVERSE)
      .libcallFor(LegalLibcallScalars)
      .clampScalar(0, s8, s64);

  computeTables();
  verify(*STI.getInstrInfo());
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeCustomMaybeLegal(LegalizerHelper &Helper,
                                           MachineInstr &MI) const {
  Helper.MIRBuilder.setInstrAndDebugLoc(MI);
  switch (MI.getOpcode()) {
  default:
    // No idea what to do.
    return LegalizerHelper::UnableToLegalize;
  case G_AND:
  case G_OR:
  case G_XOR:
    return legalizeBitwise(Helper, MI);
  case G_EXTRACT:
    return legalizeExtractInsert(Helper, MI);
  case G_FCONSTANT:
    return legalizeFConstant(Helper, MI);
  case G_VASTART:
    return legalizeVAStart(Helper, MI);
  case G_SHL:
  case G_LSHR:
  case G_ASHR:
    return legalizeShift(Helper, MI);
  case G_FSHL:
  case G_FSHR:
    return legalizeFunnelShift(Helper, MI);
  case G_ICMP:
  case G_FCMP:
    return legalizeCompare(Helper, MI);
  case G_MEMCPY:
  case G_MEMMOVE:
  case G_MEMSET:
    return legalizeMemIntrinsic(Helper, MI);
  }
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeBitwise(LegalizerHelper &Helper,
                                  MachineInstr &MI) const {
  assert((MI.getOpcode() == G_AND || MI.getOpcode() == G_OR ||
          MI.getOpcode() == G_XOR) &&
         "Unexpected opcode");
  if (!MI.getParent()->getParent()->getFunction().hasOptSize() &&
      Helper.MIRBuilder.getMRI()->getType(MI.getOperand(0).getReg()) ==
          LLT::scalar(16))
    if (Helper.narrowScalar(MI, 0, LLT::scalar(8)) ==
        LegalizerHelper::Legalized)
      return LegalizerHelper::Legalized;
  return Helper.libcall(MI);
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeExtractInsert(LegalizerHelper &Helper,
                                        MachineInstr &MI) const {
  bool OpOff = MI.getOpcode() == G_INSERT;
  assert((MI.getOpcode() == G_EXTRACT || OpOff) && "Unexpected opcode");
  return MI.getOperand(2 + OpOff).getImm() & 7
             ? LegalizerHelper::UnableToLegalize
             : LegalizerHelper::Legalized;
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeFConstant(LegalizerHelper &Helper,
                                    MachineInstr &MI) const {
  assert(MI.getOpcode() == G_FCONSTANT && "Unexpected opcode");
  Helper.Observer.changingInstr(MI);
  MI.setDesc(Helper.MIRBuilder.getTII().get(G_CONSTANT));
  MachineOperand &Imm = MI.getOperand(1);
  const ConstantFP *FPImm = Imm.getFPImm();
  Imm.ChangeToCImmediate(ConstantInt::get(
      FPImm->getContext(), FPImm->getValueAPF().bitcastToAPInt()));
  Helper.Observer.changedInstr(MI);
  return LegalizerHelper::Legalized;
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeVAStart(LegalizerHelper &Helper,
                                  MachineInstr &MI) const {
  assert(MI.getOpcode() == G_VASTART && "Unexpected opcode");
  MachineFunction &MF = Helper.MIRBuilder.getMF();
  Z80MachineFunctionInfo &FuncInfo = *MF.getInfo<Z80MachineFunctionInfo>();
  int FrameIdx = FuncInfo.getVarArgsFrameIndex();
  assert(FrameIdx && "Found va_start but never setVarArgsFrameIndex!");
  LLT p0 = LLT::pointer(0, TM.getPointerSizeInBits(0));
  Helper.MIRBuilder.buildStore(Helper.MIRBuilder.buildFrameIndex(p0, FrameIdx),
                               MI.getOperand(0).getReg(),
                               **MI.memoperands_begin());
  MI.eraseFromParent();
  return LegalizerHelper::Legalized;
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeShift(LegalizerHelper &Helper,
                                MachineInstr &MI) const {
  assert((MI.getOpcode() == G_SHL || MI.getOpcode() == G_LSHR ||
          MI.getOpcode() == G_ASHR) &&
         "Unexpected opcode");
  MachineRegisterInfo &MRI = *Helper.MIRBuilder.getMRI();
  Register DstReg = MI.getOperand(0).getReg();
  LLT Ty = MRI.getType(DstReg);
  if (auto Amt =
          getConstantVRegValWithLookThrough(MI.getOperand(2).getReg(), MRI)) {
    if (Ty == LLT::scalar(8) && Amt->Value == 1)
      return LegalizerHelper::AlreadyLegal;
    if (MI.getOpcode() == G_SHL && Amt->Value == 1) {
      Helper.Observer.changingInstr(MI);
      MI.setDesc(Helper.MIRBuilder.getTII().get(G_ADD));
      MI.getOperand(2).setReg(MI.getOperand(1).getReg());
      Helper.Observer.changedInstr(MI);
      return LegalizerHelper::Legalized;
    }
    if (MI.getOpcode() == G_ASHR && Amt->Value == Ty.getSizeInBits() - 1 &&
        (Ty == LLT::scalar(8) || Ty == LLT::scalar(16) ||
         (Subtarget.is24Bit() && Ty == LLT::scalar(24))))
      return LegalizerHelper::Legalized;
  }
  return Helper.libcall(MI);
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeFunnelShift(LegalizerHelper &Helper,
                                      MachineInstr &MI) const {
  assert((MI.getOpcode() == G_FSHL || MI.getOpcode() == G_FSHR) &&
         "Unexpected opcode");
  MachineIRBuilder &MIRBuilder = Helper.MIRBuilder;
  MachineRegisterInfo &MRI = *MIRBuilder.getMRI();
  Register DstReg = MI.getOperand(0).getReg();
  Register FwdReg = MI.getOperand(1).getReg();
  Register RevReg = MI.getOperand(2).getReg();
  Register AmtReg = MI.getOperand(3).getReg();
  LLT Ty = MRI.getType(DstReg);
  if (auto Amt = getConstantVRegValWithLookThrough(AmtReg, MRI))
    if (Ty == LLT::scalar(8) && (Amt->Value == 1 || Amt->Value == 7))
        return LegalizerHelper::AlreadyLegal;

  unsigned FwdShiftOpc = G_SHL;
  unsigned RevShiftOpc = G_LSHR;
  if (MI.getOpcode() == G_FSHR) {
    std::swap(FwdReg, RevReg);
    std::swap(FwdShiftOpc, RevShiftOpc);
  }

  auto MaskI = MIRBuilder.buildConstant(Ty, Ty.getSizeInBits() - 1);
  auto FwdAmtI = MIRBuilder.buildAnd(Ty, AmtReg, MaskI);
  auto FwdI = MIRBuilder.buildInstr(FwdShiftOpc, {Ty}, {FwdReg, FwdAmtI});
  auto RevAmtI = MIRBuilder.buildAnd(
      Ty, MIRBuilder.buildSub(Ty, MIRBuilder.buildConstant(Ty, 0), AmtReg),
      MaskI);
  auto RevI = MIRBuilder.buildInstr(RevShiftOpc, {Ty}, {RevReg, RevAmtI});
  MIRBuilder.buildOr(DstReg, FwdI, RevI);
  MI.eraseFromParent();
  return LegalizerHelper::Legalized;
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeCompare(LegalizerHelper &Helper,
                                  MachineInstr &MI) const {
  MachineIRBuilder &MIRBuilder = Helper.MIRBuilder;
  MachineRegisterInfo &MRI = *MIRBuilder.getMRI();
  Register DstReg = MI.getOperand(0).getReg();
  auto Pred = CmpInst::Predicate(MI.getOperand(1).getPredicate());
  Register LHSReg = MI.getOperand(2).getReg();
  Register RHSReg = MI.getOperand(3).getReg();
  LLT OpTy = MRI.getType(LHSReg);
  unsigned OpSize = OpTy.getSizeInBits();
  assert(MRI.getType(DstReg) == LLT::scalar(1) && !OpTy.isVector() &&
         MRI.getType(RHSReg) == OpTy && "Unexpected type");

  Type *Ty;
  RTLIB::Libcall Libcall;
  bool IsSigned, IsSwapped, IsConst;
  Z80::CondCode CC =
      Z80::GetBranchConditionForPredicate(Pred, IsSigned, IsSwapped, IsConst);
  if (IsSwapped)
    std::swap(LHSReg, RHSReg);
  auto &Ctx = MIRBuilder.getMF().getFunction().getContext();
  bool ZeroRHS = false;
  if (MI.getOpcode() == G_ICMP) {
    Ty = IntegerType::get(Ctx, OpSize);
    if (auto C = getConstantVRegVal(RHSReg, MRI))
      ZeroRHS = *C == 0;
    switch (OpSize) {
    case 32:
      Libcall = ZeroRHS ? RTLIB::CMP_I32_0 : RTLIB::CMP_I32;
      break;
    case 64:
      Libcall = ZeroRHS ? RTLIB::CMP_I64_0 : RTLIB::CMP_I64;
      break;
    default:
      llvm_unreachable("Unexpected type");
    }
  } else {
    assert(MI.getOpcode() == G_FCMP && "Unexpected opcode");
    assert(OpTy.isScalar() && "Unexpected type");
    switch (OpSize) {
    case 32:
      Ty = Type::getFloatTy(Ctx);
      Libcall = RTLIB::CMP_F32;
      break;
    case 64:
      Ty = Type::getDoubleTy(Ctx);
      Libcall = RTLIB::CMP_F64;
      break;
    default:
      llvm_unreachable("Unexpected type");
    }
  }
  if (!IsConst) {
    LLT s8 = LLT::scalar(8);
    Type *Int8Ty = Type::getInt8Ty(Ctx);
    Register FlagsReg = MRI.createGenericVirtualRegister(s8);
    CallLowering::ArgInfo FlagsArg(FlagsReg, Int8Ty);
    CallLowering::ArgInfo Args[2] = {{LHSReg, Ty}, {RHSReg, Ty}};
    createLibcall(MIRBuilder, Libcall, FlagsArg,
                  makeArrayRef(Args, 2 - ZeroRHS));
    if (IsSigned) {
      Register SignedFlagsReg = MRI.createGenericVirtualRegister(s8);
      CallLowering::ArgInfo SignedFlagsArg(SignedFlagsReg, Int8Ty);
      createLibcall(MIRBuilder, RTLIB::SCMP, SignedFlagsArg, FlagsArg);
      FlagsReg = SignedFlagsReg;
    }
    MIRBuilder.buildCopy(Register(Z80::F), FlagsReg);
  } else
    MIRBuilder.buildInstr(Z80::RCF);
  MIRBuilder.buildInstr(Z80::SetCC, {DstReg}, {int64_t(CC)});
  MI.eraseFromParent();
  return LegalizerHelper::Legalized;
}

LegalizerHelper::LegalizeResult
Z80LegalizerInfo::legalizeMemIntrinsic(LegalizerHelper &Helper,
                                       MachineInstr &MI) const {
  MachineIRBuilder &MIRBuilder = Helper.MIRBuilder;
  MIRBuilder.setInstrAndDebugLoc(MI);
  MachineRegisterInfo &MRI = *MIRBuilder.getMRI();
  MachineFunction &MF = MIRBuilder.getMF();
  const DataLayout &DL = MF.getDataLayout();
  bool Is24Bit = Subtarget.is24Bit();

  unsigned Opc = MI.getOpcode();
  assert((Opc == G_MEMCPY || Opc == G_MEMMOVE || Opc == G_MEMSET) &&
         "Unexpected opcode");

  Register DstReg = MI.getOperand(0).getReg();
  LLT DstTy = MRI.getType(DstReg);

  Register SrcReg = MI.getOperand(1).getReg();

  Register LenReg = MI.getOperand(2).getReg();
  LLT LenTy = LLT::scalar(DL.getIndexSizeInBits(DstTy.getAddressSpace()));
  LenReg = MIRBuilder.buildZExtOrTrunc(LenTy, LenReg).getReg(0);

  // We need to make sure the number of bytes is non-zero for this lowering to
  // be correct.  Since we only need to lower constant-length intrinsics for
  // now, just support those.
  if (auto ConstLen = getConstantVRegVal(LenReg, MRI)) {
    // Doing something with zero bytes is a noop anyway.
    if (!*ConstLen) {
      MI.eraseFromParent();
      return LegalizerHelper::Legalized;
    }
    // Lowering memmove generates a lot of code...
    if (!MF.getFunction().hasOptSize() || Opc != TargetOpcode::G_MEMMOVE) {
      if (Opc == TargetOpcode::G_MEMSET) {
        // Store the first byte.
        MIRBuilder.buildStore(SrcReg, DstReg, *MI.memoperands().front());
        // If we are only storing one byte, we are done now.
        // TODO: lower small len to a series of stores.
        if (*ConstLen == 1) {
          MI.eraseFromParent();
          return LegalizerHelper::Legalized;
        }
        // Read starting at the stored byte.
        SrcReg = DstReg;
        // Write starting at the following byte.
        auto One =
            MIRBuilder.buildConstant(LLT::scalar(DstTy.getSizeInBits()), 1);
        DstReg = MIRBuilder.buildPtrAdd(DstTy, DstReg, One).getReg(0);
        // Copy one less byte.
        auto NegOne =
            MIRBuilder.buildConstant(LenTy, -1);
        LenReg = MIRBuilder.buildAdd(LenTy, LenReg, NegOne).getReg(0);
        // Now it's just an ldir.
      }

      Register DE = Is24Bit ? Z80::UDE : Z80::DE;
      Register HL = Is24Bit ? Z80::UHL : Z80::HL;
      Register BC = Is24Bit ? Z80::UBC : Z80::BC;
      if (Opc == TargetOpcode::G_MEMMOVE) {
        MIRBuilder.buildCopy(HL, SrcReg);
        MIRBuilder.buildInstr(Is24Bit ? Z80::CP24ao : Z80::CP16ao, {},
                              {DstReg});
        MIRBuilder.buildInstr(Is24Bit ? Z80::LDR24 : Z80::LDR16, {},
                              {DstReg, SrcReg, LenReg}).cloneMemRefs(MI);
      } else {
        // TODO: lower small len to a series of loads and stores.
        MIRBuilder.buildCopy(DE, DstReg);
        MIRBuilder.buildCopy(HL, SrcReg);
        MIRBuilder.buildCopy(BC, LenReg);
        MIRBuilder.buildInstr(Is24Bit ? Z80::LDIR24 : Z80::LDIR16)
            .cloneMemRefs(MI);
      }
      MI.eraseFromParent();
      return LegalizerHelper::Legalized;
    }
  }

  MI.getOperand(2).setReg(LenReg);
  auto Result = createMemLibcall(MIRBuilder, MRI, MI);
  MI.eraseFromParent();
  return Result;
}

bool Z80LegalizerInfo::legalizeIntrinsic(LegalizerHelper &Helper,
                                         MachineInstr &MI) const {
  MachineIRBuilder &MIRBuilder = Helper.MIRBuilder;
  MIRBuilder.setInstrAndDebugLoc(MI);
  MachineFunction &MF = MIRBuilder.getMF();
  auto &Ctx = MF.getFunction().getContext();
  auto &CLI = *MF.getSubtarget().getCallLowering();

  switch (MI.getIntrinsicID()) {
  case Intrinsic::trap: {
    CallLowering::CallLoweringInfo Info;
    Info.CallConv = CallingConv::C;
    Info.Callee = MachineOperand::CreateES("abort");
    Info.OrigRet = CallLowering::ArgInfo({0}, Type::getVoidTy(Ctx));
    if (!CLI.lowerCall(MIRBuilder, Info))
      return false;
    break;
  }
  default:
    return false;
  }

  MI.eraseFromParent();
  return true;
}
