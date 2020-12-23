//===-- Z80RegisterInfo.cpp - Z80 Register Information --------------------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file contains the Z80 implementation of the TargetRegisterInfo class.
//
//===----------------------------------------------------------------------===//

#include "Z80RegisterInfo.h"
#include "Z80FrameLowering.h"
#include "Z80MachineFunctionInfo.h"
#include "Z80Subtarget.h"
#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "llvm/CodeGen/MachineFrameInfo.h"
#include "llvm/CodeGen/MachineFunction.h"
#include "llvm/CodeGen/MachineInstrBuilder.h"
#include "llvm/CodeGen/MachineRegisterInfo.h"
#include "llvm/CodeGen/RegisterScavenging.h"
#include "llvm/CodeGen/TargetFrameLowering.h"
#include "llvm/Support/Debug.h"
using namespace llvm;

#define DEBUG_TYPE "z80reginfo"

#define GET_REGINFO_TARGET_DESC
#include "Z80GenRegisterInfo.inc"

Z80RegisterInfo::Z80RegisterInfo(const Triple &TT)
    : Z80GenRegisterInfo(Z80::PC) {
  // Cache some information.
  Is24Bit = !TT.isArch16Bit() && TT.getEnvironment() != Triple::CODE16;

  // Use a callee-saved register as the base pointer.  These registers must
  // not conflict with any ABI requirements.
  if (Is24Bit) {
    SlotSize = 3;
    StackPtr = Z80::SPL;
  } else {
    SlotSize = 2;
    StackPtr = Z80::SPS;
  }
}

const TargetRegisterClass *
Z80RegisterInfo::getPointerRegClass(const MachineFunction &MF,
                                    unsigned Kind) const {
  switch (Kind) {
  default: llvm_unreachable("Unexpected Kind in getPointerRegClass!");
  case 0: return Is24Bit ? &Z80::G24RegClass : &Z80::G16RegClass;
  case 1: return Is24Bit ? &Z80::A24RegClass : &Z80::A16RegClass;
  case 2: return Is24Bit ? &Z80::I24RegClass : &Z80::I16RegClass;
  }
}

const TargetRegisterClass *
Z80RegisterInfo::getLargestLegalSuperClass(const TargetRegisterClass *RC,
                                           const MachineFunction &) const {
  const TargetRegisterClass *Super = RC;
  TargetRegisterClass::sc_iterator I = RC->getSuperClasses();
  do {
    switch (Super->getID()) {
    case Z80::R8RegClassID:
    case Z80::R16RegClassID:
    case Z80::R24RegClassID:
      return Super;
    }
    Super = *I++;
  } while (Super);
  return RC;
}

unsigned Z80RegisterInfo::getRegPressureLimit(const TargetRegisterClass *RC,
                                              MachineFunction &MF) const {
  return 3;

  switch (RC->getID()) {
  default:
    return 0;
  case Z80::R16RegClassID:
  case Z80::R24RegClassID:
    return 2;
  }
}

const MCPhysReg *
Z80RegisterInfo::getCalleeSavedRegs(const MachineFunction *MF) const {
  switch (MF->getFunction().getCallingConv()) {
  default: llvm_unreachable("Unsupported calling convention");
  case CallingConv::C:
  case CallingConv::Fast:
    return Is24Bit ? CSR_EZ80_C_SaveList : CSR_Z80_C_SaveList;
  case CallingConv::PreserveAll:
  case CallingConv::Z80_LibCall:
  case CallingConv::Z80_LibCall_AB:
  case CallingConv::Z80_LibCall_AC:
  case CallingConv::Z80_LibCall_BC:
  case CallingConv::Z80_LibCall_L:
  case CallingConv::Z80_LibCall_F:
    return Is24Bit ? CSR_EZ80_AllRegs_SaveList : CSR_Z80_AllRegs_SaveList;
  case CallingConv::Z80_TIFlags:
    return Is24Bit ? CSR_EZ80_TIFlags_SaveList : CSR_Z80_TIFlags_SaveList;
  }
}

const uint32_t *
Z80RegisterInfo::getCallPreservedMask(const MachineFunction &MF,
                                      CallingConv::ID CC) const {
  switch (CC) {
  default: llvm_unreachable("Unsupported calling convention");
  case CallingConv::C:
  case CallingConv::Fast:
    return Is24Bit ? CSR_EZ80_C_RegMask : CSR_Z80_C_RegMask;
  case CallingConv::PreserveAll:
  case CallingConv::Z80_LibCall:
  case CallingConv::Z80_LibCall_AB:
  case CallingConv::Z80_LibCall_AC:
  case CallingConv::Z80_LibCall_BC:
  case CallingConv::Z80_LibCall_L:
  case CallingConv::Z80_LibCall_F:
    return Is24Bit ? CSR_EZ80_AllRegs_RegMask : CSR_Z80_AllRegs_RegMask;
  case CallingConv::Z80_TIFlags:
    return Is24Bit ? CSR_EZ80_TIFlags_RegMask : CSR_Z80_TIFlags_RegMask;
  }
}
const uint32_t *Z80RegisterInfo::getNoPreservedMask() const {
  return CSR_NoRegs_RegMask;
}

BitVector Z80RegisterInfo::getReservedRegs(const MachineFunction &MF) const {
  BitVector Reserved(getNumRegs());

  // Set the stack-pointer registers as reserved.
  Reserved.set(Z80::SPS);
  Reserved.set(Z80::SPL);

  // Set the program-counter register as reserved.
  Reserved.set(Z80::PC);

  // Set the frame-pointer register and its aliases as reserved if needed.
  for (Register Reg :
       {Register(Is24Bit ? Z80::UIX : Z80::IX), getFrameRegister(MF)})
    for (MCRegAliasIterator I(Reg, this, /*IncludeSelf=*/true); I.isValid();
         ++I)
      Reserved.set(*I);

  return Reserved;
}

bool Z80RegisterInfo::saveScavengerRegister(MachineBasicBlock &MBB,
                                            MachineBasicBlock::iterator MI,
                                            MachineBasicBlock::iterator &UseMI,
                                            const TargetRegisterClass *RC,
                                            Register Reg) const {
  return false;
  const Z80Subtarget &STI = MBB.getParent()->getSubtarget<Z80Subtarget>();
  const TargetInstrInfo &TII = *STI.getInstrInfo();
  DebugLoc DL;
  if (Reg == Z80::AF)
    BuildMI(MBB, MI, DL, TII.get(Is24Bit ? Z80::PUSH24AF : Z80::PUSH16AF));
  else
    BuildMI(MBB, MI, DL, TII.get(Is24Bit ? Z80::PUSH24r : Z80::PUSH16r))
        .addReg(Reg);
  for (MachineBasicBlock::iterator II = MI; II != UseMI; ++II) {
    if (II->isDebugValue())
      continue;
    if (II->modifiesRegister(Reg, this))
      UseMI = II;
  }
  if (Reg == Z80::AF)
    BuildMI(MBB, UseMI, DL, TII.get(Is24Bit ? Z80::POP24AF : Z80::POP16AF));
  else
    BuildMI(MBB, UseMI, DL, TII.get(Is24Bit ? Z80::POP24r : Z80::POP16r), Reg);
  return true;
}

void Z80RegisterInfo::eliminateFrameIndex(MachineBasicBlock::iterator II,
                                          int SPAdj, unsigned FIOperandNum,
                                          RegScavenger *RS) const {
  MachineInstr &MI = *II;
  unsigned Opc = MI.getOpcode();
  MachineBasicBlock &MBB = *MI.getParent();
  MachineFunction &MF = *MBB.getParent();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();
  const Z80FrameLowering *TFI = getFrameLowering(MF);
  (void)TFI;
  DebugLoc DL = MI.getDebugLoc();
  int FrameIndex = MI.getOperand(FIOperandNum).getIndex();
  Register BasePtr = getFrameRegister(MF);
  LLVM_DEBUG(MF.dump(); II->dump();
             dbgs() << MF.getFunction().arg_size() << '\n');
  assert(TFI->hasFP(MF) && "Stack slot use without fp unimplemented");
  int BaseOff = MF.getFrameInfo().getObjectOffset(FrameIndex);
  int SlotSize = Is24Bit ? 3 : 2;
  // Skip any saved callee saved registers
  if (TFI->hasFP(MF))
    BaseOff += SlotSize;
  // Skip return address for arguments
  if (FrameIndex < 0)
    BaseOff += SlotSize;
  int Off = BaseOff + getFrameIndexInstrOffset(&MI, FIOperandNum);
  if (isFrameOffsetLegal(&MI, BasePtr, BaseOff) &&
      (Opc != Z80::LEA16ro || STI.hasEZ80Ops())) {
    MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
    MI.getOperand(FIOperandNum + 1).ChangeToImmediate(Off);
    return;
  }
  bool SaveFlags = RS->isRegUsed(Z80::F);
  Register OffReg = RS->scavengeRegister(
      Is24Bit ? &Z80::O24RegClass : &Z80::O16RegClass, II, SPAdj);
  if ((Opc == Z80::LEA24ro &&
       Z80::A24RegClass.contains(MI.getOperand(0).getReg())) ||
      (Opc == Z80::LEA16ro &&
       Z80::A16RegClass.contains(MI.getOperand(0).getReg()))) {
    Register Op0Reg = MI.getOperand(0).getReg();
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::LD24ri : Z80::LD16ri), OffReg)
        .addImm(Off);
    TII.copyPhysReg(MBB, II, DL, Op0Reg, BasePtr);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::PUSH24AF : Z80::PUSH16AF));
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::ADD24ao : Z80::ADD16ao), Op0Reg)
        .addReg(Op0Reg).addReg(OffReg, RegState::Kill)
        ->addRegisterDead(Z80::F, this);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::POP24AF : Z80::POP16AF));
    MI.eraseFromParent();
  } else if (Register ScratchReg = RS->FindUnusedReg(
                 Is24Bit ? &Z80::A24RegClass : &Z80::A16RegClass)) {
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::LD24ri : Z80::LD16ri), OffReg)
        .addImm(Off);
    TII.copyPhysReg(MBB, II, DL, ScratchReg, BasePtr);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::PUSH24AF : Z80::PUSH16AF));
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::ADD24ao : Z80::ADD16ao),
            ScratchReg).addReg(ScratchReg).addReg(OffReg, RegState::Kill)
        ->addRegisterDead(Z80::F, this);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::POP24AF : Z80::POP16AF));
    MI.getOperand(FIOperandNum).ChangeToRegister(ScratchReg, false);
    if ((Is24Bit ? Z80::I24RegClass : Z80::I16RegClass).contains(ScratchReg))
      MI.getOperand(FIOperandNum + 1).ChangeToImmediate(0);
    else {
      switch (Opc) {
      default: llvm_unreachable("Unexpected opcode!");
      case Z80::LD24ro:  Opc = Z80::LD24rp;  break;
      case Z80::LD16ro:  Opc = Z80::LD16rp;  break;
      case Z80::LD88ro:  Opc = Z80::LD88rp;  break;
      case Z80::LD8ro:   Opc = Z80::LD8rp;   break;
      case Z80::LD8go:   Opc = Z80::LD8gp;   break;
      case Z80::LD24or:  Opc = Z80::LD24pr;  break;
      case Z80::LD16or:  Opc = Z80::LD16pr;  break;
      case Z80::LD88or:  Opc = Z80::LD88pr;  break;
      case Z80::LD8or:   Opc = Z80::LD8pr;   break;
      case Z80::LD8og:   Opc = Z80::LD8pg;   break;
      case Z80::LD8oi:   Opc = Z80::LD8pi;   break;
      case Z80::PEA24o:  Opc = Z80::PUSH24r; break;
      case Z80::PEA16o:  Opc = Z80::PUSH16r; break;
      case Z80::RLC8o:   Opc = Z80::RLC8p;   break;
      case Z80::RRC8o:   Opc = Z80::RRC8p;   break;
      case Z80::RL8o:    Opc = Z80::RL8p;    break;
      case Z80::RR8o:    Opc = Z80::RR8p;    break;
      case Z80::SLA8o:   Opc = Z80::SLA8p;   break;
      case Z80::SRA8o:   Opc = Z80::SRA8p;   break;
      case Z80::SRL8o:   Opc = Z80::SRL8p;   break;
      case Z80::BIT8bo:  Opc = Z80::BIT8bp;  break;
      case Z80::RES8bo:  Opc = Z80::RES8bp;  break;
      case Z80::SET8bo:  Opc = Z80::SET8bp;  break;
      case Z80::INC8o:   Opc = Z80::INC8p;   break;
      case Z80::DEC8o:   Opc = Z80::DEC8p;   break;
      case Z80::ADD8ao:  Opc = Z80::ADD8ap;  break;
      case Z80::ADC8ao:  Opc = Z80::ADC8ap;  break;
      case Z80::SUB8ao:  Opc = Z80::SUB8ap;  break;
      case Z80::SBC8ao:  Opc = Z80::SBC8ap;  break;
      case Z80::AND8ao:  Opc = Z80::AND8ap;  break;
      case Z80::XOR8ao:  Opc = Z80::XOR8ap;  break;
      case Z80::OR8ao:   Opc = Z80::OR8ap;   break;
      case Z80::CP8ao:   Opc = Z80::CP8ap;   break;
      case Z80::TST8ao:  Opc = Z80::TST8ap;  break;
      case Z80::LEA24ro:
      case Z80::LEA16ro:
        Opc = TargetOpcode::COPY;
        break;
      }
      if (Opc == TargetOpcode::COPY) {
        TII.copyPhysReg(MBB, ++II, DL, MI.getOperand(0).getReg(),
                        MI.getOperand(1).getReg());
        MI.eraseFromParent();
      } else {
        MI.setDesc(TII.get(Opc));
        MI.RemoveOperand(FIOperandNum + 1);
      }
    }
  } else {
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::PUSH24r : Z80::PUSH16r))
        .addReg(BasePtr);
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::LD24ri : Z80::LD16ri), OffReg)
        .addImm(Off);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::PUSH24AF : Z80::PUSH16AF));
    BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::ADD24ao : Z80::ADD16ao),
            BasePtr).addReg(BasePtr).addReg(OffReg, RegState::Kill)
        ->addRegisterDead(Z80::F, this);
    if (SaveFlags)
      BuildMI(MBB, II, DL, TII.get(Is24Bit ? Z80::POP24AF : Z80::POP16AF));
    if (Opc == Z80::PEA24o || Opc == Z80::PEA16o) {
      MI.setDesc(TII.get(Opc == Z80::PEA24o ? Z80::EX24SP : Z80::EX16SP));
      MI.getOperand(0).ChangeToRegister(BasePtr, true);
      MI.getOperand(1).ChangeToRegister(BasePtr, false);
      MI.tieOperands(0, 1);
    } else {
      MI.getOperand(FIOperandNum).ChangeToRegister(BasePtr, false);
      MI.getOperand(FIOperandNum + 1).ChangeToImmediate(0);
      BuildMI(MBB, ++II, DL, TII.get(Is24Bit ? Z80::POP24r : Z80::POP16r),
              BasePtr);
    }
  }
}

Register Z80RegisterInfo::getFrameRegister(const MachineFunction &MF) const {
  return getFrameLowering(MF)->hasFP(MF) ? (Is24Bit ? Z80::UIX : Z80::IX)
                                         : (Is24Bit ? Z80::SPL : Z80::SPS);
}

bool Z80RegisterInfo::
shouldCoalesce(MachineInstr *MI,
               const TargetRegisterClass *SrcRC, unsigned SrcSubReg,
               const TargetRegisterClass *DstRC, unsigned DstSubReg,
               const TargetRegisterClass *NewRC, LiveIntervals &LIS) const {
  LLVM_DEBUG(
      dbgs() << getRegClassName(SrcRC) << '[' << SrcRC->getNumRegs()
             << "]:" << (SrcSubReg ? getSubRegIndexName(SrcSubReg) : "")
             << " -> " << getRegClassName(DstRC) << '[' << DstRC->getNumRegs()
             << "]:" << (DstSubReg ? getSubRegIndexName(DstSubReg) : "") << ' '
             << getRegClassName(NewRC) << '[' << NewRC->getNumRegs() << "]\n");
  // Don't coalesce if SrcRC and DstRC have a small intersection.
  return std::min(SrcRC->getNumRegs(), DstRC->getNumRegs()) <=
         NewRC->getNumRegs();
}

bool Z80RegisterInfo::requiresVirtualBaseRegisters(
    const MachineFunction &MF) const {
  return true;
}
int64_t Z80RegisterInfo::getFrameIndexInstrOffset(const MachineInstr *MI,
                                                  int FIOperandNum) const {
  return MI->getOperand(FIOperandNum + 1).getImm();
}
bool Z80RegisterInfo::needsFrameBaseReg(MachineInstr *MI,
                                        int64_t Offset) const {
  const MachineFunction &MF = *MI->getParent()->getParent();
  return !isFrameOffsetLegal(MI, getFrameRegister(MF), Offset);
}
void Z80RegisterInfo::materializeFrameBaseRegister(MachineBasicBlock *MBB,
                                                   Register BaseReg,
                                                   int FrameIdx,
                                                   int64_t Offset) const {
  MachineFunction &MF = *MBB->getParent();
  MachineRegisterInfo &MRI = MF.getRegInfo();
  const Z80Subtarget &STI = MF.getSubtarget<Z80Subtarget>();
  const Z80InstrInfo &TII = *STI.getInstrInfo();
  MachineBasicBlock::iterator II = MBB->begin();
  DebugLoc DL = MBB->findDebugLoc(II);
  MRI.setRegClass(BaseReg, Is24Bit ? &Z80::I24RegClass : &Z80::I16RegClass);
  BuildMI(*MBB, II, DL, TII.get(Is24Bit ? Z80::LEA24ro : Z80::LEA16ro), BaseReg)
      .addFrameIndex(FrameIdx).addImm(Offset);
  return;
  Register CopyReg = MRI.createVirtualRegister(Is24Bit ? &Z80::I24RegClass
                                                       : &Z80::I16RegClass);
  Register OffsetReg = MRI.createVirtualRegister(Is24Bit ? &Z80::O24RegClass
                                                         : &Z80::O16RegClass);
  TII.copyPhysReg(*MBB, II, DL, CopyReg, getFrameRegister(MF));
  BuildMI(*MBB, II, DL, TII.get(Is24Bit ? Z80::LD24ri : Z80::LD16ri), OffsetReg)
      .addImm(Offset);
  BuildMI(*MBB, II, DL, TII.get(Is24Bit ? Z80::ADD24ao : Z80::ADD16ao), BaseReg)
      .addReg(CopyReg).addReg(OffsetReg)->addRegisterDead(Z80::F, this);
}

static bool isSplitLoadStoreOpc(unsigned Opc) {
  switch (Opc) {
  default:
    return false;
  case Z80::LD88rp:
  case Z80::LD88ro:
  case Z80::LD88pr:
  case Z80::LD88or:
    return true;
  }
}
static unsigned getFIOperandNum(const MachineInstr &MI) {
  for (const auto &MO : MI.explicit_uses())
    if (MO.isFI())
      return MI.getOperandNo(&MO);
  llvm_unreachable("Instr doesn't have a FrameIndex operand!");
}

void Z80RegisterInfo::resolveFrameIndex(MachineInstr &MI, Register BaseReg,
                                        int64_t Offset) const {
  unsigned FIOperandNum = getFIOperandNum(MI);
  MI.getOperand(FIOperandNum).ChangeToRegister(BaseReg, false);
  MI.getOperand(FIOperandNum + 1)
      .ChangeToImmediate(MI.getOperand(FIOperandNum + 1).getImm() + Offset);
}
bool Z80RegisterInfo::isFrameOffsetLegal(const MachineInstr *MI,
                                         Register BaseReg,
                                         int64_t Offset) const {
  Offset += getFrameIndexInstrOffset(MI, getFIOperandNum(*MI));
  return isInt<8>(Offset) &&
         (!isSplitLoadStoreOpc(MI->getOpcode()) || isInt<8>(Offset + 1));
}
