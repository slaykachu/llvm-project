//=== lib/CodeGen/GlobalISel/Z80PostSelectCombiner.cpp --------------------===//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
// This pass does combining of machine instructions at the generic MI level,
// before the legalizer.
//
//===----------------------------------------------------------------------===//

#include "MCTargetDesc/Z80MCTargetDesc.h"
#include "Z80.h"
#include "Z80InstrInfo.h"
#include "Z80Subtarget.h"
#include "llvm/CodeGen/MachineFunctionPass.h"
#include "llvm/CodeGen/TargetPassConfig.h"
#include "llvm/InitializePasses.h"
#include "llvm/Support/Debug.h"
#include "llvm/Target/TargetMachine.h"

#define DEBUG_TYPE "z80-postselect-combiner"

using namespace llvm;

static cl::opt<unsigned> CondCallThreshold("z80-cond-call-threshold",
                                           cl::Hidden, cl::init(10));

namespace {
class Z80PostSelectCombiner : public MachineFunctionPass {
public:
  static char ID;

  Z80PostSelectCombiner();

  StringRef getPassName() const override { return "Z80 Post Select Combiner"; }

  bool runOnMachineFunction(MachineFunction &MF) override;

  void getAnalysisUsage(AnalysisUsage &AU) const override;
};

class ValLoc {
  Register Reg, Base;
  int8_t Off;

  static Register getBaseReg(const MachineInstr &MI, unsigned OpNo) {
    Register BaseReg;
    const MachineOperand &BaseMO = MI.getOperand(OpNo);
    if (BaseMO.isReg())
      BaseReg = BaseMO.getReg();
    else if (BaseMO.getIndex() >= 0)
      return Register::index2StackSlot(BaseMO.getIndex());
    return BaseReg;
  }

public:
  ValLoc() : Reg(), Base(), Off() {}

  ValLoc &setReg(Register Reg) {
    this->Reg = Reg;
    return *this;
  }
  ValLoc &setReg(const MachineInstr &MI, unsigned OpNo) {
    return setReg(MI.getOperand(OpNo).getReg());
  }
  ValLoc &setMem(Register Base, int8_t Off = 0) {
    this->Base = Base;
    this->Off = Off;
    return *this;
  }
  ValLoc &setPtr(const MachineInstr &MI, unsigned OpNo) {
    return setMem(MI.getOperand(OpNo).getReg());
  }
  ValLoc &setOff(const MachineInstr &MI, unsigned OpNo) {
    return setMem(getBaseReg(MI, OpNo), MI.getOperand(OpNo + 1).getImm());
  }

  bool matchesReg(Register Reg) const {
    return Reg.isValid() && this->Reg == Reg;
  }
  bool matchesReg(const MachineInstr &MI, unsigned OpNo) const {
    return matchesReg(MI.getOperand(OpNo).getReg());
  }
  bool matchesMem(Register Base, int8_t Off = 0) const {
    return Base.isValid() && this->Base == Base && this->Off == Off;
  }
  bool matchesPtr(const MachineInstr &MI, unsigned OpNo) const {
    return matchesMem(MI.getOperand(OpNo).getReg());
  }
  bool matchesOff(const MachineInstr &MI, unsigned OpNo) const {
    return matchesMem(getBaseReg(MI, OpNo), MI.getOperand(OpNo + 1).getImm());
  }

  void clobberDefs(const MachineInstr &MI, const TargetRegisterInfo &TRI) {
    for (const MachineOperand &DefMO : MI.defs())
      for (Register LocReg : {Reg, Base})
        if (LocReg.isValid() && TRI.regsOverlap(DefMO.getReg(), LocReg))
          return clear();
  }

  void clear() {
    *this = ValLoc();
  }
};

} // end anonymous namespace

void Z80PostSelectCombiner::getAnalysisUsage(AnalysisUsage &AU) const {
  AU.setPreservesCFG();
  MachineFunctionPass::getAnalysisUsage(AU);
}

Z80PostSelectCombiner::Z80PostSelectCombiner() : MachineFunctionPass(ID) {
  initializeZ80PostSelectCombinerPass(*PassRegistry::getPassRegistry());
}

bool Z80PostSelectCombiner::runOnMachineFunction(MachineFunction &MF) {
  MachineRegisterInfo &MRI = MF.getRegInfo();
  auto &STI = MF.getSubtarget<Z80Subtarget>();
  auto &TII = *STI.getInstrInfo();
  auto &TRI = *STI.getRegisterInfo();

  bool Changed = false;
  for (MachineBasicBlock &MBB : MF) {
    auto I = MBB.begin(), E = MBB.end();
    ValLoc SZFlagLoc;
    auto FlagLocs = {&SZFlagLoc};
    while (I != E) {
      MachineInstr &MI = *I;
      ++I;

      switch (unsigned Opc = MI.getOpcode()) {
      case TargetOpcode::COPY: {
        for (ValLoc *FlagLoc : FlagLocs)
          if (FlagLoc->matchesReg(MI, 1))
            FlagLoc->setReg(MI, 0);
        Register DstReg = MI.getOperand(0).getReg();
        if (DstReg != Z80::SPS && DstReg != Z80::SPL)
          break;
        Register TmpReg =
            MRI.createVirtualRegister(DstReg == Z80::SPL ? &Z80::A24RegClass
                                                         : &Z80::A16RegClass);
        BuildMI(MBB, MI, MI.getDebugLoc(), TII.get(TargetOpcode::COPY), TmpReg)
            .add(MI.getOperand(1));
        MI.setDesc(TII.get(DstReg == Z80::SPL ? Z80::LD24SP : Z80::LD16SP));
        MI.getOperand(0).setReg(TmpReg);
        MI.RemoveOperand(1);
        Changed = true;
        break;
      }
      case Z80::PUSH16r:
      case Z80::PUSH24r: {
        if (!STI.hasEZ80Ops())
          break;
        bool IsPush24 = Opc == Z80::PUSH24r;
        Register SrcReg = MI.getOperand(0).getReg();
        if (!MRI.hasOneUse(SrcReg))
          break;
        MachineInstr *SrcMI = MRI.getVRegDef(SrcReg);
        if (!SrcMI ||
            SrcMI->getOpcode() != (IsPush24 ? Z80::LEA24ro : Z80::LEA16ro))
          break;
        MachineOperand &BaseMO = SrcMI->getOperand(1);
        auto NewOff = SrcMI->getOperand(2).getImm();
        if (!BaseMO.isReg() || NewOff) {
          MI.RemoveOperand(0);
          MI.setDesc(TII.get(IsPush24 ? Z80::PEA24o : Z80::PEA16o));
          MachineInstrBuilder(MF, MI).add(SrcMI->getOperand(1)).addImm(NewOff);
        } else
          MI.getOperand(0).setReg(BaseMO.getReg());
        SrcMI->eraseFromParent();
        Changed = true;
        break;
      }
      case Z80::LD8rp:
      case Z80::LD8gp:
        for (ValLoc *FlagLoc : FlagLocs)
          if (FlagLoc->matchesPtr(MI, 1))
            FlagLoc->setReg(MI, 0);
        break;
      case Z80::LD8ro:
      case Z80::LD8go:
        for (ValLoc *FlagLoc : FlagLocs)
          if (FlagLoc->matchesOff(MI, 1))
            FlagLoc->setReg(MI, 0);
        break;
      case Z80::LD8pr:
      case Z80::LD8pg:
        for (ValLoc *FlagLoc : FlagLocs)
          if (FlagLoc->matchesReg(MI, 1))
            FlagLoc->setPtr(MI, 0);
        break;
      case Z80::LD8or:
      case Z80::LD8og:
        for (ValLoc *FlagLoc : FlagLocs)
          if (FlagLoc->matchesReg(MI, 2))
            FlagLoc->setOff(MI, 0);
        break;
      case Z80::OR8ar:
        if (MI.getOperand(0).getReg() == Z80::A &&
            SZFlagLoc.matchesReg(MI, 0)) {
          MI.eraseFromParent();
          break;
        }
        LLVM_FALLTHROUGH;
      case Z80::ADD8ar:
      case Z80::ADD8ai:
      case Z80::ADC8ar:
      case Z80::ADC8ai:
      case Z80::SUB8ar:
      case Z80::SUB8ai:
      case Z80::SBC8ar:
      case Z80::SBC8ai:
      case Z80::AND8ar:
      case Z80::AND8ai:
      case Z80::XOR8ar:
      case Z80::XOR8ai:
      case Z80::OR8ai:
        SZFlagLoc.setReg(Z80::A);
        break;
      case Z80::RLC8r:
      case Z80::RRC8r:
      case Z80::RL8r:
      case Z80::RR8r:
      case Z80::SLA8r:
      case Z80::SRA8r:
      case Z80::SRL8r:
      case Z80::INC8r:
      case Z80::DEC8r:
        SZFlagLoc.setReg(MI, 0);
        break;
      case Z80::ADD8ap:
      case Z80::ADC8ap:
      case Z80::SUB8ap:
      case Z80::SBC8ap:
      case Z80::AND8ap:
      case Z80::XOR8ap:
      case Z80::OR8ap:
      case Z80::RLC8p:
      case Z80::RRC8p:
      case Z80::RL8p:
      case Z80::RR8p:
      case Z80::SLA8p:
      case Z80::SRA8p:
      case Z80::SRL8p:
      case Z80::INC8p:
      case Z80::DEC8p:
        SZFlagLoc.setPtr(MI, 0);
        break;
      case Z80::ADD8ao:
      case Z80::ADC8ao:
      case Z80::SUB8ao:
      case Z80::SBC8ao:
      case Z80::AND8ao:
      case Z80::XOR8ao:
      case Z80::OR8ao:
      case Z80::RLC8o:
      case Z80::RRC8o:
      case Z80::RL8o:
      case Z80::RR8o:
      case Z80::SLA8o:
      case Z80::SRA8o:
      case Z80::SRL8o:
      case Z80::INC8o:
      case Z80::DEC8o:
        SZFlagLoc.setOff(MI, 0);
        break;
      default:
        if (MI.modifiesRegister(Z80::F, &TRI))
          for (ValLoc *FlagLoc : FlagLocs)
            FlagLoc->clear();
        break;
      }

      for (ValLoc *FlagLoc : FlagLocs)
        FlagLoc->clobberDefs(MI, TRI);
    }
  }

  for (MachineBasicBlock &MBB : MF) {
    MachineBasicBlock *TrueMBB = nullptr, *FalseMBB = nullptr;
    SmallVector<MachineOperand, 1> Cond;
    if (TII.analyzeBranch(MBB, TrueMBB, FalseMBB, Cond, false) || Cond.empty())
      continue;
    if (!FalseMBB)
      FalseMBB = &*std::next(MBB.getIterator());
    assert(TrueMBB && FalseMBB && "Expected to be nonnull");
    for (int I = 0; I != 2; ++I) {
      if (TrueMBB->succ_empty() && TrueMBB->isReturnBlock()) {
        auto II = TrueMBB->begin();
        while (II->isCopy() || II->isMetaInstruction())
          ++II;
        if (++II == TrueMBB->end()) {
          // Unimplemented until FPE works.
          //Changed = true;
        }
      }
      if (TII.reverseBranchCondition(Cond))
        break;
      std::swap(TrueMBB, FalseMBB);
    }
    // Separate loop because we want to prefer the above optimization.
    for (int I = 0; I != 2; ++I) {
      if (TrueMBB->pred_size() == 1 && TrueMBB->succ_size() == 1 &&
          TrueMBB->isSuccessor(FalseMBB)) {
        MachineBasicBlock::iterator I = TrueMBB->begin();
        MachineBasicBlock::iterator E = TrueMBB->getFirstTerminator();
        if (I != E && TII.isFrameSetup(*I) && TII.isFrameInstr(*--E) &&
            I != E) {
          unsigned Cost = 0;
          MachineInstr *CallMI = nullptr;
          struct Result {
            Register PhysReg, TrueReg, FalseReg, ResReg;
          };
          SmallVector<Result, 4> Results;
          while (++I != E) {
            ++Cost;
            unsigned Opc = I->getOpcode();
            if (Opc == Z80::CALL16 || Opc == Z80::CALL24) {
              if (CallMI) {
                CallMI = nullptr;
                break;
              }
              CallMI = &*I;
            }
            if (TII.isFrameInstr(*I) ||
                (!CallMI && I->modifiesRegister(Z80::F, &TRI))) {
              CallMI = nullptr;
              break;
            }
            if (CallMI && Opc == TargetOpcode::COPY) {
              if (Results.size() == 4) {
                CallMI = nullptr;
                break;
              }
              Result &Res = Results.emplace_back();
              Res.TrueReg = I->getOperand(0).getReg();
              Res.PhysReg = I->getOperand(1).getReg();
              assert(Res.TrueReg.isVirtual() && Res.PhysReg.isPhysical() &&
                     "Expected phys to virt reg copy inside call sequence");
            }
          }
          for (I = FalseMBB->begin(), E = FalseMBB->end(); CallMI && I != E && I->isPHI();
               ++I) {
            if (I->getNumOperands() != 5) {
              CallMI = nullptr;
              break;
            }
            Register FalseReg, TrueReg;
            for (unsigned OpNo = 1; CallMI && OpNo != 5; OpNo += 2) {
              Register Reg = I->getOperand(OpNo).getReg();
              MachineBasicBlock &PredMBB = *I->getOperand(OpNo + 1).getMBB();
              if (&PredMBB == &MBB)
                FalseReg = Reg;
              else if (&PredMBB == TrueMBB)
                TrueReg = Reg;
              else
                CallMI = nullptr;
            }
            bool Found = false;
            for (Result &Res : Results) {
              if (Res.TrueReg != TrueReg)
                continue;
              if (Res.FalseReg.isValid())
                break;
              Res.FalseReg = FalseReg;
              Res.ResReg = I->getOperand(0).getReg();
              Found = true;
              break;
            }
            if (!Found)
              CallMI = nullptr;
          }
          for (Result &Res : Results) {
            if (!Res.FalseReg.isValid()) {
              CallMI = nullptr;
              break;
            }
          }
          if (CallMI && Cost < CondCallThreshold) {
            Register TempReg = MRI.createVirtualRegister(&Z80::F8RegClass);
            DebugLoc DL = MBB.findBranchDebugLoc();
            MBB.removeSuccessor(FalseMBB);
            TII.removeBranch(MBB);
            BuildMI(&MBB, DL, TII.get(TargetOpcode::COPY), TempReg)
                .addReg(Z80::F);
            if (!MBB.isLayoutSuccessor(TrueMBB))
              TII.insertUnconditionalBranch(MBB, TrueMBB, DL);
            BuildMI(*TrueMBB, TrueMBB->begin(), DL, TII.get(TargetOpcode::COPY),
                    Z80::F).addReg(TempReg);
            CallMI->setDesc(TII.get(CallMI->getOpcode() == Z80::CALL24
                                    ? Z80::CALL24CC : Z80::CALL16CC));
            auto RegMask = CallMI->getOperand(1).getRegMask();
            CallMI->RemoveOperand(1);
            MachineInstrBuilder(MF, CallMI)
                .add(Cond[0]).addRegMask(RegMask)
                .addReg(Z80::F, RegState::Implicit);
            for (Result &Res : Results) {
              BuildMI(*TrueMBB, CallMI, CallMI->getDebugLoc(),
                      TII.get(TargetOpcode::COPY), Res.PhysReg)
                  .addReg(Res.FalseReg);
              CallMI->addRegisterKilled(Res.PhysReg, &TRI, true);
            }
            Changed = true;
          }
        }
      }
      if (TII.reverseBranchCondition(Cond))
        break;
      std::swap(TrueMBB, FalseMBB);
    }
  }

  return Changed;
}

char Z80PostSelectCombiner::ID = 0;
INITIALIZE_PASS_BEGIN(Z80PostSelectCombiner, DEBUG_TYPE,
                      "Combine Z80 machine instrs after inst selection", false,
                      false)
INITIALIZE_PASS_DEPENDENCY(TargetPassConfig);
INITIALIZE_PASS_DEPENDENCY(InstructionSelect);
INITIALIZE_PASS_END(Z80PostSelectCombiner, DEBUG_TYPE,
                    "Combine Z80 machine instrs after inst selection", false,
                    false)


namespace llvm {
FunctionPass *createZ80PostSelectCombiner() {
  return new Z80PostSelectCombiner;
}
} // end namespace llvm
