//===- Z80TargetStreamer.cpp - Z80TargetStreamer class --*- C++ -*---------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//
//
// This file implements the Z80TargetStreamer class.
//
//===----------------------------------------------------------------------===//

#include "Z80TargetStreamer.h"
#include "llvm/MC/MCContext.h"
#include "llvm/Support/FormattedStream.h"

using namespace llvm;

Z80TargetStreamer::Z80TargetStreamer(MCStreamer &S)
    : MCTargetStreamer(S) {}

Z80TargetAsmStreamer::Z80TargetAsmStreamer(MCStreamer &S,
                                           formatted_raw_ostream &OS)
    : Z80TargetStreamer(S), MAI(S.getContext().getAsmInfo()), OS(OS) {}

void Z80TargetAsmStreamer::emitAlign(Align Alignment) {
  if (Alignment > Align())
    OS << "\tALIGN\t" << ByteAlign.value() << '\n';
}

void Z80TargetAsmStreamer::emitBlock(uint64_t NumBytes) {
  if (NumBytes)
    OS << "\tDS\t" << NumBytes << '\n';
}

void Z80TargetAsmStreamer::emitGlobal(MCSymbol *Symbol) {
  OS << "\tXDEF\t";
  Symbol->print(OS, MAI);
  OS << '\n';
}

void Z80TargetAsmStreamer::emitExtern(MCSymbol *Symbol) {
  OS << "\tXREF\t";
  Symbol->print(OS, MAI);
  OS << '\n';
}
