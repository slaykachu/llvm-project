//===- lib/MC/MCSectionOMF.cpp - OMF Code Section Representation ----------===//
//
//                     The LLVM Compiler Infrastructure
//
// This file is distributed under the University of Illinois Open Source
// License. See LICENSE.TXT for details.
//
//===----------------------------------------------------------------------===//

#include "llvm/MC/MCSectionOMF.h"
#include "llvm/MC/MCAsmInfo.h"
#include "llvm/ADT/Twine.h"
#include "llvm/Support/raw_ostream.h"

using namespace llvm;

MCSectionOMF::MCSectionOMF(StringRef Name, SectionKind K, MCSymbol *Begin)
    : MCSection(SV_OMF, Name, K, Begin) {}

MCSectionOMF::~MCSectionOMF() {} // anchor.

void MCSectionOMF::PrintSwitchToSection(const MCAsmInfo &MAI, const Triple &T,
                                         raw_ostream &OS,
                                         const MCExpr *Subsection) const {
  assert(!Subsection && "Unimplemented!");
  OS << "\tSEGMENT\t" << getName() << '\n';
}
