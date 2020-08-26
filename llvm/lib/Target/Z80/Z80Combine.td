//=- Z80.td - Define Z80 Combine Rules -----------------------*- tablegen -*-=//
//
// Part of the LLVM Project, under the Apache License v2.0 with LLVM Exceptions.
// See https://llvm.org/LICENSE.txt for license information.
// SPDX-License-Identifier: Apache-2.0 WITH LLVM-exception
//
//===----------------------------------------------------------------------===//
//
//
//===----------------------------------------------------------------------===//

include "llvm/Target/GlobalISel/Combine.td"

def combine_trunc_shift_matchdata : GIDefMatchData<"Register">;
def combine_trunc_shift : GICombineRule<
  (defs root:$mi, combine_trunc_shift_matchdata:$matchinfo),
  (match (wip_match_opcode G_TRUNC):$mi,
         [{ return matchCombineTruncShift(*${mi}, MRI, ${matchinfo}); }]),
  (apply [{ applyCombineTruncShift(*${mi}, B, Observer, ${matchinfo}); }])>;

def flip_setcc_matchdata : GIDefMatchData<"MachineInstr *">;
def flip_setcc_cond : GICombineRule<
  (defs root:$mi, flip_setcc_matchdata:$matchinfo),
  (match (wip_match_opcode G_XOR):$mi,
         [{ return matchFlipSetCCCond(*${mi}, MRI, ${matchinfo}); }]),
  (apply [{ applyFlipSetCCCond(*${mi}, B, Observer, *${matchinfo}); }])>;

def Z80PreLegalizerCombinerHelper
    : GICombinerHelper<"Z80GenPreLegalizerCombinerHelper", [
      and_ext, and_trivial_mask, trivial_combines, elide_br_by_inverting_cond,
      ptr_add_immed_chain, ptr_add_global_immed, ptr_add_const_immed,
      sext_to_zext, or_to_add, funnel_shift, combine_identity,
      identity_combines, combine_ext_or_trunc, narrow_op, narrow_load,
      narrow_icmp, simplify_icmp_bool, split_brcond, combine_trunc_shift,
      flip_cmp_cond, flip_setcc_cond, lower_is_power_of_two, simplify_const,
      undef_combines
    ]> {
  let DisableRuleOption = "z80prelegalizercombiner-disable-rule";
}

def Z80PostLegalizerCombinerHelper
    : GICombinerHelper<"Z80GenPostLegalizerCombinerHelper", [
      narrow_op
    ]> {
  let DisableRuleOption = "z80postlegalizercombiner-disable-rule";
}
