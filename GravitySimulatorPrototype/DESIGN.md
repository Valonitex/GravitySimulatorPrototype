====================================================================
DESIGN DECISIONS
====================================================================

This file is authoritative. Never contradict it.
Read it before implementing any fix.

====================================================================
GRAVITY KERNEL
====================================================================

This simulator uses 3D inverse-square gravity (F ∝ 1/r²) evaluated
in 2D coordinates.

It is NOT true 2D gravity (F ∝ 1/r, potential ∝ ln r).

This is intentional. Do not change the force kernel, energy diagnostic,
or KS regularization to use ln(r) or any 2D-specific formulation.

Bugs flagged for "2D vs 3D kernel mismatch" are By Design.
Specifically: FMM-004, BH-005, KS-010, PHYS-006, EOS-001.

A previous session added namespace grav2d (lines 194–221) implementing
true 2D gravity as a mistaken fix for these non-bugs. That namespace is
dead code and must be deleted (tracked as PHYS-007). Do not use or
extend namespace grav2d under any circumstances.
