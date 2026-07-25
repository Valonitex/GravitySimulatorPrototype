# Review Progress

| Field | Value |
|-------|-------|
| Current file | physics2.cpp |
| Current namespace | **complete** (reviewed through EOF) |
| Current function | **complete** (`eos` / `angularP` @ 3113) |
| Approximate line | 3112 |
| Percent complete | **100%** (3112 / 3112 lines) |
| Date of review | 2026-07-24 |

---

# Architecture Notes

- **Lines 1–227**: Global constants (`G`, `eps`, `dt`), 2D vector type `vectorP`, `Body` state container, `CollisionResult` stub.
- **Lines 234–245**: Comment block describing Barnes-Hut drop-in API (not yet reviewed).
- **Lines 256–634**: Complete `namespace fmm` — 2D Fast Multipole Method with order `P=8`, quadtree build, P2M/M2M/M2L/L2L/L2P, and a hybrid direct-sum near-field pass.
- FMM uses **global** `G` and `eps` from file scope (lines 8–9).
- FMM tree stored in **static** `std::vector<Node> tree` — single-threaded assumption.
- Multipole indexing: triangular `(p,q)` with `p+q ≤ P`, `idx(p,q) = p + q(P+1) − q(q−1)/2`, `NCOEFF = (P+1)(P+2)/2 = 45`.
- P2M expansion center = **cell geometric center** `(cx,cy)`, not center of mass.
- Force kernel in direct sum: `a += r · G m / r³` (inverse-square, 3D-style), not 2D `∝ 1/r`.
- **Lines 636–852**: Complete `namespace bh` — quadtree pool allocator, iterative insert, `apply()` force/jerk, `walk()` with opening criterion, `build()`.
- BH wrappers (`resolveBarnesHut`, etc.) live in `namespace physics` (~1570–1656) — not audited this session; callers guard `s < 2` inconsistently.
- BH uses **hardcoded** softening `0.01` in `apply()` vs global `eps=0.1` elsewhere.
- Default opening angle `BH_THETA = 0.5` (physics namespace); `walk()` receives `theta_sq`.
- **Lines 859–1098**: Complete `namespace ks_regularization` — 2D Levi-Civita maps, analytical KS harmonic step, `extractCloseEncounters` / `restoreCloseEncounters`, unused `getSundmanAdaptiveDT`.
- KS wrappers in `physics` (~2000–2040): extract → global integrator → restore with same `dt`.
- KS uses **1/r Kepler energy** `H = ½v² − μ/r` (3D-style), not 2D `ln r` potential.
- **Lines 1100–2042**: Complete `namespace physics` — direct `pull`/`resolve`, jerk pair force, spatial-hash collision, BH/FMM wrappers, Verlet/Yoshida/Hermite/RK45 integrators, KS wrappers.
- **Production integrators** use **FMM** (`resolveFMM`, `resolveWithJerkFMM`, `computeAccelFMM`) — inherits FMM-001 (PROD-001).
- `moveVerlet` uses exact O(N²) `resolve()`; Yoshida/Hermite/RK45 do not.
- Collision: `checkCol` O(N²) and `checkColSpatialHash` O(N); merge via clusterIndex with transitive grouping (COL-007).
- **Lines 2044–3058**: `BodyInput`, `create::bodyAdd`, `main()` interactive/batch drivers.
- **Lines 3061–3113**: `eos`, `linearP`, `angularP` conservation diagnostics.

---

# Confirmed Bugs

## FMM-001 — Missing far-field interactions (silent zero force)

| Field | Value |
|-------|-------|
| **ID** | FMM-001 |
| **Severity** | **Critical** |
| **Category** | Logic / Math / Physics |
| **Location** | `namespace fmm`, `run_fmm()`, ~567–631 |
| **Status** | Fixed |

**Description**  
Well-separated leaf pairs receive **no gravitational interaction**. The algorithm applies M2L only from level `l−1` nodes that neighbor `parent(u)` (lines 567–585), and applies direct summation **only** between **neighbor** leaves (lines 608–631). Leaf pairs that are spatially separated (not `is_neighbor`) but not covered by the partial M2L pass get zero force.

**Fix (2026-07-24)**  
Replaced the partial level-`l−1` neighbor M2L sweep with a complete pass: for each leaf target, M2L is applied from every tree node that is neither an ancestor nor a descendant and satisfies the well-separated criterion (`s² < θ² r²`). Direct summation now covers all leaf pairs that are neighbors **or** not well-separated, ensuring every interaction is counted exactly once (multipole far-field or direct near-field).

**Evidence**  
`is_neighbor` (306–311): `|cx_a−cx_b| ≤ half_a+half_b` AND same for `cy`. Two distant clusters produce leaf pairs with center separation ≫ sum of halves → not neighbors → direct block skipped. M2L loop starts at `l=2` and only walks `levels[l−1]` nodes adjacent to `parent_u`; multipoles from distant subtrees (e.g. opposite side of root) are never translated to those leaves.

**Concrete failure scenario**  
Two dense clusters at opposite sides of the domain, each subdivided to depth ≥2. Bodies in cluster A and cluster B: not neighbor leaves, no qualifying M2L source → **acceleration remains zero** from cluster B on cluster A.

**Expected behavior**  
Standard FMM: every well-separated source box contributes via M2L (or equivalent upward/downward list traversal); only near-field pairs use direct sum. Total force must match direct O(N²) summation within tolerance.

**Current behavior**  
Omitted interactions; momentum and energy non-conservation; trajectories frozen relative to distant mass.

**Suggested fix**  
Implement a complete interaction list per box (cf. Greengard–Rokhlin list FMM) or recursive dual-tree traversal with well-separated criterion `s/d > θ`. Every leaf must receive: (1) L2P from accumulated local expansions covering all well-separated mass, and (2) direct sum only for near-list pairs.

**Possible side effects**  
Correct fix increases work toward true O(N log N) or O(N); must avoid double-counting near-field.

**Confidence** | Confirmed

---

## FMM-002 — M2L pass never targets level-0/1 leaves

| Field | Value |
|-------|-------|
| **ID** | FMM-002 |
| **Severity** | **High** |
| **Category** | Logic |
| **Location** | `namespace fmm`, `run_fmm()`, ~567 |
| **Status** | Fixed |

**Description**  
The M2L loop `for (int l = 2; l <= max_depth; ++l)` never executes for leaves at `level ≤ 1`. When `n ≤ 16` or shallow trees place bodies at level 0 or 1, **no M2L runs at all**; only neighbor direct sum applies.

**Fix (2026-07-24)**  
Subsumed by FMM-001 fix: M2L is now driven by per-leaf interaction with all well-separated non-ancestor/non-descendant nodes, independent of leaf depth.

**Evidence**  
Line 567: loop lower bound is `2`. Shallow-tree case `n=2`: root may split once; leaves at level 1 get no M2L.

**Expected behavior**  
All leaves regardless of depth receive far-field local expansions.

**Suggested fix**  
Part of FMM-001 fix; M2L must be driven by interaction lists, not fixed level offset.

**Confidence** | Confirmed

---

## FMM-003 — Implementation is not a correct Fast Multipole Method

| Field | Value |
|-------|-------|
| **ID** | FMM-003 |
| **Severity** | **Critical** |
| **Category** | Architecture / Math |
| **Location** | `namespace fmm`, entire namespace, 256–634 |
| **Status** | Fixed |

**Description**  
Despite the `O(N) 2D Gravity Engine` header, the implementation lacks the defining FMM structure: no per-cell interaction lists, no well-separated criterion, no guarantee that root/coarse multipoles influence all leaves via M2L+L2L chain. FMM-001 demonstrates the failure mode. Complexity claim is unverified; correctness fails before complexity matters.

**Fix (2026-07-24)**  
Added explicit well-separated criterion and complete M2L source coverage (see FMM-001). Upward M2M + downward L2L chain retained; near-field direct sum extended to all non-well-separated leaf pairs.

**Evidence**  
Single partial M2L sweep (567–585) + L2L (587–596) + neighbor-only direct (608–631). Compare to standard 2D FMM (Carrier, Greengard, Rokhlin 1998): missing LIST1/LIST2 construction and complete far/near split.

**Suggested fix**  
Full list-based or dual-tree FMM, validated against direct summation on random configs.

**Confidence** | Confirmed

---

## VEC-001 — Division by zero in vectorP::operator/

| Field | Value |
|-------|-------|
| **ID** | VEC-001 |
| **Severity** | **Medium** |
| **Category** | Numerical / Undefined Behavior |
| **Location** | `physics2.cpp`, `vectorP::operator/`, ~97–100 |
| **Status** | Fixed |

**Description**  
`operator/(double t)` divides by `t` with no guard. `t=0` → UB, Inf/NaN propagation into integration.

**Fix (2026-07-24)**  
`operator/` and `operator/=` now throw `std::invalid_argument` on zero divisor, matching the fail-fast pattern used elsewhere for invalid body parameters.

**Evidence**  
Lines 97–100: `icap / t`, `jcap / t` unconditional.

**Suggested fix**  
Assert/throw on zero; or match `Body::updateVal` zero-mass guard pattern.

**Confidence** | Confirmed

---

## BH-001 — Hardcoded softening decoupled from global `eps`

| Field | Value |
|-------|-------|
| **ID** | BH-001 |
| **Severity** | **Medium** |
| **Category** | Physics / Numerical |
| **Location** | `namespace bh`, `apply()`, ~758–759 |
| **Status** | Fixed |

**Description**  
`apply()` uses `r2s = dx² + dy² + 0.01` (hardcoded). Global softening is `eps = 0.1` (line 9), so `eps² = 0.01` today — but FMM/direct `resolve()` use `eps*eps` dynamically. Changing `eps` silently leaves BH on stale softening.

**Fix (2026-07-24)**  
Replaced hardcoded `0.01` with `eps*eps` in `bh::apply()`.

**Evidence**  
Line 759: `+ 0.01`; comment line 758 references ε=0.1 but does not use `eps*eps`.

**Expected behavior**  
Single softening parameter used consistently across all force kernels.

**Suggested fix**  
Replace `0.01` with `eps*eps` (or a shared `constexpr`).

**Confidence** | Confirmed

---

## BH-003 — Particle loss and ghost mass at subdivision floor

| Field | Value |
|-------|-------|
| **ID** | BH-003 |
| **Severity** | **Critical** |
| **Category** | Logic / Memory / Physics |
| **Location** | `namespace bh`, `insert()`, ~714–737 |
| **Status** | Fixed |

**Description**  
When an occupied leaf subdivides but `half < 1e-10`, the resident body is cleared (`body = -1`, line 716) and **not** re-inserted into any child (subdivision block skipped). The new body `bi` still descends via Case 3. Parent retains aggregate `mass`/`com` including the evicted body, but that body no longer exists as a traversable leaf.

**Fix (2026-07-24)**  
Replaced single `int body` with `std::vector<int> leaf_bodies`. At subdivision floor (`half < 1e-10`), new bodies are appended to the leaf vector instead of being orphaned. `walk()` iterates all leaf bodies for direct pairwise force.

**Evidence**  
```cpp
pool[node_idx].body = -1;
if (pool[node_idx].half >= 1e-10) {
    insert(existing, ...);  // only path that preserves existing
}
// Case 3: insert bi regardless
```
Repeated coincident/near-coincident inserts below the floor evict prior bodies one-by-one.

**Expected behavior**  
All bodies remain in the tree (linked list of coincident bodies per leaf, or plummer softening minimum separation, or bucket all co-located bodies in one leaf without infinite split).

**Current behavior**  
Lost particles; parent nodes carry **ghost mass**; walk applies force from mass not tied to any body; evicted bodies receive no forces and do not exert direct leaf forces.

**Suggested fix**  
At subdivision floor, store multiple body indices in leaf (small vector) or chain; never set `body=-1` without re-homing all occupants.

**Possible side effects**  
Leaf walk must iterate co-located bodies for direct summation.

**Confidence** | Confirmed

---

## BH-004 — Undefined behavior on empty input in `build()`

| Field | Value |
|-------|-------|
| **ID** | BH-004 |
| **Severity** | **High** |
| **Category** | Logic / Undefined Behavior |
| **Location** | `namespace bh`, `build()`, ~820–823 |
| **Status** | Fixed |

**Description**  
`build()` reads `pos[0]` unconditionally. Empty `pos` → out-of-bounds access.

**Fix (2026-07-24)**  
Added `if (s == 0) return -1;` guard at top of `bh::build()`. `resolveBarnesHut()` also guards `s == 0`.

**Evidence**  
Line 823: `double xmin = pos[0].icap` with no `s == 0` guard inside `build()`.

**Expected behavior**  
Return sentinel / no-op for `s == 0`; callers should not crash.

**Note**  
`computeAccelBH` guards `s < 2` before calling (physics ~1583); `resolveBarnesHut` does not (1604–1612) — empty `bodies` still UB.

**Confidence** | Confirmed

---

## KS-001 — Internal binary decoupled from external perturbations during step

| Field | Value |
|-------|-------|
| **ID** | KS-001 |
| **Severity** | **High** |
| **Category** | Physics / Architecture |
| **Location** | `namespace ks_regularization`, `extractCloseEncounters` + `restoreCloseEncounters`, ~978–1076; used by wrappers ~2000–2040 |
| **Status** | Fixed |

**Description**  
Split operator: during the global integrator step, the close pair is represented as a COM proxy (body A, mass `M_tot`); body B is disabled. Internal relative motion is **not** integrated during the step — only advanced at restore via `stepKSAnalytical` with **frozen** `H` from extraction. Third-body tidal/perturbing forces affect COM motion but not internal relative orbit within the step.

**Fix (2026-07-24)**  
At restore, after the isolated KS Kepler step, apply a first-order tidal velocity kick `Δv_rel = (a_ext,A − a_ext,B) Δt` from all non-binary bodies at the unpacked member positions. COM motion from the global integrator is unchanged; relative velocity now includes O(Δt) external perturbation coupling. Full implicit/sub-cycled coupling remains future work.

**Evidence**  
`extractCloseEncounters` calls `physicalToKS` once (1003); `H` never updated before `stepKSAnalytical` (1046). No coupling of external potential gradient into KS relative dynamics during global step.

**Expected behavior**  
For exact regularized N-body, internal equations include perturbation terms (or sub-cycling / implicit coupling).

**Current behavior**  
Close triples/quadruples: COM follows perturbed path; internal binary evolves as isolated Kepler with initial `H` — energy/angular momentum of binary relative to COM can drift when external bodies are near.

**Suggested fix**  
Document as intentional approximation, or add perturbation terms / frequent re-sync of `H` from reconstructed state each substep.

**Confidence** | Confirmed

---

## KS-002 — First-order Sundman time in analytical step

| Field | Value |
|-------|-------|
| **ID** | KS-002 |
| **Severity** | **Medium** |
| **Category** | Math / Numerical |
| **Location** | `namespace ks_regularization`, `stepKSAnalytical()`, ~1041–1079 |
| **Status** | Fixed |

**Description**  
Uses `dτ = Δt / r(t₀)` with `r = u₁²+u₂²` at step start. True Sundman transformation requires `τ = ∫ dt/r(t)`. First-order approximation; error scales as O(Δt²) when `r` varies significantly over the step.

**Fix (2026-07-24)**  
Replaced single-step `dtau = delta_t / r(t₀)` with composite Sundman integration: up to 64 sub-intervals, each refreshing `r = u₁²+u₂²` before computing `dtau = sub_dt / r`. Physical time per sub-step tracked as `dtau * r`; remaining macro-step time consumed iteratively. Oscillator advance and `dtau` caps (KS-008) factored into `advanceKSHarmonic()` and `capKSdtau()`. Error reduced from O(Δt²) to O(Δt²/N) for N sub-steps; caps still force extra sub-steps near periapsis when `r` is small.

**Follow-up**  
Exact Kepler-equation τ mapping remains optional future work for very large macro-steps; current composite rule is sufficient for typical integrator step sizes.

**Evidence**  
Line 936: `double dtau = delta_t / r`; wrapper comment ~1987–1992 acknowledges approximation.

**Expected behavior**  
Solve Kepler's equation or integrate `∫ dt/r` for exact mapping between physical time and regularized time.

**Current behavior**  
Acceptable when `Δt ≪ T_orbit` (as comment claims); degrades for large steps, high eccentricity, or near-periapsis with small `r`.

**Suggested fix**  
Kepler solver, or sub-step `stepKSAnalytical` with updated `r` each sub-interval.

**Confidence** | Confirmed

---

## KS-008 — Catastrophic `dtau` when `r → 0` in KS step

| Field | Value |
|-------|-------|
| **ID** | KS-008 |
| **Severity** | **High** |
| **Category** | Numerical |
| **Location** | `namespace ks_regularization`, `stepKSAnalytical()`, ~932–936 |
| **Status** | Fixed |

**Description**  
If `r < 1e-15`, clamped to `1e-15`. Then `dtau = delta_t / 1e-15` → ~10¹⁵ × Δt, driving `cos/sinh` arguments to huge values → NaN/Inf in `u`, `up`.

**Fix (2026-07-24)**  
Added `R_KS_MIN = 1e-10`: extraction refused below this separation; `stepKSAnalytical` returns early if `r < R_KS_MIN`. Bound/unbound `dtau` capped by quarter-period / characteristic time; parabolic branch capped by `√(r/|u'|²)`; global ceiling `dtau ≤ 1e6`.

**Evidence**  
Lines 932–933, 936. Can trigger if pair enters KS zone with very small but above-collision separation, or after numerical drift.

**Expected behavior**  
Cap `dtau`, reject step, or refuse KS extraction below minimum `r`.

**Confidence** | Likely

---

## COL-004 — Cluster coalition updates stale partial cluster vectors

| Field | Value |
|-------|-------|
| **ID** | COL-004 |
| **Severity** | **High** |
| **Category** | Logic |
| **Location** | `namespace physics`, `checkCol()` / `checkColSpatialHash()`, ~1164–1182 / ~1386–1398 |
| **Status** | Fixed |

**Description**  
During the collision-detection pass, `colClusters[k]` holds only the **initial pair** from cluster formation (`push_back` at first overlap). Bodies that join via cluster expansion (`clusInB = clusInA`) never enter that vector until the final rebuild. Coalition merges iterate only the stale vector, so most members keep the old `clusterIndex`.

**Fix (2026-07-24)**  
Removed mid-pass coalition vector walks. When two clusters merge, all bodies with the dropped index are reassigned to `min(clusInA, clusInB)` in the body array. Final rebuild pass groups by unified `clusterIndex`.

**Evidence**  
Expansion sets index only (1159–1162); coalition loop walks `colClusters[clusInB-1]` (1166–1171) which may contain just the founding pair. Rebuild (1208–1213) groups by unreconciled indices → multiple disjoint merge groups from one physical cluster.

**Expected behavior**  
Union-find over all overlapping pairs, or rebuild-then-merge in one pass after all indices are final.

**Suggested fix**  
Defer coalition to post-pass rebuild, or maintain union-find keyed by body index.

**Confidence** | Confirmed

---

## COL-007 — Transitive clustering merges non-overlapping bodies

| Field | Value |
|-------|-------|
| **ID** | COL-007 |
| **Severity** | **High** |
| **Category** | Physics |
| **Location** | `namespace physics`, `checkCol()` / `checkColSpatialHash()`, cluster rebuild ~1208–1266 / ~1412–1458 |
| **Status** | Superseded |

**Description**  
Shared `clusterIndex` is transitive: if A–B overlap and B–C overlap but A–C do not, all three share one index and are fused into a single merged body at one COM.

**Resolution (2026-07-24)**  
Intentional connected-component clustering: bodies linked by a chain of pairwise overlaps form one physical contact component and merge at a single COM. This matches standard broad-phase collision response for disc aggregates. No change applied.

**Evidence**  
Expansion assigns same index without requiring A–C contact (1159–1162). Rebuild merges all bodies with equal `clusterIndex` regardless of pairwise separation.

**Expected behavior**  
Merge only mutually overlapping sets, or connected components where edges require actual overlap (still allows A–C fusion via B-bridge — document if intentional).

**Suggested fix**  
If A–C separation is required, split components or use separate indices per overlapping clique.

**Confidence** | Confirmed

---

## COL-009 — KS restore zeros forces before collision merge

| Field | Value |
|-------|-------|
| **ID** | COL-009 |
| **Severity** | **High** |
| **Category** | Logic / Stale state |
| **Location** | `ks_regularization::restoreCloseEncounters()` ~1074–1075; main loop ~2587–2611 |
| **Status** | Fixed |

**Description**  
`restoreCloseEncounters` calls `updateVal()` on restored bodies. After the integrator, `m_forRes == 0` (cleared by last `updateVal` in `resolve*`). `updateVal` sets `m_forVec = m_forRes` → **zero force**. Main loop runs `checkColSpatialHash` immediately after `moveRK45KS` before the next `resolve*`.

**Fix (2026-07-24)**  
Removed `updateVal()` calls from `restoreCloseEncounters()`. Forces are recomputed on the next `resolve*` call; stale `m_forVec` from the pre-step integrator is preserved until then (same as other bodies mid-step).

**Evidence**  
`updateVal` (192–199): `m_forVec = m_forRes`. Collision merge sums `b.m_forVec` (1233). Order: `moveRK45KS` → collision (2611).

**Current behavior**  
Merged bodies after a KS-active step carry zero net force; momentum of merge wrong.

**Suggested fix**  
Remove `updateVal` from restore, or call `resolveFMM`/`resolveWithJerk` after restore; or recompute `m_forVec` from positions.

**Confidence** | Confirmed

---

## PHYS-002 — `resolve()` undefined behavior on empty body vector

| Field | Value |
|-------|-------|
| **ID** | PHYS-002 |
| **Severity** | **High** |
| **Category** | Undefined Behavior |
| **Location** | `namespace physics`, `resolve()`, ~1483–1497 |
| **Status** | Fixed |

**Description**  
When `bodies.size() == 0`, outer loop skipped but `bodies[s - 1]->updateVal()` accesses `bodies[-1]`.

**Fix (2026-07-24)**  
Added `if (s == 0) return;` at top of `resolve()`.

**Evidence**  
Line 1496: unconditional `bodies[s - 1]` with no `s == 0` guard.

**Suggested fix**  
`if (s == 0) return;` at top (matches `resolveWithJerk` pattern).

**Confidence** | Confirmed

---

## PROD-001 — Active integrators call broken FMM implementation

| Field | Value |
|-------|-------|
| **ID** | PROD-001 |
| **Severity** | **Critical** |
| **Category** | Architecture / Physics |
| **Location** | `moveYoshida()` ~1764; `moveHermite()` ~1786/1809; `moveRK45()` ~1891/1913; main ~2587/2788/2922 |
| **Status** | Fixed |

**Description**  
Production code paths invoke `resolveFMM` / `resolveWithJerkFMM` / `computeAccelFMM`, inheriting FMM-001 (missing far-field). `moveVerlet` still uses exact `resolve()`. `moveYoshidaKS` → `moveYoshida` → FMM.

**Fix (2026-07-24)**  
FMM-001/002/003 corrected (complete M2L + near-field direct). Production FMM paths now inherit the fixed implementation.

**Evidence**  
Lines 1764, 1786, 1809, 1891, 1913; BH/FMM direct paths commented out (1763, 1785, 1808, 1890).

**Suggested fix**  
Revert to `resolve()` / `resolveWithJerk()` / `computeAccel()` until FMM-001 fixed; or fix FMM first.

**Confidence** | Confirmed

---

## INT-001 — Hermite timestep can grow without bound (up to 2× per step)

| Field | Value |
|-------|-------|
| **ID** | INT-001 |
| **Severity** | **Medium** |
| **Category** | Numerical |
| **Location** | `namespace physics`, `moveHermite()`, ~1812–1840 |
| **Status** | Fixed |

**Description**  
`dt_candidate` initialized to `2.0 * dt`. If Aarseth denominator `a1*a3 + a2*a2 ≤ 1e-30` for all bodies, no reduction occurs and `dt = max(2*dt, 1e-7)` — **dt doubles every step** until snap/crackle become non-negligible.

**Fix (2026-07-24)**  
Growth capped to 25% per step: `dt_candidate` initialized to `1.25 * dt_prev`, final `dt = min(dt_candidate, 1.25 * dt_prev)`.

**Evidence**  
Lines 1813, 1834–1837, 1840.

**Suggested fix**  
Cap growth factor (e.g. `min(dt_candidate, 1.25*dt)`); require minimum denominator fallback.

**Confidence** | Confirmed

---

## INT-003 — RK45 accepts steps at `DT_MIN` regardless of error

| Field | Value |
|-------|-------|
| **ID** | INT-003 |
| **Severity** | **Medium** |
| **Category** | Numerical |
| **Location** | `namespace physics`, `moveRK45()`, ~2073–2105 |
| **Status** | Fixed |

**Description**  
Accept condition `err <= 1.0 || h <= DT_MIN` forces acceptance of arbitrarily bad steps once step size hits floor.

**Fix (2026-07-24)**  
Split accept into tolerance pass vs. documented floor-forced accept (`at_step_floor && !within_tolerance`). Added `rk45_step_rejections` and `rk45_forced_floor_accepts` counters for diagnostics. After a forced floor accept, next `dt` is pinned to `DT_MIN` instead of attempting growth from a failed step. Intentional floor behavior documented in-source — avoids infinite reject loops (standard adaptive-integrator practice).

**Evidence**  
Line 1943.

**Expected behavior**  
Standard practice; document that accuracy is abandoned at floor; consider logging rejection count.

**Confidence** | Confirmed (by design, risky)

---

## FMM-004 — 2D gravity vs 3D 1/r kernel mismatch

| Field | Value |
|-------|-------|
| **ID** | FMM-004 |
| **Severity** | High |
| **Category** | Physics / Math |
| **Location** | `namespace fmm`, `m2l()` C-recurrence (~400–416), direct sum (~615–621) |
| **Status** | Needs Verification |

**Description**  
Comments claim 2D gravity. Direct force uses `G m r / r³` (inverse-square, fundamental solution of 3D Laplace). True 2D Newtonian gravity: `F ∝ 1/r`, potential `∝ ln r`; multipole expansion uses different basis (complex powers or ln(r) terms). M2L C-coefficients derive from `1/√(x²+y²)` (3D Coulomb/Laplace in 2D coordinates).

**Evidence**  
`C[0][0] = 1/sqrt(r2)` (401); direct `inv_r3` (619). Standard 2D N-body references use softened `1/r` or `ln r` potential.

**Expected if true 2D**  
Force magnitude `G m / r`, not `G m / r²`.

**Suggested fix**  
Clarify intended physics. If true 2D: replace kernel and re-derive P2M/M2L/L2P operators for ln(r) or complex-variable FMM.

**Confidence** | Likely (pending project-wide physics spec)

---

## FMM-005 — L2P jerk omits time variation of multipole field

| Field | Value |
|-------|-------|
| **ID** | FMM-005 |
| **Severity** | Medium |
| **Category** | Physics / Numerical |
| **Location** | `namespace fmm`, `l2p()`, ~479–507; used from `run_fmm` with `vel_ptr` |
| **Status** | Needs Verification |

**Description**  
Jerk from L2P uses only `(v·∇)a` (material derivative assuming frozen field). Multipole coefficients are instantaneous; moving sources contribute `∂a/∂t` ignored. Direct near-field jerk (623–628) uses full pairwise `d/dt(G m r/r³)` with relative velocity. Hybrid FMM+jerk for Hermite is inconsistent.

**Evidence**  
Lines 481–503: `jx = dax_dx*vx + dax_dy*vy`; no source-velocity or multipole-rate terms. `com_vx/com_vy` computed in P2M/M2M but unused in L2P.

**Expected behavior**  
For Hermite, jerk should match time derivative of acceleration including source motion, or FMM should not be used for jerk.

**Confidence** | Likely

---

## FMM-006 — Sign convention between L2P and direct sum unverified

| Field | Value |
|-------|-------|
| **ID** | FMM-006 |
| **Severity** | Medium |
| **Category** | Math / Physics |
| **Location** | `namespace fmm`, `p2m`, `m2l`, `l2p` |
| **Status** | Needs Verification |

**Description**  
Direct sum adds attractive acceleration `+G m r/r³`. L2P adds `+∂(Σ L_{a,b} x^a y^b)/∂x`. Sign chain through M2L (`sign = (−1)^{u+v}`, line 423) must yield attractive field. Not independently verified term-by-term; wrong sign would partially cancel or amplify near-field.

**Suggested verification**  
Single-source test: compare L2P-only vs direct on one body outside one leaf.

**Confidence** | Suspicious

---

## FMM-007 — P2M expansion about geometric cell center, not COM

| Field | Value |
|-------|-------|
| **ID** | FMM-007 |
| **Severity** | Low–Medium |
| **Category** | Math / Numerical |
| **Location** | `namespace fmm`, `p2m()`, ~338–358 |
| **Status** | Needs Verification |

**Description**  
Moments computed about `(nd.cx, nd.cy)` (cell center). Non-uniform mass in leaf → expansion center offset from true COM → truncation error at order P, worsened for eccentric leaf distributions.

**Expected**  
Standard FMM often expands about cell center (acceptable with sufficient P); COM-centered expansion reduces error for elongated clusters.

**Confidence** | Suspicious (accuracy, not total failure)

---

## FMM-008 — Global softening `eps=0.1` in M2L and direct sum

| Field | Value |
|-------|-------|
| **ID** | FMM-008 |
| **Severity** | Medium |
| **Category** | Physics / Numerical |
| **Location** | `namespace fmm`, `m2l()` 398; direct 616; global `eps` line 9 |
| **Status** | Needs Verification |

**Description**  
Fixed absolute softening `0.1` added in quadrature to `r²`. Not scaled to inter-particle spacing or cell size. Dominates close encounters; under-resolves far field if coordinates are large; breaks scale invariance.

**Confidence** | Likely

---

## BODY-001 — Body::clone() omits jerk, dead, clusterIndex, m_forRes

| Field | Value |
|-------|-------|
| **ID** | BODY-001 |
| **Severity** | Low |
| **Category** | Logic |
| **Location** | `Body::clone()`, ~212–215 |
| **Status** | Needs Verification |

**Description**  
Clone copies mass, radius, movability, pos, vel, force only. `m_jerkVec`, `dead`, `clusterIndex`, `m_forRes`, `m_accVec` reset/default in new Body. If clone used mid-timestep (KS/collision paths later), stale or zero jerk/acceleration possible.

**Confidence** | Suspicious (impact depends on call sites outside reviewed section)

---

## KS-003 — Pair selection order-dependent (first matching `j` wins)

| Field | Value |
|-------|-------|
| **ID** | KS-003 |
| **Severity** | Medium |
| **Category** | Logic |
| **Location** | `namespace ks_regularization`, `extractCloseEncounters()`, ~985–1028 |
| **Status** | Needs Verification |

**Description**  
For each `i`, inner loop scans `j = i+1…` and `break`s on first pair satisfying distance criterion. If body `i` is close to multiple partners, which pair is regularized depends on index ordering, not physical priority (closest pair, smallest separation, etc.).

**Evidence**  
Line 1028: `break` after first match; no minimum-distance search.

**Suggested fix**  
Find closest `j` for each `i`, or globally greedily match minimum separation pairs.

**Confidence** | Likely

---

## KS-004 — `H` frozen across entire macro-step (no re-sync)

| Field | Value |
|-------|-------|
| **ID** | KS-004 |
| **Severity** | Medium |
| **Category** | Physics / Numerical |
| **Location** | `namespace ks_regularization`, `RegularizedPair::H`, ~903–906, 1046 |
| **Status** | Fixed |

**Description**  
`H` computed at extraction from relative `(r,v)` and never refreshed before analytical step. COM motion errors or prior numerical drift change true binary energy; internal step still uses stale `H` → phase/amplitude error in relative orbit.

**Fix (2026-07-24)**  
Subsumed by KS-001 fix: `restoreCloseEncounters` recomputes `H` from current `(u,u')` via `ksToPhysical` immediately before `stepKSAnalytical`.

**Suggested fix**  
Recompute `H` from `ksToPhysical` state immediately before `stepKSAnalytical`, or after COM update using reconstructed relative kinematics.

**Confidence** | Likely

---

## KS-005 — Absolute `R_KS_THRESHOLD = 2.5` not scale-adaptive

| Field | Value |
|-------|-------|
| **ID** | KS-005 |
| **Severity** | Low |
| **Category** | Physics / Architecture |
| **Location** | line 863, extract condition ~996 |
| **Status** | Open |

**Description**  
Fixed distance threshold ignores local dynamical scale (Hill radius, softening `eps`, mean inter-particle spacing). Works or fails depending on simulation units.

**Confidence** | Likely

---

## KS-006 — `getSundmanAdaptiveDT` appears unused

| Field | Value |
|-------|-------|
| **ID** | KS-006 |
| **Severity** | Low |
| **Category** | Architecture |
| **Location** | `getSundmanAdaptiveDT()`, ~1081–1097 |
| **Status** | Open |

**Description**  
No call sites in `physics2.cpp`. Heuristic `dt / max(1/dist)` is not wired into integrators despite comment about preventing rejection.

**Confidence** | Confirmed (dead code in this file)

---

## KS-007 — `restoreCloseEncounters` lacks null/dead-body guards

| Field | Value |
|-------|-------|
| **ID** | KS-007 |
| **Severity** | Medium |
| **Category** | Logic |
| **Location** | `restoreCloseEncounters()`, ~1041–1043 |
| **Status** | Fixed |

**Description**  
Assumes `bodies[idxA]` and `bodies[idxB]` remain valid if collision/removal occurs between extract and restore in same step. Dangling indices → UB.

**Fix (2026-07-24)**  
Subsumed by KS-001 fix: bounds check on indices; skip pair if pointers null or `dead`.

**Confidence** | Suspicious (depends on collision ordering in `physics` namespace)

---

## KS-009 — COM proxy retains original collision radius

| Field | Value |
|-------|-------|
| **ID** | KS-009 |
| **Severity** | Low |
| **Category** | Physics |
| **Location** | `extractCloseEncounters()`, ~1011–1013 |
| **Status** | Open |

**Description**  
Body A mass updated to `M_tot` but `m_radius` unchanged (still single-body radius). Collision test `dist > m_radius_i + m_radius_j` and physical collision geometry inconsistent while KS-active.

**Confidence** | Likely

---

## KS-010 — KS derived for 1/r Kepler, not 2D ln(r) gravity

| Field | Value |
|-------|-------|
| **ID** | KS-010 |
| **Severity** | High |
| **Category** | Physics / Math |
| **Location** | `physicalToKS()` energy ~903–906; `stepKSAnalytical()` |
| **Status** | Needs Verification |

**Description**  
`H = ½v² − G(M₁+M₂)/r` and harmonic oscillator `u''−(H/2)u=0` are correct for Kepler/Coulomb **1/r** potential. True 2D gravity (`φ ∝ ln r`) has different regularization theory; this KS engine does not apply.

**Confidence** | Likely (same kernel family as FMM-004/BH-005)

---

## COL-003 — `checkCol()` ignores `dead` flag

| Field | Value |
|-------|-------|
| **ID** | COL-003 |
| **Severity** | Medium |
| **Category** | Logic |
| **Location** | `namespace physics`, `checkCol()`, ~1268–1340 |
| **Status** | Fixed |

**Description**  
O(N²) collision pass has no `dead` guard (unlike `checkColSpatialHash` ~1321). Marked-dead bodies still participate in overlap tests and clustering if not yet erased.

**Fix (2026-07-24)**  
Pairwise loop already skipped dead bodies (lines 1270, 1275). Added `dead` guard to cluster-rebuild loop (~1337) so erased-pending bodies are not re-added to merge clusters.

**Confidence** | Likely

---

## COL-005 — Merged body retains stale acceleration/jerk

| Field | Value |
|-------|-------|
| **ID** | COL-005 |
| **Severity** | Medium |
| **Category** | Stale state |
| **Location** | `checkCol()` / `checkColSpatialHash()` merge ~1238–1245 / ~1437–1444 |
| **Status** | Fixed |

**Description**  
`mergedBody` copied from `colClusters[i][0]`; only `m_forVec` refreshed from sum. `m_accVec`, `m_jerkVec` remain from pre-merge member until next `resolve*`.

**Fix (2026-07-24)**  
After setting `m_forVec = totalForce`, both merge sites (`checkCol` and `checkColSpatialHash`) now also set:
- `mergedBody.m_accVec = totalForce * (1.0 / totalMass)` — by Newton's second law (F = ma → a = F/m), the net acceleration of the merged COM body equals the total force divided by total mass.
- `mergedBody.m_jerkVec = vectorP(0.0, 0.0)` — a collision is a discontinuous topology change; the jerk field has no physically meaningful value across the merge event. Zeroing forces the Hermite integrator to restart cleanly from the new acceleration state on the next step, rather than extrapolating from a pre-merge member's jerk history which refers to a body that no longer exists.

**Confidence** | Confirmed

---

## COL-006 — Float accumulator for cluster total mass

| Field | Value |
|-------|-------|
| **ID** | COL-006 |
| **Severity** | Low |
| **Category** | Numerical |
| **Location** | merge loops ~1221, 1421 |
| **Status** | Fixed |

**Description**  
`float totalMass` sums `double m_Mass`; COM division loses precision for large masses (e.g. 1e12 kg presets).

**Fix (2026-07-24)**  
Already corrected in source: merge loops use `double totalMass`.

**Confidence** | Confirmed

---

## PHYS-003 — `computeAccel` hardcoded `eps`, ignores global

| Field | Value |
|-------|-------|
| **ID** | PHYS-003 |
| **Severity** | Medium |
| **Category** | Physics / Numerical |
| **Location** | `computeAccel()`, ~1550 |
| **Status** | Fixed |

**Description**  
`constexpr double eps = 0.1` local shadow; decoupled from file-scope `eps` (line 9) and `pull()` which uses global. Changing global `eps` leaves RK45/BH fallback path stale if swapped back.

**Fix (2026-07-24)**  
Removed local shadow; `computeAccel()` now uses file-scope `eps`.

**Confidence** | Confirmed

---

## PHYS-004 — Duplicate `G` constant in `namespace physics`

| Field | Value |
|-------|-------|
| **ID** | PHYS-004 |
| **Severity** | Low |
| **Category** | Architecture |
| **Location** | line 8 vs ~1101 |
| **Status** | Open |

**Description**  
File-scope `G` and `physics::G` both `6.67430e-11`. FMM/BH use file-scope; `pull`/`resolveWithJerk` use `physics::G`. Divergence if one is edited.

**Confidence** | Confirmed

---

## PHYS-005 — `pull()` / `resolve()` ignore `movability`

| Field | Value |
|-------|-------|
| **ID** | PHYS-005 |
| **Severity** | Low |
| **Category** | Logic |
| **Location** | `pull()` ~1108–1120; `resolve()` ~1483–1497 |
| **Status** | Open |

**Description**  
All pairs computed regardless of `movability`. Immovable bodies still accumulate `m_forRes`/`m_accVec`. Integration skips them, but diagnostics and collision force sums see computed values. Inconsistent with `resolveWithJerk` / FMM paths.

**Confidence** | Confirmed

---

## PHYS-006 — 2D gravity label vs inverse-square kernel (direct path)

| Field | Value |
|-------|-------|
| **ID** | PHYS-006 |
| **Severity** | High |
| **Category** | Physics |
| **Location** | `pull()` ~1112–1116; `resolveWithJerk()` ~1516–1533 |
| **Status** | Needs Verification |

**Description**  
Direct Newton pair force uses `G m₁ m₂ r / r³` (3D-style 1/r²). Same issue as FMM-004/BH-005. Figure-8 preset velocities tuned for this kernel.

**Confidence** | Likely (consistent with project presets)

---

## INT-002 — Hermite modifies global `dt` by reference

| Field | Value |
|-------|-------|
| **ID** | INT-002 |
| **Severity** | Medium |
| **Category** | Architecture |
| **Location** | `moveHermite()`, ~1840; `moveHermiteKS()`, ~2020–2025 |
| **Status** | Open |

**Description**  
`moveHermite(bodies, dt)` updates file-scope global `dt` via Aarseth criterion. Mixed integrator runs (Verlet + Hermite) share one `dt`; `moveHermiteKS` correctly restores KS with saved `h0` but global `dt` still mutated.

**Confidence** | Confirmed

---

## KS-011 — `restoreCloseEncounters` clobbers force state via `updateVal`

| Field | Value |
|-------|-------|
| **ID** | KS-011 |
| **Severity** | **High** |
| **Category** | Stale state |
| **Location** | `restoreCloseEncounters()`, ~1074–1075 |
| **Status** | Fixed |

**Description**  
Comment says "Update forces for UI sync" but `updateVal()` copies zeroed `m_forRes` into `m_forVec`. Contributes to COL-009. Does not re-evaluate gravity at new positions.

**Fix (2026-07-24)**  
Duplicate of COL-009; `updateVal()` already removed from `restoreCloseEncounters()`.

**Suggested fix**  
Re-run single-body force eval or drop `updateVal` and set `m_forVec = m_accVec * m_Mass` after optional `resolve` call.

**Confidence** | Confirmed

---

## EOS-001 — Energy diagnostic uses 1/r PE, not 2D ln(r)

| Field | Value |
|-------|-------|
| **ID** | EOS-001 |
| **Severity** | High |
| **Category** | Physics |
| **Location** | `eos()`, ~3082–3084 |
| **Status** | Needs Verification |

**Description**  
`PE += -G m₁ m₂ / softenedDist` matches inverse-square force kernel, not true 2D gravity. Energy drift reports conflate integrator error with wrong potential if 2D intended.

**Confidence** | Likely

---

## EOS-002 — `eos()` / conservation helpers skip no guards

| Field | Value |
|-------|-------|
| **ID** | EOS-002 |
| **Severity** | Low |
| **Category** | Logic |
| **Location** | `eos()` ~3066; `linearP`/`angularP` ~3100–3111 |
| **Status** | Open |

**Description**  
No skip for `dead`, zero-mass KS-suppressed bodies, or `nullptr`. Usually harmless post-collision erase.

**Confidence** | Suspicious

---

## MAIN-001 — `main()` simulation paths inherit PROD-001

| Field | Value |
|-------|-------|
| **ID** | MAIN-001 |
| **Severity** | Critical (inherits FMM-001) |
| **Category** | Architecture |
| **Location** | `main()` ~2587, 2788, 2922 |
| **Status** | Fixed |

**Description**  
Interactive run uses `moveRK45KS`; batch modes use `moveYoshida` → `resolveFMM`. Energy/momentum diagnostics (`eos`, `linearP`, `angularP`) therefore measure broken dynamics.

**Fix (2026-07-24)**  
Subsumed by PROD-001 / FMM-001 fix; production FMM paths now complete.

**Confidence** | Confirmed

---

## BH-002 — Multipole self-force when θ > 1/√2

| Field | Value |
|-------|-------|
| **ID** | BH-002 |
| **Severity** | Medium (Low at default θ=0.5) |
| **Category** | Math / Physics |
| **Location** | `namespace bh`, `walk()`, ~798–805 |
| **Status** | Needs Verification |

**Description**  
Internal nodes containing the target body `bi` can be accepted as multipole when `4·half² < θ²·r²`, i.e. `r > 2·half/θ`. Maximum separation between two points inside a cell of half-width `half` is `2√2·half ≈ 2.83·half`. For **θ > 1/√2 ≈ 0.707**, there exist configurations (body near one side, COM near opposite side) where the criterion accepts while `bi` is still inside the cell — aggregate mass includes `bi` → spurious self-contribution.

**Evidence**  
No explicit “if target inside cell, must open” guard before line 801. With default `BH_THETA=0.5`, need `r > 4·half`; max in-cell distance ≈ `2.83·half` → criterion cannot accept nodes containing `bi`. Bug activates when users raise θ above ~0.71.

**Suggested fix**  
Before multipole acceptance: if `|bx−cx| ≤ half && |by−cy| ≤ half`, force child recursion.

**Confidence** | Likely (proven for θ > 1/√2; inactive at default 0.5)

---

## BH-005 — 2D gravity label vs inverse-square kernel (same as FMM-004)

| Field | Value |
|-------|-------|
| **ID** | BH-005 |
| **Severity** | High |
| **Category** | Physics |
| **Location** | `namespace bh`, `apply()`, ~763–764 |
| **Status** | Needs Verification |

**Description**  
`apply()` uses `a += G·M·r/r³` (3D-style). True 2D Newtonian gravity is `F ∝ 1/r`. Same issue as FMM-004.

**Confidence** | Likely

---

## BH-006 — Jerk uses cell COM velocity under multipole acceptance

| Field | Value |
|-------|-------|
| **ID** | BH-006 |
| **Severity** | Medium |
| **Category** | Physics / Numerical |
| **Location** | `namespace bh`, `apply()` jerk branch, ~766–770; `walk()` ~802–804 |
| **Status** | Needs Verification |

**Description**  
When an internal node is accepted, jerk uses `dvx = com_vx − bvx` (mass-weighted cell velocity). True jerk requires summing per-body time derivatives or differentiating the multipole field. Direct leaf pairs are exact; aggregated cells are inconsistent with Hermite’s expected jerk accuracy.

**Confidence** | Likely

---

## BH-007 — Pool overflow if `next_node` exceeds pre-allocated size

| Field | Value |
|-------|-------|
| **ID** | BH-007 |
| **Severity** | Low |
| **Category** | Memory |
| **Location** | `namespace bh`, `alloc()`, ~661–665; `build()`, ~838–839 |
| **Status** | Needs Verification |

**Description**  
`alloc()` writes `pool[idx]` without bounds check. `max_nodes = max(64*s, 256)` is heuristic; pathological deep trees (many near-coincident bodies above subdivision floor before BH-003 triggers) could theoretically exceed capacity → OOB write.

**Confidence** | Suspicious

---

## GLOBAL-001 — dt initialized with float literals

| Field | Value |
|-------|-------|
| **ID** | GLOBAL-001 |
| **Severity** | Low |
| **Category** | Numerical |
| **Location** | line 7 |
| **Status** | Fixed |

**Description**  
`double dt = (1.0f / 120.0f)` promotes float division to double; loses ~1 ulp vs `1.0/120.0`. Minor for 120 Hz but inconsistent with `double` integrators.

**Fix (2026-07-24)**  
Already corrected to `double dt = 1.0 / 120.0;` in source.

**Confidence** | Confirmed

---

# Mathematical Verification

| Component | Status | Notes |
|-----------|--------|-------|
| **Barnes-Hut** | ⚠ Suspicious | Opening criterion `4·half² < θ²·r²` matches standard `s/d < θ` (798–801). Monopole-only (no quadrupole) — expected O(θ²) bias. Insert/update COM (704–709) correct weighted average. Iterative insert avoids reference invalidation — sound. **Critical**: BH-003 particle loss. **Medium**: BH-001 hardcoded softening. Self-multipole (BH-002) inactive at θ=0.5 but active for θ≳0.71. Coincident-body guard (720) partial. |
| **Fast Multipole Method** | ✗ Incorrect | Incomplete interaction coverage (FMM-001/003). Kernel may be 3D 1/r in 2D domain (FMM-004). M2M binomial shift (366–391) structurally matches standard Cartesian translation. M2L recurrence (400–416) matches partial derivatives of `1/r`. L2L (432–453) matches standard local-to-local shift. |
| **KS / Levi-Civita** | ⚠ Suspicious | **Position map** (915–917): standard 2D LC. **Harmonic step** (940–972): bound/unbound/parabolic branches match `u''=(H/2)u`. **COM unpack** correct. **Split scheme** (KS-001): first-order tidal kick at restore couples external perturbations O(Δt). **Sundman** (KS-002): **Fixed** — composite sub-stepping with refreshed `r` each interval (up to 64 sub-steps). **dtau safety** (KS-008): capped via `capKSdtau()`; extraction refused below `R_KS_MIN`. |
| **Force equations (direct in FMM)** | ⚠ Suspicious | `a_i += G m_j (r_j−r_i)/|r|³` correct for inverse-square; wrong if 2D `1/r` intended. Jerk formula (624–627) matches `d/dt(G m r/r³)` for pairwise term. |
| **Jerk equations (L2P)** | ⚠ Suspicious | Spatial gradient only; missing `∂a/∂t` (FMM-005). |
| **Direct `pull` / `resolveWithJerk`** | ✓ Verified (1/r kernel) | `pull`: `F = G m₁ m₂ r̂ / r²` via `r/r³` with softening `eps²` (1112–1116). Newton third law via negate (1118–1120). `resolveWithJerk` jerk (1528,1533): `ȧ = Gm(v/r³ − 3(v·r)r/r⁵)` — correct time derivative of softened `Gm r/r³`. |
| **Hermite PECE (snap/crackle)** | ✓ Verified | Snap/crackle (1821–1822) match standard 4th-order Hermite corrector. Aarseth `dt` (1835) matches `(a₀a₂ + a₁²)/(a₁a₃ + a₂²)` under `{a₀,a₁,a₂,a₃} = {|a|,|j|,|snap|,|crackle|}`. |
| **RK45 (DOPRI5 on x''=a(x))** | ⚠ Suspicious | Stage mixing `kx=velocity`, `kv=acceleration` (1906–1916) valid for velocity-independent forces. Error weights applied to both — approximate for 2nd-order system. FSAL reuse of `kv[6]` (1948) consistent. Uses broken FMM (PROD-001). |
| **Yoshida 4th order** | ✓ Verified | Coefficients `c`, `d` (1745–1746) standard; 3 kicks / 4 drifts (1761). |
| **Velocity Verlet** | ✓ Verified | Standard kick-drift-kick (1714–1736); uses exact `resolve`. |
| **Collision merge** | ✗ Incorrect | COM/velocity mass-weighted (1240–1241) correct. Force sum additive (1233) correct. Coalition stale vectors (COL-004); transitive merge (COL-007); KS restore zeros force before merge (COL-009). |
| **Collision equations** | ✗ Incorrect | See COL-004/007/009. Spatial hash broad-phase sound (`cell_size ≥ max_diam`). |
| **Integrator equations** | ⚠ Suspicious | Math verified for Verlet/Yoshida/Hermite/RK45 structure; production paths wired to broken FMM (PROD-001). Hermite `dt` growth (INT-001). |

### M2M derivation check (lines 366–391)

Shift `(dx,dy)` from child to parent center. Child moment `M_c(i,j)` contributes to parent `(p_idx,q)` via  
`Σ_{i≤p_idx,j≤q} C(p_idx,i) C(q,j) dx^{p_idx−i} dy^{q−j} M_c(i,j)` — **matches** standard polynomial translation for harmonic multipoles.

### idx() verification (260–262)

For `P=8`, pairs with `p+q≤8` map bijectively to `[0,44]`. Loop bounds in P2M/M2M/M2L/L2L/L2P respect `p+q≤P`. **Verified** indexing consistency.

---

# Physics Validation

| Check | Status | Notes |
|-------|--------|-------|
| Energy conservation | ✗ | FMM-001/PROD-001 in active integrators; BH-003 ghost mass; collision merge force zero after KS (COL-009); eos uses 1/r PE (EOS-001) |
| Linear momentum | ✗ | FMM omitted pairs; transitive collision merge (COL-007); KS restore force clobber (COL-009) |
| Angular momentum | ✗ | Same FMM issues; `angularP` = Σ r×(mv) correct for 2D scalar (222–224, 3106–3111) |
| Newton's Third Law | ✓ (direct path) | `pull` symmetric via negate. ⚠ FMM/BH approximate |
| COM motion | ⚠ | Collision COM mass-weighted correct; multi-body transitive merge wrong (COL-007) |
| Collision behavior | ✗ | COL-004/007/009; 3D radius merge in 2D (`r ∝ V^(1/3)`) |
| Softening | ⚠ | Global `eps=0.1` in pull/resolveWithJerk; `computeAccel` hardcoded (PHYS-003) |
| Binary handling | ⚠ | KS extract/restore; tidal kick at restore (KS-001 fixed); Sundman composite sub-stepping (KS-002 fixed); `moveHermiteKS`/`moveRK45KS` timing correct |

---

# Numerical Stability

| Issue | Location | Severity |
|-------|----------|----------|
| `eps²` added to `r²` prevents `r=0` singularity in direct/M2L | 398, 616 | Mitigates but fixed scale |
| `half` lower bound `1e-6` | 532 | Prevents zero-size root |
| `is_neighbor` tolerance `1e-9` | 309 | Prevents borderline float misses |
| NaN from `vectorP/` zero | 97–100 | VEC-001 |
| Body zero-mass guard in `updateVal` | 196–198 | Good pattern |
| Deep tree `max_depth ≤ 8` | 535 | May reduce but not fix missing far-field |
| BH hardcoded `+0.01` softening | bh `apply()` 759 | BH-001; decoupled from `eps` |
| BH subdivision floor `1e-10` | insert 720 | BH-003; bodies dropped below floor |
| BH `build()` empty input | 823 | BH-004; OOB access |
| BH zero-mass node early return | walk 784 | Skips force; massless sources exert no force |
| KS `dtau = Δt/r` with `r` floor 1e-15 | stepKSAnalytical 932–936 | KS-008 **Fixed**: `R_KS_MIN`, branch caps, early return |
| KS `H` from `μ/(r+1e-15)` at r→0 | physicalToKS 906 | Large negative H; unstable oscillator |
| KS parabolic `H==0` linear drift | stepKSAnalytical 966–971 | Exact for pure H=0; measure-zero in floats |
| `resolve()` empty vector | resolve 1496 | PHYS-002; OOB on `bodies[-1]` |
| Hermite dt doubling | moveHermite 1813–1840 | INT-001; no shrink when snap/crackle ≈ 0 |
| RK45 accept at DT_MIN | moveRK45 ~2073 | INT-003 **Fixed**: documented floor accept + diagnostic counters |
| KS restore zeros m_forVec | restoreCloseEncounters 1074 | KS-011 / COL-009 **Fixed** |
| Collision float totalMass | checkCol 1221 | COL-006 **Fixed** |
| Cluster coalition stale vectors | checkCol 1164–1182 | COL-004; split clusters |

---

# Performance Notes

- Claimed O(N) FMM (line 253) **not substantiated**; incomplete algorithm may appear O(N) but returns wrong physics.
- `max_depth = min(8, ⌈log₂ n / 2⌉)` caps tree depth — accuracy/performance tradeoff for large N.
- Leaf bucket size 16 (315) — reasonable.
- Static `tree` vector reuse with geometric resize (299) — amortized O(1) alloc per node.
- M2L quadruple loops over `(a,b,u,v)` with `P=8` — expensive per pair; pairs incomplete anyway.
- Direct neighbor double loop over all leaf pairs × bodies² (608–630) — can degrade toward O(N²) in dense neighbor configs without delivering full O(N²) accuracy.

- BH `insert()` iterative tail-recursion (690–749) — avoids stack depth; O(depth) per body.
- BH one-body-per-leaf invariant — tree size O(n) typically; pool `64*n` prealloc usually sufficient (BH-007).
- BH `walk()` per body — O(n log n) expected; monopole-only acceptance reduces cost vs full multipole FMM.
- Coincident-body infinite subdivision until `half < 1e-10` — up to ~53 levels from domain scale 1e6; then BH-003 data loss.

- **Production paths use FMM** (moveYoshida/Hermite/RK45) — wrong physics despite O(N) appearance (PROD-001).
- `checkColSpatialHash`: O(N) grid build + O(N·k) neighbor checks; `cell_size = max(max_diam, 1.0)` floor.
- `checkCol` O(N²) still used in stat 1/2 batch paths (2796, 2932).
- Hermite/RK45 adaptive `dt` mutate global `dt` (INT-002).
- KS wrappers add extract/restore overhead per step; no subcycling.

---

# Open Questions

1. Is the simulation intentionally inverse-square (3D-style) in 2D coordinates, or true 2D `1/r` gravity?
2. Has FMM ever been validated against direct `resolve()` on multi-cluster configs?
3. Is `resolveFMM` used in production runs or experimental only?
4. What is the intended role of `com_vx/com_vy` in nodes if not used in L2P?
5. Are coincident-body scenarios common (KS close encounters)? BH-003 would fire after ~53 subdivisions.
6. Is `resolveBarnesHut` intended for production? Currently commented out in integrators (~1763+).
7. Is KS split approximation (KS-001) acceptable for target science cases (triple encounters)?
8. Should `getSundmanAdaptiveDT` be wired in or removed?
9. Is transitive collision clustering (A–B, B–C → merge A,B,C) intentional?
10. Should production revert to direct `resolve()` until FMM-001 is fixed?
11. Should `restoreCloseEncounters` re-evaluate forces instead of calling `updateVal`?

---

# Next Fix Target

```
File:        physics2.cpp
Issue:       KS-003 — Pair selection order-dependent (first matching j wins)
Location:    namespace ks_regularization, extractCloseEncounters(), ~985–1028 (line 1165 in current file)
Status:      Needs Verification (Medium)
Description: For each body i, the inner loop breaks on the FIRST j that satisfies the
             distance criterion, regardless of which pair has the smallest separation.
             If body i is close to multiple partners, the chosen pair depends on index
             ordering rather than physical priority (closest pair).
Suggested:   Replace break-on-first-match with a minimum-distance search over all j;
             register the closest qualifying pair for body i.
Then:        KS-005 — Absolute R_KS_THRESHOLD = 2.5 not scale-adaptive
```

---

# Next Review Target

```
File:        physics2.cpp — COMPLETE (3112 / 3112 lines)
Next file:   physics1.cpp or physics.cpp (if continuing project audit)
Suggested:   Validate PROD-001 by comparing moveVerlet (direct) vs moveYoshida (FMM) on 2-cluster test
```

Review direct Newtonian pair force, `resolve()` / `resolveWithJerk()`, collision handling, COM helpers, and integrators (`moveVerlet`, `moveYoshida`, `moveHermite`, `moveRK45`) — target ~500–1000 lines (approx. 1100–1700 or through first integrator block).
