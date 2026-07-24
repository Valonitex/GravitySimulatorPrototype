# physics.cpp — Bug Report & Fix Guide

---

## Bug 1 — FMM kernel mismatch  *(primary cause of 5-order energy drift)*

### What it is

The complex-multipole machinery (P2M → M2M → M2L → L2L → L2P) encodes the
kernel $1/(z_s - z)$, which in real components gives:

$$F_x = \frac{Gm(x_s - x)}{r^2}, \quad F_y = \frac{Gm(y_s - y)}{r^2}$$

That is **2D log-law gravity** ($|F| \propto 1/r$), the force law of charged
wires or 2D point vortices — *not* 3D Newtonian gravity.

Proof from P2M: `M[k] += G*m * (z_j − z_C)^k`.  
Proof from M2L: `L[l] = Σ_k C(l+k,k) * M[k] / D^{l+k+1}` with `D = z_src − z_tgt`.  
For a pure monopole ($M_0 = Gm$, all others 0) this gives `L[l] = Gm / D^{l+1}`.  
L2P evaluates $\sum_l L_l\,dz^l = Gm/(D - dz) = Gm/(z_s - z)$ → 2D log-law. ✓

Every other part of the code — `pull()`, `computeAccel()`, `resolveWithJerk()`,
BH `apply()`, and `eos()` — uses **3D Newtonian gravity** ($|F| \propto 1/r^2$,
softened potential $-Gm/\sqrt{r^2+\varepsilon^2}$).

`eos()` measures the Hamiltonian $H_{3D} = \frac12\sum m_i v_i^2 - \sum_{i<j}
\frac{Gm_im_j}{\sqrt{r^2+\varepsilon^2}}$.  The FMM integrates the trajectory
under the **completely different** Hamiltonian $H_{2D\text{-log}} = \frac12\sum
m_i v_i^2 + \sum_{i<j} Gm_im_j \ln r_{ij}$.  These share no conserved
quantity, so apparent energy conservation is catastrophic.

### Why it only appears at large N

`max_depth = min(8, ceil(log2(N)/2))`.  For $N < 8$, max\_depth = 1.  The M2L
loop starts at level 2, so it never executes — all interactions fall through to
near-field P2P (Bug 2 below) which used the 3D kernel.  The FMM accidentally
gave correct 3D results for small $N$ by degeneration.

For $N \geq 8$ (max\_depth $\geq 2$), M2L activates and starts contributing
2D log-law far-field forces while the P2P still applies 3D forces — a
discontinuous, non-Hamiltonian mixed force law that compounds with every step.

### Fix

The FMM's near-field P2P is corrected to use the same 2D log-law kernel as the
far-field, making the FMM internally consistent (see Bug 2).

**The FMM still implements 2D log-law gravity, not 3D Newtonian.**  For 3D
Newtonian gravity use `computeAccelBH` / `resolveBarnesHut` — those are correct
and consistent with every other integrator and with `eos()`.

---

## Bug 2 — FMM near-field P2P uses 3D kernel while far-field uses 2D log-law

### What it is

Inside `fmm::run_fmm`, the near-field P2P loop for neighbouring leaves was:

```cpp
double r2    = r.magSq() + eps*eps;     // softened
double inv_r = 1.0 / std::sqrt(r2);
double inv_r3 = inv_r * inv_r * inv_r;
acc_out[i] += r * (G * masses[j] * inv_r3);     // |F| ∝ 1/r²  (3D Newton)

// jerk:
double inv_r5 = inv_r3 * inv_r * inv_r;
jerk[i] += (v * inv_r3 - r * (3.0 * v_dot_r * inv_r5)) * G * masses[j]; // 3D jerk
```

The correct 2D log-law near-field:

$$F = Gm\,\mathbf{r}/r^2 \quad(|\mathbf{F}| \propto 1/r)$$
$$\dot{\mathbf{F}} = Gm\left[\frac{\mathbf{v}_{rel}}{r^2} - \frac{2(\mathbf{v}_{rel}\cdot\mathbf{r})\,\mathbf{r}}{r^4}\right]$$

Note also: no softening ($\varepsilon = 0$) — the complex multipole expansion
has no softening term, so adding softening only to the near-field creates another
discontinuity at the near/far boundary.

### Fix (applied)

```cpp
double r2     = r.magSq();
if (r2 < 1e-20) continue;
double inv_r2 = 1.0 / r2;

acc_out[i] += r * (G * masses[j] * inv_r2);     // |F| ∝ 1/r  (2D log-law) ✓

// jerk for F = Gm r/r²:
double v_dot_r = v.icap*r.icap + v.jcap*r.jcap;
double inv_r4  = inv_r2 * inv_r2;
jerk[i] += (v * inv_r2 - r * (2.0 * v_dot_r * inv_r4)) * G*masses[j];  // ✓
```

---

## Bug 3 — KS extraction does not zero body B's mass → double-counting gravity

### What it is

`extractCloseEncounters` sets body A's mass to $M_{tot} = m_A + m_B$ so that
third bodies feel the correct combined gravity.  But body B's mass is left at
$m_B$ and body B is only flagged `movability = false`.

`pull()` and the BH tree both include body B in force calculations regardless of
movability.  So a third body C feels:

$$\text{gravity from A} \;(M_{tot} = m_A + m_B) \;+\; \text{gravity from B} \;(m_B)
= m_A + 2m_B$$

instead of the correct $m_A + m_B$.  This silently corrupts linear momentum,
angular momentum, and energy whenever any two bodies are within `R_KS_THRESHOLD`.

### Fix (applied)

```cpp
// in extractCloseEncounters, immediately after movability = false:
bodies[j]->m_Mass = 0.0;       // suppress B's ghost gravity
```

`restoreCloseEncounters` already saves and restores `m_B` via `pair.m2`, so
this is safe.  The mass zeroing also requires the `updateVal()` guard (Bug 4).

---

## Bug 4 — `updateVal()` divides by zero when mass is zeroed

### What it is

`resolve()` calls `updateVal()` on every body unconditionally.  With Bug 3
fixed, body B has `m_Mass = 0`, so the line:

```cpp
m_accVec = m_forVec / m_Mass;   // → NaN when m_Mass == 0
```

produces NaN that propagates into positions, velocities, and the energy
accumulator, corrupting the entire simulation for the remainder of the run.

### Fix (applied)

```cpp
void updateVal() {
    m_forVec = m_forRes;
    m_accVec = (m_Mass > 0.0) ? (m_forVec / m_Mass) : vectorP(0.0, 0.0);
    m_forRes = vectorP(0.0f, 0.0f);
}
```

---

## Bug 5 — RK45 BH fallback comment passes wrong arrays in stage loop

### What it is

```cpp
// Stages k2..k7
for (int st = 1; st <= 6; st++) {
    ...build xs, vs from Butcher sum...
    kx[st][i] = vs[i];
    //computeAccelBH(x0, kv[0], bodies);  ← WRONG
```

If this comment were uncommented, every stage would evaluate BH accelerations
at the **initial position** `x0` and write into the **k1 buffer** `kv[0]` —
making k2 through k7 all identical to k1.  The Butcher tableau would silently
collapse to a first-order method (Euler), and the error estimator would report
zero error, letting the step-size controller grow `h` to `DT_MAX` in one step.
Energy errors of many orders of magnitude would follow immediately.

### Fix (applied)

```cpp
//computeAccelBH(xs, as, bodies);  // ← xs/as: correct per-stage arrays
```

---

## Bug 6 — All movement methods lack KS wrappers

### What it is

`ks_regularization::extractCloseEncounters` and `restoreCloseEncounters` existed
but were never connected to any of the four integrators.  There was no
`moveVerletKS`, `moveYoshidaKS`, `moveHermiteKS`, or `moveRK45KS`.

### Fix (new functions added in `namespace physics`)

```
moveVerletKS(bodies)           — extract → moveVerlet   → restore(dt)
moveYoshidaKS(bodies)          — extract → moveYoshida  → restore(dt)
moveHermiteKS(bodies, dt_ref)  — extract → moveHermite  → restore(h₀)
moveRK45KS(bodies)             — extract → moveRK45     → restore(h_used)
```

#### Hermite: which dt to pass to restore?

`moveHermite` receives `dt_ref` on entry (the step to apply) and **overwrites**
it on exit with the Aarseth next-step suggestion.  The KS binary must be advanced
by the step that was *actually applied*, not the suggested next one.  The wrapper
saves `h0 = dt_ref` before calling `moveHermite` and passes `h0` to restore.

#### RK45: step rejection is invisible to KS

`moveRK45` may internally retry with a smaller `h` before accepting.  The KS
state is never advanced during a rejected trial — only the returned `h_used`
(the accepted physical step) is passed to `restoreCloseEncounters`, so internal
and external evolution stay perfectly in sync.

---

## Summary table

| # | Location | Severity | Effect |
|---|----------|----------|--------|
| 1 | `fmm` far-field multipoles | **Critical** | 2D log-law ≠ 3D Newtonian; eos measures wrong Hamiltonian → 5-order energy drift |
| 2 | `fmm::run_fmm` P2P | **Critical** | Mixed kernels within FMM; force discontinuity at near/far boundary |
| 3 | `ks_regularization::extractCloseEncounters` | **High** | Double-counts body B mass → corrupts momentum and energy whenever KS active |
| 4 | `Body::updateVal()` | **High** | Division by zero (NaN) after Bug 3 fix, or if any body ever gets mass 0 |
| 5 | `physics::moveRK45` BH comment | Medium | RK45 collapses to Euler if BH is switched in; instant energy blow-up |
| 6 | `namespace physics` | Medium | No KS-aware move functions existed; close encounters could not be regularised |

---

## Recommendations

1. **Use `computeAccelBH` / `resolveBarnesHut` for production runs.** BH
   correctly implements the same softened 3D Newtonian gravity as `pull()`,
   `computeAccel()`, and `eos()`.  The FMM (now internally consistent) implements
   a *different* force law and should only be used if you specifically want 2D
   log-law gravity.

2. **Switch the main loop to KS-wrapped integrators** for any scenario with close
   encounters or highly eccentric orbits:
   ```cpp
   physics::moveYoshidaKS(bodys);   // instead of moveYoshida
   physics::moveRK45KS(bodys);      // instead of moveRK45
   ```

3. **The Sundman approximation** $\delta\tau \approx \delta t / r(t_0)$ inside
   `stepKSAnalytical` is first-order in $\delta t$.  For large steps or very
   tight binaries, implementing Kepler's equation (eccentric anomaly solver)
   would give exact physical-to-regularised time mapping.  For the typical use
   case (adaptive RK45 with small $h$ near close encounters) the approximation
   is acceptable.

4. **Perturbation forces** on the KS binary from external bodies are not included
   in `stepKSAnalytical` (it solves the pure Kepler problem).  For high-accuracy
   N-body work the Stumpff/Brouwer perturbation terms should be added to the KS
   equations of motion.
