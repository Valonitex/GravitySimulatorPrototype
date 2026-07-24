# FMM Rewrite Log

## Observations & Analysis
- The original FMM used complex numbers which natively models 2D gravity ($F \propto 1/r$).
- The main integrator and rest of the engine models 3D Newtonian gravity ($F \propto 1/r^2$) projected onto a 2D plane.
- Mismatch between FMM and rest of the engine caused energy drift.
- A 2D Cartesian Taylor expansion of the 3D potential $\Phi(x,y) = -Gm / \sqrt{x^2 + y^2 + \epsilon^2}$ is needed.

## Plan
1. **Remove complex arrays**: Replace `Cplx M[P+1]` and `L[P+1]` with real arrays for a 2D Cartesian Taylor expansion. For order $P=8$, the number of coefficients is $(8+1)(8+2)/2 = 45$.
2. **P2M**: Compute moments $M_{p,q} = \sum m_i \cdot \Delta x^p \cdot \Delta y^q / (p! q!)$.
3. **M2M**: Shift moments to parent center.
4. **M2L**: Compute derivatives of the softened 1/r kernel $D_{p,q} = \partial_x^p \partial_y^q (1/\sqrt{\Delta x^2 + \Delta y^2 + \epsilon^2})$. Multiply with moments to get local expansion coefficients.
5. **L2L**: Shift local coefficients to child center.
6. **L2P**: Evaluate local expansion and its derivatives for acceleration and jerk.
7. **P2P**: Re-implement exact $1/r^2$ softened near-field calculation.

## Progress
- Implementation plan approved.
- Decided to use $P=8$ for efficiency and $\epsilon=0.1$ for consistency.
- Test scenario will use `moga == 2` (eccentric orbit).
