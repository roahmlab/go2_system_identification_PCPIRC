# Go2 System Identification — Knowledge Base

Porting Khorshidi/Khadiv PCPIRC (ICRA 2025, arXiv 2409.09850) from Spot to Unitree Go2.
All numbers measured on this machine unless noted.

---

## 0. Dev plan

Working list of what to build next. Edit freely.

| # | item | status | notes |
|---|---|---|---|
| 1 | **Update the LMI solver for Go2** | todo | Blocked on the ellipsoid builder — see §4. Needs: `visuals`→`collisions` switch, union over fused children, `rpy` handling. `go2_config.yaml` still to be written (§5) |
| 2 | **Design exciting trajectories for Go2** | todo | Nothing to port — the authors shipped no trajectory code (§6). Multisine, all 6 channels simultaneous, coprime harmonics. Target the directions §7 shows are invisible |
| 3 | **Run and validate in simulation** | todo | MuJoCo as the independent plant. G0.3 already passes (§8). Inject a known Δφ into the plant and check the metric recovers it |

**Notes carried into these:**
- (1) and (3) are independent — sim validation of the *metric* doesn't need the LMI working.
- (2) is the actual research contribution; (1) and (3) are infrastructure for it.
- The trunk is the worst-identified link in every run so far (§7). That's the thing (2) has to fix.

---

## 1. Environment

**`uv`-managed venv** (`pyproject.toml`, Python 3.11) — replaces the earlier `conda env go2sysid`.
`uv sync` installs: pinocchio 3.9.0 (PyPI package name `pin`), cvxpy 1.9.2, casadi 3.7.2,
mujoco 3.10.0, numpy 2.4.6 — all ship as prebuilt wheels, no conda-forge needed.

- `pinocchio` pinned to `3.9.0` on purpose — unpinned gives 4.1.0; authors' code targets 3.7.0.
- `urdf-parser-py` added to `dependencies` — was missing before; `rigid_body_dynamics.py` imports
  it at module level.
- MOSEK is an optional extra (`uv sync --extra mosek`). License still at `~/mosek/mosek.lic`,
  academic, expires 12-aug-2027, unaffected by the env switch — `.gitignore` covers `*.lic`.
  **Optional overall** — CLARABEL (ships with cvxpy) solves the same SDP identically.
- `environment.yml` is now historical only — the authors' full conda export, pins
  `_x86_64-microarch-level=3=2_skylake` (fails on non-Skylake x86_64), never usable as-is.

---

## 2. Go2 model as Pinocchio builds it

`nq=19, nv=18, njoints=14` → **13 movable bodies**, `phi` = 130. Pinocchio welds fixed-joint
children into parents, **transitively**: 42 URDF links → 13 bodies.

| body | fused in | mass |
|---|---|---|
| `base` | base(6.921) + 4× hip_rotor(0.089) + Head_upper(.001) + Head_lower(.001) | **7.2790** |
| `*_hip` | hip(0.678) + thigh_rotor(0.089) | 0.7670 |
| `*_thigh` | thigh(1.152) + calf_rotor(0.089) | 1.2410 |
| `*_calf` | calf(0.154) + foot(0.040) + 2 massless shells | 0.1940 |

`7.279 + 4×(0.767+1.241+0.194) = 16.0870` ✓

Trunk fixture: `mass 7.2790, com (0.0201531, 0, -0.0051090), Izz 0.112809`

> **Corrected from an earlier 7.277.** That counted only the four hip rotors. `Head_upper` and
> `Head_lower` (the latter fused *through* the former) add 2 g. **Enumerate the transitive closure
> of fixed joints, not direct children.** G0.1 must assert 7.2790 or it false-fails.

---

## 3. URDF: visual vs collision vs inertial

Three independent blocks. **Nothing validates them against each other.**

| block | used by | affects dynamics |
|---|---|---|
| `<inertial>` | RNEA, regressor — this **is** `phi` | **yes, only this** |
| `<visual>` | renderers (meshcat, RViz) | no |
| `<collision>` | contact queries (MuJoCo, PyBullet) | no |

`FL_calf` inertial CoM is at `z=-0.115`; its collision cylinder is centred at `z=-0.06`. They
disagree, and that's legal.

**That independence is what makes the ellipsoid constraint meaningful** — geometry comes from CAD,
inertials from a separate calculation. If the ellipsoid were derived from `phi`, constraining `phi`
with it would be circular.

---

## 4. Bounding ellipsoids (LMI only)

**Purpose:** produce `semi_axes`/`center` per link for two LMI constraints — `tr(J Q) ≥ 0` (density
realizability, eq. 13) and CoM-containment. Without them you still get `J ≻ 0`, but the body could be
implausibly large. **NLS/log-Cholesky needs none of this** (`mesh_dir=None`); `J ≻ 0` holds by
construction.

**Why it works for Spot and not Go2.** Spot: 20 links → 13 bodies, but *twelve of thirteen have
nothing fused*. Its only fusions are massless, geometry-free frames (`*_foot`, rails) plus a
`base_link → body` rename. Fused-child masses across all bodies: `[0.0, 16.52]`. So "one shape per
link" **is** the union. Go2 models the drivetrain explicitly — all 13 bodies have massive fused
children with geometry at nonzero offsets.

Three defects for Go2, none fixable by fetching meshes:

1. **One shape ≠ the fused body.** Worst on the trunk: base collision box half-extents
   `(0.1881, 0.04675, 0.057)`, but fused hip rotors sit at `x=±0.1934` — **5 mm outside**. Only 4.9%
   of trunk mass, but at the extremes where it drives inertia. `tr(J Q) ≥ 0` becomes **too tight**,
   can exclude the true parameters, and solves cleanly → silent failure.
2. **Rotated primitives dropped — and 8 of them are 90°, not small tilts.** `_compute_bounding_
   ellipsoids` reads only `.origin.xyz` and discards `rpy`. Audit of all 42 links found **12
   collision shapes with nonzero `rpy`**:

   ```
   *_hip     cylinder  rpy = 1.5708 0 0      (90 deg about x)  len 0.04,  r 0.046
   *_thigh   box       rpy = 0 1.5708 0      (90 deg about y)  0.213 x 0.0245 x 0.034
   *_calf    cylinder  rpy = 0 -0.20..-0.21 0                  len 0.12,  r 0.012..0.013
   ```

   This is **not** an under-approximation you can fix by inflating — it is the wrong shape
   *orientation*:
   - `*_thigh`: code gives `semi_axes = (0.1065, 0.01225, 0.017)`; truth is
     `(0.017, 0.01225, 0.1065)`. **The 21 cm long axis lands on x instead of z** — it constrains
     thigh mass into a volume perpendicular to the actual link.
   - `*_hip`: code gives `(0.046, 0.046, 0.02)`; truth is `(0.046, 0.02, 0.046)`.

   Not a one-liner to fix: `Q = inv(diag(semi_axes)**2)` is axis-aligned with no orientation term
   anywhere in the formulation. Bound the rotated shape by an enlarged axis-aligned one instead.
   Spot dodges this entirely — `mesh.bounding_box` is axis-aligned by construction.
3. **visual→collision is a trade, not a free swap.** Code reads `link.visuals` (principled: a visual
   mesh is a true *upper bound*). Go2's `.dae` files **don't exist on this machine**; its collisions
   are all primitives the existing Box/Cylinder/Sphere branches handle. But collision primitives are
   *shrunk simplifications*, not bounds — Go2's `base` box excludes the shoulders, `FL_calf` is a
   12 mm rod. Too-small ellipsoid = wrong constraint.

**Collision-geometry audit, all 42 links** (`go2.urdf`, source of the numbers above):

```
box 5    cylinder 17    sphere 5    mesh 0        <- zero collision meshes: "entirely primitives" holds
15 links have no <collision> at all: 12 *_rotor + imu, radar, front_camera
```

Note the 12 rotors have **no collision block**, so even a correct union builder gets nothing from
them; their offset mass (0.089 kg each at `x = ±0.1934` on the trunk) must come from the joint
origins in the kinematic tree, not from geometry.

**Correct fix:** ellipsoid of the **union of fused collision shapes**, each transformed into the body
frame *including its `rpy`*, then bounded axis-aligned. ~30 lines reusing the transitive fixed-joint
walk. No meshes.

> **Principle: err large.** A too-big ellipsoid is a loose but *valid* constraint. A too-small or
> mis-oriented one is a *wrong* constraint that can exclude the true parameters while still solving
> cleanly — silent failure.

**Order: LMI, not NLS** — see §7. NLS was originally recommended first because it needs no geometry;
the Spot run falsified that.

---

## 5. `go2_config.yaml`

`link_names` is matched against **URDF link names**, not Pinocchio body names (Spot's config says
`base_link` while Pinocchio's body is `body` — the author picked the geometry-carrying one).

> **Rule:** for each Pinocchio body, in Pinocchio's order, name the URDF link carrying its geometry.

`num_links` and `phi_nom` **bypass the config** (`rmodel.njoints-1`, `inertias[jid]
.toDynamicParameters()`), so the 7.279 lumping is automatic. `_compute_phi_nom_urdf()` would get it
wrong but is never called.

**Never list fused links** (`Head_upper`, `*_rotor`, `*_foot`). Ellipsoids are appended in **URDF**
order but read by `lmi_solver` in **Pinocchio** order, unchecked. Adding `Head_upper` shifts every
subsequent link onto the *previous* link's geometry — feasible, silent, wrong.
Verified `ORDER MATCH: True` for Go2's 13 — coincidence, so assert it.

Config: `name: go2`, `mass: 16.087`, feet `FL/FR/RL/RR_foot`, `link_names` = `base` + per-leg
`hip, thigh, calf` in FL, FR, RL, RR order. **Unverified:** foot *frame* names (`getFrameId` returns
garbage rather than raising if absent).

---

## 6. The authors' dataset — no excitation design

**No trajectory-generation code exists in the repo** (verified by full listing + keyword grep).
Shipped output only: `q(19), dq(18), ddq(18), tau(12), contact(4)` × 10500 @ 100 Hz, **samples are
columns**.

Paper: Solo12 used BiConMP [30] — planned, but as *locomotion*, no observability objective. Spot is
described only as *"various trajectories"*, no method stated. Related work cites optimal-excitation
design [16] then doesn't use it.

Measured on the Spot log (105 s): heave 0-15 s, rotation wobble 16-57 s, crawl 60-105 s.

- **`nc` is only ever 4 or 3** — no trot, no flight.
- Wobble **saturates** at Spot's pose limits; pitch parked within 5% of extreme for **30.5%** of
  samples. Spectrum is smooth 1/f, **no discrete harmonics**, 78-89% of power under 3 Hz.
- Channels move **one or two at a time** → near-blind to off-diagonals `I_xy, I_xz, I_yz`, which need
  *coupled* rotation.
- **The crawl excites `ω̇` ~3× harder than the wobble** (rms wx 14.1 vs 4.15), via impacts not design.
  For Go2 `Izz ≈ 0.11`, wobble-level `ω̇z` gives ~0.2 N·m — same order as their reported RMSE, i.e.
  **inertia sits near the noise floor**.

*Provenance:* trapezoidal saturation proves commands went through Spot's high-level pose interface
(no torque control). Human vs script is **not determinable** (plateau CV 0.52-0.60).

**Implication:** their numbers are what the *estimator* achieves on undesigned motion. Beating that
with designed excitation is a clean, separable claim.

---

## 7. Baseline runs on Spot data — LMI vs NLS

Both solvers run on the authors' own data, 2026-08-17. **Read every inertia number below with the
label swap of §8 applied** (printed `I_xz` is really `Iyy`, printed `I_yy` is really `Ixz`).

**Convergence.** LMI: MOSEK `PRIMAL_AND_DUAL_FEASIBLE`, `OPTIMAL`, 20 interior-point iterations,
22.7 s solve — but **59 s of cvxpy compilation** (`ConeMatrixStuffing`, 189k scalar variables from
the stacked 10500-sample `Y`). Expect the same shape on Go2. NLS: hit `max_iters=500` with cost still
falling ~0.1%/iter, line search backtracking to `step=4.88e-04` while `|dtheta|` reached `1.5e+03`.
**Not converged** — the NLS numbers are just wherever iteration 500 landed.

**The decisive test: Spot's four legs are identical hardware**, so any correct identification must
return near-identical parameters. Needs no CAD ground truth.

| link | | LMI | NLS |
|---|---|---|---|
| hip | masses | 1.426, 1.113, 1.075, 1.065 | 4.272, 5.893, 2.051, **9.264** |
| | CV | **12.7%** | 49.0% |
| upper leg | masses | 1.316, 1.353, 1.272, 1.458 | 0.303, **0.081**, 0.796, **0.089** |
| | CV | **5.1%** | 91.5% |
| lower leg | masses | 0.851, 0.807, 0.759, 0.771 | 0.648, 0.834, 0.582, 0.640 |
| | CV | **4.5%** | 14.0% |

CoM check, prior lower-leg `c_z = -0.180 m`:

```
LMI:  -0.161, -0.156, -0.209, -0.212      near the prior
NLS:  +0.015, -0.006, +0.007, +0.012      CoM at the knee -- impossible for a shin
```

Total mass: LMI `33.999999999817895` (the `mass_sum == 34.0` equality holds to 1e-10);
NLS `31.615` — **`nls_solver` has no such constraint**.

Both guarantee `J > 0`, so both are "physically consistent". Only LMI is physically *plausible*.
That gap is exactly what the bounding-ellipsoid and CoM-containment constraints buy.

**Counterintuitive but central: NLS fits better.**

```
                per-joint RMSE (Nm)              total*   rooted
nominal   [0.840, 1.890, 2.177, ...]             43.75     6.61
NLS       [0.697, 1.256, 1.939, ...]  best fit   24.24     4.92
LMI       [0.822, 1.382, 2.177, ...]             32.55     5.71
```

\* `total` has no square root — see §8, `rigid_body_dynamics.py:709`.

NLS wins on residual by ~25% while producing garbage parameters. **Unconstrained fit quality is not
evidence of correct identification.** Constraints cost residual and buy meaningful parameters.

**What LMI still gets wrong — and it is the link we care about:**

```
base_link   mass  16.52 -> 20.73   (+25.5%)
            Ixx   0.1315 -> 0.0650  (-50.6%)
            Iyy   0.1321 -> 0.0155  (-88.2%)
            Izz   0.1321 -> 0.0636  (-51.9%)
```

The legs shed ~4.2 kg and the trunk absorbed almost exactly that: the total-mass equality enforces
the *sum* but cannot say *where* mass belongs. This is not solver failure — it is the observability
limit computed in §6 (`nc` never below 3, quasi-static wobble, stance `wdot` rms only 1.5-4 rad/s²).

> **Why this matters for the Go2 project:** the trunk is the worst-identified body, using the paper's
> own method on the paper's own data. That is measured confirmation of the premise behind the
> excitation-design work — not an assumption that needs arguing.

*Caveat on the priors:* Spot's URDF inertias are placeholders —
`ixx = iyy = izz = 0.13144`, all off-diagonals exactly `0.0`, an isotropic dummy for a 16.5 kg body.
So large deviations from prior are **expected for Spot** and are not a bug. Go2 differs: its URDF
carries real per-link inertias, so the same deviation there would mean something.

---

## 8. Defects in the authors' repo

| location | issue |
|---|---|
| **`rigid_body_dynamics.py:743-744`** | **Iyy/Ixz labels swapped in every printed table.** `toDynamicParameters()` returns Pinocchio order `[m, hx, hy, hz, Ixx, Ixy, **Iyy, Ixz**, Iyz, Izz]`, but the print calls label slot 2 as `I_xz` and slot 3 as `I_yy` — the paper's eq. (3) order. Makes nominal values look impossible (a negative `I_yy`, an `I_xz` larger than every diagonal). Exactly the ordering trap the plan warned about. Fix below |
| `spot.urdf` | refs `base/visual/body.obj`; shipped file is `base_link.obj`. 1 of 18 broken. Fix: symlink. **Blocks visualization only** — `buildGeomFromUrdf`; identification uses `buildModelFromUrdf` and builds its own mesh paths, which resolve |
| `lmi_solver.py:61` | `dtype=np.float32` in a float64 pipeline |
| `:107,111` | `eigvals` on a symmetric matrix; `eigvalsh` is correct (nls_solver does it right) |
| `:173` | authors' TODO: `entropic` regularizer *"doesn't converge!"* |
| `:76-79` | `cp.reshape` without `order=`; cvxpy 1.9.2 is migrating this to required |
| `:203` | hard-codes `cp.MOSEK`; CLARABEL works license-free |
| `rigid_body_dynamics.py:709` | `rmse_total` has **no square root** — a mean-square despite the name; also normalized by sample count, not `Σr_k`, so not comparable across contact states. Line 710 (per-joint) is correct |
| `:684` | otherwise correct and reusable — projects *then* slices `[base_dof:]`, matching Table II |
| `run_identification.py` | loads `float32`; uses full `P` (18×18) not orthonormal `N`, so 18 rows/sample of which only `r` are independent |

Fix for the label swap (highest priority — it corrupts every result you read):

```diff
-            self._print_table("I_xz (kg.m^2)", inertia_prior[2], inertia_ident[2])
-            self._print_table("I_yy (kg.m^2)", inertia_prior[3], inertia_ident[3])
+            self._print_table("I_yy (kg.m^2)", inertia_prior[2], inertia_ident[2])
+            self._print_table("I_xz (kg.m^2)", inertia_prior[3], inertia_ident[3])
```

Also ignore the giant `error %` values (`666250.2%`): priors of ~1e-6 against a 1e-8 masking
threshold. Noise amplification, and the source of the `np.divide ... where=` uninitialized-memory
warning at `:757`.

---

## 9. Gates

| gate | result |
|---|---|
| **G0.1** trunk fixture | 7.2790 / (0.0201531, 0, -0.0051090) / Izz 0.112809 — fixture corrected |
| **G0.3** regressor identity | worst `‖Yφ − rnea‖` over 200 states = **1.275e-13 PASS** |
| model sanity | mass 16.0870, nq=19, nv=18, 13 bodies ✓ |
| MOSEK | 4×4 SDP optimal, agrees with CLARABEL |

> **Trap:** `pin.randomConfiguration` on a free-flyer draws `±inf` position limits → NaN. Build `q`
> explicitly: random translation, **normalized** quaternion in `q[3:7]` (Pinocchio order `x,y,z,w`),
> joints from `model.lower/upperPositionLimit[7:]`.

---

## 10. Open

- Foot **frame** names unverified
- Union-of-fused-shapes ellipsoid builder — not written (dev plan item 1)
- Go2 `.dae` meshes absent — **not needed**; collisions are primitives (§4)
- **Gradient check:** `dφ/dθ` is exact CasADi AD, needs no verification. The hand-assembled chain
  rule + GN Hessian at `nls_solver.py:296-356` are the target — since `G,K,H,d,M_link` are constant
  numpy, rebuild the whole cost symbolically in CasADi and compare `ca.gradient` (exact, beats finite
  differences). The **GN Hessian is deliberately approximate** — do *not* test it against `∇²cost`.
