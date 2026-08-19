# Go2 PCPIRC Port — Session Handoff

Reference document for continuing work on porting Khorshidi/Khadiv's physically-consistent
inertial-parameter identification pipeline from Spot to the Unitree Go2.

Everything below was **verified against code, URDF XML, or the paper PDF** during the session.
Numbers marked *(hypothesis)* were not verified and are flagged as such.

---

## 0. Working constraints (read first)

From `CLAUDE.md` in this repo — these override default agent behaviour:

- The user writes or copy-pastes **all** code. Do not write, edit, or run `.py`/`.cpp`/`.h`/`.m` files.
- Markdown files are the only thing an assistant may write directly.
- Code changes are delivered in the chat as ```diff blocks showing **only changed lines**
  (`-` deletions, `+` additions). No unchanged context lines.
- Workflow: Opus/Sonnet = architect + reviewer, writes the spec. **Sonnet implements.**
- Do not run terminal commands that execute or modify code without explicit permission.
  (Reading files and grepping is fine.)

---

## 1. Project

**Paper:** Khorshidi, Dawood, Nederkorn, Bennewitz, Khadiv, *Physically-Consistent Parameter
Identification of Robots in Contact*, ICRA 2025 (arXiv 2409.09850).
PDF: `Literature Review/khadiv.pdf`.

**Goal:** identify Go2 inertial parameters — the **trunk** in particular — using contact-nullspace
projection, then design excitation trajectories (the actual research contribution, absent from the
paper).

**Repo:** `/home/triakshunn/ROAHM_Lab/go2_system_identification_PCPIRC/`
**Env:** conda `go2sysid` — pinocchio 3.9.0, cvxpy, MOSEK, mujoco.
**MOSEK licence:** `~/mosek/mosek.lic`, mode 600. Deliberately outside the repo — `.gitignore`
does not cover `*.lic`.

### Method

Floating-base dynamics, `nq=19`, `nv=18`, 13 movable bodies, `φ ∈ R¹³⁰`:

```
M v̇ + n = Sᵀτ + Jcᵀλ            S = [0₁₂ₓ₆  1₁₂]
P = 1₁₈ − Jc†Jc                  ⇒  P Jcᵀ ≡ 0   (kills unknown contact forces)
res = P( Yφ − Sᵀ(τ − B_v v − B_c sign(v)) )
```

For stacking, prefer the orthonormal basis form `P = NNᵀ`, `N ∈ R^{18×r}`, `r = 18 − rank(Jc)` —
only `r` rows are independent; using the full `P` gives `18−r` redundant, ill-conditioned rows.

---

## 2. Go2 model as Pinocchio builds it

`go2.urdf` has 42 links; Pinocchio's fixed-joint lumping is **transitive**, leaving **13 bodies**.

**Trunk after lumping** (measured, pinocchio 3.9.0, `JointModelFreeFlyer`):

```
mass 7.2790 kg    com (0.0201531, 0, -0.0051090)    Izz 0.112809 about CoM
```

Not the URDF `base` link's 6.921 kg. Composition:

```
6.921 (base) + 4 x 0.089 (*_hip_rotor) + 0.001 (Head_upper) + 0.001 (Head_lower) = 7.279
```

`Head_lower` is a child of `Head_upper`, fused **transitively** — enumerate the transitive closure
of fixed joints or the fixture comes out grams light. `imu`, `radar`, `front_camera` are massless.

Downstream lumping: `hip += thigh_rotor (0.767)`, `thigh += calf_rotor (1.241)`,
`calf += foot (0.194)`. Total **16.087 kg**, matches Pinocchio exactly.

**Rotor inertia vs reflected armature — different things.** URDF `FL_hip_rotor ixx = 1.11842e-4`
is the *unreflected* rotor body and is already inside `Yφ`. The reflected armature `I_r·N²` with
N=6.22 is ≈4.33e-3, absent from the URDF, and **not in the paper's model** (eq. 14 has friction
only). They differ by exactly `N² = 38.72`.

### Parameter ordering — a live source of bugs

```
Pinocchio: [m, hx, hy, hz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]     ← Iyy at 6, Ixz at 7
Paper (3): [ ...            ixx, ixy, ixz, iyy, iyz, izz]     ← swapped
```

Transcribing the paper's order swaps `Iyy ↔ Ixz`. For a trunk that's a factor of ~47, and it will
**not** show as a residual blow-up in a symmetric test.

---

## 3. Verified defects in the authors' code

### (a) Label swap in the results table — **highest priority, silently corrupts every reported number**

`src/dynamics/rigid_body_dynamics.py:743-744`

```diff
-            self._print_table("I_xz (kg.m^2)", inertia_prior[2], inertia_ident[2])
-            self._print_table("I_yy (kg.m^2)", inertia_prior[3], inertia_ident[3])
+            self._print_table("I_yy (kg.m^2)", inertia_prior[2], inertia_ident[2])
+            self._print_table("I_xz (kg.m^2)", inertia_prior[3], inertia_ident[3])
```

Every printed table so far has the `I_xz` and `I_yy` rows transposed. Rows labelled `I_xz` are
really `I_yy` and vice versa.

### (b) `rmse_total` is not an RMSE

`rigid_body_dynamics.py:709`

```python
rmse_total = np.mean(np.square(np.linalg.norm(error, axis=1)))   # no sqrt
```

The paper's "Overall RMSE" is `sqrt(Σ_j r_j²)` over the 12 per-joint RMSEs — **verified
numerically** against Table II (see §6). So printed totals must be square-rooted before comparing
to the paper.

### (c) Bounding-ellipsoid builder is wrong for Go2

`rigid_body_dynamics.py:185-220`, `_compute_bounding_ellipsoids`. Three independent problems:

1. Iterates `for visual in link.visuals:` — should be `link.collisions` for Go2 (see §4).
2. Reads only `.origin.xyz` and **discards `.origin.rpy`**.
3. Appends in **URDF order** while `lmi_solver` indexes in **Pinocchio order**.
4. One shape per link — no union over fused children.

### (d) Dead code that would be wrong if called

`_compute_phi_nom_urdf()` uses `link_names` and would mis-handle lumping. `__init__` never calls
it; `_compute_phi_nom_pin` (lines ~130-134) is used instead and bypasses the config entirely, so
the 7.279 lumping is automatic and correct.

### (e) Numpy warning

`rigid_body_dynamics.py:757` — `np.divide(..., where=prior!=0)` without `out=`, gives uninitialised
memory in the `error %` column wherever the prior is exactly zero. Cosmetic but the resulting
percentages (e.g. `453592.2`) are meaningless anyway.

---

## 4. URDF geometry — why Spot works and Go2 doesn't

Three URDF blocks are **mutually independent**; nothing validates them against each other:

| block | affects |
|---|---|
| `<inertial>` | = φ. The only block affecting dynamics. |
| `<visual>` | rendering only |
| `<collision>` | contact queries only |

This independence is what makes the bounding-ellipsoid constraint **non-circular** — the geometry
is genuinely separate information from the inertia being identified.

**Why one shape per link is enough for Spot:** Spot's fused children are all massless, geometry-free
frames (fused-child masses `[0.0, 16.52]`). One shape *is* the union.
**Why it fails for Go2:** Go2 models the drivetrain explicitly — 12 rotor links carrying real mass.

### Go2 collision audit (from raw XML)

```
box 5    cylinder 17    sphere 5    mesh 0        <- zero collision meshes
15 links have no <collision> at all: 12 x *_rotor + imu, radar, front_camera
```

**Consequence: the LMI route is NOT blocked by missing meshes.** Go2's collisions are entirely
primitives, handled by the Box/Cylinder/Sphere branches — no file is ever opened. (An earlier
belief that missing meshes blocked LMI was wrong; only the `Mesh` branch reads files, and Go2 never
reaches it.) Visual meshes reference `package://go2_description/dae/*.dae` and **no `dae/`
directory exists** — that blocks *visualisation* only.

### The rotation problem

12 collision shapes have nonzero `rpy`, **8 of them 90°**:

```xml
<link name="FL_hip">    <origin rpy="1.5707963267948966 0 0" xyz="0 0.08 0"/>
                        <cylinder length="0.04" radius="0.046"/>
<link name="FL_thigh">  <origin rpy="0 1.5707963267948966 0" xyz="0 0 -0.1065"/>
                        <box size="0.213 0.0245 0.034"/>
<link name="FL_calf">   <origin rpy="0 -0.21 0" xyz="0.008 0 -0.06"/>
                        <cylinder length="0.12" radius="0.012"/>
```

Because the code drops `rpy`:

| link | code gives | truth |
|---|---|---|
| `*_thigh` | `(0.1065, 0.01225, 0.017)` | `(0.017, 0.01225, 0.1065)` |
| `*_hip` | `(0.046, 0.046, 0.02)` | `(0.046, 0.02, 0.046)` |

The 21 cm long axis lands on **x** instead of **z**. This is a wrong *orientation*, not an
under-approximation — inflating the ellipsoid does not fix it.

Note the formulation has **no orientation term**: `Q = inv(diag(semi_axes)**2)`, axis-aligned only.
So a rotated shape must be handled by taking the axis-aligned bounding box of the rotated shape.

> **Principle: err large.** A too-big ellipsoid is a loose but *valid* constraint. A too-small or
> mis-oriented one is *wrong* and fails silently.

### Required fix (dev item 1)

Union of all collision shapes over the link **and its fused children**, each transformed by its own
`origin.xyz` + `origin.rpy` and by the fixed-joint transform into the parent frame, then take the
axis-aligned bounding box of that union, then the enclosing ellipsoid. Emit in **Pinocchio joint
order**.

---

## 5. The two solvers

`src/solver/lmi_solver.py` and `src/solver/nls_solver.py`.

| | LMI (`solve_fully_consistent`) | NLS (`solve_gn_exp`) |
|---|---|---|
| `J ⪰ 0` | ✓ explicit, `+1e-6·I` | ✓ by log-Cholesky construction |
| CoM ∈ ellipsoid | ✓ | ✗ |
| `tr(J·Q) ≥ 0` (density realizability, eq. 13) | ✓ | ✗ |
| `Σ m = total_mass` | ✓ **hard equality** | ✗ |
| `b_v ≥ 0`, `b_c ≥ 0` | ✓ | — |
| default `lambda_reg` | `1e-4` | `1e-7` |
| regulariser | constant-pullback Riemannian (default); `entropic` exists but **does not converge**; `euclidean` | — |
| backend | cvxpy → MOSEK / CLARABEL (SDP) | CasADi + Gauss-Newton |

`lmi_solver.py:188` — `self._constraints.append(mass_sum == self.total_mass)`.
`lmi_solver.py:198` — `Minimize( (1/2)*sum_squares(error)/N + lambda_reg * reg )`.

**NLS's feasible set strictly contains LMI's, with a 1000× weaker regulariser.** NLS therefore
*must* fit better or equal in-sample. That is arithmetic, not evidence of quality.

Friction **is** identified in both paths (`B_v`, `B_c` passed from `run_identification.py:114`).
Reflected armature is **not** modelled anywhere.

**There is no train/test split in the repo.** `run_identification.py:121-122` calls
`print_tau_prediction_rmse` on the *same* `(q, dq, ddq, tau, cnt)` used for fitting. All reported
RMSEs are training error.

---

## 6. The paper's validation metric

### What it is

An **open-loop prediction residual** — one forward pass over a recorded log. No controller in the
loop, no re-simulation.

```python
tau_pred = P @ (Y @ phi + S.T @ (diag(b_v)@dq + diag(b_c)@sign(dq)))
tau_meas = P @ S.T @ torque
error    = (tau_pred - tau_meas)[6:]        # project FIRST, slice after
```

- `q, dq, ddq, torque` all come from the **same log**. The FF+PD controller that produced the
  motion is upstream of the metric — it is how the data was collected, not part of the computation.
- `tau_meas` is the **total** applied/measured joint torque, never decomposed into FF and PD parts.
  This is why teleop data works.
- Uses `pin.computeJointTorqueRegressor` → `Yφ`, not `rnea`. Identical at φ = model params, but the
  regressor form lets you swap φ without rebuilding the model.
- Friction is added to the *prediction* side, not subtracted from the measurement.
- Project with `P` **first**, then take rows `6:`. Reversing is wrong.

### Aggregation — verified

`Overall RMSE = sqrt(Σ_j r_j²)` over the 12 per-joint RMSEs. Confirmed by reconstructing Table II:

```
Crawl LMI [1.015,1.856,3.089, 1.037,1.823,3.180, 1.452,2.257,3.506, 1.113,1.818,3.928]
  -> sqrt(sum of squares) = 8.246     (table: 8.246)
Walk  LMI [1.404,2.549,3.563, 1.316,2.829,3.284, 1.540,2.585,3.973, 1.402,2.450,4.247]
  -> sqrt(sum of squares) = 9.619     (table: 9.620)
```

So: **`sqrt(printed rmse_total)` = paper-comparable Overall RMSE.**

### Reference numbers

**Table I — Solo12, PyBullet, m = 2.5 kg:**

| Motion | LMI | SVD | MLP |
|---|---|---|---|
| Validation | 0.4019 | 0.9867 | **0.3707** |
| OOD Trot | 0.6470 | 1.7184 | 1.0742 |
| OOD Task (Jump) | **0.7148** | 3.4063 | 2.3986 |

**The MLP beats LMI in-distribution and is 3.4× worse out of it.** This is the paper's central
argument and the direct answer to "doesn't lower RMSE mean a better model."

**Table II — Spot, real hardware, m = 34 kg.** Joint order per leg: hip ab/ad, hip fl/ex, knee fl/ex.

| Motion | Model | Overall RMSE |
|---|---|---|
| Crawl (Validation) | LMI | 8.246 |
| Crawl (Validation) | MLP | 13.039 |
| Walk (New Task) | LMI | 9.620 |
| Walk (New Task) | MLP | 20.931 |

LMI degrades only +17% on an unseen task. The knee is the worst joint everywhere (3.0–4.2 N·m),
hip ab/ad the best (~1.0–1.5).

---

## 7. Baseline runs on the authors' Spot data

Both solvers run on the shipped Spot dataset. **In-sample.**

| | printed `total` | = N·m (sqrt) |
|---|---|---|
| LMI | 32.55 | **5.71** |
| NLS | 24.24 | **4.92** |

Both are in the paper's ballpark (8.246 for crawl) — good evidence the port functions. NLS fits
better and its parameters are garbage. Specifics:

**Leg-symmetry test (four physically identical legs, estimated from independent columns — nothing
in the formulation couples them). This is a free repeatability test the paper never uses, and it
separates the solvers far more decisively than any RMSE:**

| link | LMI CV | NLS CV |
|---|---|---|
| calf | 4.5% | |
| thigh | 5.1% | 14 – 91.5% |
| hip | 12.7% | |

**Lower-leg CoM `c_z`** (prior −0.180): LMI −0.156 … −0.212. NLS +0.015 … −0.006 — i.e. NLS places
the calf's centre of mass **at the knee**. Physically impossible, and invisible to the RMSE because
crawl data barely swings the knee.

**Mass redistribution under LMI** — the total is a *hard constraint*, so it cannot fail; only the
distribution is informative:

| | prior | identified | Δ |
|---|---|---|---|
| all four legs | 17.48 | 13.27 | **−4.21** |
| trunk | 16.52 | 20.73 | **+4.21** |

One transfer of 4.21 kg from legs to trunk, to three decimals — not 13 independent errors. A second
consistent mode superimposes on it: within every leg, hip+thigh → calf (−6.00 / +1.79).

**Why that direction:** `nc` in this dataset is only ever 4 or 3 — never a trot, never flight. At
least three feet are pinned at all times, and a pinned leg transmits its weight through the contact
force, which is exactly what `P` deletes. At `nc=4` you keep `r=6` of 18 rows. Leg mass is barely
excited; the trunk is the one body always free to accelerate. `lambda_reg=1e-4` is far too weak to
hold the split at the prior.

**Trunk is the worst-identified body in both runs** (LMI: mass +25.5%, Ixx −50.6%, Iyy −88.2%,
Izz −51.9%) — measured confirmation of the premise behind the excitation-design contribution.

### Caveat: Spot's URDF priors are partly fiction

Spot's trunk inertia is `Ixx/Iyy/Izz = 0.1315/0.1321/0.1321` — isotropic. For a 16.5 kg body ~1.1 m
long, `Iyy` should be ≈1.7 kg·m², so the prior is ~13× too small; Boston Dynamics clearly scrubbed
it. Deviations against the trunk row therefore mean much less than they appear.

The calf prior, by contrast, is internally consistent: `m·L²/3 = 0.35·0.1296/3 = 0.01512` against a
URDF `0.01535` (1.5%) — a uniform-rod idealisation, defensible.

**Go2 does not share this problem** — it has real per-link inertias. Do not carry Spot's deviation
magnitudes over as expectations for Go2.

### Anomaly worth checking

`rear_left_hip` `c_y` identified **+0.013571** against a prior of **−0.012842** — a sign flip. FL
(+0.0115 vs +0.0128), FR (−0.0178 vs −0.0128) and RR (−0.0279 vs −0.0128) all follow their priors.
Note the Spot URDF gives RL the same `c_y` and `I_xy` signs as the *right* legs, which is itself odd.
Unresolved: either a URDF sign-convention issue or an unreliable RL estimate.

---

## 8. How to judge whether identified parameters are correct

The key result, from the runs above: **torque RMSE and parameter correctness are only loosely
coupled.** NLS scored better while placing a CoM at a joint. The residual is `Nᵀ Y Δφ`; any `Δφ` in
the small-singular-value subspace of `W = NᵀY` moves the RMSE by ~nothing.

The projected RMSE is a **good metric used wrongly**:

| use | valid? |
|---|---|
| in-sample, comparing models of different capacity | **no** — always favours the looser model |
| out-of-sample on held-out **motion types** | **yes** — this is Table I/II |
| as a certificate that parameters are right | only for the observable subspace |

Hold out entire motions, never random samples — at 100 Hz, neighbouring samples are nearly
identical and random holdout is almost free to fit.

**Checks that carry information (things the solver was never told):**

1. **Cross-leg consistency (CV).** Four identical legs, independent columns. Strongest cheap test.
2. **Left/right sign antisymmetry** in `c_y`, `I_xy`, `I_yz`.
3. **Radius of gyration** `sqrt(I/m)` vs link length.
4. **Held-out motion type** — the paper's "New Task" column.

**Checks that carry NO information:** anything the LMI enforces as a constraint — `J ⪰ 0`, CoM in
ellipsoid, `tr(JQ) ≥ 0`, `Σm = total`. Satisfying these is not evidence. Neither is closeness to the
prior when the prior is fiction.

---

## 9. Data format

`load_data()` in `demo/run_identification.py` reads five tab-delimited `.dat` files:

```
<robot>_robot_q.dat        19 rows  (3 pos + 4 quat + 12 joint)
<robot>_robot_dq.dat       18 rows
<robot>_robot_ddq.dat      18 rows   <- LOGGED, not differentiated by this repo
<robot>_robot_tau.dat      12 rows
<robot>_robot_contact.dat   4 rows   (binary per foot, feeds P)
```

- **`q̈` is read from file.** Free in MuJoCo (`qacc`); a real gap on hardware.
- **`q` must include full base pose** — state estimation, not encoders.
- The base block of `dq`/`ddq` is in Pinocchio's **local body frame**. Getting this convention
  wrong multiplies straight into trunk inertia.
- Filtering (`butterworth` default, 5th order, `filtfilt`; or `savitzky`) is applied to
  **`dq, ddq, tau` only — `q` is left raw.** Defensible, but it means `q` and `dq` are no longer
  exactly mutually consistent.

---

## 10. Excitation trajectories — there are none to port

Exhaustively searched the authors' repo: **no trajectory-generation code ships with it.** grep hits
for "trajectory" are `motion_subspace` false positives.

- **Solo12** motions were planned with BiConMP (reference [30], Meduri et al., T-RO 2023) — a
  kino-dynamic locomotion MPC with **no parameter-observability objective**.
- **Spot** motions are described in the paper only as "various trajectories," with no method stated.
  Whether they were teleoperated or scripted is **not determinable** from the data (plateau CV
  0.52–0.60).

Either way: **no excitation-designed trajectories exist anywhere in the work** — the paper's own
related-work section concedes this. It is the gap the Go2 project is meant to fill.

### Characterisation of the shipped Spot dataset

- `nc` only ever 4 or 3 — no trot, no flight phase.
- The "wobble" is trapezoidal and saturates: pitch is within 5% of its extreme for 30.5% of samples.
- Spectrum is a smooth 1/f with **no discrete harmonics** — not a multisine.
- Channels move one or two at a time, so the data is largely **blind to off-diagonal inertia**
  (`I_xy`, `I_xz`, `I_yz`), which only appears under *coupled* rotation.
- The crawl excites `ω̇` roughly 3× harder than the "wobble" does, via foot impacts.

---

## 11. Dev plan

| # | item | status | notes |
|---|---|---|---|
| 1 | Update the LMI solver for Go2 | todo | Blocked on the ellipsoid builder (§4). Needs `visuals`→`collisions`, union over fused children, `rpy` handling, Pinocchio ordering. `go2_config.yaml` still to be written. |
| 2 | Design exciting trajectories for Go2 | todo | Nothing to port (§10). Multisine, all 6 trunk channels simultaneously, coprime harmonics. **The actual research contribution.** |
| 3 | Run and validate in simulation | todo | MuJoCo as an independent plant. Inject a known Δφ and check the metric recovers it. |

Items 1 and 3 are independent — validating the *metric* in sim does not require a working LMI.

### `go2_config.yaml` — DOES NOT EXIST YET

Proposed. `link_names` must list only the **13 Pinocchio bodies**, not all 42 URDF links — fused
children are already inside their parents' φ.

```yaml
robot:
  name: "go2"
  mass: 16.087 # Kg
  end_effectors_frame_names: ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]   # UNVERIFIED
  link_names: [base, FL_hip, FL_thigh, FL_calf, FR_hip, FR_thigh, FR_calf,
               RL_hip, RL_thigh, RL_calf, RR_hip, RR_thigh, RR_calf]
```

### Gates already passing

**G0.3** — `‖Y(q,v,a)·φ − pin.rnea(model,data,q,v,a)‖ = 1.275e-13`. The most important check in the
pipeline; if it fails everything downstream is meaningless.

> Note on testing G0.3: `pin.randomConfiguration` on a free-flyer draws ±inf position limits and
> returns NaN. Build `q` explicitly — random translation, normalised quaternion in `q[3:7]` with
> order `(x,y,z,w)`, joints from `model.lower/upperPositionLimit[7:]`.

---

## 12. Open questions

1. **Foot *frame* names unverified** for Go2 (`FL_foot` etc. are a guess; the URDF link is `FL_foot`
   but the Pinocchio **frame** id must be confirmed).
2. `urdf-parser-py` not installed in the `go2sysid` env — needed by the ellipsoid builder.
3. **Observability spectrum not yet computed.** The decisive diagnostic: SVD of the stacked
   `W = NᵀY`, projected onto the 13 mass columns. If the legs→trunk direction sits at the bottom of
   the spectrum, the 4.21 kg transfer is confirmed *unobservable* rather than *measured*, and dev
   item 2 gets a quantitative target. **Not yet run.**
4. **Held-out validation not yet run.** Refit with crawl held out, or walk held out, and re-compare
   LMI vs NLS out-of-sample. Prediction: NLS loses. Untested — the data is already on disk.
5. Gradient verification of `nls_solver.py:296-356` via symbolic CasADi rebuild. Note the GN Hessian
   is *deliberately* approximate and must **not** be tested against `∇²cost`.
6. *(hypothesis, unverified)* Missing reflected armature may explain the calf inertia inflation:
   Spot's knee `Iyy` rose +0.0075 kg·m², squarely in the plausible armature range, and physical
   consistency would then drag mass up with it while the mass equality pushes the deficit onto the
   thighs. Fits the observed sign structure but is not established.

---

## 13. Corrections to earlier beliefs (do not re-derive these)

Recorded so a fresh reader does not repeat them:

- **"LMI is blocked for Go2 by missing meshes"** — **false.** Go2's collisions are 100% primitives;
  no file is ever opened. Only the `Mesh` branch reads files.
- **"Use NLS first because it needs no geometry"** — **falsified** by the Spot runs. NLS has no
  total-mass and no geometry constraint and produces physically impossible parameters. **Order: LMI
  first.**
- **"The rpy problem is a ~12° tilt"** — **understated.** 12 rotated shapes, 8 at 90°.
- **"Trunk lumped mass is 7.277 kg"** — **wrong**, 7.2790. The hand calculation missed the two head
  links fused transitively.
- **"The authors teleoperated the Spot motions"** — **over-claimed.** Not determinable. What *is*
  supported: no excitation-designed trajectories exist anywhere in the work.
