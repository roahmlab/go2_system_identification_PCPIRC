# Go2 Trunk System Identification — Implementation Plan

**Scope.** Identify the 10 inertial parameters of the Unitree Go2 trunk (base link) — mass, first moment `h = mc`, and the 6 independent entries of the rotational inertia — using the contact-nullspace projection of Khorshidi et al. (arXiv 2409.09850), cross-checked against direct metrology and the Ayusawa flight-phase route.

**Premise.** Leg parameters are already identified and treated as known. Only proprioception is available: encoders, IMU, current-derived joint torques, and (if EDU) foot force sensors. No force plates, no motion capture assumed.

**How to use this document.** Sections 1–3 are theory you should be able to re-derive before writing code. Sections 4–10 are the build order. Section 11 is the extension that would constitute a contribution. Code blocks are reference implementations — verify API signatures against your Pinocchio version rather than pasting blindly.

---

## 1. Model, notation, and dimensions

### 1.1 The floating-base equation

```
M(q) v̇ + n(q, v)  =  Sᵀ τ  +  Jcᵀ λ  −  Sᵀ f_fric  −  Sᵀ f_rotor
```

| Symbol | Meaning | Go2 dimension |
|---|---|---|
| `q` | configuration, `SE(3) × R¹²` | `nq = 19` (3 pos + 4 quat + 12 joint) |
| `v` | generalized velocity | `nv = m = 18` |
| `M(q)` | mass matrix | 18×18 |
| `n(q,v)` | Coriolis + centrifugal + gravity | 18 |
| `S = [0₁₂ₓ₆  I₁₂]` | actuation selection | 12×18 |
| `τ` | joint torques | 12 |
| `Jc` | contact Jacobian | `3nc × 18` |
| `λ` | contact forces | `3nc` |

**Why 3 rows per foot.** A Go2 foot is a rubber ball: it transmits force in 3 directions but essentially no moment, so `λᵢ ∈ R³`. Equivalently, the constraint "this foot does not move" is 3 scalar equations (`v_x = v_y = v_z = 0` at the contact point). A flat foot with a rigid sole would contribute 6 rows — 3 force + 3 moment — constraining rotation as well. This is why the code slices `[:3, :]` from the 6-row frame Jacobian: the bottom 3 rows are the angular part, which a point contact does not constrain.

**Why 18, and what it is not.** `nv = 18` is the number of degrees of freedom: 6 for the floating base + 12 joints. The Go2 has **12** joints, not 18. The equation of motion yields exactly **one scalar equation per DoF**, so `M v̇ + n = …` is an 18×1 vector — nothing is stacked or reshaped beyond that.

A common confusion: the base *configuration* is an element of SE(3), often written as a 4×4 homogeneous matrix, which tempts one to expect factors of 4 somewhere. But SE(3) is a 6-dimensional Lie group; its velocity (an element of `se(3)`, i.e. a spatial twist) is a **6-vector**. The 4×4 form represents pose, never dynamics. Hence:

| Quantity | Dim | Composition |
|---|---|---|
| `q` | 19 | 3 position + 4 quaternion + 12 joint angles |
| `v` | 18 | 6 twist + 12 joint velocities |
| equations of motion | 18 | one per DoF |

`nq ≠ nv` because the quaternion spends 4 numbers on 3 rotational DoF. This is also why `q̇ = v` is invalid for a floating base — use `pin.integrate` / `pin.difference`.

### 1.2 Parameter vector

Per rigid body, 10 parameters. **Pinocchio's ordering** (from `Inertia.toDynamicParameters()`) is:

```
p = [ m,  h_x, h_y, h_z,  I_xx, I_xy, I_yy, I_xz, I_yz, I_zz ]
```

with `h = m·c` the first moment and `Ī` taken **about the joint origin**, expressed in the **link frame**. Note this is *not* the ordering used in the Khorshidi paper (which lists `Ixx, Ixy, Ixz, Iyy, Iyz, Izz`). Getting this wrong silently swaps `I_yy ↔ I_xz`. Write a unit test.

With a free-flyer root, `model.njoints = 14` (universe + freeflyer + 12), so:

```
φ ∈ R^130,   φ_trunk = φ[0:10]   (the free-flyer block comes first)
```

130 matches the count in the projective-geometric-algebra analysis of the Go2, which found a 36-dimensional regressor nullspace → 94 independently identifiable parameters.

### 1.3 Extra linear parameters

Three nuisance blocks, all entering **linearly** — this is what keeps the problem convex:

| Block | Model | Params | Why it matters |
|---|---|---|---|
| Reflected rotor inertia | `I_r,j · N_j² · v̇_j` | 12 | Gear ratio 6.33 → `N² ≈ 40`. Omit it and it leaks straight into link inertias. |
| Viscous friction | `b_v,j · v_j` | 12 | |
| Coulomb friction | `b_c,j · sign(v_j)` | 12 | `sign(v)` is *data*, not a decision variable |

Full unknown vector: `θ = [φ (130); a (12); b_v (12); b_c (12)] ∈ R^166`. With legs frozen you solve for `θ = [φ_trunk (10); a; b_v; b_c] ∈ R^46`.

---

## 2. The projection, derived

### 2.1 Why λ vanishes

`λ` enters only through `Jcᵀλ`, which is a linear combination of the **rows** of `Jc`. So for *any* λ:

```
Jcᵀ λ  ∈  row(Jc)
```

Let `P = I − Jc⁺Jc` be the orthogonal projector onto `null(Jc)`. Since `Jc⁺ = Jcᵀ(JcJcᵀ)⁻¹` for full-row-rank `Jc`:

```
P Jcᵀ = Jcᵀ − Jcᵀ(JcJcᵀ)⁻¹JcJcᵀ = Jcᵀ − Jcᵀ = 0
```

Premultiplying the dynamics by `P` annihilates the contact term **identically** — no estimate of λ, no assumption about its magnitude or direction. In regressor form:

```
P Y(q,v,v̇) φ  =  P Sᵀ τ
```

The complementary half, `(I−P)(Mv̇ + n) = (I−P)Sᵀτ + Jcᵀλ`, is exactly the set of equations that would determine λ. You discard it.

### 2.2 Physical reading

`null(Jc)` = generalized velocities producing zero foot motion. With four feet planted the robot is a 4-branch parallel mechanism whose only remaining freedom is trunk pose — 6 DoF. The projected dynamics *is* the equation of motion of the wobble, and contact forces don't appear because the feet don't move, so those forces do no work along these directions.

Note that `Sᵀ` has zeros in its first 6 rows (unactuated base), but `P Sᵀ` generally does **not**. The projection mixes joint torques into base-direction equations — that's the mechanism by which motor currents become informative about trunk inertia.

### 2.3 Use a basis, not the projector

`P Y φ = P Sᵀτ` still *has* 18 rows, but only `18 − 3nc` of them are independent. The reason is that `rank(P) = dim null(Jc) = 18 − rank(Jc)`, and since `rank(AB) ≤ min(rank A, rank B)`, the product `P Y` inherits that ceiling. With four feet down, 6 rows carry information and the other 12 are linear combinations of them — redundant and ill-conditioned to stack.

Define

```
r = dim null(Jc) = 18 − rank(Jc)        ( = 18 − 3nc generically )
```

Factor `P = N Nᵀ` with `N ∈ R^{18×r}` orthonormal — the right singular vectors of `Jc` associated with its zero singular values — and regress on:

```
Nᵀ Y φ = Nᵀ Sᵀ τ          (r rows, well-conditioned)
```

Same solution set, `r` rows instead of 18, far cheaper across 10⁴ samples.

### 2.4 Frame conventions — a correction worth internalizing

I earlier called frame choice the most common bug. That's overstated in one specific respect, and the precise statement is more useful:

**The choice of output frame for `Jc` does not change `P`.** Expressing foot velocity in `LOCAL` vs `LOCAL_WORLD_ALIGNED` rotates each foot's 3-row block by an invertible `R_i`; the stack is transformed by an invertible block-diagonal matrix, and `null(R·Jc) = null(Jc)`. So `P` is invariant. Pick either.

What *does* matter:
- **Column convention.** `Jc`'s columns and `Y`'s rows must both be functions of the same `v`. Pinocchio guarantees this since both come from the same `model`/`data` — but only if you don't hand-assemble either.
- **Pinocchio's free-flyer velocity is expressed in the local body frame.** Your IMU gives angular velocity in body frame (good) but linear acceleration also in body frame — you must be deliberate about converting to `v̇` in Pinocchio's convention, including the `ω × v` term.
- **Parameter ordering** (§1.2).
- **Gravity.** `model.gravity.linear = [0,0,-9.81]` in world; with a free-flyer the base orientation in `q` handles the rest. Verify by checking that a static, level, four-feet-planted sample has residual ≈ 0 under CAD parameters.

---

## 3. What is and isn't identifiable

### 3.1 Rank–nullity per contact state

`dim null(Jc) = 18 − rank(Jc)`. Assuming full row rank:

| Contact state | rank(Jc) | equations kept per sample |
|---|---|---|
| 4 feet | 12 | **6** |
| 3 feet | 9 | 9 |
| 2 feet (trot) | 6 | 12 |
| 1 foot | 3 | 15 |
| flight | 0 | 18 |

**Do not hard-code `3nc`.** Near a kinematic singularity (fully extended knee) `Jc` loses rank and the null space grows. Detect rank numerically from the SVD and discard samples where `s_min/s_max` is below threshold.

### 3.2 The tension you have to manage

Four-feet-planted wobbling gives the best trunk angular excitation but the *fewest* equations per sample. Trot and jump give more rows but noisier `v̇` and harder contact classification. Khorshidi et al. resolve this by pooling ~10⁴ samples across base wobbling, walking, and crawling. Plan on mixing motions.

### 3.3 Total mass is structurally weak here

Gravity supports the body through the contact directions you just projected out. `m` is therefore poorly conditioned in this formulation — you can be off by a kilogram and see little residual change during planted motion.

**Consequence:** measure `m` on a scale and impose it as an equality constraint. Do not ask the regression to find it. This is the same conclusion Ayusawa reaches from the other direction (total mass is the one parameter unidentifiable from free-flight motion).

### 3.4 Low residual ≠ correct parameters

The residual sees `φ` only through the stacked `[N_kᵀY_k]`. Anything in that matrix's null space, **or merely poorly excited by your trajectories**, has zero effect on the residual. SPI-Active's Go2 identification is the cautionary case: `Ixx` and `Iyy` grew from 0.025/0.098 to 0.391/0.515 kg·m², which the authors themselves attribute to overfitting from insufficient roll/pitch excitation. The fit was fine; the parameters weren't.

Always report per-parameter relative standard deviations alongside RMSE (§9.2).

---

## 4. Data collection protocol

### 4.1 Sensing checklist

| Signal | Source | Notes |
|---|---|---|
| `q_joint` | encoders, 22-bit | clean |
| `v_joint` | encoders | usually differentiated on-board; check |
| `τ` | `tau_est`, current-derived | **not** a torque sensor; see §10 |
| base orientation | onboard estimator / IMU | quaternion, check xyzw vs wxyz |
| base `ω` | IMU gyro | body frame |
| base linear accel | IMU | body frame, includes gravity — subtract carefully |
| foot contact | `foot_force[4]` (EDU only) | see §6 |

**Verify your foot sensors are real.** Foot-end force sensors are an EDU-only feature; the `foot_force` field exists in the `LowState` struct on all variants but may be zeros or a current-derived estimate on non-EDU units. Lift a leg by hand and watch whether the value actually responds. If it's software-derived from τ, you have circular reasoning — you'd be using torque to decide which torque equations to write.

### 4.2 Motion set

Log at **≥ 500 Hz** if the SDK allows, decimate later. Aim for ~10⁴ usable samples after filtering.

| Motion | Contact | Purpose |
|---|---|---|
| Static poses (~20 configs) | 4 feet | Gravity/CoM excitation; near-zero `v̇` so it isolates static terms |
| Base wobble — roll, pitch, yaw sweeps | 4 feet | **The key trunk-inertia motion.** Large `ω̇` with feet planted |
| Base wobble — combined 6-DoF Fourier | 4 feet | Off-diagonal inertia terms `Ixy, Ixz, Iyz` |
| Slow crawl | 3 feet | More equations/sample |
| Trot, varied `v_x, v_y` | 2 feet | More equations/sample; validation set |
| Jumps | flight | Ayusawa cross-check (§8) |
| Isolated single-joint sweeps, leg in air | 0 (that leg) | Friction + rotor inertia, identified separately |

**Hold out entire motion types for validation**, not random samples — random holdout is nearly free to fit and tells you almost nothing. Khorshidi et al. train on crawl + wobble and validate on walk, and separately test out-of-distribution on a jump.

### 4.3 Signal processing

- **Zero-phase filtering.** Forward-backward (`scipy.signal.filtfilt`) Butterworth. Khorshidi et al. use 5th-order at 10 Hz cutoff on 100 Hz data. Filter *before* differentiating.
- **Differentiation.** Central differences on the filtered signal, or better, fit a local polynomial (Savitzky-Golay) and take its derivative analytically. Never differentiate raw encoder data.
- **Base acceleration.** If you have only the IMU, `v̇` for the base is the noisiest quantity in the whole pipeline and it multiplies the trunk inertia directly. Consider deriving base acceleration from a kinematics-based estimate during stance (feet fixed → base motion from leg kinematics) and fusing with the IMU.
- **Quaternion handling.** Normalize, and watch for sign flips between consecutive samples before differentiating.

---

## 5. Pipeline architecture

```
go2_sysid/
├── model.py          # Pinocchio model, frame IDs, parameter layout, φ↔URDF
├── contacts.py       # contact detection → boolean mask per sample
├── signals.py        # filtering, differentiation, base state assembly
├── regressor.py      # Y_k, Jc_k, N_k, extra blocks → stacked (W, y)
├── identify.py       # convex programs: LMI / log-Cholesky / plain LS
├── validate.py       # residual metrics, diagnostics, covariance
├── excite.py         # (Phase 2) Ds-optimal trajectory design
└── tests/
    ├── test_ordering.py     # φ layout round-trip
    ├── test_projection.py   # P Jcᵀ = 0, residual ≈ 0 with ground truth
    └── test_sim.py          # end-to-end on simulated data
```

Build and test in this order. **Do not touch hardware data until `test_projection.py` passes.**

---

## 6. Stage A — model, layout, and the ground-truth test

### 6.1 Load with a free-flyer root

```python
import pinocchio as pin
import numpy as np

model = pin.buildModelFromUrdf("go2_description.urdf", pin.JointModelFreeFlyer())
data  = model.createData()

assert model.nq == 19 and model.nv == 18
n_bodies = model.njoints - 1          # exclude universe → 13
assert n_bodies * 10 == 130

FOOT_FRAMES = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
foot_ids = [model.getFrameId(f) for f in FOOT_FRAMES]
```

### 6.2 Parameter layout round-trip

```python
def phi_from_model(model):
    """Stack per-body dynamic parameters in Pinocchio's own ordering."""
    return np.concatenate([model.inertias[i].toDynamicParameters()
                           for i in range(1, model.njoints)])

def apply_phi(model, phi):
    for i in range(1, model.njoints):
        model.inertias[i] = pin.Inertia.FromDynamicParameters(phi[10*(i-1):10*i])

# Round-trip test — must be exact
phi0 = phi_from_model(model)
assert np.allclose(phi_from_model(model), phi0)
```

The trunk block is `phi0[0:10]`. Print it and compare against the URDF text (mass 6.921 kg, CoM `(0.021112, 0, −0.005366)`, `Ixx 0.02448 … Izz 0.107`) to confirm you're reading the right block and the right ordering.

### 6.3 The single most important test

Generate simulated data with known parameters. Then, for each sample:

```python
def regressor_and_projection(model, data, q, v, a, in_contact):
    pin.computeJointTorqueRegressor(model, data, q, v, a)
    Y = data.jointTorqueRegressor.copy()        # 18 × 130

    pin.computeJointJacobians(model, data, q)
    pin.updateFramePlacements(model, data)
    blocks = [pin.getFrameJacobian(model, data, fid,
                                   pin.LOCAL_WORLD_ALIGNED)[:3, :]
              for fid, c in zip(foot_ids, in_contact) if c]

    if blocks:
        Jc = np.vstack(blocks)
        U, s, Vt = np.linalg.svd(Jc, full_matrices=True)
        r = int((s > 1e-6 * s[0]).sum())
        N = Vt[r:, :].T                          # 18 × (18−r)
    else:
        N = np.eye(model.nv)                     # flight phase
    return Y, N
```

Then assert:

```python
S_T = np.zeros((18, 12)); S_T[6:, :] = np.eye(12)
res = N.T @ (Y @ phi_true - S_T @ tau_true)
assert np.linalg.norm(res) < 1e-8
```

If this is not near machine precision, the fault is in contact set, `v̇` convention, gravity, or parameter ordering — **never** in the theory. Debug here. Everything downstream is worthless until this passes.

---

## 7. Stage B — contact detection

With real foot force sensors this is mostly solved, but three things still need care.

**Hysteresis, not a bare threshold.**

```python
ON, OFF = 20.0, 10.0          # N; well above noise floor on a ~15 kg robot
def detect(force_series):
    state, out = False, []
    for f in force_series:
        state = (f > ON) if not state else (f > OFF)
        out.append(state)
    return np.array(out)
```

**Drop transition windows regardless.** Near touchdown/liftoff the rigid-contact assumption fails independently of what the sensor says: rubber is compressing, micro-slip is likely, and `v̇` is at its noisiest. Discard ~20–50 ms either side of every transition. You have 10⁴ samples; be picky.

**Contact ≠ stationary.** The sensor confirms a normal force exists, not that the foot is still. If the foot slides, `Jc v = 0` is false and the projection is invalid while the sensor reads happily. Cross-check:

```python
foot_vel = J_foot @ v
reject = np.linalg.norm(foot_vel) > VEL_TOL      # ~0.02 m/s
```

**Free diagnostic.** Compute `f̂ᵢ = (Jᵢᵀ)⁺ τᵢ` per leg and compare against the sensor. Systematic disagreement is an early warning about torque-constant calibration or gearbox efficiency asymmetry — both of which will corrupt the identification downstream.

---

## 8. Stage C — assembling the design matrix

### 8.1 Extra parameter blocks

For sample `k`, with joint acceleration `a_j = v̇[6:]` and joint velocity `v_j = v[6:]`:

```python
def extra_blocks(v, a, nv=18, nj=12):
    """Regressor columns for [rotor inertia | viscous | Coulomb], sign convention
       matching:  M v̇ + n + Sᵀ(a_rot + b_v·v + b_c·sign(v)) = Sᵀτ + Jcᵀλ"""
    Z = np.zeros((nv, nj))
    A_rot = Z.copy(); A_rot[6:, :] = np.diag(a[6:])
    A_vis = Z.copy(); A_vis[6:, :] = np.diag(v[6:])
    A_cou = Z.copy(); A_cou[6:, :] = np.diag(np.sign(v[6:]))
    return np.hstack([A_rot, A_vis, A_cou])       # 18 × 36
```

`sign(v)` is data. Near-zero velocity makes it ill-defined — either use `tanh(v/ε)` or drop samples with `|v_j| < ε` from the friction columns.

### 8.2 Stack

```python
def build_system(samples, freeze_legs=True, phi_nominal=None):
    W_rows, y_rows = [], []
    for s in samples:
        Y, N = regressor_and_projection(model, data, s.q, s.v, s.a, s.contact)
        E    = extra_blocks(s.v, s.a)
        rhs  = S_T @ s.tau

        if freeze_legs:
            # move known leg contribution to the right-hand side
            rhs = rhs - Y[:, 10:] @ phi_nominal[10:]
            Yv  = Y[:, :10]
        else:
            Yv  = Y

        W_rows.append(N.T @ np.hstack([Yv, E]))
        y_rows.append(N.T @ rhs)

    return np.vstack(W_rows), np.concatenate(y_rows)
```

**Both configurations are legitimate identification runs — run both.** The method is not trunk-specific; Khorshidi et al. identify the whole body. Freezing the legs reflects *your* premise, not a limitation.

| Config | Unknowns | Gives you | Trade-off |
|---|---|---|---|
| legs frozen | 46 | trunk + nuisance | Far better conditioned; inherits any error in your leg estimates |
| all free | 166 | every inertial parameter | No inherited bias; needs much richer excitation, and only ~94 of 130 are identifiable at all |

Compare the trunk block across the two. If it shifts materially, your prior leg identification was absorbing trunk error (exactly the fixed-base pitfall RPNA warns about) and you should trust neither result until you understand the discrepancy.

---

## 9. Stage D — the estimator

### 9.1 Ladder of models

Run these in order. Each isolates one error source; the differences are what you learn from.

| # | Model | Interpretation of a large residual |
|---|---|---|
| 0 | `φ = 0` | Baseline floor |
| 1 | `φ_URDF`, no friction/rotor | Everything mixed together — uninformative alone |
| 2 | `φ_URDF` frozen, fit friction + rotor only | **Now isolates inertia error** |
| 3 | `φ_URDF` legs frozen, fit trunk + nuisance | How much the trunk alone absorbs |
| 4 | Full identification | Your answer |

**Row 2 is the CAD audit.** The drop from row 2 to row 4 is the honest measure of "how wrong was the URDF." Skipping row 2 means you can't distinguish bad inertia from forgotten friction. For calibration: Ayusawa found a humanoid CAD total mass of 6.9 kg against 8.0 kg measured — a 14% error that propagated into most other parameters. Your Go2's URDF link-sum (~16 kg) vs spec (~15 kg) hints at something similar.

### 9.2 Convex program with physical consistency

Pseudo-inertia for one body, from Pinocchio-ordered `p`:

```python
import cvxpy as cp

def pseudo_inertia(p):
    """p = [m, hx, hy, hz, Ixx, Ixy, Iyy, Ixz, Iyz, Izz]  (Pinocchio order)"""
    m = p[0]; h = p[1:4]
    Ibar = cp.bmat([[p[4], p[5], p[7]],
                    [p[5], p[6], p[8]],
                    [p[7], p[8], p[9]]])
    K = 0.5 * cp.trace(Ibar) * np.eye(3) - Ibar
    return cp.bmat([[K,               cp.reshape(h, (3,1))],
                    [cp.reshape(h, (1,3)), cp.reshape(m, (1,1))]])
```

The program:

```python
theta = cp.Variable(W.shape[1])
phi_t = theta[:10]

# bounding ellipsoid for the trunk, from CAD geometry (collision box 0.376×0.094×0.114)
xc = np.array([0.021, 0.0, -0.005])
semi = np.array([0.21, 0.09, 0.09])          # generous
Qs = np.diag(semi**2)
Qj = np.block([[-np.linalg.inv(Qs),                 (np.linalg.inv(Qs)@xc)[:,None]],
               [(np.linalg.inv(Qs)@xc)[None,:],      1 - xc@np.linalg.inv(Qs)@xc]])

J = pseudo_inertia(phi_t)
cons = [
    J >> 1e-9 * np.eye(4),                    # full physical consistency
    cp.trace(J @ Qj) >= 0,                    # density realizable in ellipsoid
    phi_t[0] == M_SCALE_TRUNK,                # §3.3 — pin mass from the scale
    theta[10:22] >= 0,                        # rotor inertia ≥ 0
    theta[22:34] >= 0, theta[34:46] >= 0,     # friction coefficients ≥ 0
]

gamma = 1e-2
G = np.eye(10)                                # replace with geodesic weight, §9.3
obj = cp.Minimize(cp.sum_squares(W @ theta - y) / W.shape[0]
                  + gamma * cp.quad_form(phi_t - phi_prior[:10], G))
cp.Problem(obj, cons).solve(solver=cp.MOSEK)
```

`J ≻ 0` plus the trace condition together give **full physical consistency** — positive mass, PSD inertia, triangle inequalities on principal moments, and non-degeneracy (the body can't collapse to a point mass).

### 9.3 Regularization: don't use plain Euclidean

Euclidean distance on inertial parameters is coordinate-dependent — your answer changes between kg·m² and lb·in². Lee, Wensing & Park's **entropic divergence** is the convex, coordinate-invariant alternative; Khorshidi et al. use its second-order geodesic approximation as the weight `G`. Reference implementation: `github.com/alex07143/Geometric-Robot-DynID`.

Start with `G = I` to get the pipeline running, then swap it in and check how much the estimate moves. If it moves a lot, your data is weakly informative and the regularizer is doing the work — which is diagnostic in itself.

### 9.4 Alternative: log-Cholesky

If you'd rather use an unconstrained nonlinear optimizer (or fold this into a simulator-in-the-loop fit later), parameterize via Rucker & Wensing's log-Cholesky coordinates: a singularity-free smooth map `R¹⁰ → {physically consistent inertias}`. Consistency becomes structural rather than constrained. You lose global optimality; you gain compatibility with any optimizer.

---

## 10. Stage E — validation and diagnostics

### 10.1 Metrics

```python
def metrics(W, y, theta, ranks):
    res = W @ theta - y
    rmse = np.sqrt(res @ res / ranks.sum())          # N·m, rank-normalized
    rho  = np.sqrt((res @ res) / (y @ y))            # unitless fraction unexplained
    return rmse, rho
```

**Report `ρ`, not just RMSE.** Comparing raw RMSE across a slow wobble and a jump is meaningless — the jump has ~10× the torque magnitude. `ρ` is the fraction of projected-torque signal your model fails to explain, and is comparable across motions.

### 10.2 Parameter uncertainty — the check that catches SPI-Active's failure

```python
sigma2 = res @ res / (W.shape[0] - W.shape[1])
Cov    = sigma2 * np.linalg.pinv(W.T @ W)
rel_sd = np.sqrt(np.diag(Cov)[:10]) / np.abs(theta[:10])
```

**Any trunk parameter with `rel_sd > ~10%` is not identified**, regardless of how good the residual looks. Print this table every time. It is also your quantitative motivation for §11.

Also inspect `np.linalg.svd(W[:, :10])` directly — small singular values name the specific parameter *combinations* your trajectories failed to excite.

### 10.3 Residual structure tells you what's missing

Correlate the residual against candidate regressors:

| Correlates with | Missing model term |
|---|---|
| `sign(v)` | Coulomb friction |
| `v` | Viscous friction |
| joint `v̇` | Reflected rotor inertia |
| gravity direction in body frame | mass / CoM error |
| base `ω̇` | trunk rotational inertia error |

**Attribute residual to the trunk specifically** by projecting it onto the column space of `N_kᵀY_k^{trunk}` versus the leg blocks. If most of it lives in trunk-sensitive directions, that confirms your premise rather than assuming it.

**Slice by time and contact state.** Plot per-sample `‖res_k‖²/r_k` with contact events marked. Spikes at touchdown mean your contact windowing or rigid-contact assumption is off, not that your inertia is wrong. Systematically higher residual in one contact configuration points at a bad `Jc` — i.e. kinematic calibration.

### 10.4 Use the residual as a pre-fit data filter

Run the CAD-`φ` residual over raw data *before* fitting and reject the top ~1% of samples. Those are almost always corrupted (slip, mis-detected contact, differentiation artifacts), not informative. More effective than robustifying the estimator after the fact.

### 10.5 The test that actually validates parameters

Everything above validates *fit*, not *parameters* (§3.4). Two things validate parameters:

**Known-mass perturbation.** Bolt a calibrated mass at a surveyed point on the trunk, re-identify, and check the **delta** against exact ground truth. Differencing cancels most systematic actuator bias. This is the single best test in the whole plan.

**Metrology cross-check.** See §11.

---

## 11. Independent ground truth (do this early, it's cheap)

| Measurement | Method | Gives |
|---|---|---|
| Total mass | scale | `m` exactly — pin it (§3.3) |
| CoM x,y | 4 load cells or force plate, many stance configs | `c_x, c_y` sub-mm |
| CoM z | re-weigh on a ramp at known tilt (aircraft weighing method) | `c_z` |
| Trunk inertia | bifilar/trifilar pendulum, legs locked in known configs | `I` principal axes |

**Pendulum accuracy.** Trifilar methods reach ~1% with care; du Bois et al. (Experimental Mechanics 2009) found CoM misalignment forgiving (10% misalignment → ~1% inertia error) while suspension length/spacing and period measurement dominate error. Repeat with 3–4 leg configurations and subtract the known leg contributions each time — consistency across configurations is your internal check.

This completely bypasses the actuator-torque and contact models, which are your two largest error sources. Treat it as the reference against which the dynamic identification is judged, not the other way round.

### Ayusawa flight-phase cross-check

During the ballistic phase of a jump, no external forces act: `Jc` is empty, `P = I`, all 18 equations survive, and **no torque, friction, or actuator model is needed**. Ayusawa's theorem says everything except total mass is identifiable from free-flight motion alone.

Design a "jump and flail" primitive: launch, then execute an optimized leg motion during the ~200–300 ms airborne window, fitting the base equations with angular momentum conserved. Aggregate over many jumps. Short windows, but the nuisance-parameter burden drops to zero — and its error modes are uncorrelated with the projected-dynamics method, which is exactly what you want in a cross-check.

---

## 12. Practical traps specific to the Go2

- **`tau_est` is current-derived, not sensed.** Torque-constant error, gearbox efficiency asymmetry (positive vs negative torque differ), and temperature drift all leak into inertia estimates. Identify `K_t` and friction per joint on isolated leg-in-air sweeps first. The official reduction ratio is **6.33** (Unitree's GO-M8010-6 spec) — some reseller pages quote ~1:10; use the official figure.
- **Battery.** Large, removable, and the most likely source of CAD/reality divergence in the trunk. Identify with-battery and without-battery separately, and check whether it seats repeatably.
- **Foot compliance.** Rubber deforms; the rigid-contact assumption is an approximation. Window aggressively around transitions.
- **Onboard state estimator is a black box.** If you can borrow motion capture even for one session, use it to validate your base velocity/acceleration pipeline, then proceed without it.
- **Keep CAD as the prior, not as truth.** It's the right regularization target even though it's wrong.

---

## 13. Milestones and acceptance criteria

| # | Milestone | Gate to proceed |
|---|---|---|
| 1 | Model loads, parameter round-trip exact | assertion passes |
| 2 | `P Jcᵀ = 0`, simulated residual < 1e-8 | assertion passes |
| 3 | Metrology: mass, CoM, pendulum inertia | pendulum repeatability < 3% |
| 4 | Friction + rotor identified on isolated sweeps | `ρ` < 0.15 on leg-in-air validation |
| 5 | CAD audit (ladder row 2) on real data | quantified — report the number |
| 6 | Trunk identified, legs frozen | all trunk `rel_sd` < 10%; `ρ` improves ≥ 30% over row 2 |
| 7 | Held-out motion validation | `ρ` on held-out ≤ 1.5× training `ρ` |
| 8 | Known-mass perturbation | recovered Δm within 5%, Δc within 1 cm |
| 9 | Agreement with pendulum | principal inertias within combined uncertainty |

Milestone 6's `rel_sd` gate is the one most likely to fail first, and failing it is *informative* — it's the empirical case for the next section.

---

## 14. Phase 2 — Ds-optimal excitation for the trunk block

### 14.1 The gap

Essentially all excitation-design literature optimizes a scalar criterion over the **whole** parameter vector: Gautier & Khalil's condition number, Swevers' Fourier-series D-optimality, Ayusawa's condition-number optimization for legged systems, Lee & Park's frame-invariant geometric criteria, SPI-Active's `tr(F⁻¹)`.

You care about 10 parameters (6 after pinning mass and CoM), with 156 others as **nuisance**. The statistically correct object is the Schur complement of the Fisher information:

```
F = [ F_tt   F_tn ]        Cov(φ_trunk) ∝ ( F_tt − F_tn F_nn⁻¹ F_nt )⁻¹
    [ F_nt   F_nn ]

Ds-optimality:   maximize  det( F_tt − F_tn F_nn⁻¹ F_nt )
```

This is textbook in the statistics DoE literature and essentially absent from robot inertial identification. The nearest robotics neighbours are Lee & Park's reduced-parameter formulation (reduces the parameter set rather than treating the complement as statistical nuisance) and QOED's information-subspace eigenanalysis. Neither targets a single link's inertial block by formal Ds-optimality.

### 14.2 Formulation

Parameterize a four-feet-planted base trajectory as a finite Fourier series in the 6 base-pose DoF:

```
x_base(t) = x₀ + Σ_{h=1..H} [ a_h sin(hω t) + b_h cos(hω t) ] / (hω)
```

Solve joint angles by inverse kinematics with feet fixed. Then:

```
maximize    log det( F_tt(a,b) − F_tn F_nn⁻¹ F_nt )
subject to  joint position / velocity / torque limits
            friction cone at each foot
            CoM projection inside support polygon (or ZMP margin)
            base pose within reachable workspace
```

with `F = Σ_k (N_kᵀ W_k)ᵀ (N_kᵀ W_k) / σ²`.

The Schur complement is differentiable; `log det` keeps it concave-ish and well-behaved. Analytic gradients follow Lee & Park's recursive approach, or start with finite differences plus CMA-ES if you want something working fast.

### 14.3 What to benchmark

| Design | Comparison |
|---|---|
| Hand-designed wobble (your milestone-6 data) | baseline |
| Whole-vector D-optimal | the standard approach |
| SPI-Active-style `tr(F⁻¹)` | the recent learning-based approach |
| **Ds-optimal on trunk block** | yours |

Metric: trunk-block posterior variance (especially the off-diagonals `Ixy, Ixz, Iyz`, which SPI-Active explicitly failed to excite) at **equal experiment duration**. Secondary: agreement with the pendulum ground truth.

### 14.4 Framing for a reviewer

The novelty is not Ds-optimality itself — cite the classical nuisance-parameter DoE literature honestly. The contribution is the combination: formal subset-focused optimal design, applied to a specific link's inertial block, under contact-consistent projected dynamics, subject to balance and friction-cone constraints, validated against independent metrology on hardware. Distinguish explicitly from Lee & Park (parameter reduction) and QOED (subspace weighting).

---

## 15. Quick failure-mode reference

| Symptom | Likely cause |
|---|---|
| Simulated residual ≫ 0 with true `φ` | contact set, `v̇` convention, gravity, or parameter ordering |
| Residual spikes at touchdown/liftoff | contact windowing too narrow; foot compliance |
| Residual high in one contact config only | bad `Jc` → kinematic calibration |
| Great RMSE, absurd inertia values | insufficient excitation — check `rel_sd`, not RMSE (§3.4) |
| Estimate moves a lot when `γ` changes | data weakly informative; regularizer is doing the work |
| Mass estimate wanders | expected — pin it from the scale (§3.3) |
| Identified `φ` hits an LMI constraint boundary | over-constrained bounding ellipsoid, or genuinely bad data |
| Leg parameters shift when unfrozen | your prior leg identification absorbed trunk error |

---

## Appendix A — Recurring confusions

**"Isn't 18 the number of joints?"** No — 12 joints + 6 base DoF. See §1.1.

**"Shouldn't there be a factor of 4 somewhere, since SE(3) is 4×4?"** No. SE(3) is a 6-dimensional group; its 4×4 matrix form encodes pose, not dynamics. Velocities are 6-vectors. See §1.1.

**"Why does the rank drop when I premultiply by P?"** Because `P` itself is rank-deficient by construction — it projects onto a subspace of dimension `18 − rank(Jc)`. Multiplying by it cannot preserve rank. See §2.3.

**"Can this method identify legs too, or only the trunk?"** Whole body. See §8.2.

**"So how do I actually get the trunk mass?"** Weigh it. Either weigh the whole robot and subtract your identified leg masses (propagates leg error), or weigh the trunk assembly directly (cleaner). Then impose it as an equality constraint. See §3.3, §11.

**"Is `getFrameJacobian(...)[:3, :]` the right way to build `Jc`?"** Yes, with one caveat: `getFrameJacobian` is a *getter* and requires `computeJointJacobians(model, data, q)` followed by `updateFramePlacements(model, data)` to have run first, or it returns stale values **silently**. `computeFrameJacobian(model, data, q, fid, ref)` performs the kinematics internally and is safer when not batching. The `[:3, :]` slice takes the linear rows because point contacts constrain translation only.

**"Does the reference frame argument matter?"** Not for `P` — see §2.4. It matters if you later want to reconstruct `λ` in a specific frame.

---

## Key references

- Khorshidi, Dawood, Nederkorn, Bennewitz & Khadiv, *Physically-Consistent Parameter Identification of Robots in Contact*, ICRA 2025, arXiv 2409.09850 — the projection method
- Ayusawa, Venture & Nakamura, IJRR 33(3):446–468, 2014 — base-link identifiability, free-flight result
- Wensing, Niemeyer & Slotine, *A Geometric Characterization of Observability in Inertial Parameter Identification*, arXiv 1711.03896 / IJRR 2024 — RPNA, per-limb identification pitfalls
- Wensing, Kim & Slotine, RA-L 3(1):60–67, 2018 — pseudo-inertia LMIs
- Lee, Wensing & Park, T-RO 36(2):348–365, 2020 — entropic divergence; code at `alex07143/Geometric-Robot-DynID`
- Rucker & Wensing, RA-L 2022 — log-Cholesky parameterization
- Lee, Lee & Park, Automatica 131:109773, 2021 — invariant excitation criteria, reduced parameter sets
- Sobanbabu, He, He, Yang & Shi, *SPI-Active*, CoRL 2025, arXiv 2505.14266 — Go2 base params; code at `LeCAR-Lab/SPI-Active`
- Mistry & Righetti, RSS 2012 — constrained/underactuated projection
- Tools: Pinocchio (regressors, Jacobians), FIGAROH (`gitlab.laas.fr/gepetto/figaroh`), BIRDy (estimator benchmark)
