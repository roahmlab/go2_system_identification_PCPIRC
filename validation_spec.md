# Validation pipeline spec — Stage 0 + Stage 1

Implements steps 1–2 of the validation plan: environment, model layout, and the projection core.
Scope is **validation only** — no identification.

**Relationship to `stage_a_spec.md`:**

| `stage_a_spec.md` | Status |
|---|---|
| §0 (URDF lumping, rotor vs armature, legs-known premise) | **Still valid.** Read it; the fixtures below come from there. |
| §1 environment | Superseded by §1 here (adds `pybullet`). |
| §2 `model.py`, §3 `tests/test_ordering.py` | **Still valid, unchanged.** Restated here so this file is self-contained. |
| §4 `sim.py` Level 1 (`lstsq` for `τ, λ`) | **Superseded** — least-norm returns feet pulling upward on the ground. Replaced by a friction-cone SOCP in the Stage 2 spec (not yet written). |
| §5 `tests/test_projection.py` + implementation notes | **Still valid, unchanged.** Restated here. |
| §7 open decisions | 1 and 2 still open; 3 resolved — standalone package with `paths.py`. |

Reference for the method: `Literature Review/khadiv.pdf` (Khorshidi et al., ICRA 2025).

**Do not build Stage 2 until every gate below passes.**

---

## 1. Environment

Nothing needed is installed. `pinocchio 2.7.0` in the `hopscotch` env is too old.

```bash
conda create -n go2sysid -c conda-forge python=3.11 \
    pinocchio">=3.0" cvxpy scipy numpy matplotlib pytest pybullet
conda activate go2sysid
python -c "import pinocchio as pin; print(pin.__version__)"   # expect 3.x
```

`cvxpy` ships Clarabel/SCS — both handle the Stage 2 SOCP. `pybullet` is for Stage 4; installing it
now avoids a second env rebuild. No mesh files needed: `buildModelFromUrdf` ignores
`<visual>`/`<collision>`, so the `package://go2_description/dae/...` paths never resolve.

---

## 2. `paths.py`

The only cross-repo coupling in the package. One constant, nothing else.

```python
from pathlib import Path

URDF = Path(__file__).resolve().parents[1] / \
    "RAPTOR_sysid_go1" / "Robots" / "unitree-go2" / "go2_description.urdf"
```

Assert it exists at import time — a missing URDF should fail loudly at import, not as a confusing
Pinocchio parse error 40 lines deep.

---

## 3. `model.py`

Single module, no classes. Everything pure and side-effect free except `apply_phi`.

```python
FOOT_FRAMES = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
```

**`load_model() -> (model, data, foot_ids)`**
- `pin.buildModelFromUrdf(str(URDF), pin.JointModelFreeFlyer())`
- assert `model.nq == 19`, `model.nv == 18`, `model.njoints == 14` (universe + freeflyer + 12)
- resolve foot frame ids; assert each `getFrameId` returns `< model.nframes`. Pinocchio returns
  `nframes` on a miss instead of raising — a silent failure worth guarding.

**`phi_from_model(model) -> (130,)`**
- `np.concatenate([model.inertias[i].toDynamicParameters() for i in range(1, model.njoints)])`
- body `i` occupies `phi[10*(i-1) : 10*i]`; trunk is `phi[0:10]`
- **Note:** loop `range(1, model.njoints)`, i.e. 13 bodies. The existing implementation at
  `RAPTOR_sysid_go1/Examples/Unitree_Go2/python/sysid_eval_sim.py:311` loops `range(model.nv)`,
  which is correct only for the 3-DoF leg where `nv == njoints−1`. Floating base has `nv=18` but
  `njoints−1=13`. **This is the single most likely port bug.**

**`apply_phi(model, phi) -> None`** — inverse, via `pin.Inertia.FromDynamicParameters`. The
parallel-axis and `m·c → c` handling in `sysid_eval_sim.py:317-325` is correct; reuse that logic.

**`S_T -> (18,12)`** — module constant, `S_T[6:, :] = np.eye(12)`.

**`body_slice(model, joint_name) -> slice`** — name → 10-wide slice into `φ`. Used by tests and by
the freeze-legs logic, so nobody hand-writes `10:` again.

### Ordering hazard — read this before writing any array literal

Pinocchio's `toDynamicParameters` order is

```
[ m, h_x, h_y, h_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz ]
```

`I_yy` at index **6**, `I_xz` at index **7**. The paper's eq. (3) uses
`ixx, ixy, ixz, iyy, iyz, izz`. Transcribing the paper's order swaps `I_yy ↔ I_xz`, which for the
trunk is `0.106` vs `0.0023` — a factor of 47 — and it will **not** show as a residual blow-up in a
symmetric test case. This is why the fixture below is written out explicitly.

---

## 4. `tests/test_ordering.py`

**T1 — round trip.** `apply_phi(model, phi_from_model(model))`, re-extract, `assert_allclose` at
`rtol=0, atol=1e-12`. Necessary but not sufficient: it passes even if both functions share the same
ordering bug. Hence T2.

**T2 — trunk fixture.** Hard-coded, computed by parallel-axis directly from the URDF rather than by
asking Pinocchio, so it is a genuine cross-check. Pinocchio fuses the four `*_hip_rotor` fixed
joints into `base`, so the trunk body is **not** the URDF `base` link — 7.277 kg, not 6.921. See
`stage_a_spec.md` §0.1 for the derivation.

```python
PHI_TRUNK_EXPECTED = np.array([
    7.277000000e+00,   # m
    1.461163380e-01,   # h_x
    0.000000000e+00,   # h_y
   -3.713816950e-02,   # h_z
    2.590471460e-02,   # I_xx   about the base-frame ORIGIN, not the CoM
    1.216600000e-04,   # I_xy
    1.060773140e-01,   # I_yy   <-- index 6
    2.268964730e-03,   # I_xz   <-- index 7
   -3.120000000e-05,   # I_yz
    1.155790890e-01,   # I_zz
])
assert_allclose(phi_from_model(model)[0:10], PHI_TRUNK_EXPECTED, rtol=1e-6)
```

`toDynamicParameters` reports inertia **about the joint origin**, not the CoM. The CoM-referred
diagonal is `(0.02572, 0.10295, 0.11265)` — if you see those, an
`I_origin = I_com + m(cᵀc·1₃ − ccᵀ)` shift is missing.

**T2b — total mass.** `sum(model.inertias[i].mass for i in range(1, model.njoints))` ≈ 16.087 kg.
Cross-checks the lumping as a whole: `7.277 + 4×(0.767 + 1.241 + 0.194) = 16.085`.

**T3 — regressor identity.** The cheapest check that gravity, sign, and ordering all agree.

```python
q = pin.randomConfiguration(model)       # normalize the quaternion block afterwards
v, a = np.random.randn(18), np.random.randn(18)
pin.computeJointTorqueRegressor(model, data, q, v, a)
assert_allclose(data.jointTorqueRegressor @ phi0,
                pin.rnea(model, data, q, v, a), rtol=1e-9)
```

Also assert the regressor is `(18, 130)`. **If T3 fails, stop.** Nothing downstream can be right.

---

## 5. `regressor.py` — per-sample projection

Stage 1 builds only the per-sample function. Stacking, friction blocks and design-matrix assembly
come with Stage 2/3, when there is data to stack. Keep this module minimal.

```
regressor_and_projection(model, data, q, v, a, in_contact) -> (Y, N, r, Jc)

    pin.computeJointTorqueRegressor(model, data, q, v, a)
    Y = data.jointTorqueRegressor.copy()                    # (18, 130)

    blocks = [ pin.computeFrameJacobian(model, data, q, fid, ref)[:3, :]
               for fid, c in zip(foot_ids, in_contact) if c ]

    if blocks:
        Jc = np.vstack(blocks)                              # (3*nc, 18)
        U, s, Vt = np.linalg.svd(Jc, full_matrices=True)
        r = int((s > 1e-6 * s[0]).sum())                    # NUMERICAL rank
        N = Vt[r:, :].T                                     # (18, 18-r)
    else:
        Jc = np.zeros((0, model.nv))
        N  = np.eye(model.nv)                               # flight: r = 0, N = I
    return Y, N, r, Jc
```

Implementation notes:

- **Use `pin.computeFrameJacobian`, not `getFrameJacobian`.** The latter is a getter that silently
  returns stale data unless `computeJointJacobians` **and** `updateFramePlacements` ran first.
  `computeFrameJacobian` does the kinematics internally and cannot go stale. Swap to the batched
  getter only once a test exists that would catch staleness.
- **Reference frame does not matter.** `LOCAL` vs `LOCAL_WORLD_ALIGNED` rotates each 3-row block by
  an invertible `R_i`, and `null(R·Jc) = null(Jc)`, so `N` spans the same subspace. Pick
  `LOCAL_WORLD_ALIGNED` and be consistent. T4b below documents the claim.
- **Return `r`.** The RMSE is rank-normalized by `Σ r_k`, not by sample count.
- **Never hard-code `3nc`.** A fully-extended knee loses rank and the null space grows. That is
  what T6 enforces.
- Point-contact feet → the `[:3, :]` slice of the 6-row frame Jacobian. Rotational rows are dropped
  because a point contact transmits no moment.
- **This function takes only the feet in contact.** `traj.py` (§7) needs a *different* Jacobian built
  from **all four** feet. Two distinct objects — see the warning box in §7.2. Name them differently:
  `projection_jacobian(...)` here, `kinematic_jacobian(...)` there.

---

## 6. `tests/test_projection.py`

**T4 — the projector annihilates contacts.** For each `nc ∈ {1, 2, 3, 4}`, at well-conditioned
standing configurations:

```python
assert np.linalg.norm(N.T @ Jc.T) < 1e-10
assert N.shape[1] == 18 - 3*nc
assert_allclose(N.T @ N, np.eye(N.shape[1]), atol=1e-10)     # orthonormality
```

**T4b — frame invariance.** `N` from `LOCAL` and from `LOCAL_WORLD_ALIGNED` span the same subspace:

```python
assert np.linalg.norm(N_local @ N_local.T - N_lwa @ N_lwa.T) < 1e-10
```

**T4c — flight.** `in_contact = [False]*4` → `r == 0`, `N` is `(18,18)` identity, and `Jc` is empty
without raising.

**T6 — rank detected, not assumed.** Construct a near-singular configuration (fully extended knee,
`q_calf` at its limit), assert the SVD-based `r` is **less** than `3nc` — so `N.shape[1] > 18 − 3nc`
— and that the sample is flagged for rejection by a conditioning threshold.

> **T5 (projected residual < 1e-8 against true φ) belongs to Stage 2**, because it needs ground-truth
> `(τ, λ)` and that requires the friction-cone SOCP. Do not attempt it with `lstsq`.

---

## 7. `traj.py` — the validation trajectories

This module produces `(q, v, a, foot_contact_mask)` only. Ground-truth `(τ, λ)` is Stage 2's job
(`sim.py`), and none of the kinematics below depends on it — so `traj.py` can be built alongside
Stage 0/1.

### 7.0 Notation and conventions — read before writing a line of this module

Three separate conventions live here and two of them fail *silently*. Fix the vocabulary first.

#### Symbols

```
v   = [ v_b ; v_j ] ∈ R¹⁸        v_b ∈ R⁶ base twist,   v_j ∈ R¹² joint rates

J_i ∈ R^{3×18}    per-foot: the [:3,:] rows of foot i's frame Jacobian, LOCAL_WORLD_ALIGNED
                  defined by  J_i·v = ṗ_i  = LINEAR velocity of foot i's origin, world-aligned axes
Jc  ∈ R^{12×18}   the four J_i stacked        ("c" = the contact stack)

J_i = [ J_i,b | J_i,j ]      3×6  | 3×3        subscript b = base block, j = joint block
Jc  = [ Jc_b  | Jc_j  ]     12×6  | 12×12
```

`Jc_b` **stacks dense** (the base moves every foot); `Jc_j` is **block-diagonal** (leg `i`'s joints
move only foot `i`). So `Jc_j⁻¹` is never a real 12×12 inverse — it is four independent 3×3 solves,
reusing the same `J_i,j` already built for the IK Newton step.

**`Jc_b · v_b` is not a spatial velocity.** It is a 12-vector of four stacked 3D *point* velocities:
the velocity each foot would have if the legs were welded rigid. The only 6D object in the equation
is `v_b` itself. Everything downstream of a `[:3,:]` slice is a point velocity, because a point
contact constrains position only, not orientation. Read the velocity equation as

```
v_j = Jc_j⁻¹ ( ṗ_c − Jc_b·v_b )        "joints must supply (what I want) − (what the trunk caused)"
```

#### The six trunk channels — name them individually

There is **no single `x(t)`**. There are six independent scalar multisines:

```
s_x(t)  surge      φ(t)  roll
s_y(t)  sway       θ(t)  pitch
s_z(t)  heave      ψ(t)  yaw
```

Harmonics sum **within** a channel (that is the multisine). Channels are **never** summed with each
other — they are six coordinates of one pose, and you cannot add metres to radians. Assembly is by
placement and matrix product, not addition:

```
p_base(t) = [ s_x, s_y, 0.30 + s_z ]        R_base(t) = Rz(ψ)·Ry(θ)·Rx(φ)
T_wb(t)   = [[R_base, p_base], [0,0,0,1]]                       ← q[0:7], no solving
```

All six run **simultaneously** — off-diagonal inertia (`I_xy, I_xz, I_yz`) only appears under coupled
rotation, so sequential single-axis sweeps would make those parameters invisible.

#### Frames

| Quantity | Frame | Role |
|---|---|---|
| `s_x…ψ`, i.e. `T_wb` | **world** | prescribed trunk pose. `T_wb` reads *world ← base*: `p^world = T_wb·p^base` |
| `p_i^world` | **world** | measured once at `t=0`, frozen forever — the pins |
| `p_i^base = R_wbᵀ(p_i^world − p_base)` | **base** | the IK **target** |
| `q_joint` | — | the IK **output** |

The IK works in the base frame because the leg's FK chain is built from offsets bolted to the trunk
and knows nothing about the world. Pulling the target in (rather than pushing the FK out) makes the
right-hand side a per-timestep constant, keeps `T_wb` out of the Newton loop, and is what makes the
four legs decouple.

#### ⚠ Trap 1 — Jacobian reference frame

| `pin.ReferenceFrame` | what `J·v` gives |
|---|---|
| `LOCAL` | foot velocity in the **foot's own** frame |
| `LOCAL_WORLD_ALIGNED` | velocity of the **foot point**, world-aligned axes — **use this** |
| `WORLD` | the spatial/screw velocity referred to the **world origin** — *not* the foot point's velocity |

`WORLD` is the trap: the name suggests "foot velocity in world coordinates," but it differs by
`ω × p`. We prescribe `ṗ_c` as world-frame *point* velocities, so `LOCAL_WORLD_ALIGNED` it is.

#### ⚠ Trap 2 — Pinocchio's `v_b` is in the LOCAL (base) frame

```
v_b[:3] = R_baseᵀ · ṗ_base        linear velocity of base origin, in BASE coords
v_b[3:] = ω_body                   angular velocity,               in BASE coords
```

Differentiating the multisines gives `ṗ_base` in **world** coordinates. You must rotate it before
packing into `v`. Skipping the rotation yields a `v` that looks smooth and plausible and is wrong by
the full base rotation — and it propagates into `a`, which multiplies trunk inertia directly.

#### ⚠ Trap 3 — Euler rates are not angular velocity

`(φ̇, θ̇, ψ̇)` is **not** `ω`. For `R = Rz(ψ)Ry(θ)Rx(φ)`:

```
ω_x = φ̇ − ψ̇·sinθ
ω_y = θ̇·cosφ + ψ̇·cosθ·sinφ
ω_z = −θ̇·sinφ + ψ̇·cosθ·cosφ
```

At small angles this collapses to `(φ̇, θ̇, ψ̇)`, which is exactly why it gets missed. At the §7.4
amplitudes (0.15 rad) `sinθ ≈ 0.15`, so the cross-coupling is **order 15%** — far above anything the
calibration is trying to resolve. `ω̇` needs the derivative of this map, not just of the Euler rates.

#### Gates that catch all three

```
G7.0a  finite-difference:  ‖v_b − [R ᵀṗ_base ; ω] from pin.difference(q(t),q(t+h))/h‖ < 1e-6
G7.0b  constraint (G7.1):  ‖Jc·a + J̇c·v − p̈_c‖ < 1e-10
```

`G7.0a` catches Traps 2 and 3; `G7.0b` catches Trap 1 and any sign error. Neither is optional — every
one of these failures produces a `v`/`a` that *looks* physical.

### 7.1 Design principle: do NOT optimize these

Optimal excitation (Swevers condition-number optimization, and your Phase-2 Ds-optimal work) exists
to minimize *parameter variance during identification*. That is not what this is. A validation
trajectory's bar is **held-out and sufficient**, not optimal — and scoring a model on a trajectory
that was optimized for it biases the result toward the directions the optimizer happened to favour.
Validation trajectories in the Gautier/Swevers tradition are deliberately arbitrary.

"Sufficient" is checked, not designed: Stage 3a's `svd(W[:, :10])` says which trunk directions the
motion can see. Trajectories are **screened** by that, never optimized against it.

### 7.2 One kinematics routine covers wobble and trot

#### Why the joint angles are determined at all

`q = [p_base(3), quat_base(4), q_joint(12)]`. For the wobble you **choose** the trunk pose — the six
channels of §7.0, dropped straight into `p_base`/`quat_base`. Nothing is computed there.
The 12 joint angles come from a counting argument:

> Each foot is on the ground and not sliding ⇒ its **world** position is fixed. 4 feet × 3
> coordinates = **12 constraints**, against exactly **12 unknown joint angles**. Determined.

Physically: the trunk moves, the feet stay glued, the legs bend to accommodate — and there is exactly
one way to bend them (per leg, given a knee-bend branch, and the calf limit `−2.7227 … −0.83776`
already forbids the other branch).

#### Trunk pose → foot position in base frame → joint angles

Forward kinematics of leg `i`, with `T_wb` the world←base transform:

```
p_i^world  =  T_wb · f_i(q_i)
```

`f_i(q_i)` is the foot position **in the base frame** — a pure function of that leg's 3 angles and
fixed geometry. You know `p_i^world` (pinned) and `T_wb(t)` (you prescribed it), so invert the rigid
transform, which is free:

```
p_i^base(t)  =  T_wb(t)⁻¹ · p_i^world           ← known at every instant
```

then solve `f_i(q_i) = p_i^base(t)`: **3 equations, 3 unknowns, per leg, independently.** That single
solve is the only nontrivial step.

*Worked example.* Nominal FL foot at base-frame `(0.1934, 0.142, −0.30)`, knee `γ = −1.580 rad`.
Translate the trunk `+0.04 m` in x: the foot has not moved in the world, but in the base frame it now
sits at `(0.1534, 0.142, −0.30)` — it slid 4 cm *backward*, so the leg must reach backward. Thigh
joint is at `(0.1934, 0.142, 0)`, so thigh→foot is `(−0.04, 0, −0.30)`, `d = 0.3027 m`. Law of
cosines with `L1 = L2 = 0.213`:

```
cos γ = (d² − L1² − L2²)/(2·L1·L2) = (0.0916 − 0.0908)/0.0908 = 0.0088   ->   γ = −1.562 rad
```

A 4 cm trunk shift moves that knee ~1°, and `d = 0.3027` sits well under the `0.3556 m` cap set by
the knee limit. Use this as a hand-check on the IK before trusting it.

**Solve it by Newton, not closed form.** A closed form exists (strip the abduction about x, then it
is a planar 2-link problem), but `q_i ← q_i + J_i⁻¹(p_i^base_des − f_i(q_i))` seeded from the previous
timestep converges in 2–3 iterations — consecutive samples are ~1 mm apart — is not Go2-specific, and
gives you `J_i`, which you need next anyway.

#### Velocities and accelerations — never differentiate the IK

Differentiating a Newton solver numerically is a bad idea. Use the constraint directly. Let each foot
have a prescribed **world** trajectory `p_i(t), ṗ_i(t), p̈_i(t)` — constant for stance, a swing arc
for swing — stacked into `p_c, ṗ_c, p̈_c ∈ R^12`. Split the kinematic Jacobian `Jc = [Jc_b | Jc_j]`,
`Jc_b ∈ R^{12×6}`, `Jc_j ∈ R^{12×12}` block-diagonal (3×3 per leg, because leg `i`'s joints move only
foot `i` — so "inverting" it is four independent 3×3 solves reusing `J_i`). Then from `Jc·v = ṗ_c`:

```
v_j  =  Jc_j⁻¹ ( ṗ_c − Jc_b · v_b )

a_j  =  Jc_j⁻¹ ( p̈_c − Jc_b · a_b − J̇c·v )
```

For the wobble `ṗ_c = p̈_c = 0`, so `v_j = −Jc_j⁻¹ Jc_b v_b`, which reads: *the trunk moves with twist
`v_b`, which would drag the feet at rate `Jc_b v_b`; the joints must move at exactly the rate that
cancels it.*

Trot changes only one thing: `ṗ_c` is no longer all zeros — swing entries carry the arc's velocity.
The swing legs are still *constrained*, just by your chosen trajectory instead of by the ground, and
the algebra cannot tell the difference. That is why one routine covers both.

> **⚠ Two different contact Jacobians. Do not conflate them.** Both are called `Jc`, both come from
> the same Pinocchio call, and mixing them is the most likely structural bug in the pipeline.
>
> | | Used in | Which feet | Shape |
> |---|---|---|---|
> | **kinematic** | `traj.py` — solving for `q_j, v_j, a_j` | **all 4, always** — every foot has a prescribed path | 12×18 |
> | **projection** | `regressor.py` — building `N` | **only feet in contact** — only those exert `λ` | `3nc`×18 |
>
> During trot a swing foot belongs in the kinematic Jacobian but must be **absent** from the
> projection Jacobian. If it leaks in you annihilate force directions for a foot touching nothing and
> `r` comes out 6 instead of 12. Give the two functions different names.

**Trap — `J̇c·v` must be the CLASSICAL acceleration, not the spatial one.** With
`LOCAL_WORLD_ALIGNED`, `Jc·v` is the classical linear velocity of the frame origin, so its
derivative is the classical acceleration. Get it as:

```python
pin.forwardKinematics(model, data, q, v, np.zeros(model.nv))   # a = 0 -> leaves exactly J̇v
Jdot_v = np.concatenate([
    pin.getFrameClassicalAcceleration(model, data, fid,
                                      pin.LOCAL_WORLD_ALIGNED).linear
    for fid in foot_ids])
```

Using `getFrameAcceleration` (spatial) instead differs by `ω × v` and will silently corrupt `a`,
which multiplies trunk inertia directly. Assert `‖Jc·a + J̇c·v − p̈_c‖ < 1e-10` on every sample —
that single assertion catches this and every sign error in the block.

### 7.3 Geometry (from `go2_description.urdf`)

```
hip origin (from base)   (±0.1934, ±0.0465, 0)
abduction offset          0.0955        thigh L1 = 0.213    calf L2 = 0.213
nominal foot stance       FL(+0.1934, +0.1420)  FR(+0.1934, −0.1420)
                          RL(−0.1934, +0.1420)  RR(−0.1934, −0.1420)     support polygon 0.387 × 0.284 m
nominal trunk height      0.30 m   ->  thigh ≈ 0.79 rad, calf ≈ −1.58 rad

joint limits   hip   ±1.0472        thigh  −1.5708 … 3.4907   calf  −2.7227 … −0.83776
torque limits  hip/thigh 23.7 N·m   calf 45.43 N·m
vel limits     hip/thigh 30.1 rad/s calf 15.70 rad/s
```

The calf limit `−0.83776` means the knee can never straighten past 48°, which bounds `Jc_j`
conditioning from below — but stay well clear of it anyway (§7.7).

### 7.4 M1 — four-feet trunk wobble  (nc = 4, r = 6)

The key trunk-inertia motion: large `ω̇` with the feet planted, and the only regime where absolute
mass is observable because the trunk is actually loaded.

Feet pinned at nominal stance. Each of the six channels of §7.0 is its **own** periodic multisine —
Swevers' finite Fourier series structure, base period `T0 = 10 s`, `f0 = 0.1 Hz`. The template below
is applied **six times**, once per channel, with a distinct harmonic set each time:

```
c(t) = c0 + Σ_k  (A/2)·sin(2π · k·f0 · t + φ_k)          c ∈ {s_x, s_y, s_z, φ, θ, ψ}
```

| channel | `c0` | harmonics `k` | total amplitude `A` | resulting freqs |
|---|---|---|---|---|
| `s_x` surge | 0 | 3, 7 | 0.040 m | 0.3, 0.7 Hz |
| `s_y` sway | 0 | 4, 9 | 0.040 m | 0.4, 0.9 Hz |
| `s_z` heave | 0.30 m | 5, 11 | 0.030 m | 0.5, 1.1 Hz |
| `φ` roll | 0 | 2, 13 | 0.15 rad | 0.2, 1.3 Hz |
| `θ` pitch | 0 | 6, 17 | 0.15 rad | 0.6, 1.7 Hz |
| `ψ` yaw | 0 | 8, 19 | 0.12 rad | 0.8, 1.9 Hz |

Distinct harmonics matter twice over: they keep the six channels spectrally separable when you later
attribute residual to a DoF, and they guarantee the *combined* rotation that makes `Ixy, Ixz, Iyz`
observable at all. Sequential single-axis sweeps would leave those three invisible.

Periodicity is deliberate: it lets you average over cycles on hardware to lower the noise floor.
Ramp amplitudes in and out with `raised_cosine` (`sysid_eval_trajectory_generator_real.py:134-164`)
so the motion starts and ends at rest — otherwise the first sample has a step in `a`.

Differentiate each channel **analytically**. Then — per §7.0 Traps 2 and 3 — rotate `ṗ_base` into the
base frame and map the Euler rates through to `ω` before packing `v_b`. Do **not** pack `(ṡ, φ̇, θ̇, ψ̇)`
directly; at `sinθ ≈ 0.15` that is a 15% error. Never finite-difference in sim.

**Duration 60 s @ 100 Hz → 6000 samples × 6 rows = 36 000 equations.**

### 7.5 M2 — trot  (nc = 2, r = 12)

Diagonal pairs `(FL,RR)` and `(FR,RL)` alternating. Twice the equations per sample of the wobble,
and the paper's primary motion.

```
gait period      T = 0.4 s          duty factor 0.5  ->  0.2 s stance per pair
step height      0.05 m             swing arc: quintic in x/y, sine-squared in z (zero vel+accel
                                    at both ends, so p̈_c is continuous through touchdown)
base forward vel vx ∈ {0, ±0.3, ±0.5} m/s     base height 0.30 m, ±0.01 m bob at 1/T
```

Stance feet: `ṗ = p̈ = 0`. Swing feet: the arc above. Footholds from a Raibert-style placement
(`p_hip + vx·T_stance/2`), which is all BiConMP uses for the acyclic cases too.

At `nc = 2` the 6 unactuated equations have exactly 6 force unknowns, so `λ` is **unique** — no SOCP,
but the resulting `λ` can still violate the friction cone. That check is mandatory (Stage 2, G2.2);
if it fires, reduce `vx` or lengthen `T`.

**Note the discontinuity:** contact state changes at touchdown/liftoff, so `r` jumps 12 ↔ 18 and
`Jc_j` changes block structure. Window out ±2 samples around every transition — impact makes the
rigid-contact assumption briefly false, and foot rubber compliance makes it worse on hardware.

**Duration 40 s @ 100 Hz → 4000 samples × 12 rows = 48 000 equations.**

### 7.6 M3 — jump  (flight: nc = 0, r = 18)

The clean channel: no torque, no friction, no `K_t` in the base rows. Flight cannot be generated by
prescribing base motion — the base is *determined* by the joints through momentum conservation. Solve
the underactuated hybrid-dynamics problem instead: prescribe **joint accelerations**, get base
acceleration and `τ` from the square 18×18 system `M a + n = Sᵀτ`:

```
a_base = −M[:6,:6]⁻¹ ( M[:6,6:] · a_joint + n[:6] )        # 6 unactuated rows
tau    =  M[6:,:] · a + n[6:]                              # 12 actuated rows
```

Then integrate with `pin.integrate(model, q, v*dt)` — **never** `q += v*dt`, the quaternion block
will drift off the manifold. Use RK4 or `dt ≤ 1 ms`, and renormalize the quaternion each step.

Flail profile during flight: sinusoidal leg swings at ~4–6 Hz, opposite phase front/rear, amplitude
~0.3 rad on thigh and calf. That is what makes the angular rows informative — the trunk must
counter-rotate by exactly the amount the leg inertias dictate.

```
takeoff v_z = 1.5 m/s  ->  flight time 2·1.5/9.81 ≈ 0.31 s  ->  ~31 samples/hop @ 100 Hz
20 hops -> ~620 flight samples × 18 rows = 11 000 equations
```

Log at 500 Hz for jumps and decimate — 31 samples per hop is thin, and flight is the one regime
worth oversampling. Generate takeoff/landing (nc=4) too, but **flight is the payload**; the stance
phases of a jump are just another 4-feet sample.

### 7.7 Feasibility screens — run before anything reaches `sim.py`

| Screen | Threshold | If it fires |
|---|---|---|
| joint limits | 5% margin inside URDF limits | reduce amplitudes |
| joint velocity | 50% of URDF limit (30.1 / 15.70 rad/s) | lower frequencies |
| knee singularity | `cond(Jc_j) < 100`, calf angle ≥ 0.15 rad off `−0.83776` | raise the crouch |
| IK convergence | residual < 1e-10, ≤ 10 iterations | pose unreachable — shrink the wobble |
| constraint identity | `‖Jc·a + J̇c·v − p̈_c‖ < 1e-10` | **bug**, not a tuning issue (§7.2) |
| torque limits | 23.7 / 23.7 / 45.43 N·m | detune — checked post-hoc in Stage 2 |
| friction cone | Stage 2 SOCP feasibility | detune `vx`, raise `μ`, or reject the sample |

Do **not** add a separate static "CoM inside support polygon" check. The SOCP's `λ_z ≥ f_min` plus
the cone is the correct *dynamic* condition and subsumes it; a static polygon test would reject valid
dynamic motions and accept invalid ones.

### 7.8 Sample budget and splits

| Motion | Duration | Samples | `r` | Equations |
|---|---:|---:|---:|---:|
| M1 wobble | 60 s | 6 000 | 6 | 36 000 |
| M2 trot, `vx ∈ {0, ±0.3}` | 40 s | 4 000 | 12 | 48 000 |
| M3 jump, 20 hops (flight only) | ~6 s | ~620 | 18 | ~11 000 |
| **total** | | **~10 600** | | **~95 000** |

~10⁴ samples at 100 Hz, matching the paper's `n_s = 10⁴`.

Split by **entire motion type**, never randomly — random holdout is nearly free to fit and tells you
almost nothing:

| Role | Motions | Mirrors |
|---|---|---|
| in-distribution | wobble + trot `vx ∈ {0, ±0.3}` | paper's training set |
| held-out | trot `vx = ±0.5` — same structure, unseen velocity | paper's "OOD Trot" (0.6470 N·m) |
| out-of-distribution | jump | paper's "OOD Task (Jump)" (0.7148 N·m) |

### 7.9 Gates

- **G7.0** Frame/convention check (§7.0). `‖v_b − v_b^fd‖ < 1e-6`, where `v_b^fd` comes from
  `pin.difference(model, q(t), q(t+h))/h`. Catches Trap 2 (unrotated `ṗ_base`) and Trap 3 (Euler
  rates packed as `ω`). Repeat for `a` against a central difference of `v`.
- **G7.1** Constraint identity `‖Jc·a + J̇c·v − p̈_c‖ < 1e-10` on every sample of M1 and M2.
- **G7.2** All §7.7 screens pass on the shipped trajectories, and each screen demonstrably fires on a
  deliberately over-aggressive variant.
- **G7.3** M3 flight conserves angular momentum about the CoM to `< 1e-8` over each flight phase —
  an independent check on the integrator that does not go through the regressor.
- **G7.4** Contact mask transitions match the commanded gait schedule exactly; transition windowing
  removes the expected number of samples.

---

## 8. Gates

| # | Gate | Test |
|---|---|---|
| G0.1 | trunk fixture matches independent parallel-axis computation | T2 |
| G0.1b | total mass 16.087 kg; `nq=19, nv=18, njoints=14` | T2b, T1 |
| G0.2 | `φ` round-trip exact to 1e-12 | T1 |
| G0.3 | `Y φ = rnea`, regressor is `(18,130)` | **T3** |
| G1.1 | `Nᵀ Jcᵀ = 0`, `N` orthonormal, `N.shape[1] == 18 − rank(Jc)` | T4, T4c |
| G1.2 | `N` frame-invariant | T4b |
| G1.3 | rank detected numerically, not assumed | T6 |
| G7.0 | `v_b`/`a_b` match finite differences — frame + Euler-rate conventions | §7.9 |
| G7.1 | constraint identity on every wobble/trot sample | §7.9 |
| G7.2 | feasibility screens pass, and each fires when it should | §7.9 |
| G7.3 | flight conserves angular momentum to 1e-8 | §7.9 |
| G7.4 | contact mask matches the commanded gait schedule | §7.9 |

`pytest -q` green on all of the above is the handoff point for the Stage 2 spec.

**Build order within this spec:** `paths.py` → `model.py` (+T1,T2,T3) → `regressor.py` (+T4,T6) →
`traj.py` (+G7.x). `traj.py` depends on `regressor.py` for `Jc`, but on nothing from Stage 2.

---

## 9. Still open

1. **Trunk mass constraint convention** — pin to the lumped 7.277-style value, or redefine as
   `m_total − Σ m_leg`? Only matters once identification starts; not blocking validation.
2. **Leg freeze values** — URDF for now, pending the leg-pipeline audit
   (`sysid_comparison_urdf_sim_real_go1sim.md` shows off-diagonal leg inertias off by 1–2+ orders of
   magnitude; `Ixx_hip` at 303× is in the *identifiable* subset, so something is genuinely wrong).
