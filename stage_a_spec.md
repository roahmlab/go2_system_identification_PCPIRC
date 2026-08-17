# Stage A spec — model layout + projection ground-truth test

Implements milestones 1–2 of `go2_trunk_sysid_plan.md`. Sim backend: **Pinocchio only**.
Gate: nothing downstream gets built until `test_projection.py` passes at machine precision.

---

## 0. Two corrections to the plan, found in the URDF

Both were verified directly against `RAPTOR_sysid_go1/Robots/unitree-go2/go2_description.urdf`.

### 0.1 Pinocchio's trunk is NOT the URDF `base` link

The plan (§6.2) says to sanity-check `phi0[0:10]` against "mass 6.921 kg, CoM (0.021112, 0, −0.005366)".
That check will fail, and "fixing" it would inject a real 5% error.

`go2_description.urdf` has 35 joints — 12 revolute, 23 **fixed**. Pinocchio fuses fixed joints and
shifts the child inertia into the parent by parallel axis. Four of those fixed children are attached
to `base`:

| fixed joint | child | mass | xyz in base frame |
|---|---|---|---|
| `FL_hip_rotor_joint` | `FL_hip_rotor` | 0.089 | ( 0.1122,  0.0468, 0) |
| `FR_hip_rotor_joint` | `FR_hip_rotor` | 0.089 | ( 0.1122, −0.0468, 0) |
| `RL_hip_rotor_joint` | `RL_hip_rotor` | 0.089 | (−0.1122,  0.0468, 0) |
| `RR_hip_rotor_joint` | `RR_hip_rotor` | 0.089 | (−0.1122, −0.0468, 0) |

(`imu`, `radar`, `front_camera` are also fixed to base but carry zero mass.)

So the trunk body Pinocchio actually builds is **base + 4 hip rotors**:

```
mass  7.277 kg          (not 6.921 — +0.356 kg, +5.1%)
com   (0.0200792, 0.0, -0.0051035)
Izz   0.11265 about lumped CoM   (not 0.107 — +5.3%)
```

The same lumping applies down the legs (this is why the KB's calf block is `calf + foot = 0.194 kg`):

| Pinocchio body | absorbs | lumped mass |
|---|---|---|
| trunk | `base` + 4×`hip_rotor` | 7.277 |
| `*_hip` | + `*_thigh_rotor` | 0.767 |
| `*_thigh` | + `*_calf_rotor` | 1.241 |
| `*_calf` | + `*_foot` + `*_calflower*` | 0.194 |

Cross-check: `7.277 + 4×(0.767 + 1.241 + 0.194) = 16.085` ≈ URDF link-mass total `16.087`. ✓

**Consequences.**

1. The milestone-1 fixture is the lumped value, not the URDF text (§2 below).
2. §9.2's `phi_t[0] == M_SCALE_TRUNK` must be pinned to the **lumped** trunk mass. A scale
   reading of a bare trunk assembly is not that number — you must add the four hip rotors, or
   redefine the constraint in terms of `total robot mass − Σ identified leg masses`. Decide this
   explicitly and write it in the docstring; it is a silent 0.36 kg bias otherwise.
3. Every "URDF vs identified" comparison table must state which convention it uses. The existing
   `sysid_comparison_urdf_sim_real_go1sim.md` quotes calf `m = 0.194` (lumped) but hip `m = 0.678`
   (unlumped — the lumped value is 0.767). Worth re-checking before that table is reused.

### 0.2 URDF rotor inertia ≠ reflected armature inertia — they differ by exactly N²

Easy to double-count, so state it once and be done:

```
FL_hip_rotor  ixx = 1.11842e-4 kg·m²      ← URDF, unreflected physical rotor body
KB I_armature = 4.330e-3 kg·m²            ← reflected:  I_rotor × N²,  N = 6.22
                                             4.330e-3 / 1.11842e-4 = 38.72 = 6.22²   ✓
```

These are **two different, non-overlapping effects**:

- The URDF rotor link is the rotor treated as a *rigidly attached mass* — it contributes to the
  parent link's lumped inertia and is already inside `Y φ`.
- The plan's §1.3 block `I_r,j · N_j² · v̇_j` is the rotor *spinning through the gearbox*. It is
  **not** in the URDF and must be identified.

So the 12 rotor-inertia columns in `extra_blocks` are correct and do not double-count — but their
expected magnitude is ~4.3e-3 (hip/thigh) and ~1.6e-2 (calf, per the KB's N ≈ 11.93), not ~1e-4.
Use those as the prior / sanity band. An identified value near 1e-4 means you fitted the wrong thing.

### 0.3 On the "legs are known" premise

`sysid_comparison_urdf_sim_real_go1sim.md` reports every off-diagonal leg inertia off by 1–2+ orders
of magnitude in sim *and* real. Part of that is expected — the KB documents 13 of 30 leg params as
structurally unidentifiable from fixed-base joint torque, and an unidentified direction returns
whatever the regularizer left there. But `Ixx_hip` at 303× is in the *identifiable* subset, so
something is genuinely wrong.

**Therefore: `phi_nominal` in §8.2 `build_system` = URDF values, not the identified leg values**,
until the leg pipeline is re-audited. Make it an explicit argument with no default so the choice is
never implicit. Run the "all free" (166-param) config alongside, as the plan already recommends —
that comparison is the diagnostic for whether the legs absorbed trunk error.

Two questions to answer before trusting that table at all: were those numbers produced before or
after the Tikhonov ridge landed, and do the 17 *identifiable* directions look any better than the
13 dead ones?

---

## 1. Environment

Nothing needed is currently installed. `pinocchio 2.7.0` exists in the `hopscotch` env — too old.
Build fresh:

```bash
conda create -n go2sysid -c conda-forge python=3.11 \
    pinocchio">=3.0" cvxpy scipy numpy matplotlib pytest
conda activate go2sysid
python -c "import pinocchio as pin; print(pin.__version__)"   # expect 3.x
```

`cvxpy` ships Clarabel/SCS, both of which handle the §9.2 SDP. MOSEK only if the SDP gets slow —
it needs a license and is not needed for Stage A.

No mesh files are required: `buildModelFromUrdf` ignores `<visual>`/`<collision>`, so the
`package://go2_description/dae/...` paths never resolve. Only `buildGeomFromUrdf` would need them.

---

## 2. `model.py`

Single module, no classes. Everything here is pure and side-effect free except `apply_phi`.

```python
URDF = ".../RAPTOR_sysid_go1/Robots/unitree-go2/go2_description.urdf"
FOOT_FRAMES = ["FL_foot", "FR_foot", "RL_foot", "RR_foot"]
```

**`load_model() -> (model, data, foot_ids)`**
- `pin.buildModelFromUrdf(URDF, pin.JointModelFreeFlyer())`
- assert `model.nq == 19`, `model.nv == 18`, `model.njoints == 14`  (universe + freeflyer + 12)
- resolve foot frame ids; assert each `getFrameId` returns `< model.nframes` (Pinocchio returns
  `nframes` for a miss instead of raising — a silent failure worth guarding)

**`phi_from_model(model) -> (130,)`**
- `np.concatenate([model.inertias[i].toDynamicParameters() for i in range(1, model.njoints)])`
- body `i` occupies `phi[10*(i-1) : 10*i]`; trunk is `phi[0:10]`

**`apply_phi(model, phi) -> None`** — inverse of the above, via `pin.Inertia.FromDynamicParameters`.

**`S_T -> (18,12)`** — module constant, `S_T[6:, :] = I₁₂`.

**`body_slice(model, joint_name) -> slice`** — name → 10-wide slice into φ. Used by tests and by
the freeze-legs logic so no one hand-writes `10:` again.

### Ordering hazard

Pinocchio's `toDynamicParameters` order is

```
[ m, h_x, h_y, h_z, I_xx, I_xy, I_yy, I_xz, I_yz, I_zz ]
```

— note `I_yy` sits at index 6 and `I_xz` at index 7. The Khorshidi paper uses
`Ixx, Ixy, Ixz, Iyy, Iyz, Izz`. Transcribing the paper's order swaps `I_yy ↔ I_xz`, which for the
trunk is `0.106` vs `0.0023` — a factor of 47, and it will **not** show up as a residual blow-up in
a symmetric test case. This is why the fixture below is written out explicitly.

---

## 3. `tests/test_ordering.py`

Three tests. All are cheap and all three have caught real bugs in this class of pipeline.

**T1 — round trip.** `apply_phi(model, phi_from_model(model))` then re-extract; `assert_allclose`
at `rtol=0, atol=1e-12`. Catches ordering bugs only if `apply_phi` and `phi_from_model` disagree,
so it is necessary but not sufficient — hence T2.

**T2 — trunk fixture.** Hard-code the independently computed lumped value (derived by parallel-axis
from the URDF, *not* by asking Pinocchio, so it is a real cross-check):

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
value is `diag ≈ (0.02572, 0.10295, 0.11265)`; if you see those instead, you have a
`I_origin = I_com + m(cᵀc·I₃ − ccᵀ)` shift missing somewhere.

**T3 — regressor consistency.** The single cheapest check that gravity, sign, and ordering all agree:

```python
q  = pin.randomConfiguration(model)      # normalize the quaternion afterwards
v, a = np.random.randn(18), np.random.randn(18)
pin.computeJointTorqueRegressor(model, data, q, v, a)
assert_allclose(data.jointTorqueRegressor @ phi0,
                pin.rnea(model, data, q, v, a), rtol=1e-9)
```

If T3 fails, stop — nothing else in the pipeline can be right.

---

## 4. `sim.py` — synthetic data, two levels

### Level 1: inverse-dynamics construction (build this first)

No contact solver, no integration. Purpose is solely to manufacture a `(q, v, a, τ, λ, contact)`
tuple that satisfies the floating-base EoM *exactly*, so the projection test has ground truth.

```
given q, v, a, contact mask:
    rhs = pin.rnea(model, data, q, v, a)              # (18,)  = M a + n
    Jc  = stack of getFrameJacobian(..., LOCAL_WORLD_ALIGNED)[:3,:] over feet in contact
    solve  [S_T | Jc.T] @ [tau; lam] = rhs            # 18 eqs, 12+3nc unknowns
        -> least-norm via np.linalg.lstsq  (underdetermined for nc >= 2)
    return tau, lam
```

Notes:
- Sample `q` around a standing pose, not `pin.randomConfiguration` — random configurations put feet
  through the trunk and give a garbage `Jc`. Normalize the quaternion block regardless.
- `τ` may exceed hardware limits. Irrelevant here; do not clip (clipping breaks the identity).
- For `nc = 4` the system is 18×24 and always solvable. For `nc = 0` it is 18×12 and generally
  **not** — the base rows can only be satisfied if `a` is a genuine free-flight acceleration. Handle
  flight by generating `a` from `pin.aba` instead (see below).

### Level 2: constrained forward rollout (for Stage B onward)

`pin.constraintDynamics` with one `RigidConstraintModel` per foot,
`pin.ContactType.CONTACT_3D`, reference frame `LOCAL_WORLD_ALIGNED`, at the foot frame placement.
Call `pin.initConstraintDynamics(model, data, contact_models)` once. Integrate with
`pin.integrate(model, q, v*dt)` — never `q += v*dt`.

Use Level 2 to produce the actual wobble / crawl / trot trajectories of §4.2, plus flight phases
(empty constraint set → `pin.aba`). Level 1 remains the unit-test generator.

Add noise **only** as an opt-in argument, default off. Milestone 2 must run noise-free.

---

## 5. `tests/test_projection.py`

**T4 — the projector annihilates contacts.** For each contact count `nc ∈ {1,2,3,4}`:

```python
assert np.linalg.norm(N.T @ Jc.T) < 1e-10
assert N.shape[1] == 18 - nc*3          # only for well-conditioned q; see below
assert_allclose(N.T @ N, np.eye(N.shape[1]), atol=1e-10)   # orthonormality
```

**T5 — the milestone-2 gate.** Over ≥ 200 Level-1 samples spanning all four contact counts:

```python
res = N.T @ (Y @ phi_true - S_T @ tau_true)
assert np.linalg.norm(res) < 1e-8
```

**T6 — rank is detected, not assumed.** Construct a near-singular case (fully extended knee,
`q_calf → limit`) and assert the SVD-based `r` grows and the sample is flagged for rejection.
The plan is explicit that `3nc` must never be hard-coded; this test is what enforces it.

### Implementation notes for `regressor_and_projection`

- `getFrameJacobian` is a **getter**. It silently returns stale data unless
  `pin.computeJointJacobians(model, data, q)` **and** `pin.updateFramePlacements(model, data)` ran
  first. Prefer `pin.computeFrameJacobian(model, data, q, fid, ref)` while building Stage A — it
  does the kinematics internally and cannot go stale. Swap to the batched getter later, once
  there's a test that would catch the staleness.
- Reference frame choice does not affect `N` (plan §2.4 — it rotates each 3-row block by an
  invertible `R_i`, and `null(R·Jc) = null(Jc)`). Pick `LOCAL_WORLD_ALIGNED` and be consistent.
  Add a test asserting `N` from `LOCAL` and from `LOCAL_WORLD_ALIGNED` span the same subspace
  (`norm(N₁N₁ᵀ − N₂N₂ᵀ) < 1e-10`) — it documents the claim and costs nothing.
- Take `N = Vt[r:, :].T` from `np.linalg.svd(Jc, full_matrices=True)`, with
  `r = (s > 1e-6*s[0]).sum()`. Return `r` alongside `N`; §10.1's RMSE is rank-normalized and needs it.

---

## 6. Gates

| # | Gate | Test |
|---|---|---|
| 1 | model loads, `nq/nv/njoints` correct | T1 |
| 1 | trunk fixture matches independent parallel-axis computation | T2 |
| 1 | `Y φ = rnea` | T3 |
| 2 | `Nᵀ Jcᵀ = 0`, `N` orthonormal, rank correct | T4 |
| 2 | **projected residual < 1e-8 with true φ** | T5 |
| 2 | rank deficiency detected numerically, not assumed | T6 |

Only after T5 passes: Stage B (contact detection) and Stage C (design matrix).

---

## 7. Open decisions

1. **Trunk mass constraint (§0.1 pt 2).** Lumped-with-rotors, or redefine as
   `m_total − Σ m_leg`? Affects §9.2's equality constraint directly.
2. **Leg freeze values (§0.3).** Confirmed as URDF for now — revisit after the leg-pipeline audit.
3. **Where does this code live?** `go2_sysid/` standalone Python package, or under
   `RAPTOR/Examples/Unitree_Go2/python/`? Standalone is cleaner — this method shares no code with
   the C++ IPOPT pipeline — but then the URDF path is a cross-repo reference. Suggest standalone
   with the URDF path in a single `paths.py` constant.
