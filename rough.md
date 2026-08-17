That's my notation's fault — I wrote x(t) for a generic channel and then a table with six rows, which reads as if x is the x-coordinate. Let me fix that and walk the whole thing through.

There are six independent scalar functions, not one

s_x(t)   surge      translation along world x
s_y(t)   sway       translation along world y
s_z(t)   heave      translation along world z
φ(t)     roll       rotation about x
θ(t)     pitch      rotation about y
ψ(t)     yaw        rotation about z

Each one is its own multisine. The x(t) = x0 + Σ_k A_k sin(2π k f0 t + φ_k) formula is a template applied six times with different harmonic sets:

s_x(t) = 0.020·sin(2π·0.3·t + φ₁) + 0.020·sin(2π·0.7·t + φ₂)      # k = 3, 7
φ(t)   = 0.075·sin(2π·0.2·t + φ₃) + 0.075·sin(2π·1.3·t + φ₄)      # k = 2, 13
...

Two different kinds of summing, and only one of them is real:
- Within a channel, harmonics are summed → that's the multisine, giving one scalar.
- Across channels, nothing is summed. Six separate numbers that jointly specify one pose.

Yes — all six move simultaneously

Deliberately. Sequential single-axis sweeps would be a mistake here, because off-diagonal inertia terms only appear under coupled rotation. I_xy enters the angular dynamics through terms like ω_x·ω_y and ω̇_y — roll alone leaves it invisible. If you want I_xy, I_xz, I_yz identifiable at all, you must roll and pitch and yaw at once.

The distinct frequencies are what keeps that legible: each DoF has its own spectral signature, so when you later look at where residual lives, you can tell which channel it came from. That's Swevers' whole reason for the finite-Fourier structure.

No — these are world-frame, not base-relative

This is the crux of your confusion, and there are genuinely two frames doing two different jobs:

┌──────────────────────────┬───────┬──────────────────────────────────────────────┐
│         Quantity         │ Frame │                     Role                     │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ s_x, s_y, s_z, φ, θ, ψ   │ world │ the trunk's pose in the world. This is T_wb. │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ foot positions p_i^world │ world │ frozen at t=0, never change                  │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ p_i^base(t)              │ base  │ the IK target                                │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ q_joint                  │ —     │ the IK output                                │
└──────────────────────────┴───────┴──────────────────────────────────────────────┘

Base-relative trunk coordinates would be meaningless — the base's pose relative to itself is always identity.

The pipeline at one timestep

Setup at t = 0: trunk at world (0, 0, 0.30), identity orientation. Measure where the feet are and freeze them forever:

p_FL^world = ( 0.1934,  0.1420, 0)      p_FR^world = ( 0.1934, −0.1420, 0)
p_RL^world = (−0.1934,  0.1420, 0)      p_RR^world = (−0.1934, −0.1420, 0)

Now at some time t, say the six channels evaluate to s = (0.031, −0.018, 0.022), (φ,θ,ψ) = (0.082, −0.114, 0.047):

1. Build the trunk pose — pure evaluation, no solving:
p_base(t) = (0.031, −0.018, 0.322)
R_base(t) = Rz(0.047)·Ry(−0.114)·Rx(0.082)
These 7 numbers (position + quaternion) are q[0:7]. Done, no computation.

2. Ask where each foot is, as seen from the base:
p_i^base(t) = R_base(t)ᵀ · ( p_i^world − p_base(t) )
The feet haven't moved in the world. But because the trunk moved under them, they've moved in the base frame. One matrix-vector product per foot. Still no solving.

3. Solve the IK — the only real solve:
find q_i (3 angles)  such that  f_i(q_i) = p_i^base(t)
Four independent 3×3 Newton solves. Output: q[7:19], the 12 joint angles.

4. v_j, a_j from the constraint equations. 5. RNEA on the complete (q, v, a).

The mechanics, with numbers I can actually verify

Take the simplest case — pure surge, s_x = +0.04, everything else zero. FL foot in base frame:

p_FL^base = I·( (0.1934, 0.1420, 0) − (0.04, 0, 0.30) ) = (0.1534, 0.1420, −0.30)

It was at x = 0.1934; now it reads 0.1534. The foot didn't move — the trunk moved forward, so from the trunk's point of view the foot slid backward. The leg must reach back. Thigh joint sits at (0.1934, 0.142, 0), so thigh→foot is (−0.04, 0, −0.30), length 0.3027. Law of cosines with L1 = L2 = 0.213:

cos γ = (0.0916 − 0.0908)/0.0908 = 0.0088   →   γ = −1.562 rad     (was −1.580)

The knee straightens by about 1°. Add roll/pitch/yaw and all four legs do this simultaneously in different directions — front legs extending while rear tuck, one side lifting while the other compresses. That's the wobble.

What's fixed, what moves

feet in WORLD      ────────── frozen at t=0, forever
trunk in WORLD     ~~~~~~~~~~ six multisines, prescribed
feet in BASE       ~~~~~~~~~~ moving (because the trunk moves under them)
joint angles       ~~~~~~~~~~ solved, one 3×3 per leg per timestep

And yes — your endpoint is right: you come out of this with all 12 joint positions plus v and a, and that is what RNEA consumes.

Let me record the notation fix, since §7.4 of the spec has the same ambiguous xThat's my notation's fault — I wrote x(t) for a generic channel and then a table with six rows, which reads as if x is the x-coordinate. Let me fix that and walk the whole thing through.

There are six independent scalar functions, not one

s_x(t)   surge      translation along world x
s_y(t)   sway       translation along world y
s_z(t)   heave      translation along world z
φ(t)     roll       rotation about x
θ(t)     pitch      rotation about y
ψ(t)     yaw        rotation about z

Each one is its own multisine. The x(t) = x0 + Σ_k A_k sin(2π k f0 t + φ_k) formula is a template applied six times with different harmonic sets:

s_x(t) = 0.020·sin(2π·0.3·t + φ₁) + 0.020·sin(2π·0.7·t + φ₂)      # k = 3, 7
φ(t)   = 0.075·sin(2π·0.2·t + φ₃) + 0.075·sin(2π·1.3·t + φ₄)      # k = 2, 13
...

Two different kinds of summing, and only one of them is real:
- Within a channel, harmonics are summed → that's the multisine, giving one scalar.
- Across channels, nothing is summed. Six separate numbers that jointly specify one pose.

Yes — all six move simultaneously

Deliberately. Sequential single-axis sweeps would be a mistake here, because off-diagonal inertia terms only appear under coupled rotation. I_xy enters the angular dynamics through terms like ω_x·ω_y and ω̇_y — roll alone leaves it invisible. If you want I_xy, I_xz, I_yz identifiable at all, you must roll and pitch and yaw at once.

The distinct frequencies are what keeps that legible: each DoF has its own spectral signature, so when you later look at where residual lives, you can tell which channel it came from. That's Swevers' whole reason for the finite-Fourier structure.

No — these are world-frame, not base-relative

This is the crux of your confusion, and there are genuinely two frames doing two different jobs:

┌──────────────────────────┬───────┬──────────────────────────────────────────────┐
│         Quantity         │ Frame │                     Role                     │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ s_x, s_y, s_z, φ, θ, ψ   │ world │ the trunk's pose in the world. This is T_wb. │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ foot positions p_i^world │ world │ frozen at t=0, never change                  │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ p_i^base(t)              │ base  │ the IK target                                │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ q_joint                  │ —     │ the IK output                                │
└──────────────────────────┴───────┴──────────────────────────────────────────────┘

Base-relative trunk coordinates would be meaningless — the base's pose relative to itself is always identity.

The pipeline at one timestep

Setup at t = 0: trunk at world (0, 0, 0.30), identity orientation. Measure where the feet are and freeze them forever:

p_FL^world = ( 0.1934,  0.1420, 0)      p_FR^world = ( 0.1934, −0.1420, 0)
p_RL^world = (−0.1934,  0.1420, 0)      p_RR^world = (−0.1934, −0.1420, 0)

Now at some time t, say the six channels evaluate to s = (0.031, −0.018, 0.022), (φ,θ,ψ) = (0.082, −0.114, 0.047):

1. Build the trunk pose — pure evaluation, no solving:
p_base(t) = (0.031, −0.018, 0.322)
R_base(t) = Rz(0.047)·Ry(−0.114)·Rx(0.082)
These 7 numbers (position + quaternion) are q[0:7]. Done, no computation.

2. Ask where each foot is, as seen from the base:
p_i^base(t) = R_base(t)ᵀ · ( p_i^world − p_base(t) )
The feet haven't moved in the world. But because the trunk moved under them, they've moved in the base frame. One matrix-vector product per foot. Still no solving.

3. Solve the IK — the only real solve:
find q_i (3 angles)  such that  f_i(q_i) = p_i^base(t)
Four independent 3×3 Newton solves. Output: q[7:19], the 12 joint angles.

4. v_j, a_j from the constraint equations. 5. RNEA on the complete (q, v, a).

The mechanics, with numbers I can actually verify

Take the simplest case — pure surge, s_x = +0.04, everything else zero. FL foot in base frame:

p_FL^base = I·( (0.1934, 0.1420, 0) − (0.04, 0, 0.30) ) = (0.1534, 0.1420, −0.30)

It was at x = 0.1934; now it reads 0.1534. The foot didn't move — the trunk moved forward, so from the trunk's point of view the foot slid backward. The leg must reach back. Thigh joint sits at (0.1934, 0.142, 0), so thigh→foot is (−0.04, 0, −0.30), length 0.3027. Law of cosines with L1 = L2 = 0.213:

cos γ = (0.0916 − 0.0908)/0.0908 = 0.0088   →   γ = −1.562 rad     (was −1.580)

The knee straightens by about 1°. Add roll/pitch/yaw and all four legs do this simultaneously in different directions — front legs extending while rear tuck, one side lifting while the other compresses. That's the wobble.

What's fixed, what moves

feet in WORLD      ────────── frozen at t=0, forever
trunk in WORLD     ~~~~~~~~~~ six multisines, prescribed
feet in BASE       ~~~~~~~~~~ moving (because the trunk moves under them)
joint angles       ~~~~~~~~~~ solved, one 3×3 per leg per timestep

And yes — your endpoint is right: you come out of this with all 12 joint positions plus v and a, and that is what RNEA consumes.

Let me record the notation fix, since §7.4 of the spec has the same ambiguous xThat's my notation's fault — I wrote x(t) for a generic channel and then a table with six rows, which reads as if x is the x-coordinate. Let me fix that and walk the whole thing through.

There are six independent scalar functions, not one

s_x(t)   surge      translation along world x
s_y(t)   sway       translation along world y
s_z(t)   heave      translation along world z
φ(t)     roll       rotation about x
θ(t)     pitch      rotation about y
ψ(t)     yaw        rotation about z

Each one is its own multisine. The x(t) = x0 + Σ_k A_k sin(2π k f0 t + φ_k) formula is a template applied six times with different harmonic sets:

s_x(t) = 0.020·sin(2π·0.3·t + φ₁) + 0.020·sin(2π·0.7·t + φ₂)      # k = 3, 7
φ(t)   = 0.075·sin(2π·0.2·t + φ₃) + 0.075·sin(2π·1.3·t + φ₄)      # k = 2, 13
...

Two different kinds of summing, and only one of them is real:
- Within a channel, harmonics are summed → that's the multisine, giving one scalar.
- Across channels, nothing is summed. Six separate numbers that jointly specify one pose.

Yes — all six move simultaneously

Deliberately. Sequential single-axis sweeps would be a mistake here, because off-diagonal inertia terms only appear under coupled rotation. I_xy enters the angular dynamics through terms like ω_x·ω_y and ω̇_y — roll alone leaves it invisible. If you want I_xy, I_xz, I_yz identifiable at all, you must roll and pitch and yaw at once.

The distinct frequencies are what keeps that legible: each DoF has its own spectral signature, so when you later look at where residual lives, you can tell which channel it came from. That's Swevers' whole reason for the finite-Fourier structure.

No — these are world-frame, not base-relative

This is the crux of your confusion, and there are genuinely two frames doing two different jobs:

┌──────────────────────────┬───────┬──────────────────────────────────────────────┐
│         Quantity         │ Frame │                     Role                     │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ s_x, s_y, s_z, φ, θ, ψ   │ world │ the trunk's pose in the world. This is T_wb. │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ foot positions p_i^world │ world │ frozen at t=0, never change                  │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ p_i^base(t)              │ base  │ the IK target                                │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ q_joint                  │ —     │ the IK output                                │
└──────────────────────────┴───────┴──────────────────────────────────────────────┘

Base-relative trunk coordinates would be meaningless — the base's pose relative to itself is always identity.

The pipeline at one timestep

Setup at t = 0: trunk at world (0, 0, 0.30), identity orientation. Measure where the feet are and freeze them forever:

p_FL^world = ( 0.1934,  0.1420, 0)      p_FR^world = ( 0.1934, −0.1420, 0)
p_RL^world = (−0.1934,  0.1420, 0)      p_RR^world = (−0.1934, −0.1420, 0)

Now at some time t, say the six channels evaluate to s = (0.031, −0.018, 0.022), (φ,θ,ψ) = (0.082, −0.114, 0.047):

1. Build the trunk pose — pure evaluation, no solving:
p_base(t) = (0.031, −0.018, 0.322)
R_base(t) = Rz(0.047)·Ry(−0.114)·Rx(0.082)
These 7 numbers (position + quaternion) are q[0:7]. Done, no computation.

2. Ask where each foot is, as seen from the base:
p_i^base(t) = R_base(t)ᵀ · ( p_i^world − p_base(t) )
The feet haven't moved in the world. But because the trunk moved under them, they've moved in the base frame. One matrix-vector product per foot. Still no solving.

3. Solve the IK — the only real solve:
find q_i (3 angles)  such that  f_i(q_i) = p_i^base(t)
Four independent 3×3 Newton solves. Output: q[7:19], the 12 joint angles.

4. v_j, a_j from the constraint equations. 5. RNEA on the complete (q, v, a).

The mechanics, with numbers I can actually verify

Take the simplest case — pure surge, s_x = +0.04, everything else zero. FL foot in base frame:

p_FL^base = I·( (0.1934, 0.1420, 0) − (0.04, 0, 0.30) ) = (0.1534, 0.1420, −0.30)

It was at x = 0.1934; now it reads 0.1534. The foot didn't move — the trunk moved forward, so from the trunk's point of view the foot slid backward. The leg must reach back. Thigh joint sits at (0.1934, 0.142, 0), so thigh→foot is (−0.04, 0, −0.30), length 0.3027. Law of cosines with L1 = L2 = 0.213:

cos γ = (0.0916 − 0.0908)/0.0908 = 0.0088   →   γ = −1.562 rad     (was −1.580)

The knee straightens by about 1°. Add roll/pitch/yaw and all four legs do this simultaneously in different directions — front legs extending while rear tuck, one side lifting while the other compresses. That's the wobble.

What's fixed, what moves

feet in WORLD      ────────── frozen at t=0, forever
trunk in WORLD     ~~~~~~~~~~ six multisines, prescribed
feet in BASE       ~~~~~~~~~~ moving (because the trunk moves under them)
joint angles       ~~~~~~~~~~ solved, one 3×3 per leg per timestep

And yes — your endpoint is right: you come out of this with all 12 joint positions plus v and a, and that is what RNEA consumes.

Let me record the notation fix, since §7.4 of the spec has the same ambiguous xThat's my notation's fault — I wrote x(t) for a generic channel and then a table with six rows, which reads as if x is the x-coordinate. Let me fix that and walk the whole thing through.

There are six independent scalar functions, not one

s_x(t)   surge      translation along world x
s_y(t)   sway       translation along world y
s_z(t)   heave      translation along world z
φ(t)     roll       rotation about x
θ(t)     pitch      rotation about y
ψ(t)     yaw        rotation about z

Each one is its own multisine. The x(t) = x0 + Σ_k A_k sin(2π k f0 t + φ_k) formula is a template applied six times with different harmonic sets:

s_x(t) = 0.020·sin(2π·0.3·t + φ₁) + 0.020·sin(2π·0.7·t + φ₂)      # k = 3, 7
φ(t)   = 0.075·sin(2π·0.2·t + φ₃) + 0.075·sin(2π·1.3·t + φ₄)      # k = 2, 13
...

Two different kinds of summing, and only one of them is real:
- Within a channel, harmonics are summed → that's the multisine, giving one scalar.
- Across channels, nothing is summed. Six separate numbers that jointly specify one pose.

Yes — all six move simultaneously

Deliberately. Sequential single-axis sweeps would be a mistake here, because off-diagonal inertia terms only appear under coupled rotation. I_xy enters the angular dynamics through terms like ω_x·ω_y and ω̇_y — roll alone leaves it invisible. If you want I_xy, I_xz, I_yz identifiable at all, you must roll and pitch and yaw at once.

The distinct frequencies are what keeps that legible: each DoF has its own spectral signature, so when you later look at where residual lives, you can tell which channel it came from. That's Swevers' whole reason for the finite-Fourier structure.

No — these are world-frame, not base-relative

This is the crux of your confusion, and there are genuinely two frames doing two different jobs:

┌──────────────────────────┬───────┬──────────────────────────────────────────────┐
│         Quantity         │ Frame │                     Role                     │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ s_x, s_y, s_z, φ, θ, ψ   │ world │ the trunk's pose in the world. This is T_wb. │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ foot positions p_i^world │ world │ frozen at t=0, never change                  │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ p_i^base(t)              │ base  │ the IK target                                │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ q_joint                  │ —     │ the IK output                                │
└──────────────────────────┴───────┴──────────────────────────────────────────────┘

Base-relative trunk coordinates would be meaningless — the base's pose relative to itself is always identity.

The pipeline at one timestep

Setup at t = 0: trunk at world (0, 0, 0.30), identity orientation. Measure where the feet are and freeze them forever:

p_FL^world = ( 0.1934,  0.1420, 0)      p_FR^world = ( 0.1934, −0.1420, 0)
p_RL^world = (−0.1934,  0.1420, 0)      p_RR^world = (−0.1934, −0.1420, 0)

Now at some time t, say the six channels evaluate to s = (0.031, −0.018, 0.022), (φ,θ,ψ) = (0.082, −0.114, 0.047):

1. Build the trunk pose — pure evaluation, no solving:
p_base(t) = (0.031, −0.018, 0.322)
R_base(t) = Rz(0.047)·Ry(−0.114)·Rx(0.082)
These 7 numbers (position + quaternion) are q[0:7]. Done, no computation.

2. Ask where each foot is, as seen from the base:
p_i^base(t) = R_base(t)ᵀ · ( p_i^world − p_base(t) )
The feet haven't moved in the world. But because the trunk moved under them, they've moved in the base frame. One matrix-vector product per foot. Still no solving.

3. Solve the IK — the only real solve:
find q_i (3 angles)  such that  f_i(q_i) = p_i^base(t)
Four independent 3×3 Newton solves. Output: q[7:19], the 12 joint angles.

4. v_j, a_j from the constraint equations. 5. RNEA on the complete (q, v, a).

The mechanics, with numbers I can actually verify

Take the simplest case — pure surge, s_x = +0.04, everything else zeThat's my notation's fault — I wrote x(t) for a generic channel and then a table with six rows, which reads as if x is the x-coordinate. Let me fix that and walk the whole thing through.

There are six independent scalar functions, not one

s_x(t)   surge      translation along world x
s_y(t)   sway       translation along world y
s_z(t)   heave      translation along world z
φ(t)     roll       rotation about x
θ(t)     pitch      rotation about y
ψ(t)     yaw        rotation about z

Each one is its own multisine. The x(t) = x0 + Σ_k A_k sin(2π k f0 t + φ_k) formula is a template applied six times with different harmonic sets:

s_x(t) = 0.020·sin(2π·0.3·t + φ₁) + 0.020·sin(2π·0.7·t + φ₂)      # k = 3, 7
φ(t)   = 0.075·sin(2π·0.2·t + φ₃) + 0.075·sin(2π·1.3·t + φ₄)      # k = 2, 13
...

Two different kinds of summing, and only one of them is real:
- Within a channel, harmonics are summed → that's the multisine, giving one scalar.
- Across channels, nothing is summed. Six separate numbers that jointly specify one pose.

Yes — all six move simultaneously

Deliberately. Sequential single-axis sweeps would be a mistake here, because off-diagonal inertia terms only appear under coupled rotation. I_xy enters the angular dynamics through terms like ω_x·ω_y and ω̇_y — roll alone leaves it invisible. If you want I_xy, I_xz, I_yz identifiable at all, you must roll and pitch and yaw at once.

The distinct frequencies are what keeps that legible: each DoF has its own spectral signature, so when you later look at where residual lives, you can tell which channel it came from. That's Swevers' whole reason for the finite-Fourier structure.

No — these are world-frame, not base-relative

This is the crux of your confusion, and there are genuinely two frames doing two different jobs:

┌──────────────────────────┬───────┬──────────────────────────────────────────────┐
│         Quantity         │ Frame │                     Role                     │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ s_x, s_y, s_z, φ, θ, ψ   │ world │ the trunk's pose in the world. This is T_wb. │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ foot positions p_i^world │ world │ frozen at t=0, never change                  │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ p_i^base(t)              │ base  │ the IK target                                │
├──────────────────────────┼───────┼──────────────────────────────────────────────┤
│ q_joint                  │ —     │ the IK output                                │
└──────────────────────────┴───────┴──────────────────────────────────────────────┘

Base-relative trunk coordinates would be meaningless — the base's pose relative to itself is always identity.

The pipeline at one timestep

Setup at t = 0: trunk at world (0, 0, 0.30), identity orientation. Measure where the feet are and freeze them forever:

p_FL^world = ( 0.1934,  0.1420, 0)      p_FR^world = ( 0.1934, −0.1420, 0)
p_RL^world = (−0.1934,  0.1420, 0)      p_RR^world = (−0.1934, −0.1420, 0)

Now at some time t, say the six channels evaluate to s = (0.031, −0.018, 0.022), (φ,θ,ψ) = (0.082, −0.114, 0.047):

1. Build the trunk pose — pure evaluation, no solving:
p_base(t) = (0.031, −0.018, 0.322)
R_base(t) = Rz(0.047)·Ry(−0.114)·Rx(0.082)
These 7 numbers (position + quaternion) are q[0:7]. Done, no computation.

2. Ask where each foot is, as seen from the base:
p_i^base(t) = R_base(t)ᵀ · ( p_i^world − p_base(t) )
The feet haven't moved in the world. But because the trunk moved under them, they've moved in the base frame. One matrix-vector product per foot. Still no solving.

3. Solve the IK — the only real solve:
find q_i (3 angles)  such that  f_i(q_i) = p_i^base(t)
Four independent 3×3 Newton solves. Output: q[7:19], the 12 joint angles.

4. v_j, a_j from the constraint equations. 5. RNEA on the complete (q, v, a).

The mechanics, with numbers I can actually verify

Take the simplest case — pure surge, s_x = +0.04, everything else zero. FL foot in base frame:

p_FL^base = I·( (0.1934, 0.1420, 0) − (0.04, 0, 0.30) ) = (0.1534, 0.1420, −0.30)

It was at x = 0.1934; now it reads 0.1534. The foot didn't move — the trunk moved forward, so from the trunk's point of view the foot slid backward. The leg must reach back. Thigh joint sits at (0.1934, 0.142, 0), so thigh→foot is (−0.04, 0, −0.30), length 0.3027. Law of cosines with L1 = L2 = 0.213:

cos γ = (0.0916 − 0.0908)/0.0908 = 0.0088   →   γ = −1.562 rad     (was −1.580)

The knee straightens by about 1°. Add roll/pitch/yaw and all four legs do this simultaneously in different directions — front legs extending while rear tuck, one side lifting while the other compresses. That's the wobble.

What's fixed, what moves

feet in WORLD      ────────── frozen at t=0, forever
trunk in WORLD     ~~~~~~~~~~ six multisines, prescribed
feet in BASE       ~~~~~~~~~~ moving (because the trunk moves under them)
joint angles       ~~~~~~~~~~ solved, one 3×3 per leg per timestep

And yes — your endpoint is right: you come out of this with all 12 joint positions plus v and a, and that is what RNEA consumes.

Let me record the notation fix, since §7.4 of the spec has the same ambiguous xo. FL foot in base frame:

p_FL^base = I·( (0.1934, 0.1420, 0) − (0.04, 0, 0.30) ) = (0.1534, 0.1420, −0.30)

It was at x = 0.1934; now it reads 0.1534. The foot didn't move — the trunk moved forward, so from the trunk's point of view the foot slid backward. The leg must reach back. Thigh joint sits at (0.1934, 0.142, 0), so thigh→foot is (−0.04, 0, −0.30), length 0.3027. Law of cosines with L1 = L2 = 0.213:

cos γ = (0.0916 − 0.0908)/0.0908 = 0.0088   →   γ = −1.562 rad     (was −1.580)

The knee straightens by about 1°. Add roll/pitch/yaw and all four legs do this simultaneously in different directions — front legs extending while rear tuck, one side lifting while the other compresses. That's the wobble.

What's fixed, what moves

feet in WORLD      ────────── frozen at t=0, forever
trunk in WORLD     ~~~~~~~~~~ six multisines, prescribed
feet in BASE       ~~~~~~~~~~ moving (because the trunk moves under them)
joint angles       ~~~~~~~~~~ solved, one 3×3 per leg per timestep

And yes — your endpoint is right: you come out of this with all 12 joint positions plus v and a, and that is what RNEA consumes.

Let me record the notation fix, since §7.4 of the spec has the same ambiguous x