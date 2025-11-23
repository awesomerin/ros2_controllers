# Examples: Complex Poles and Unstable Systems

Real-world examples with complete mathematical derivations and physical interpretations.

---

## Example 1: Complex Poles - Mass-Spring-Damper System

### Physical System

Imagine a mass on a spring with a damper (shock absorber):

```
    Ceiling
      ║
   ┌──╫──┐  ← Spring (k)
   │  ║  │
   └──╫──┘
      ║
    ╱═╫═╲  ← Damper (c)
   ╱  ║  ╲
  ╱═══╬═══╲
  │   │   │ ← Mass (m)
  │   M   │
  └───────┘
      ↓ x (position)
```

**Parameters:**
- Mass: m = 1 kg
- Spring constant: k = 25 N/m
- Damping coefficient: c = 3 N·s/m

### Differential Equation

**Newton's Second Law:**
```
m·ẍ = -k·x - c·ẋ + F

where:
  x = position [m]
  ẋ = velocity [m/s]
  ẍ = acceleration [m/s²]
  F = applied force [N]
```

**Rearrange:**
```
m·ẍ + c·ẋ + k·x = F
1·ẍ + 3·ẋ + 25·x = F
```

### Transfer Function

**Apply Laplace transform:**
```
s²·X(s) + 3s·X(s) + 25·X(s) = F(s)
X(s)·(s² + 3s + 25) = F(s)
```

**Transfer function:**
```
         X(s)           1
H(s) = ────── = ──────────────
         F(s)    s² + 3s + 25
```

### Finding the Poles

**Set denominator = 0:**
```
s² + 3s + 25 = 0
```

**Use quadratic formula:** s = (-b ± √(b² - 4ac)) / 2a
```
a = 1, b = 3, c = 25

s = (-3 ± √(9 - 100)) / 2
s = (-3 ± √(-91)) / 2
s = (-3 ± j·9.54) / 2
```

**The poles:**
```
s₁ = -1.5 + j·4.77
s₂ = -1.5 - j·4.77
```

**These are COMPLEX CONJUGATE poles!**

### Pole Locations on s-Plane

```
        jω
         ↑
     5j  │
         │
   4.77j │  ×  ← s₁ = -1.5 + j4.77
         │   ╲
         │    ╲
─────────┼─────×────→ σ
        -1.5   ╲
         │      ╲ Distance from origin = 5.0
         │       ↘ Angle = 107.6°
  -4.77j │  ×  ← s₂ = -1.5 - j4.77
         │
    -5j  │
```

**Pole properties:**
- **Real part:** σ = -1.5 (negative → stable!)
- **Imaginary part:** ω = ±4.77 rad/s (oscillation frequency)
- **Magnitude:** |s| = √(1.5² + 4.77²) = 5.0
- **Natural frequency:** ωₙ = 5.0 rad/s
- **Damping ratio:** ζ = 0.3 (underdamped)

### Time Response to Step Input

**Apply force F = 10 N at t = 0:**

```
x(t) = 0.4 - 0.4·e^(-1.5t)·cos(4.77t) - 0.126·e^(-1.5t)·sin(4.77t)
```

**Simplified form:**
```
x(t) ≈ 0.4·[1 - e^(-1.5t)·cos(4.77t - 18°)]
```

**Graphically:**
```
Position x(t) [m]
    0.6│
       │     ╱╲
    0.5│    ╱  ╲  ╱─────
       │   ╱    ╲╱   Overshoots!
    0.4│  ╱          Final value
       │ ╱  ╲  ╱
    0.3│╱    ╲╱  Oscillates
       │
    0.2│
       │
    0.1│
       │
    0.0└──────────────────────> Time [s]
        0   0.5   1.0   1.5   2.0   2.5   3.0

Key features:
  - Oscillation period = 2π/4.77 = 1.32 seconds
  - Envelope decays with τ = 1/1.5 = 0.67 seconds
  - Overshoot ≈ 40%
  - Settling time ≈ 4/1.5 = 2.67 seconds
```

### Physical Interpretation

**Why does it oscillate?**
- Spring pulls mass back (restoring force)
- Mass overshoots equilibrium (inertia)
- Damper slows it down but doesn't stop oscillation
- Eventually settles to equilibrium

**Effect of each pole component:**

**Real part (σ = -1.5):**
- Controls decay rate
- Envelope: e^(-1.5t)
- More negative → faster decay

**Imaginary part (ω = ±4.77):**
- Controls oscillation frequency
- Period = 2π/4.77 = 1.32s
- Frequency = 4.77 rad/s = 0.76 Hz

### What Happens with Different Damping?

**Case 1: Less damping (c = 1 N·s/m)**
```
s² + s + 25 = 0
s = -0.5 ± j·4.97

Poles closer to imaginary axis:
  - Slower decay (σ = -0.5)
  - More oscillation
  - Higher overshoot (≈70%)
```

**Case 2: More damping (c = 10 N·s/m)**
```
s² + 10s + 25 = 0
s = -5 ± j·0  (REAL poles now!)

No oscillation:
  - Pure exponential decay
  - No overshoot
  - Critically damped
```

**Case 3: No damping (c = 0)**
```
s² + 25 = 0
s = ±j·5  (PURE IMAGINARY)

On imaginary axis:
  - Oscillates forever!
  - No decay
  - Marginally stable
```

---

## Example 2: Complex Poles - RLC Circuit

### Physical System

```
    ┌─[R]─┬─[L]─┐
    │     │     │
   ─┴─   ─┴─    )
   Vin    C     ) L
   ─┬─   ─┬─    )
    │     │     │
    └─────┴─────┘
          │
         Vout
```

**Components:**
- Resistor: R = 10 Ω
- Inductor: L = 0.1 H
- Capacitor: C = 0.001 F

### Circuit Equation

**Kirchhoff's voltage law:**
```
L·d²q/dt² + R·dq/dt + q/C = Vin

where q = charge on capacitor
      i = dq/dt = current
```

**In terms of voltage across capacitor (Vout = q/C):**
```
LC·d²Vout/dt² + RC·dVout/dt + Vout = Vin
```

**Substitute values:**
```
0.0001·d²Vout/dt² + 0.01·dVout/dt + Vout = Vin
```

**Divide by 0.0001:**
```
d²Vout/dt² + 100·dVout/dt + 10000·Vout = 10000·Vin
```

### Transfer Function

**Laplace transform:**
```
         Vout(s)           10000
H(s) = ─────────── = ──────────────────
          Vin(s)      s² + 100s + 10000
```

### Finding Poles

```
s² + 100s + 10000 = 0

s = (-100 ± √(10000 - 40000)) / 2
s = (-100 ± √(-30000)) / 2
s = (-100 ± j·173.2) / 2
```

**Poles:**
```
s₁ = -50 + j·86.6
s₂ = -50 - j·86.6
```

### s-Plane Plot

```
        jω
         ↑
    100j │
         │
     86.6│  ×  ← Fast oscillation
         │
         │
─────────┼×───────→ σ
       -50  ← Fast decay
         │
         │
   -86.6│  ×
         │
   -100j │
```

### Time Response

**Step input: Vin = 5V at t=0**

```
Vout(t) = 5·[1 - e^(-50t)·cos(86.6t) - 0.577·e^(-50t)·sin(86.6t)]
```

**Graphically:**
```
Vout [V]
    7│    ╱╲
     │   ╱  ╲
    6│  ╱    ─────  Overshoots to 7V
     │ ╱  ╲ ╱
    5│╱    ╲   Final value = 5V
     │     ╱
    4│
     │
    3│
     │
    0└──────────────────> Time [ms]
      0   10   20   30   40

Very fast oscillation: T = 2π/86.6 = 72.5 ms
Very fast decay: τ = 1/50 = 20 ms
```

**This is a resonant circuit!** Common in:
- Radio tuners
- Filter circuits
- Power converters

---

## Example 3: Unstable System - Inverted Pendulum

### Physical System

```
      ↑ u (control force)
      │
    ┌─┴─┐
    │   │ ← Cart (M = 1 kg)
    └─┬─┘
      │
      │  ← Rod (massless, length L = 1 m)
      │
      ●  ← Ball (m = 0.1 kg)
     θ
```

**The inverted pendulum naturally falls over (unstable!)**

### Linearized Equation

**For small angles θ:**
```
(M + m)L·θ̈ - mg·θ = u

(1 + 0.1)·1·θ̈ - 0.1·9.8·θ = u
1.1·θ̈ - 0.98·θ = u
```

**Rearrange:**
```
θ̈ = 0.891·θ + 0.909·u
```

### Transfer Function

**Laplace transform:**
```
s²·Θ(s) = 0.891·Θ(s) + 0.909·U(s)
s²·Θ(s) - 0.891·Θ(s) = 0.909·U(s)
Θ(s)·(s² - 0.891) = 0.909·U(s)
```

**Transfer function:**
```
         Θ(s)          0.909
H(s) = ────── = ──────────────
         U(s)      s² - 0.891
```

### Finding Poles

```
s² - 0.891 = 0
s² = 0.891
s = ±√0.891
```

**Poles:**
```
s₁ = +0.944  ← RIGHT HALF-PLANE (UNSTABLE!)
s₂ = -0.944  ← Left half-plane (stable)
```

### s-Plane Plot

```
        jω
         ↑
         │
         │
───┬─────┼─────┬──→ σ
 -0.944  │  +0.944
   ✓     │     ✗
 (stable)│ (UNSTABLE!)
         │
         │

One pole in right half-plane → UNSTABLE SYSTEM
```

### Time Response (No Control)

**Tiny initial angle: θ(0) = 0.01 rad (0.57°)**

```
θ(t) = 0.005·e^(-0.944t) + 0.005·e^(+0.944t)
         └─ stable mode      └─ UNSTABLE mode dominates!
```

**Simplified (unstable mode dominates):**
```
θ(t) ≈ 0.005·e^(+0.944t)
```

**Graphically:**
```
Angle θ [rad]
    10│                      ╱
      │                    ╱
     8│                  ╱  GROWS EXPONENTIALLY!
      │                ╱
     6│              ╱
      │            ╱
     4│          ╱
      │        ╱
     2│      ╱
      │    ╱
  0.01│──╱───────────────────> Time [s]
      0  1  2  3  4  5  6  7

At t = 1s: θ = 0.013 rad
At t = 2s: θ = 0.034 rad
At t = 3s: θ = 0.087 rad
At t = 5s: θ = 0.57 rad (33°) - FALLING OVER!
At t = 7s: θ = 3.8 rad (218°) - COMPLETE TOPPLE!
```

### Physical Interpretation

**Why unstable?**
- Gravity pulls pendulum downward
- Any deviation from vertical gets amplified
- Positive feedback loop → exponential growth

**Growth rate:**
- Time constant τ = 1/0.944 = 1.06 seconds
- Doubles every: 0.693/0.944 = 0.73 seconds

### Making It Stable (Feedback Control)

**Add control: u = -K·θ (proportional feedback)**

**Closed-loop equation:**
```
θ̈ = 0.891·θ + 0.909·(-K·θ)
θ̈ = (0.891 - 0.909K)·θ
```

**New transfer function:**
```
         1
H(s) = ──────────────────────
       s² - (0.891 - 0.909K)
```

**New poles:**
```
s² = 0.891 - 0.909K
s = ±√(0.891 - 0.909K)
```

**For stability, need both poles in left half-plane:**
```
Need: 0.891 - 0.909K < 0
      0.909K > 0.891
      K > 0.98
```

**Choose K = 5:**
```
s² = 0.891 - 0.909·5 = -3.65
s = ±j·1.91  ← PURE IMAGINARY (oscillates)
```

**Better: Choose K = 10 (more damping)**
```
Need to add derivative feedback too:
u = -Kp·θ - Kd·θ̇

With Kp = 10, Kd = 5:
Poles: s = -2.5 ± j·1.3  ← Stable complex poles!
```

---

## Example 4: Unstable System - Positive Feedback Amplifier

### Circuit Diagram

```
      ┌───[R₂]───┐
      │          │
  Vin ─┬─[R₁]─┬──┴─┐
      │       │    │
      │       │  ──┴──
      │       │  \ Op / Vout
      │       │   \─/
      │       │    │
      └───────┴────┘
```

**This is POSITIVE feedback (feeds output back non-inverted)!**

### Transfer Function

**Op-amp with positive feedback:**
```
         Vout        R₂/R₁
H(s) = ────── = ───────────────
         Vin      1 - R₂/R₁
```

**Let R₁ = 1 kΩ, R₂ = 1.5 kΩ:**
```
         Vout         1.5
H(s) = ────── = ──────────
         Vin       1 - 1.5

              1.5
       = ────────
          -0.5

       = -3  (negative!!)
```

### Finding the Pole

**Transfer function with dynamics:**
```
Including op-amp bandwidth (ω₀ = 1000 rad/s):

         -3·ω₀
H(s) = ──────────
        s - ω₀

         -3000
     = ─────────
        s - 1000
```

**Pole:**
```
s - 1000 = 0
s = +1000  ← RIGHT HALF-PLANE!
```

### s-Plane

```
        jω
         ↑
         │
         │
─────────┼────┬─────→ σ
         │  +1000
         │    ✗
         │  UNSTABLE!
```

### Time Response

**Apply 0.001V input:**

```
Vout(t) = -0.003·e^(+1000t)
```

**Graphically:**
```
Vout [V]
   10│                ╱
     │              ╱  RAIL SATURATION
     │            ╱    (op-amp limit)
    5│          ╱
     │        ╱
     │      ╱
     │    ╱
     │  ╱    Exponential growth!
-0.003│─╱──────────────> Time [ms]
      0  2  4  6  8  10

At t = 1ms: Vout = -0.008V
At t = 3ms: Vout = -0.06V
At t = 5ms: Vout = -0.45V
At t = 7ms: Vout = -3.3V
At t = 9ms: Vout = -24V → SATURATES!

Time constant: τ = 1/1000 = 1 ms (VERY FAST growth!)
```

**What happens in reality?**
- Output rail-saturates almost instantly
- Acts like a **comparator** (digital output)
- Used intentionally in **Schmitt triggers**

---

## Summary Table

| System | Poles | Type | Behavior | Real-World Example |
|--------|-------|------|----------|-------------------|
| **Hydraulic motor** | s = -1.67 | Real, stable | Exponential approach | Your mower |
| **Mass-spring-damper** | s = -1.5 ± j4.77 | Complex, stable | Damped oscillation | Car suspension |
| **RLC circuit** | s = -50 ± j86.6 | Complex, stable (fast) | Resonance | Radio tuner |
| **Inverted pendulum** | s = ±0.944 | Real, one unstable | Exponential growth | Segway (needs control!) |
| **Positive feedback amp** | s = +1000 | Real, unstable | Very fast growth | Comparator circuit |

---

## Pole Location → Response Cheat Sheet

```
s-plane location          Time response          Common examples
──────────────────────────────────────────────────────────────────
s = -a (real, stable)     e^(-at)                Motors, RC circuits
                          Smooth decay

s = +a (real, unstable)   e^(+at)                Inverted pendulum
                          GROWS!                  Positive feedback

s = ±jω (imaginary)       sin(ωt)                Undamped oscillator
                          Oscillates forever      LC circuit (ideal)

s = -a ± jω               e^(-at)·sin(ωt)        Suspension, RLC
(complex, stable)         Decays while           Damped systems
                          oscillating

s = +a ± jω               e^(+at)·sin(ωt)        Flutter instability
(complex, unstable)       GROWING oscillation    Aircraft wing
```

---

## How to Identify Pole Type from Transfer Function

### Recipe:

**Step 1:** Write transfer function denominator
**Step 2:** Set denominator = 0
**Step 3:** Solve for s (use quadratic formula if needed)
**Step 4:** Check discriminant

**For second-order:** `s² + 2ζωₙs + ωₙ² = 0`

**Discriminant:** `Δ = (2ζωₙ)² - 4ωₙ² = 4ωₙ²(ζ² - 1)`

```
ζ > 1: Δ > 0  → Two REAL poles (overdamped)
ζ = 1: Δ = 0  → Repeated REAL pole (critically damped)
ζ < 1: Δ < 0  → COMPLEX poles (underdamped)
```

**Examples:**

**1) `H(s) = 1/(s² + 6s + 9)`**
```
Δ = 36 - 36 = 0 → Repeated real poles
s = -3, -3
Response: (1 + 3t)·e^(-3t)  (critically damped)
```

**2) `H(s) = 1/(s² + 2s + 10)`**
```
Δ = 4 - 40 = -36 → Complex poles
s = (-2 ± √-36)/2 = -1 ± j3
Response: e^(-t)·sin(3t)  (underdamped)
```

**3) `H(s) = 1/(s² - 4)`**
```
s² = 4
s = ±2  → One positive! UNSTABLE
Response: 0.5·e^(+2t) + 0.5·e^(-2t)  (grows!)
```

---

## Practice Problems

### Problem 1: Mystery System
**Given:** H(s) = 4/(s² + 4s + 20)
**Find:** Pole locations, damping ratio, natural frequency, predict behavior

<details>
<summary>Solution</summary>

Poles: s² + 4s + 20 = 0
s = (-4 ± √(16-80))/2 = (-4 ± j8)/2 = -2 ± j4

Natural frequency: ωₙ = √20 = 4.47 rad/s
Damping ratio: ζ = 2/(2·4.47) = 0.447 (underdamped)

Behavior: Oscillates at 4 rad/s while decaying with τ = 0.5s
</details>

### Problem 2: Stability Check
**Given:** H(s) = 10/(s² + 2s - 8)
**Is this stable?**

<details>
<summary>Solution</summary>

Poles: s² + 2s - 8 = 0
s = (-2 ± √(4+32))/2 = (-2 ± 6)/2

s₁ = 2 (POSITIVE → UNSTABLE!)
s₂ = -4 (negative → stable)

One pole in right half-plane → UNSTABLE SYSTEM
</details>

### Problem 3: From Poles to Transfer Function
**Given:** Poles at s = -3 ± j5
**Find:** Transfer function (assume gain = 1 at DC)

<details>
<summary>Solution</summary>

Poles: s = -3 ± j5

Transfer function:
H(s) = 1 / [(s - (-3+j5))·(s - (-3-j5))]
     = 1 / [(s+3-j5)·(s+3+j5)]
     = 1 / [(s+3)² + 25]
     = 1 / (s² + 6s + 9 + 25)
     = 1 / (s² + 6s + 34)

To get DC gain = 1:
H(s) = 34 / (s² + 6s + 34)
</details>
