# Transfer Functions, Poles, and the Complex Plane - Detailed Explanation

A step-by-step guide to understanding these foundational control theory concepts.

---

## Part 1: From Differential Equation to Transfer Function

### Starting Point: Your Hydraulic System

**Physical System:**
Your hydraulic motor has a lag - when you command a velocity, it takes time to respond.

**Differential Equation:**
```
dv/dt = (u - v) / τ

where:
  v = actual velocity (output)
  u = commanded velocity (input)
  τ = time constant (0.6 seconds for your system)
  dv/dt = rate of change of velocity
```

**What this means in English:**
"The velocity changes at a rate proportional to the difference between what you commanded and what you currently have."

### Step 1: Rearrange the Differential Equation

```
dv/dt = (u - v) / τ

Multiply both sides by τ:
τ · dv/dt = u - v

Rearrange:
τ · dv/dt + v = u
```

**This is the standard form of a first-order system!**

### Step 2: What is the Laplace Transform?

The Laplace transform converts **time-domain** equations into **frequency-domain** equations.

**Key idea:** Replace `d/dt` with `s`

**Laplace Transform Rules:**
```
Time Domain          Laplace Domain (s-domain)
──────────────────────────────────────────────
v(t)                 V(s)
dv/dt                s·V(s)
d²v/dt²              s²·V(s)
u(t)                 U(s)
```

**Why `s`?** It's a complex variable: `s = σ + jω`
- σ (sigma) = real part (decay/growth rate)
- jω (j·omega) = imaginary part (oscillation frequency)

We'll explain this in Part 2!

### Step 3: Apply Laplace Transform to Your Hydraulic Equation

**Time-domain equation:**
```
τ · dv/dt + v = u
```

**Apply Laplace transform to each term:**
```
τ · [s·V(s)] + V(s) = U(s)
```

**Factor out V(s):**
```
V(s) · [τ·s + 1] = U(s)
```

**Solve for V(s)/U(s):**
```
V(s)/U(s) = 1/(τ·s + 1)
```

**This ratio is the TRANSFER FUNCTION!**

### Step 4: Standard Form of First-Order Transfer Function

```
         Output         V(s)          1
H(s) = ────────── = ────────── = ─────────
         Input         U(s)       τ·s + 1
```

**For your hydraulic system (τ = 0.6s):**
```
         1
H(s) = ─────────
       0.6s + 1
```

**Alternative form (multiply by 1/τ):**
```
         1/τ         1.67
H(s) = ─────── = ─────────
        s + 1/τ     s + 1.67
```

Both forms are equivalent! The second form makes the **pole** more obvious.

---

## Part 2: Understanding Poles and Zeros

### What are Poles?

**Mathematical definition:**
Poles are values of `s` where the transfer function **goes to infinity** (denominator = 0).

**Physical meaning:**
Poles determine how fast the system responds and whether it's stable.

### Finding Poles for Your Hydraulic System

**Transfer function:**
```
         1
H(s) = ─────────
       0.6s + 1
```

**Set denominator = 0:**
```
0.6s + 1 = 0
0.6s = -1
s = -1/0.6
s = -1.67
```

**Your system has ONE pole at s = -1.67**

### What are Zeros?

**Mathematical definition:**
Zeros are values of `s` where the transfer function **equals zero** (numerator = 0).

**Your hydraulic system:**
```
         1
H(s) = ─────────
       0.6s + 1
```

Numerator = 1 (constant, never zero)
**→ No zeros!**

### Example with a Zero

**Different system:**
```
         s + 2
H(s) = ─────────
       s + 5
```

**Zero:** Set numerator = 0
```
s + 2 = 0
s = -2   ← Zero at s = -2
```

**Pole:** Set denominator = 0
```
s + 5 = 0
s = -5   ← Pole at s = -5
```

---

## Part 3: The Complex Plane (s-plane)

### What is `s`?

`s` is a **complex number**: `s = σ + jω`

- σ (sigma) = **real part**
- ω (omega) = **imaginary part**
- j = √(-1) (imaginary unit, engineers use `j` instead of `i`)

**Why complex?** Systems can have both:
- Exponential growth/decay (real part)
- Oscillations (imaginary part)

### The s-Plane Visualization

```
        jω (imaginary axis)
         ↑
         │
      3j │
         │
      2j │        × Pole here = oscillation + growth (UNSTABLE)
         │
       j │
         │
─────────┼─────────────→ σ (real axis)
   -3  -2│-1    1   2   3
         │
      -j │
         │
     -2j │    × Pole here = oscillation + decay (stable, damped)
         │
     -3j │
```

### What Each Region Means

**Left Half-Plane (σ < 0):**
- Poles here → **STABLE**
- System responses **decay to zero** over time
- More negative σ → faster decay

**Right Half-Plane (σ > 0):**
- Poles here → **UNSTABLE**
- System responses **grow without bound**
- Never want poles here for real systems!

**Imaginary Axis (σ = 0):**
- Poles here → **MARGINALLY STABLE**
- System **oscillates forever** at constant amplitude
- Example: Undamped spring-mass system

**On Real Axis (ω = 0):**
- Poles here → **NO OSCILLATION**
- Pure exponential response
- Your hydraulic system is here!

### Your Hydraulic System's Pole

**Pole at s = -1.67:**

```
        jω
         ↑
         │
         │
         │
─────────┼×────────→ σ
        -1.67
         │
         │
```

**Location:** On the real axis, left of origin

**Meaning:**
- Real axis → **No oscillation** (pure exponential response)
- Left of origin → **Stable** (velocity approaches commanded value)
- At -1.67 → **Time constant** = 1/1.67 = 0.6 seconds

### Example: Spring-Mass-Damper System

**Different example to show oscillation:**

**Transfer function:**
```
              1
H(s) = ─────────────────
       s² + 2ζωₙs + ωₙ²

where:
  ζ (zeta) = damping ratio (0.3 = underdamped)
  ωₙ (omega_n) = natural frequency (5 rad/s)
```

**Specific numbers:**
```
              1
H(s) = ──────────────
       s² + 3s + 25
```

**Finding poles:** Set denominator = 0
```
s² + 3s + 25 = 0

Using quadratic formula:
s = (-3 ± √(9 - 100)) / 2
s = (-3 ± √(-91)) / 2
s = (-3 ± j·9.54) / 2
s = -1.5 ± j·4.77
```

**Two poles:**
1. s₁ = -1.5 + j·4.77
2. s₂ = -1.5 - j·4.77 (complex conjugate)

**Plot on s-plane:**
```
        jω
         ↑
     5j  │
         │
   4.77j│  ×  ← Pole 1: -1.5 + j4.77
         │
         │
─────────┼─────────→ σ
      -1.5
         │
         │
  -4.77j│  ×  ← Pole 2: -1.5 - j4.77
         │
    -5j  │
```

**Meaning:**
- **Real part (σ = -1.5):** Decay rate → decays with time constant τ = 1/1.5 = 0.67s
- **Imaginary part (ω = 4.77):** Oscillation frequency → oscillates at 4.77 rad/s
- **Both negative real parts:** STABLE (oscillations die out)

**Time response:**
```
    Output
      │    ╱╲  ╱╲ ╱ ╲
      │   ╱  ╲╱  ╲   ╲___
      │  ╱
      │ ╱   Oscillates while decaying
      │╱
      └────────────────────> Time

      Oscillation period = 2π/ω = 2π/4.77 = 1.32s
      Decay time = 4/σ = 4/1.5 = 2.67s
```

---

## Part 4: Step-by-Step Examples

### Example 1: Your Hydraulic System (First-Order)

**Given:**
- Time constant τ = 0.6s
- No oscillation (pure lag)

**Step 1: Write differential equation**
```
τ·dv/dt + v = u
0.6·dv/dt + v = u
```

**Step 2: Laplace transform**
```
0.6·s·V(s) + V(s) = U(s)
V(s)·(0.6s + 1) = U(s)
```

**Step 3: Transfer function**
```
H(s) = V(s)/U(s) = 1/(0.6s + 1)
```

**Step 4: Find poles**
```
Denominator = 0
0.6s + 1 = 0
s = -1/0.6 = -1.67
```

**Step 5: Interpret pole location**
```
Pole at s = -1.67 (on real axis, left half-plane)

→ Stable
→ No oscillation
→ Time constant = 1/1.67 = 0.6s
→ 95% response time = 3τ = 1.8s
```

**Step 6: Time response to step input**

If you command u = 1.0 m/s (step input):

```
v(t) = 1 - e^(-t/0.6)

    v(t)
    1.0│        ┌────────
       │       ╱
       │      ╱
       │     ╱
    0.63│    ├─ ← 63% at t=0.6s
       │   ╱
       │  ╱
       │ ╱
    0.0└─┴──────────────> t
        0  0.6   1.2  1.8

At t=0.6s: v = 63% of final
At t=1.8s: v = 95% of final
```

### Example 2: Comparing Different Systems

**Fast system (electric motor, τ = 0.1s):**
```
H(s) = 1/(0.1s + 1)
Pole: s = -10
Time constant: 0.1s
95% response: 0.3s
```

**Your hydraulic system (τ = 0.6s):**
```
H(s) = 1/(0.6s + 1)
Pole: s = -1.67
Time constant: 0.6s
95% response: 1.8s
```

**Very slow system (large hydraulic, τ = 2.0s):**
```
H(s) = 1/(2.0s + 1)
Pole: s = -0.5
Time constant: 2.0s
95% response: 6.0s
```

**On s-plane:**
```
        jω
         ↑
         │
─────┬──┬┼┬───────→ σ
  -10  -1.67 -0.5
   ↑    ↑    ↑
  Fast  You  Slow
```

**Key insight:**
- **Further left** (more negative) = **faster response**
- **Closer to origin** = **slower response**

---

## Part 5: Why This Matters for Control

### Pole Placement in Action

**Your hydraulic system:**
- Current pole: s = -1.67 (τ = 0.6s)
- Want faster response: 3× faster

**Goal:** Move pole to s = -5.0 (τ = 0.2s)

**Method:** State feedback control

**Original system:**
```
dv/dt = -1.67·v + 1.67·u
```

**Add feedback:** u = -K·v + r (r = reference)
```
dv/dt = -1.67·v + 1.67·(-K·v + r)
      = -1.67·v - 1.67·K·v + 1.67·r
      = -(1.67 + 1.67·K)·v + 1.67·r
```

**New pole:** -(1.67 + 1.67·K)

**Set equal to desired pole:**
```
-(1.67 + 1.67·K) = -5.0
1.67 + 1.67·K = 5.0
1.67·K = 3.33
K = 2.0
```

**Result:** With feedback gain K = 2.0, pole moves from -1.67 to -5.0

**On s-plane:**
```
        jω
         ↑
         │
────┬────┼┬──────→ σ
   -5.0  -1.67
    ↑     ↑
   New  Original
  (fast) (slow)
```

**Time response comparison:**
```
Original (s = -1.67):  ────────────╱──────
                              ↑
                           Reaches 95% at 1.8s

With feedback (s = -5.0): ───╱──────────
                          ↑
                    Reaches 95% at 0.6s (3× faster!)
```

---

## Part 6: Visual Summary

### The s-Plane Map

```
           jω (OSCILLATION frequency)
            ↑
            │
   Unstable │  Unstable
  (growing  │  oscillation
   decay)   │  (BAD!)
            │
   -2  -1   │   1   2  → σ (DECAY/GROWTH rate)
────────────┼──────────
            │
   Stable   │  Stable
  (damped   │  (pure
   osc.)    │   decay)
            │
```

**Where you want poles:**
- ✅ **Left half-plane** (stable)
- ✅ **Not too close to origin** (reasonably fast)
- ✅ **Not too far left** (too fast = noise sensitivity)

**Where you DON'T want poles:**
- ❌ **Right half-plane** (unstable)
- ❌ **On imaginary axis** (sustained oscillation)

### Relationship Between Pole Location and Response

```
Pole Position          Time Response         Characteristics
───────────────────────────────────────────────────────────────
s = -a (real axis)     Exponential decay     No oscillation
                       v(t) = (1-e^(-at))    Fast if a large

s = ±jω (imag axis)    Sustained osc.        Never settles
                       v(t) = sin(ωt)        Period = 2π/ω

s = -a ± jω            Damped oscillation    Decays while
(complex pair)         v(t) = e^(-at)sin(ωt) oscillating
```

---

## Summary

### Transfer Functions in 4 Steps:

1. **Write differential equation** (physical system)
2. **Apply Laplace transform** (time → frequency domain)
3. **Solve for H(s) = Y(s)/U(s)** (transfer function)
4. **Find poles** (set denominator = 0)

### The s-Plane in Simple Terms:

**s-plane** = A map showing where poles live

- **Horizontal axis (σ):** How fast system responds
  - More negative = faster
  - Positive = unstable (bad!)

- **Vertical axis (jω):** Oscillation frequency
  - Zero = no oscillation (your hydraulic system)
  - Non-zero = oscillates at frequency ω

### For Your Hydraulic System:

```
System: dv/dt = (u - v)/0.6

Transfer function: H(s) = 1/(0.6s + 1)

Pole: s = -1.67

Location on s-plane: Real axis, left of origin

Meaning:
  ✓ Stable (left half-plane)
  ✓ No oscillation (on real axis)
  ✓ Time constant = 0.6s
  ✓ 95% response = 1.8s
```

---

## Practice Problems

### Problem 1
**Given:** A system with τ = 1.2s
**Find:** Pole location and 95% response time

<details>
<summary>Solution</summary>

Transfer function: H(s) = 1/(1.2s + 1)

Pole: 1.2s + 1 = 0 → s = -1/1.2 = -0.833

95% response: 3τ = 3 × 1.2 = 3.6 seconds
</details>

### Problem 2
**Given:** Pole at s = -4.0
**Find:** Time constant and transfer function

<details>
<summary>Solution</summary>

Pole s = -1/τ = -4.0
τ = 1/4.0 = 0.25 seconds

Transfer function: H(s) = 1/(0.25s + 1) = 4/(s + 4)
</details>

### Problem 3
**Given:** Two poles at s = -2 ± j3
**What type of response?**

<details>
<summary>Solution</summary>

Complex poles (have imaginary parts)
→ Oscillatory response

Real part σ = -2 (negative)
→ Stable, oscillations decay

Imaginary part ω = 3 rad/s
→ Oscillation frequency = 3 rad/s

Period = 2π/3 = 2.09 seconds
Decay time constant = 1/2 = 0.5 seconds
</details>

---

## Next Steps

Now that you understand transfer functions and poles:

1. **Read about Bode plots** - visualize frequency response
2. **Study root locus** - see how poles move with feedback
3. **Learn pole placement design** - put poles where you want them
4. **Explore state-space** - modern approach using matrices

For your mower project:
- You now understand WHY τ = 0.6s means slow response
- You can predict response time without testing (3τ = 1.8s)
- You understand how feedback can move poles to speed up response
