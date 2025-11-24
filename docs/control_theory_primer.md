# Control Theory Primer for Robotics Engineers

A practical refresher on control concepts you'll encounter in ROS2 controllers.

---

## Table of Contents
1. [State-Space Representation](#state-space)
2. [Transfer Functions & Poles/Zeros](#transfer-functions)
3. [Pole Placement](#pole-placement)
4. [PID Control](#pid-control)
5. [State Feedback Control](#state-feedback)
6. [Observers](#observers)
7. [Practical Examples](#examples)

---

## 1. State-Space Representation <a name="state-space"></a>

### What is "State"?

**State** = the minimum information needed to predict future behavior

**Example: Mass-Spring-Damper System**
```
     ┌──┐
     │ m│───▶ Force F
     └──┘
      ║
   Spring k
      ║
  ═══════
```

**States:** `x = [position, velocity]`

Why both? Because:
- Position alone isn't enough (could be moving fast or slow)
- Velocity alone isn't enough (don't know where you are)
- Together, they fully describe the system

### State-Space Equations

**Continuous Time:**
```
ẋ = Ax + Bu    (state equation)
y = Cx + Du    (output equation)

where:
  x = state vector
  u = input (control)
  y = output (measurement)
  ẋ = dx/dt (derivative of state)
```

**Example: Mass-Spring-Damper**
```
States: x = [position]
             [velocity]

Dynamics:
  d/dt[position] = velocity
  d/dt[velocity] = -k/m*position - c/m*velocity + 1/m*force

In matrix form:
  ẋ = [0        1    ] x + [0  ] u
      [-k/m   -c/m  ]     [1/m]

  A = [0        1    ]    B = [0  ]
      [-k/m   -c/m  ]        [1/m]
```

### Why Use State-Space?

**Advantages:**
- ✅ Handles multi-input, multi-output (MIMO) systems easily
- ✅ Easy to incorporate modern control (LQR, MPC, observers)
- ✅ Natural for computer implementation
- ✅ Can represent nonlinear systems

**Disadvantages:**
- ❌ Less intuitive than transfer functions
- ❌ Requires choosing state variables

---

## 2. Transfer Functions & Poles/Zeros <a name="transfer-functions"></a>

### Transfer Function Basics

**Definition:** Ratio of output to input in Laplace domain

```
           Y(s)
H(s) = ─────────
           U(s)

where s = jω (complex frequency)
```

**Example: First-Order System (like your hydraulics!)**
```
         K
H(s) = ─────
       τs + 1

where:
  K = gain (steady-state output/input ratio)
  τ = time constant (how fast it responds)
```

**Time response:**
```
Step input: u(t) = 1

Output: y(t) = K(1 - e^(-t/τ))

  y
  K│        ┌────── (steady state)
   │       ╱
   │      ╱
   │     ╱
   │    ╱  ← 63% at t=τ
   │   ╱
   0└──┴────────────> t
      0   τ   2τ  3τ

Time to 63% of final value = τ
Time to 95% = 3τ
Time to 99% = 5τ
```

**For your hydraulics:**
```
τ = 0.6s (measured)

Time to reach 95% of commanded velocity = 3 × 0.6 = 1.8 seconds
```

### Poles and Zeros

**Poles:** Values of s where H(s) → ∞ (denominator = 0)
**Zeros:** Values of s where H(s) = 0 (numerator = 0)

**First-order example:**
```
         K
H(s) = ─────
       τs + 1

Pole: τs + 1 = 0 → s = -1/τ
```

**Pole location determines response:**
```
Complex Plane (s = σ + jω):

      jω (imaginary)
       ↑
       │    × faster oscillation
       │  ×
       │×
───────┼────────────→ σ (real)
   ×   │
faster │
decay  │

LEFT half-plane (σ < 0): STABLE
RIGHT half-plane (σ > 0): UNSTABLE
```

**Rules of Thumb:**
- **More negative real part** → **Faster response**
- **Imaginary part** → **Oscillation frequency**
- **On imaginary axis** → **Sustained oscillation**
- **Right half-plane** → **Exponential growth (BAD!)**

---

## 3. Pole Placement <a name="pole-placement"></a>

### The Core Idea

**You can choose where the poles go (within limits)!**

This lets you **design the response speed**.

### Example: State Feedback Control

**System:**
```
ẋ = Ax + Bu
```

**Control Law:**
```
u = -Kx  (feedback gain)
```

**Closed-loop dynamics:**
```
ẋ = Ax + B(-Kx)
ẋ = (A - BK)x
```

The eigenvalues of **(A - BK)** are the closed-loop poles.

**Pole Placement:** Choose K so that eigenvalues are where you want.

### Practical Example

**Your hydraulic system (simplified):**
```
State: x = [velocity]
Input: u = [command]

Dynamics:
  ẋ = -1/τ * x + 1/τ * u

  A = -1/τ = -1/0.6 = -1.67
  B = 1/τ = 1.67
```

**Current pole:** s = -1.67 (time constant 0.6s)

**Goal:** Make response 3× faster → pole at s = -5.0

**State feedback:**
```
u = -K*x + r  (r = reference)

Closed-loop:
  ẋ = (A - BK)x + Br
  ẋ = (-1.67 - 1.67*K)x + 1.67*r
```

**For pole at -5.0:**
```
-1.67 - 1.67*K = -5.0
K = (-5.0 + 1.67) / 1.67 = 2.0
```

**Result:** With K=2.0, system responds 3× faster!

### Pole Placement for Observers

**Same idea, but for estimation error dynamics:**

```
Observer: x̂̇ = Ax̂ + Bu + L(y - Cx̂)

Estimation error: e = x - x̂
Error dynamics: ė = (A - LC)e
```

**Choose L to make (A - LC) poles fast** → errors decay quickly

**Typical choice:** Observer poles 3-10× faster than system poles

---

## 4. PID Control <a name="pid-control"></a>

### The Classic Controller

```
         ┌──────────────────────────────┐
error ──>│  Kp + Ki/s + Kd*s  │──> control
         └──────────────────────────────┘

u(t) = Kp*e(t) + Ki*∫e(τ)dτ + Kd*de(t)/dt

where e(t) = reference - measurement
```

### Each Term's Role

**Proportional (Kp):**
- **Effect:** Output proportional to current error
- **Increases:** Response speed
- **Decreases:** Steady-state error
- **Too high:** Oscillation, overshoot

**Integral (Ki):**
- **Effect:** Accumulates error over time
- **Eliminates:** Steady-state error
- **Handles:** Constant disturbances, bias
- **Too high:** Slow, oscillatory, overshoot

**Derivative (Kd):**
- **Effect:** Responds to rate of change
- **Damps:** Oscillations
- **Improves:** Stability
- **Too high:** Amplifies noise, jittery

### Tuning Rules of Thumb

**Ziegler-Nichols (manual tuning):**

1. Set Ki = 0, Kd = 0
2. Increase Kp until system oscillates steadily
3. Note: Kp_critical and oscillation period T_critical
4. Set:
   ```
   Kp = 0.6 * Kp_critical
   Ki = 2*Kp / T_critical
   Kd = Kp * T_critical / 8
   ```

**Conservative tuning (for hydraulics):**
```
Kp = 1 / (desired_settling_time)
Ki = Kp / 10    (slow integration)
Kd = Kp * 0.1   (light damping)
```

**For 500-700ms hydraulics:**
```
Desired settling time: 2 seconds

Kp = 1/2 = 0.5
Ki = 0.05
Kd = 0.05
```

### PID Limitations

**Cannot handle:**
- ❌ Multiple coupled outputs (MIMO)
- ❌ Constraints (e.g., max pressure)
- ❌ Prediction/anticipation
- ❌ Optimal control

**But it's simple and works for 90% of applications!**

---

## 5. State Feedback Control <a name="state-feedback"></a>

### LQR (Linear Quadratic Regulator)

**Optimal state feedback control**

**Problem:** Minimize cost function
```
J = ∫[xᵀQx + uᵀRu] dt

Q = state cost (how much you care about state errors)
R = control cost (how much you penalize control effort)
```

**Solution:** Optimal gain K computed from Q, R
```
u = -Kx

K = R⁻¹BᵀP  (where P solves Riccati equation)
```

**Tuning:** Adjust Q and R matrices

**Example:**
```
Q = [100   0 ]  ← Care a lot about position
    [ 0    1 ]  ← Care less about velocity

R = [1]  ← Control effort cost

→ Compute K
→ Apply u = -Kx
```

**Benefits:**
- ✅ Mathematically optimal
- ✅ Guaranteed stability
- ✅ Handles MIMO naturally

**Drawbacks:**
- ❌ Requires full state measurement (or observer!)
- ❌ Tuning Q, R is still somewhat trial-and-error
- ❌ No integral action (won't eliminate steady-state error)

---

## 6. Observers <a name="observers"></a>

### Why Observers?

**Problem:** LQR needs all states, but you can only measure some.

**Solution:** Estimate unmeasured states from measurements.

### Luenberger Observer (Deterministic)

**Idea:** Run a model of the system, correct it with measurements

```
System:     ẋ = Ax + Bu
            y = Cx

Observer:   x̂̇ = Ax̂ + Bu + L(y - Cx̂)
                              └────┘
                            innovation
                         (measurement error)
```

**How it works:**
1. Predict state using model: `Ax̂ + Bu`
2. Compute what measurement should be: `Cx̂`
3. Compare to actual measurement: `y - Cx̂`
4. Correct estimate proportional to error: `L(y - Cx̂)`

**Pole placement:** Choose L so errors decay fast

### Kalman Filter (Stochastic)

**Same structure, but L is computed optimally:**

```
Predict:
  x̂⁻ = Ax̂ + Bu
  P⁻ = APAᵀ + Q  (covariance prediction)

Update:
  K = P⁻Cᵀ(CP⁻Cᵀ + R)⁻¹  (Kalman gain)
  x̂ = x̂⁻ + K(y - Cx̂⁻)
  P = (I - KC)P⁻
```

**Q:** Process noise covariance (model uncertainty)
**R:** Measurement noise covariance (sensor noise)

**Benefit:** Statistically optimal for Gaussian noise

---

## 7. Practical Examples <a name="examples"></a>

### Example 1: Your Hydraulic System

**Physical Model:**
```
Hydraulic motor with first-order lag

dv/dt = (u - v) / τ

where:
  v = wheel velocity
  u = commanded velocity
  τ = time constant (0.6s)
```

**State-Space:**
```
x = [v]  (1×1 state)
u = [cmd]

ẋ = -1/τ * x + 1/τ * u

A = [-1.67]
B = [1.67]
C = [1]  (measure velocity directly)
```

**Pole:** s = -1.67

**Transfer function:**
```
        1.67
H(s) = ───────
       s + 1.67
```

### Example 2: Differential Drive Robot

**States:**
```
x = [x_pos, y_pos, heading, v_left, v_right]ᵀ
```

**Inputs:**
```
u = [cmd_left, cmd_right]ᵀ
```

**Dynamics:**
```
ẋ = v*cos(θ)
ẏ = v*sin(θ)
θ̇ = ω
v̇_left = (cmd_left - v_left) / τ
v̇_right = (cmd_right - v_right) / τ

where:
  v = (v_left + v_right) / 2
  ω = (v_right - v_left) / wheelbase
```

**This is nonlinear!** (due to cos, sin)

---

## Quick Reference: When to Use What

| Control Method | Use When | Complexity |
|----------------|----------|------------|
| **On/Off** | Thermostat, simple yes/no | Trivial |
| **PID** | Single-input single-output, unknown model | Easy |
| **PID + Feedforward** | Known model, simple dynamics | Easy |
| **Pole Placement** | Known model, want specific response | Medium |
| **LQR** | Full state available, want optimal | Medium |
| **LQR + Observer** | Partial state, want optimal | Hard |
| **MPC** | Constraints, prediction needed | Very Hard |

---

## Recommended Learning Path

1. **Week 1:** Understand state-space representation
   - Convert your hydraulic system to state-space
   - Simulate in Python/MATLAB

2. **Week 2:** PID tuning
   - Implement PID for velocity control
   - Tune manually, understand each term

3. **Week 3:** Pole placement
   - Design state feedback for your system
   - Compare PID vs. state feedback

4. **Week 4:** Observers
   - Implement Luenberger observer
   - Understand observer poles

5. **Week 5:** Kalman filter
   - Add noise to simulation
   - Compare Luenberger vs. Kalman

---

## Tools & Resources

### Software:
- **Python Control**: `pip install control`
- **MATLAB Control Toolbox**: `tf()`, `ss()`, `lqr()`, `place()`
- **Simulink**: Visual simulation

### Books:
- *Feedback Control of Dynamic Systems* - Franklin, Powell, Emami-Naeini
- *Modern Control Engineering* - Ogata
- *Control System Design* - Goodwin, Graebe, Salgado

### Online:
- Brian Douglas YouTube series on Control Systems
- Steve Brunton's Control Bootcamp (YouTube)

---

## Summary

**Core Concepts:**
1. **State-space** = modern way to represent dynamics
2. **Poles** = determine response speed and stability
3. **Pole placement** = design response by choosing pole locations
4. **PID** = practical, simple, works for SISO
5. **State feedback** = optimal, requires full state
6. **Observers** = estimate unmeasured states from measurements

**For your mower:**
- Start with **PID + feedforward** (accel/jerk limits)
- Add **observer** only if you need state estimation
- Use **pole placement** if you want precise response tuning

**The #1 rule:** Keep it as simple as possible while meeting requirements!
