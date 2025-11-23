# Look-Ahead Behavior in Local Planners for Slow Actuators

A practical guide to understanding and configuring prediction horizons for hydraulic mobile robots.

---

## Table of Contents
1. [What is Look-Ahead?](#what-is-look-ahead)
2. [How Look-Ahead is Implemented](#implementation)
3. [Impact on TEB Planner](#teb-impact)
4. [Configuration for Hydraulic Systems](#configuration)
5. [Trade-offs and Tuning](#tuning)

---

## 1. What is Look-Ahead? <a name="what-is-look-ahead"></a>

### The Core Concept

**Look-ahead** = how far into the future the planner predicts robot motion

```
Current State          Look-ahead window
     │                      ┊
     ▼                      ┊
   ┌─┴──┐                   ┊
   │ 🤖 │─────────────────► ┊─────────────> Future
   └────┘                   ┊
     │                      ┊
   (t=0)                 (t=horizon)

   ◄─────────────────────►
     Prediction horizon
```

### Why It Matters for Slow Actuators

**Fast actuators (electric motors):**
```
Command issued at t=0
         │
         ▼
  ┌─────────────┐
  │ Motor       │
  │ responds    │──> Achieves velocity in ~100ms
  │ instantly   │
  └─────────────┘

Look-ahead needed: ~1-2 seconds
```

**Slow actuators (hydraulics - your mower):**
```
Command issued at t=0
         │
         ▼
  ┌─────────────┐
  │ Hydraulics  │
  │ lag 600ms   │──> Still ramping up at 600ms
  │             │──> Reaches velocity at ~1800ms
  └─────────────┘

Look-ahead needed: ~3-5 seconds
```

### The Problem Illustrated

**Scenario:** Obstacle appears 2 meters ahead, robot traveling at 0.5 m/s

**With fast actuators (100ms response):**
```
Time t=0:  Planner sees obstacle at 2m
           Commands "slow down"

Time t=0.1s:  Robot slows down
              Distance traveled: 0.05m

Time t=1s:  Robot safely navigates around
```

**With slow actuators (600ms response):**
```
Time t=0:  Planner sees obstacle at 2m
           Commands "slow down"

Time t=0.6s:  Robot JUST starting to slow down!
              Distance traveled: 0.3m (still at 0.5 m/s)

Time t=1.8s:  Robot finally slowed down
              Distance traveled: 0.75m
              CRASH into obstacle! 💥
```

### The Solution: Look Further Ahead

**Increase prediction horizon** so planner sees the obstacle sooner relative to when action takes effect:

```
Standard look-ahead (2s):
  │
  ▼
🤖─────────[prediction]─────> 🌲
  0s      1s      2s         3s
          ▲
          Obstacle detected, but robot takes 1.8s to respond!
          = CRASH


Extended look-ahead (4s):
  │
  ▼
🤖─────────────────[prediction]──────────────> 🌲
  0s      1s      2s      3s      4s          5s
          ▲
          Obstacle detected at 2s
          Robot starts slowing at 0.6s
          Robot slow enough by 2.8s
          = SAFE
```

---

## 2. How Look-Ahead is Implemented <a name="implementation"></a>

### Core Mechanism: Trajectory Rollout

All local planners fundamentally do this:

```python
def plan_trajectory(current_state, goal, obstacles):
    """
    Local planner algorithm (simplified)
    """
    trajectories = []

    # Generate candidate trajectories
    for vel_cmd in sample_velocities():
        trajectory = []
        state = current_state

        # ROLLOUT: Simulate forward in time
        for t in range(0, horizon_time, dt):
            # Predict next state
            state = predict_motion(state, vel_cmd, dt)
            trajectory.append(state)

            # Check collision along predicted path
            if collides(state, obstacles):
                trajectory.cost = INFINITY  # Reject this trajectory
                break

        # Evaluate trajectory quality
        trajectory.cost = compute_cost(trajectory, goal)
        trajectories.append(trajectory)

    # Return best trajectory
    return min(trajectories, key=lambda t: t.cost)
```

**Key parameter:** `horizon_time` - how far to simulate

### Motion Prediction Model

**Simple kinematic model (standard planners):**
```python
def predict_motion(state, vel_cmd, dt):
    """Assumes instant response"""
    x, y, theta = state
    v, omega = vel_cmd

    # Update position (assumes velocity achieved instantly)
    x_new = x + v * cos(theta) * dt
    y_new = y + v * sin(theta) * dt
    theta_new = theta + omega * dt

    return (x_new, y_new, theta_new)
```

**Problem:** This assumes robot **instantly** achieves commanded velocity!

**Dynamic model (accounts for lag):**
```python
def predict_motion_with_lag(state, vel_cmd, dt, tau=0.6):
    """Accounts for actuator lag"""
    x, y, theta, v_actual, omega_actual = state  # Track actual velocity!
    v_cmd, omega_cmd = vel_cmd

    # First-order lag model (exponential response)
    alpha = exp(-dt / tau)
    v_new = alpha * v_actual + (1 - alpha) * v_cmd
    omega_new = alpha * omega_actual + (1 - alpha) * omega_cmd

    # Update position using ACTUAL velocity (not commanded)
    x_new = x + v_new * cos(theta) * dt
    y_new = y + v_new * sin(theta) * dt
    theta_new = theta + omega_new * dt

    return (x_new, y_new, theta_new, v_new, omega_new)
```

### Different Planners' Implementations

**DWB (Dynamic Window Approach):**
```yaml
DWBLocalPlanner:
  sim_time: 2.0  # Prediction horizon (seconds)
  sim_granularity: 0.025  # Time step for simulation (dt)

  # Samples many (v, omega) pairs and simulates each forward
  vx_samples: 20
  vtheta_samples: 20

  # Total trajectories evaluated: 20 × 20 = 400
```

**Visualization:**
```
DWB samples velocity space and simulates forward:

  ω (angular velocity)
  ↑
  │  × × × × ×   Each × = one trajectory
  │  × × × × ×   Simulated forward for sim_time
  │  × × × × ×
  │  × × × × ×
  └──────────────> v (linear velocity)

Each trajectory:
  Length = sim_time
  Points = sim_time / sim_granularity
         = 2.0 / 0.025 = 80 points
```

**TEB (Timed Elastic Band):**
```yaml
TebLocalPlanner:
  teb_autosize: True
  dt_ref: 0.3  # Desired time resolution
  dt_hysteresis: 0.1
  max_samples: 500  # Max trajectory points

  # Horizon determined by:
  min_obstacle_dist: 0.5  # How close to obstacles
  costmap_converter_rate: 5.0
```

**Visualization:**
```
TEB creates elastic band that optimizes path:

Start ●─────╱╲─────╱╲─────● Goal
            │  │    │  │
         Waypoints (poses with timestamps)

Each waypoint has:
  - Position (x, y)
  - Orientation (θ)
  - TIME (t)  ← Key difference from DWB!
```

**MPC (Model Predictive Control):**
```yaml
MPCLocalPlanner:
  prediction_horizon: 20  # Number of steps
  control_horizon: 5      # How many controls to optimize
  dt: 0.1                 # Time step

  # Total look-ahead: prediction_horizon × dt = 2.0s
```

**Visualization:**
```
MPC optimizes control sequence:

  u₀   u₁   u₂   u₃   u₄  ...
  │    │    │    │    │
  ▼    ▼    ▼    ▼    ▼
x₀ → x₁ → x₂ → x₃ → x₄ → ... → x₂₀
(now)                        (horizon)

Optimize u₀...u₄ to minimize cost over x₀...x₂₀
```

---

## 3. Impact on TEB Planner <a name="teb-impact"></a>

### TEB Background

**TEB = Timed Elastic Band**

Key idea: Trajectory is a sequence of poses **with timestamps**

```
Trajectory = [(x₀,y₀,θ₀,t₀), (x₁,y₁,θ₁,t₁), ..., (xₙ,yₙ,θₙ,tₙ)]
```

The "elastic band" minimizes a cost functional:
```
Cost = w₁·obstacle_cost +
       w₂·smoothness +
       w₃·velocity_profile +
       w₄·time_optimality +
       ...
```

### How TEB Look-Ahead Works

**TEB doesn't have a fixed horizon like DWB!**

Instead, it adapts based on:
1. Distance to goal
2. Obstacle proximity
3. Velocity limits

**Horizon determination:**
```python
# Simplified TEB horizon logic
def compute_horizon(current_pos, goal, obstacles):
    # Distance-based
    dist_to_goal = distance(current_pos, goal)
    time_to_goal = dist_to_goal / max_velocity

    # Obstacle-based
    nearest_obstacle_dist = min_distance_to_obstacles(obstacles)
    safety_time = nearest_obstacle_dist / current_velocity

    # Choose longer of the two
    horizon = max(time_to_goal, safety_time, min_horizon)
    horizon = min(horizon, max_horizon)  # Cap it

    return horizon
```

### Parameters Affecting Look-Ahead in TEB

**1. Time Resolution (`dt_ref`, `dt_hysteresis`)**

```yaml
dt_ref: 0.3  # Desired temporal resolution
dt_hysteresis: 0.1  # Tolerance band

# For hydraulics, INCREASE these:
dt_ref: 0.5  # Coarser time steps
dt_hysteresis: 0.2
```

**Why:** Slower systems don't need fine temporal resolution

**Effect on horizon:**
```
If path length = 10 meters, max_vel = 0.5 m/s

Small dt_ref (0.3s):
  Time to traverse: 20s
  Number of waypoints: 20 / 0.3 = 67 points

Large dt_ref (0.5s):
  Time to traverse: 20s  (same)
  Number of waypoints: 20 / 0.5 = 40 points

Fewer points = faster optimization, but coarser trajectory
```

**2. Obstacle Lookahead (`min_obstacle_dist`, `obstacle_poses_affected`)**

```yaml
min_obstacle_dist: 0.5  # Minimum distance to obstacles [m]

# For slow hydraulics, INCREASE:
min_obstacle_dist: 1.0  # Stay farther from obstacles

obstacle_poses_affected: 30  # How many poses consider obstacles
# INCREASE for slower response:
obstacle_poses_affected: 50
```

**Visualization:**
```
Standard (min_obstacle_dist = 0.5m):

🤖────────────> 🌲
   0.5m buffer

Robot can get close because it stops quickly


Hydraulics (min_obstacle_dist = 1.0m):

🤖──────────────────────> 🌲
      1.0m buffer

Need more buffer because slow to stop!
```

**3. Trajectory Timing (`max_vel_x`, `acc_lim_x`)**

```yaml
max_vel_x: 0.5  # Lower for hydraulics
acc_lim_x: 0.3  # MUST match hydraulic capability!

# If these don't match hardware:
# - TEB plans trajectory assuming 0.3 m/s²
# - But hardware can only do 0.3 m/s²
# - Robot won't track trajectory!
```

**4. Weight on Time Optimality**

```yaml
weight_optimaltime: 1.0  # Default

# For slow hydraulics, DECREASE:
weight_optimaltime: 0.5
```

**Why:** Fast traversal isn't possible, so don't try!

**5. Planning Horizon Limits**

```yaml
# Implicit horizon via max trajectory points
max_samples: 500  # Maximum waypoints

# Effective horizon = max_samples × dt_ref
# = 500 × 0.5 = 250 seconds (if dt_ref=0.5)
```

### TEB Example: Fast vs. Slow Actuators

**Scenario:** Navigate around obstacle

**Fast actuators (electric motors):**
```yaml
teb_local_planner:
  dt_ref: 0.3
  min_obstacle_dist: 0.3
  max_vel_x: 1.0
  acc_lim_x: 1.5  # Can accelerate quickly!
  weight_optimaltime: 1.0
```

**Resulting trajectory:**
```
     🌲
    ╱  ╲
   ╱    ╲
🤖─╯      ╰─────> Goal
   Quick dodge,
   tight turn

Timeline:
t=0.0s: Start turning
t=0.3s: Already turning (fast response)
t=1.0s: Past obstacle
```

**Slow actuators (hydraulics - 600ms lag):**
```yaml
teb_local_planner:
  dt_ref: 0.5           # ↑ Coarser time steps
  dt_hysteresis: 0.2    # ↑
  min_obstacle_dist: 1.0  # ↑ More buffer
  max_vel_x: 0.5        # ↓ Slower
  acc_lim_x: 0.3        # ↓ Conservative
  weight_optimaltime: 0.5  # ↓ Don't rush
  obstacle_poses_affected: 50  # ↑ More poses check obstacles
```

**Resulting trajectory:**
```
        🌲
       ╱    ╲
      ╱      ╲
     ╱        ╲
🤖──╯          ╰──────> Goal
   Wide berth,
   gentle turn

Timeline:
t=0.0s: Start turning command
t=0.6s: Hydraulics catch up, now turning
t=2.0s: Turning
t=3.5s: Past obstacle
```

---

## 4. Configuration for Hydraulic Systems <a name="configuration"></a>

### Recommended TEB Parameters for 500-700ms Response

```yaml
TebLocalPlannerROS:
  # ===== TIMING PARAMETERS =====
  dt_ref: 0.5
    # WHY: Coarser time resolution matches slower dynamics
    # EFFECT: Fewer waypoints, faster planning

  dt_hysteresis: 0.2
    # WHY: More tolerance for timing deviations

  # ===== VELOCITY & ACCELERATION =====
  max_vel_x: 0.5
  max_vel_x_backwards: 0.2
  max_vel_theta: 0.6

  acc_lim_x: 0.3
    # CRITICAL: Must match measured hydraulic capability!
    # Use value from measure_velocity_profile.py

  acc_lim_theta: 0.3

  # ===== JERK LIMITS (NEW IN RECENT VERSIONS) =====
  max_vel_x_deriv: 0.8
    # This is jerk (rate of acceleration change)
    # Use value from measurement script

  # ===== OBSTACLE AVOIDANCE =====
  min_obstacle_dist: 1.0
    # WHY: Need more stopping distance due to slow response
    # FORMULA: min_dist = v² / (2 × a) × safety_factor
    #        = 0.5² / (2 × 0.3) × 2.0 = 0.83m → round to 1.0m

  inflation_dist: 0.3
    # Additional buffer around min_obstacle_dist

  obstacle_poses_affected: 50
    # WHY: More trajectory points need obstacle checking
    # DEFAULT: 30 (for fast robots)
    # INCREASE: For slow response

  # ===== TRAJECTORY OPTIMIZATION =====
  weight_optimaltime: 0.5
    # WHY: De-emphasize speed (can't go fast anyway)
    # DEFAULT: 1.0

  weight_obstacle: 50.0
    # WHY: Emphasize obstacle avoidance over speed
    # DEFAULT: 50.0 (keep high)

  weight_viapoint: 1.0
  weight_inflation: 0.1

  # ===== ROBOT FOOTPRINT =====
  footprint_model:
    type: "circular"
    radius: 0.5  # Actual robot radius + safety margin
    # For slow robots, add 20-30% safety margin

  # ===== PLANNING FREQUENCY =====
  controller_frequency: 10.0
    # WHY: Don't need high-frequency replanning for slow robots
    # DEFAULT: 20 Hz
    # LOWER: 10 Hz saves computation

  # ===== LOOK-AHEAD INDICATORS =====
  max_global_plan_lookahead_dist: 3.0
    # WHY: Look further along global path
    # FORMULA: max_vel × response_time × safety_factor
    #        = 0.5 m/s × 1.8s × 2.0 = 1.8m → round to 3.0m

  # ===== TRAJECTORY FEASIBILITY =====
  feasibility_check_no_poses: 5
    # Check more poses for actuator lag
    # DEFAULT: 4

  # ===== ADVANCED: TRAJECTORY PREDICTION =====
  include_dynamic_obstacles: true
  predict_obstacles_time: 3.0
    # WHY: Predict obstacles 3s ahead (our response time)
```

### Critical Relationships

**1. Stopping Distance:**
```
stopping_distance = v² / (2 × a) + v × reaction_time

For your mower:
  v = 0.5 m/s (max velocity)
  a = 0.3 m/s² (deceleration capability)
  reaction_time = 0.6s (hydraulic lag)

stopping_distance = 0.5² / (2 × 0.3) + 0.5 × 0.6
                  = 0.42 + 0.3
                  = 0.72m

→ Set min_obstacle_dist ≥ 1.0m (with safety factor)
```

**2. Look-Ahead Distance:**
```
lookahead_dist = max_vel × (response_time + safety_margin)

For your mower:
  lookahead_dist = 0.5 × (1.8 + 1.0) = 1.4m

→ Set max_global_plan_lookahead_dist ≥ 2.0m
```

**3. Trajectory Duration:**
```
trajectory_duration = lookahead_dist / avg_velocity
                    = 2.0 / 0.5 = 4.0s

Number of waypoints = trajectory_duration / dt_ref
                    = 4.0 / 0.5 = 8 waypoints

→ This is reasonable (not too many, not too few)
```

---

## 5. Trade-offs and Tuning <a name="tuning"></a>

### The Central Trade-Off

```
        Longer Look-Ahead
               │
       ┌───────┴───────┐
       │               │
       ▼               ▼
  More Safety     Slower Planning
  Smoother Path   Less Reactive
  Early Avoidance Computational Cost
       │               │
       └───────┬───────┘
               │
         Must Balance!
```

### Impact on Performance

**Computation Time:**
```
Planning time ∝ (horizon / dt)²

Example:
  horizon = 2s, dt = 0.3s
  points = 2/0.3 = 7
  Planning time ∝ 7² = 49 units

  horizon = 4s, dt = 0.5s
  points = 4/0.5 = 8
  Planning time ∝ 8² = 64 units (30% slower)
```

**Recommendation:** Increase horizon but also increase dt to compensate

**Reactivity:**
```
Longer horizon = Sees far obstacles early (GOOD)
                 But slower to react to new obstacles (BAD)
```

**Solution:** Balance horizon with planning frequency

### Tuning Procedure

**Step 1: Measure Your System**
```bash
python3 scripts/measure_velocity_profile.py
```

**Step 2: Compute Safety Parameters**
```python
# From measurements:
max_vel = 0.5  # m/s
max_accel = 0.3  # m/s²
tau = 0.6  # time constant

# Compute:
stopping_dist = max_vel**2 / (2 * max_accel) + max_vel * tau
lookahead_dist = max_vel * (3 * tau)  # 3τ to reach 95%

print(f"min_obstacle_dist: {stopping_dist * 1.5}")  # 1.5× safety
print(f"max_global_plan_lookahead_dist: {lookahead_dist * 1.2}")
```

**Step 3: Configure TEB**
```yaml
# Use computed values
min_obstacle_dist: <computed_value>
max_global_plan_lookahead_dist: <computed_value>
acc_lim_x: <measured_max_accel>
max_vel_x: <measured_max_vel * 0.8>  # 80% for safety
```

**Step 4: Test Incrementally**

1. **Test 1: Straight line**
   - Does robot track path smoothly?
   - If oscillating: Increase dt_ref

2. **Test 2: Obstacle avoidance (static)**
   - Does robot avoid early enough?
   - If too close: Increase min_obstacle_dist

3. **Test 3: Tight spaces**
   - Can robot navigate?
   - If stuck: Decrease min_obstacle_dist or increase weight_optimaltime

4. **Test 4: Dynamic obstacles**
   - Does robot react in time?
   - If reactive but jerky: Increase dt_ref
   - If smooth but late: Increase predict_obstacles_time

### Common Issues & Fixes

**Problem:** Robot oscillates around path
```yaml
# FIX: Reduce planning frequency, increase dt
controller_frequency: 5.0  # Down from 10
dt_ref: 0.8  # Up from 0.5
```

**Problem:** Robot gets too close to obstacles
```yaml
# FIX: Increase safety buffers
min_obstacle_dist: 1.5  # Up from 1.0
inflation_dist: 0.5  # Up from 0.3
```

**Problem:** Robot too slow/conservative
```yaml
# FIX: Increase velocity limits (carefully!)
max_vel_x: 0.6  # Up from 0.5 (only if safe)
weight_optimaltime: 0.8  # Up from 0.5
```

**Problem:** Planning takes too long
```yaml
# FIX: Reduce horizon or increase dt
dt_ref: 0.7  # Up from 0.5
max_samples: 300  # Down from 500
```

---

## Summary

### Key Principles for Slow Actuators

1. **Look further ahead** (increase effective horizon)
2. **Stay further from obstacles** (increase min_obstacle_dist)
3. **Use coarser time steps** (increase dt_ref)
4. **Match acceleration limits to hardware** (critical!)
5. **De-emphasize time optimality** (decrease weight_optimaltime)
6. **Reduce planning frequency** if needed (computational savings)

### Recommended Starting Point

```yaml
TebLocalPlannerROS:
  dt_ref: 0.5
  dt_hysteresis: 0.2
  min_obstacle_dist: 1.0
  max_vel_x: 0.5
  acc_lim_x: 0.3  # FROM MEASUREMENTS!
  weight_optimaltime: 0.5
  obstacle_poses_affected: 50
  controller_frequency: 10.0
  max_global_plan_lookahead_dist: 2.5
```

**Then tune based on real-world testing!**

### The Bottom Line

**For 500-700ms hydraulic response:**
- Standard look-ahead (1-2s): **TOO SHORT** → collisions
- Extended look-ahead (3-5s): **BETTER** → early avoidance
- Coarse time steps (dt=0.5s): **EFFICIENT** → matches dynamics

**The planner must "see" obstacles far enough ahead that the robot's delayed response still has time to act!**
