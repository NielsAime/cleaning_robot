# Vacuum Cleaner Autonomous Robot — Control Strategy & Simulation Approach
**Author:** *[Name Surname]*  
**Course / Project:** Mobile Robotics Project — Pôle Léonard de Vinci  
**Date:** *[update date]*

---

## 1) Executive Summary
This document presents (i) the **proposed control strategy** for an autonomous vacuum cleaner robot and (ii) the **approach for the simulation**. After a short **state of the art** of cleaning strategies used in practice, we justify the choice of a **hybrid strategy** combining **reactive obstacle avoidance**, **stochastic exploration**, and a **coverage memory** (using the proprioceptive pose \((x,y,\theta)\) provided by the project) to increase cleaning efficiency with minimal sensing (one ultrasonic sensor).

---

## 2) Rapid State of the Art of Coverage/Navigation Strategies (Domestic Robots)
### 2.1 Reactive (Obstacle Avoidance / Wall-Following)
- **Principle:** Move forward; when the front distance falls below a threshold, turn and follow the walls at a safe distance.
- **Pros:** Extremely simple, robust, cheap; works with very limited sensing.
- **Cons:** No guarantee of complete coverage; many revisits; can loop in corners.
- **Typical use:** Early/entry-level cleaners; baseline behavior in many products.

### 2.2 Random / Stochastic Exploration (“Random Walk” with Bump/Distance Cues)
- **Principle:** Move straight until detecting an obstacle; then choose a new heading randomly (e.g., 90–180°) and continue.
- **Pros:** Easy to implement; good average coverage over long durations.
- **Cons:** Unpredictable paths; possible inefficiencies in the short term; revisits remain frequent.
- **Typical use:** Widely used historically; still a component in many hybrid commercial strategies.

### 2.3 Systematic Coverage Patterns (Lawnmower / Spiral / Cell-Decomposition)
- **Principle:** Follow structured patterns (zig-zag / spiral) or visit a grid of cells.
- **Pros:** Near-complete coverage and clear visual results; efficient if the environment is known or pose is reliable.
- **Cons:** Requires reliable localization; obstacle handling complicates the pattern with minimal sensing.
- **Typical use:** Mid/high-end robots with better localization; often combined with local obstacle handling.

### 2.4 Map-Based / SLAM-Driven Coverage (High-End)
- **Principle:** Build a map, plan globally, then execute systematic coverage.
- **Pros:** High efficiency, repeatability, room segmentation.
- **Cons:** Requires richer sensors (LIDAR/cameras) and significant compute; out-of-scope for this project.
- **Typical use:** Premium robots; not aligned with the “one ultrasonic sensor” constraint.

**Conclusion:** With a **single forward ultrasonic sensor** and the **availability of \((x,y,\theta)\)** from proprioception (given in the project), fully map-based strategies are out-of-scope. The best cost/benefit is a **hybrid approach** that leverages pose to avoid revisits while keeping a simple, reactive core.

---

## 3) Project Constraints & Design Implications
- **Robot model:** Unicycle/differential-drive. Control directly in **linear speed \(v\)** and **angular speed \(\omega\)** (no per-wheel control required per instructor note).
- **Sensors available:**
  - **Proprioception:** Provides **pose** \((x,y,\theta)\) (treated as available at each step for control and logging).
  - **Exteroception:** **One front ultrasonic distance** to the nearest obstacle along the robot’s x-axis.
- **Environment (simulation):** Square room **20 m × 20 m**, center at \((0,0)\); one cylindrical obstacle at \((3,3)\), radius \(0.25\) m; random initial pose; max speed **0.3 m/s**.
- **Cleaning efficiency bias:** Prefer **forward motion** when free space is available.

**Implication:** Since \((x,y,\theta)\) is available, the controller can reason globally enough to **remember already-visited areas** and bias future headings towards **less-visited regions**, while the ultrasonic distance ensures **safe local avoidance**.

---

## 4) Proposed Control Strategy: Hybrid Reactive + Stochastic + Coverage Memory
### 4.1 Rationale
- **Reactive avoidance** is necessary with a single ultrasonic sensor → immediate safety and wall/corner handling.
- **Stochastic exploration** prevents being trapped in cycles and adds robustness to unforeseen layouts.
- **Coverage memory** (via a **2D occupancy/coverage grid** indexed by the known pose) actively **discourages revisiting** and **encourages exploration** of “cold” zones, improving the **coverage vs. time** KPI.

### 4.2 High-Level Algorithm
1. **Coverage Grid Maintenance (Memory):** Discretize the room into an \(N \times N\) grid (e.g., \(200\times 200\)). Each time step, convert \((x,y)\) into a cell index and **increment** the visit count; compute **Coverage(t)** as the ratio of visited cells.
2. **Obstacle Avoidance (Reactive):** If ultrasonic distance \(d < d_{\text{th}}\):  
   - Set \(v \rightarrow v_{\text{min}}\) (or 0),  
   - Rotate with \(|\omega|\rightarrow \omega_{\max}\) until \(d \ge d_{\text{th}}\).  
   Optionally choose the turning side that leads to **less-visited** lateral cells.
3. **Heading Selection (Stochastic, Bias to Novelty):** When free ahead (\(d \ge d_{\text{th}}\)), select a desired heading \(\theta^\*\) within a forward sector \([\theta-\Delta,\ \theta+\Delta]\). For candidate headings \(\alpha_k\), compute a **score**:
   \[
   S(\alpha_k) = w_{\text{nov}}\cdot(1 - \text{avg\_visits\_ahead}(\alpha_k)) - w_{\text{rot}}\cdot |\alpha_k - \theta|
   \]
   Then pick \(\theta^\*\) using a **softmax** over \(\{S(\alpha_k)\}\) (stochastic but biased towards novelty and small rotations).
4. **Velocity Command:**  
   \(\omega = k_\psi \cdot \text{wrap}(\theta^\* - \theta)\) with saturation;  
   \(v = v_{\max} \cdot \sigma\big(|\text{wrap}(\theta^\* - \theta)|\big)\) (reduce speed for large turns to keep forward motion dominant).
5. **Safety & Limits:** Saturations on \(v\) and \(\omega\); timeout/stagnation detector → if progress is low for T seconds, trigger a larger randomized reorientation.

### 4.3 Expected Behavior
- Rapid initial coverage thanks to forward-biased motion.
- Decreasing revisits because the heading choice favors **less-visited** areas.
- Robustness to corners and obstacle thanks to **reactive** loop.

### 4.4 Strengths & Limitations
- **Strengths:** Simple, fast to implement, good coverage/time tradeoff, visually compelling (coverage map + path), aligns with constraints.
- **Limitations:** Heuristic (no optimality proof); with only a front distance, complex obstacle geometries are handled locally; performance depends on grid resolution and scoring weights.

---

## 5) Simulation Approach
### 5.1 Model Structure (Simulink / MATLAB)
```
[Pose (x,y,θ)] ─┐
                │        ┌───────────────────────┐       ┌──────────────────┐
[Ultrasonic d] ─┼──────▶ │  Control Strategy     │ ───▶  │ Unicycle Model   │ ───▶ (x,y,θ)
                │        │ (react + bias memory) │  v,ω  │  (kinematics)    │
[Coverage Grid] ◀────────┘                       │       └──────────────────┘
      ▲             updates visits & scores      │
      └────────────── ray-cast & novelty ◀───────┘
```

### 5.2 Components
- **Unicycle Kinematics:**  
  \(\dot{x}=v\cos\theta,\ \dot{y}=v\sin\theta,\ \dot{\theta}=\omega\).  
  Inputs: \(v,\omega\). Outputs: \((x,y,\theta)\). Integrators + saturation at \(v_{\max}=0.3\ \text{m/s}\).
- **Ultrasonic Sensor (Ray-Casting):** Compute distance \(d\) from robot’s pose/heading to the nearest intersection with room walls or the circular obstacle.
- **Coverage Grid Module:**  
  - World \([-10,10] \times [-10,10]\) m → grid \(N\times N\).  
  - Update visited cells; compute **Coverage(t)** and maintain a **visit heatmap**.  
  - Provide a **local novelty estimate** in candidate directions for the controller’s scoring function.
- **Control Strategy Block:** Implements the **reactive** threshold logic, **stochastic heading** selection with novelty bias, and the \((v,\omega)\) command law with saturations and stagnation recovery.

### 5.3 Simulation Parameters
- **Time step:** e.g., \( \Delta t = 0.05\)–\(0.1\) s.
- **Thresholds:** \(d_{\text{th}}\) (e.g., 0.35–0.6 m), \(v_{\max}=0.3\ \text{m/s}\), \(\omega_{\max}\) tuned to achieve a comfortable turn rate.
- **Scoring Weights:** \(w_{\text{nov}}, w_{\text{rot}}\); forward sector width \(\Delta\); softmax temperature \(\tau\).
- **Stagnation timeout:** e.g., 5–10 s of low advance → trigger a larger randomized reorientation.

### 5.4 Outputs & KPIs
- **Coverage(t):** \(\frac{\#\text{visited cells}}{\#\text{total cells}}\times 100\%\).
- **Path visualization:** Robot track overlay on the heatmap.
- **Optional:** Revisit rate, average distance to unvisited cells, time to reach 80% coverage.

### 5.5 Validation Plan
- Baseline with simple reactive only → compare with hybrid (expected faster coverage growth).  
- Sensitivity to \(d_{\text{th}}\), grid resolution, novelty weight, sector width.  
- Multiple random seeds (initial pose / stochastic choices).

---

## 6) Why This Strategy Fits the Project
- **Aligned with constraints:** single ultrasonic sensor + pose availability; preference for forward motion.
- **Feasible in days:** modular blocks; minimal math overhead; immediately demonstrable results.
- **Pedagogical:** links robot kinematics, sensing, and decision-making; clear KPI improvement over a reactive baseline.

---

## 7) Implementation Notes (for the next steps)
- Provide a MATLAB init script to set room, obstacle, grid, thresholds, and random seed.
- Package Simulink model with subsystems: `Unicycle`, `Ultrasonic`, `Coverage`, `Controller`.
- Generate figures automatically at the end of a run: Coverage vs. time, final heatmap + path.
- Prepare short instructions for running (single `run_simulation.m`).

---

*End of document.*
