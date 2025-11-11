# Mobile Robotics Project — Requirements & Project Init
**Author:** *[Name Surname]*  
**Course / Project:** Mobile Robotics Project — Pôle Léonard de Vinci  
**Date:** *[update date]*

---

## 1) Purpose & Scope
Design, simulate, and evaluate a **control strategy** for an autonomous **vacuum cleaner** robot (unicycle/differential drive) in a **20×20 m** room with one circular obstacle. Deliver a working **MATLAB/Simulink** simulation and a report with **coverage indicators**.

---

## 2) Functional Requirements (What the system must do)
1. **Pose-driven control:** Command linear speed \(v\) and angular speed \(\omega\); no per-wheel control.
2. **Obstacle avoidance:** Use one **front ultrasonic** distance to avoid walls/obstacles.
3. **Coverage strategy:** Explore and clean the room using a **hybrid** approach (reactive + stochastic + coverage memory).
4. **Coverage tracking:** Maintain a **coverage grid** and compute **Coverage(t)** \[% visited over time].
5. **Visualization:** Provide the **robot path** and a **final heatmap** of visited cells.
6. **Random start:** Initialize pose \((x_0,y_0,\theta_0)\) randomly inside the room.
7. **KPIs export:** Export Coverage(t) and key parameters for plots/analysis.

---

## 3) Non-Functional Requirements (Qualities & constraints)
- **Runtime:** Simulate typical runs (1 000–5 000 s) within reasonable time on a student laptop.
- **Reliability:** Stable behavior (no numerical explosions); saturations on \(v\) and \(\omega\).
- **Reproducibility:** Simulations reproducible via a **random seed**.
- **Maintainability:** Modular Simulink subsystems (`Unicycle`, `Ultrasonic`, `Coverage`, `Controller`).  
- **Usability:** One-click script to run (e.g., `run_simulation.m`) and clear instructions.
- **Versioning:** Use **Git** (or at least ZIPs) with a clean repo structure.

---

## 4) Assumptions
- **Pose availability:** \((x,y,\theta)\) provided at each time step (idealized proprioception).
- **Sensing:** One forward **ultrasonic** sensor with a maximum range \(d_{\max}\) and threshold \(d_{\text{th}}\).
- **Environment:** Flat floor; room is square, centered at \((0,0)\); 1 circular obstacle at \((3,3)\), \(R=0.25\) m.
- **Kinematics:** Unicycle model with \(v_{\max}=0.3\ \text{m/s}\).

---

## 5) Constraints & Limits
- **No per-wheel control:** We do not model motor dynamics; we command \((v,\omega)\) directly.
- **Limited perception:** Only one distance measurement in the robot’s forward direction.
- **No prior map:** The controller has **no a-priori map**; memory comes from the **coverage grid** built online.
- **Heuristic strategy:** Not mathematically optimal; performance depends on weights, thresholds, and grid resolution.

---

## 6) Interfaces
- **Inputs:** \((x,y,\theta)\), ultrasonic distance \(d\), simulation time \(t\), random seed.
- **Outputs:** \((v,\omega)\), Coverage(t), path, final heatmap (images/plots), logs to workspace/files.
- **Scripts:** `init_project.m` (init), `run_simulation.m` (main run), optional `plot_results.m`.

---

## 7) KPIs (Evaluation)
- **Coverage(t)**: \(\frac{\#\text{visited}}{\#\text{total}}\times 100\%\).
- **Revisit rate** (optional): \(\frac{\#\text{cells visited}>1}{\#\text{visited}}\).
- **Time-to-X%**: Time to reach 60/80/90% coverage.
- **Sensitivity**: Effect of \(d_{\text{th}}\), \(w_{nov}\), grid resolution.

---

## 8) Out of Scope (for this project)
- Full **SLAM** or visual/LIDAR-based mapping.
- Detailed **motor dynamics** and low-level per-wheel control.
- Multi-room planning, docking/charging behavior, carpet detection, etc.

---

## 9) Risks & Mitigations
- **Local minima / loops** → stochastic reorientation + stagnation timer.
- **Parameter sensitivity** → simple parameter sweep and defaults documented.
- **Numerical instability** → saturation, time-step selection, guard conditions in ray-casting.
- **Performance vs. resolution** → choose \(N\) for the grid to balance detail and runtime.

---

## 10) What is `init_project.m` (and why you need it)
**`init_project.m`** is a single **MATLAB initialization script** that centralizes **all project constants and paths** so every run starts from a **clean, reproducible configuration**.

### Purpose
- **Define constants:** room size, obstacle, \(v_{\max}\), \(\omega_{\max}\), thresholds, grid size \(N\), time step \(\Delta t\).
- **Set random seed:** reproducible randomness (initial pose, stochastic heading).
- **Add paths:** ensure MATLAB finds `/Models`, `/Scripts`, `/Data` folders.
- **Reset state:** clear variables, close figures, create output directories.
- **Save config:** store parameters in a struct (e.g., `P`) pushed to base workspace.

### Benefits
- **Reproducibility:** same config → same results.
- **Maintainability:** one place to change parameters.
- **Collaboration:** teammates run the same config without hunting for variables.

### Typical Structure (Skeleton)
```matlab
function P = init_project()
  % Housekeeping
  close all; clc;
  rng(42); % reproducible runs
  addpath(genpath('Models')); addpath(genpath('Scripts')); addpath('Data');

  % World & robot
  P.room.half = 10;           % world ∈ [-10,10] × [-10,10]
  P.obs.center = [3, 3]; P.obs.R = 0.25;
  P.robot.v_max = 0.3;        % m/s
  P.robot.omega_max = 1.5;    % rad/s (tune)
  P.dt = 0.1; P.T = 3000;     % step and horizon (s)

  % Sensor
  P.sonar.d_max = 3.0;        % m
  P.sonar.d_th  = 0.5;        % m

  % Coverage grid
  P.grid.N = 200;             % resolution
  P.grid.decay = 0.0;         % optional

  % Strategy weights
  P.ctrl.w_nov = 1.0;
  P.ctrl.w_rot = 0.2;
  P.ctrl.sector_deg = 120;    % forward sector width
  P.ctrl.temp = 1.0;          % softmax temperature
  P.ctrl.stag_T = 7.0;        % stagnation timeout (s)

  % Initial pose (random)
  P.x0 = (rand*2-1)*P.room.half*0.8;
  P.y0 = (rand*2-1)*P.room.half*0.8;
  P.th0 = rand*2*pi - pi;

  % Output folders
  if ~exist('Output','dir'), mkdir('Output'); end
  save(fullfile('Output','last_config.mat'),'P');
end
```
> **How to use:** In MATLAB, call `P = init_project();` once, then run `run_simulation.m` which reads `P` from the workspace or loads `last_config.mat`.

---

## 11) Git — Minimal Workflow for This Project
### Why Git here?
- **Version control:** track changes to models/scripts.
- **Reproducibility & backup:** revert if needed, safe collaboration.
- **Packaging:** easy delivery to the instructor (tag or release).

### Suggested Repo Structure
```
mobile-robotics-project/
├─ Models/           % Simulink models (.slx)
├─ Scripts/          % MATLAB scripts (.m)
├─ Data/             % saved configs, logs
├─ Output/           % figures, results
├─ Docs/             % Markdown, report, images
└─ README.md
```

### Minimal Command Set
```bash
# 1) Init repository (run once at the project root)
git init
git add .
git commit -m "Initial commit: baseline structure, init_project.m skeleton"

# 2) Work cycle
# ... edit files ...
git add Models Scripts Docs
git commit -m "Add unicycle model and sonar ray-cast block"

# 3) Create a checkpoint (tag) before submission
git tag -a v1.0 -m "Submission version"
git log --oneline --graph

# 4) (Optional) push to remote
git branch -M main
git remote add origin <your-repo-url>
git push -u origin main --tags
```

### Good Practices
- **One feature per commit** (small, frequent).
- **Meaningful messages** (“Implement coverage grid and KPI”).
- **Ignore large outputs** (`Output/`) with a `.gitignore`.
- **Tag** the version you submit (e.g., `v1.0`).

---

*End of requirements & init notes.*
