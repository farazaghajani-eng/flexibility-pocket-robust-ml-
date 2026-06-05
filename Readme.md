# Flexibility Pocket — Deterministic MPC + Repowering (Generation Flexibility Research) ⚡️📈

A compact research/engineering codebase combining:
- mid-term generation rehabilitation (repowering) decisions, and
- a deterministic receding-horizon Model Predictive Control (MPC) operational scheduler.

This repository is intended for experimenting with infeasibility diagnosis, operational flexibility bottlenecks (ramp limits, reserve shortages, storage constraints), and optimization-based corrective actions (slack penalties, temporary operational "boosts", and Stackelberg-style leader–follower investment decisions).

---

Table of contents
- [Project overview](#project-overview)
- [Features](#features)
- [Installation & Quick start](#installation--quick-start)
- [Project structure](#project-structure)
- [Usage examples](#usage-examples)
- [Research vision](#research-vision)
- [Results & visualization](#results--visualization)
- [Contributing](#contributing)
- [License](#license)

---

## Project overview

This project explores how strategic investment-like repowering decisions (ΔPmax, ΔRU) interact with deterministic operational control under ramping & reserve constraints. 
We couple a two-stage planning mindset (investment -> operation) with a receding-horizon MPC follower and provide tools to:

- detect infeasibility sources,
- restore feasibility with minimum intervention using optimization-based enhancements (slacks and temporary operational boosts),
- and extend to hierarchical Stackelberg-style design where the planner (leader) anticipates the MPC's (follower) response.

---

## Features

- Generation repowering model (Pyomo) with:
  - endogenous ΔRU and ΔPmax investment variables,
  - headroom dynamics (pbar), ramping constraints, and reserve calculations.
- Deterministic receding-horizon MPC (PuLP) for operational scheduling:
  - multi-resource battery + hydrogen-like storage models,
  - ramping constraints in time-sequences,
  - automatic enhancement strategies (slack variables, temporary RU boost with penalty).
- Stackelberg planner wrapper:
  - leader chooses investment deltas anticipating the MPC response (follower),
  - black-box outer optimization (derivative-free: Nelder–Mead / random search).
- Diagnostics:
  - slack usage, infeasible time steps detection,
  - heuristic binding/saturation detection (headroom/ramp saturations).
- Visualization helpers (matplotlib + seaborn) to inspect dispatch, SOC, objective, and slack usage.

---

## Installation & quick start

Prerequisites (Ubuntu / macOS / WSL):
1. Python 3.8+ (recommend: 3.9 or 3.10)
2. Install required Python packages:

```bash
python -m pip install --upgrade pip
pip install pyomo pulp matplotlib seaborn scipy
