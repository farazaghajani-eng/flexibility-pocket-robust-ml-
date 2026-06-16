# Results and Research Artifacts  
**Flexibility Pocket Repository**

## Repository Overview
This repository presents a focused research framework for **flexibility-oriented power system planning and operation**. It integrates generation repowering decisions with deterministic Model Predictive Control (MPC) in a hierarchical (Stackelberg) architecture.

The core objective is to quantify and mitigate system flexibility deficits under high renewable penetration through coordinated investment (repowering) and operational strategies.

Key components include:
- Thermal unit rehabilitation (repowering)
- Flexibility-aware decision making
- Receding-horizon MPC control
- Stackelberg leader-follower coordination
- Energy storage (battery & hydrogen-like) and demand-side flexibility modeling

Models are implemented in **Python** using **Pyomo** (strategic planning/repowering) and **PuLP** (operational MPC), providing reproducible testbeds for low-carbon power systems research.

---

## Research Outputs

### 1. Conference Research Paper
**`Aghajani_Faraz_Conference2.pdf`**

**Description**  
This conference paper serves as the principal scientific reference for the repository. It details the theoretical foundations, mathematical formulations, case studies, and validation of the implemented models.

**Research Scope**  
The work investigates flexibility enhancement strategies through:
- Thermal generation rehabilitation (repowering)
- Flexibility-oriented scheduling
- Renewable uncertainty management
- Pumped-storage hydro and multi-resource storage integration
- Demand-side flexibility aggregation
- Market-aware operational planning

**Methodological Contributions**  
Integrated framework combining:
1. Demand Envelope Characterization
2. Stochastic Unit Commitment & Scheduling
3. Flexibility-Constrained Generation Expansion Planning (GEP)
4. MPC-Based Operational Decision Making
5. Repowering Investment Decisions

**Key Findings**  
Coordinated repowering and flexibility planning deliver:
- Significant reduction in flexibility deficits
- Elimination of involuntary load shedding
- Lower operating expenditures
- Reduced CO₂ emissions
- Improved renewable integration

**Repository Relevance**  
Provides the foundation for the core scripts:  
`hydro_flex.py`, `mpc_control.py`, `rep_flex_mpc.py`, and `stackelberg_mpc.py`.

**Date Added**: June 2026

---

### 2. Research Visualizations

#### **IMG_5550.png** – Overall Flexibility Framework Infographic
**Description**  
Professional research infographic presenting the complete flexibility enhancement pipeline.

**Contents**  
- The Flexibility Challenge under high VRE penetration
- Integrated three-step framework (Demand Envelope → Stochastic Scheduling → GEP with repowering, storage, and household flexibility)
- Key numerical results from a realistic 18-unit system with PSH

**Representative Results**
| Performance Metric     | Improvement                  |
|------------------------|------------------------------|
| Flexibility Deficit    | **65% Reduction** (130 → 45.5 MW/min) |
| Load Shedding          | **Completely Eliminated** (100 MW → 0 MW) |
| Operating Costs        | **22.6% Reduction**          |
| CO₂ Emissions          | **27.3% Reduction**          |

**Date Added**: June 2026

---

#### **IMG_5572.png** – Stackelberg Architecture & MPC-Repowering Results
**Description**  
High-impact visualization of the two-level Stackelberg framework implemented in the repository.

**Contents**  
- **Level I (Strategic Leader)**: Repowering Planner (Pyomo) – μPmax investment, μRU ramp targets, headroom dynamics
- **Level II (Operational Follower)**: Receding-Horizon MPC (PuLP) with Battery & Hydrogen facilities
- Robust infeasibility management with dynamic slacks
- Key benchmarks: Baseline infeasibility (69.6 MW deficit) → +139.2 MW added capacity → Final $10.6k total cost (66× reduction)

**Research Significance**  
Clearly demonstrates the value of hierarchical coordination between long-term repowering decisions and short-term MPC operations.

**Date Added**: June 2026

---

## Repository Structure

| File                          | Purpose                                              |
|-------------------------------|------------------------------------------------------|
| `README.md`                   | Project overview and usage instructions              |
| `RESULTS.md`                  | Research outputs and validation summary              |
| `Aghajani_Faraz_Conference2.pdf` | Scientific publication and methodology            |
| `IMG_5550.png`                | Overall flexibility framework & numerical results    |
| `IMG_5572.png`                | Stackelberg architecture & MPC-repowering results    |
| `hydro_flex.py`               | Pumped-storage flexibility modeling                  |
| `mpc_control.py`              | Receding-horizon MPC implementation                  |
| `rep_flex_mpc.py`             | Repowering + flexibility coordination                |
| `stackelberg_mpc.py`          | Strategic planning & operational interaction         |

---

## Current Development Status
**Research Maturity**  
- ✅ Conceptual framework & two-level Stackelberg architecture established  
- ✅ Optimization models fully implemented  
- ✅ Conference publication completed  
- ✅ Numerical validation & infeasibility diagnostics performed  
- ✅ Professional research infographics developed  
- 🔄 Ongoing work on stochastic extensions and market integration  

---

## Future Research Directions
- Large-scale IEEE benchmarks
- Enhanced stochastic scenario generation
- Multi-stage MPC and full stochastic MILP-MPC
- Electricity market integration & flexibility pricing
- Sector coupling and Power-to-X (hydrogen) integration

---

## Citation
If this repository contributes to your work, please cite the associated conference publication and acknowledge the **Flexibility Pocket** research framework.

**Author**: Faraz Aghajani Aalizamini  
**Research Area**: Power System Flexibility • Generation Expansion Planning • Repowering • Stochastic Optimization • Model Predictive Control • Energy Transition

---

*Last updated: June 16, 2026*
