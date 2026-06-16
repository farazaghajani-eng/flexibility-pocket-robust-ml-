# Results and Research Artifacts  
**Flexibility Pocket Repository**

## Repository Overview
This repository presents a focused research framework for **flexibility-oriented power system planning and operation**. It integrates generation repowering decisions with deterministic Model Predictive Control (MPC) in a hierarchical (Stackelberg) architecture.

The core objective is to quantify and mitigate system flexibility deficits under high renewable penetration through coordinated investment (repowering) and operational strategies.  

Key components include:
- Thermal unit rehabilitation (repowering)
- Flexibility-aware decision making
- Stochastic/robust elements under renewable uncertainty
- Receding-horizon MPC control
- Stackelberg leader-follower coordination
- Energy storage (battery & hydrogen-like) and demand-side flexibility modeling

Models are implemented in **Python** using **Pyomo** (planning/repowering) and **PuLP** (MPC), providing reproducible testbeds for low-carbon power systems research.

---

## Research Outputs

### 1. Conference Research Paper
**`Aghajani_Faraz_Conference2.pdf`**

**Description**  
This conference paper serves as the principal scientific reference for the repository. It details the theoretical foundations, mathematical formulations, and validation of the implemented models.

**Research Scope**  
The work investigates flexibility enhancement strategies through:
- Thermal generation rehabilitation (repowering)
- Flexibility-oriented scheduling
- Renewable uncertainty management
- Pumped-storage hydro and storage integration
- Demand-side flexibility aggregation
- Market-aware operational planning

**Methodological Contributions**  
Integrated framework combining:
1. Demand Envelope Characterization
2. Stochastic Unit Commitment & Scheduling
3. Flexibility-Constrained Generation Expansion Planning (GEP)
4. MPC-Based Operational Decision Making
5. Repowering Investment Decisions under uncertainty

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

### 2. Research Visualization / Infographic
**`IMG_5572.png`**

**Description**  
High-quality research infographic summarizing the conceptual framework, methodology, and numerical outcomes of the flexibility-oriented planning study.

**Contents**  
- Two-level Stackelberg architecture: Strategic Leader (Repowering Planner – Pyomo) ↔ Operational Follower (Receding-Horizon MPC – PuLP)
- Key decisions: μPmax investment, μRU ramp targets, headroom dynamics
- System features: Battery & Hydrogen facilities
- Robust infeasibility management with dynamic slacks

**Representative Results** (from the infographic)
- **Baseline**: Model infeasible (69.6 MW flexibility deficit)
- **With Repowering**: +139.2 MW added capacity
- **Final Outcome**: $10.6k total cost (66× reduction)

**Research Significance**  
Demonstrates the substantial value of coordinated long-term investment and short-term operational flexibility for supporting the energy transition while ensuring system adequacy and economic performance.

**Date Added**: June 2026

---

## Repository Structure

| File                    | Purpose                                      |
|-------------------------|----------------------------------------------|
| `README.md`             | Project overview and usage instructions      |
| `RESULTS.md`            | Research outputs and validation summary      |
| `Aghajani_Faraz_Conference2.pdf` | Scientific publication and methodology |
| `IMG_5572.png`          | Visual summary of research framework & results |
| `hydro_flex.py`         | Pumped-storage flexibility modeling          |
| `mpc_control.py`        | Receding-horizon MPC implementation          |
| `rep_flex_mpc.py`       | Repowering + flexibility coordination        |
| `stackelberg_mpc.py`    | Strategic planning & operational interaction |

---

## Current Development Status
**Research Maturity**  
- ✅ Conceptual framework & two-level architecture established  
- ✅ Optimization models fully implemented  
- ✅ Conference publication completed  
- ✅ Numerical validation & infeasibility diagnostics performed  
- ✅ Professional research infographic developed  
- 🔄 Ongoing work on market integration and stochastic extensions  

---

## Future Research Directions
The repository continues to evolve toward a comprehensive **flexibility adequacy assessment and investment planning** platform.

**Planned Extensions**:
- **Near-term**: Large-scale IEEE benchmarks, enhanced stochastic scenarios, multi-stage MPC, advanced dashboards
- **Mid-term**: Electricity market integration, flexibility pricing, endogenous valuation, risk-aware planning
- **Long-term**: Sector coupling, Power-to-X (hydrogen), seasonal storage, AI-assisted forecasting & control

---

## Citation
If this repository contributes to your work, please cite the associated conference publication and acknowledge the **Flexibility Pocket** research framework.

**Author**: Faraz Aghajani Aalizamini  
**Research Area**: Power System Flexibility • Generation Expansion Planning • Repowering • Stochastic Optimization • Model Predictive Control • Energy Transition

---

*Last updated: June 16, 2026*
