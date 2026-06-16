# REQUIREMENTS

## Project overview
Flexibility-oriented power system planning using Pyomo and PuLP. This repository implements optimization models and experiments for assessing system flexibility (unit commitment, storage scheduling, reserves, and scenario-based planning) using Python-based optimization frameworks and common solvers.

---

## Python environment
- Recommended Python version: 3.10 or 3.11
  - (These versions are well-supported by Pyomo, PuLP, and the scientific stack. Python 3.9 will often work but 3.8 and older are not recommended.)

---

## Core dependencies

The table below lists the primary Python packages used in optimization and power system research for this project.

| Package | Purpose | Suggested minimum version |
|--------:|---------|--------------------------:|
| pyomo | Modeling language for algebraic modeling (MILP, NLP) | >= 6.4 |
| pulp | LP/MIP modeling interface (lightweight) | >= 2.7 |
| numpy | Numerical arrays & linear algebra | >= 1.24 |
| pandas | Data handling and time series | >= 2.1 |
| matplotlib | Plotting and figure generation | >= 3.7 |
| scipy | Scientific routines (optimization, linear algebra) | >= 1.10 |
| networkx | Network modeling & graph algorithms (topology tests) | >= 3.1 |
| pandapower (optional but recommended) | Power system modelling and power flow checks | >= 2.5 |
| seaborn (optional) | Statistical plotting / visuals | >= 0.12 |

Notes:
- Use the versions above as a baseline; exact pins depend on your environment and other packages.
- Solver binaries (GLPK, CBC, Gurobi, CPLEX) are separate from Python packages — see "Solver requirements".

---

## Installation instructions

Two recommended workflows are shown below: virtualenv (venv) and conda (recommended for consistent solver/system package installation).

A. Using python -m venv (virtualenv)
1. Create and activate a virtual environment:
   - Linux/macOS:
     - python3 -m venv .venv
     - source .venv/bin/activate
   - Windows (PowerShell):
     - python -m venv .venv
     - .\.venv\Scripts\Activate.ps1
2. Upgrade pip and install Python dependencies:
   - pip install --upgrade pip setuptools wheel
   - pip install -r requirements.txt
3. Install solvers (system-level) — see "Solver requirements" below.

B. Using conda (recommended for solvers & reproducibility)
1. Create and activate a conda environment:
   - conda create -n flex-ps python=3.11 -y
   - conda activate flex-ps
2. Install core packages from conda-forge:
   - conda install -c conda-forge pyomo pulp numpy pandas matplotlib scipy networkx seaborn pandapower -y
3. Install solvers via conda where possible (see "Solver requirements").

C. Notes on editable development install:
- If you plan to develop code inside the repo:
  - pip install -e .  (if you add a setup.py / pyproject later)

---

## Solver requirements

Optimization solvers are external executables; install and configure at least one solver before running models.

Open-source solvers:
- GLPK
  - Good for LPs and small MILPs.
  - Install:
    - Debian/Ubuntu: sudo apt-get install glpk-utils libglpk-dev
    - macOS (Homebrew): brew install glpk
    - conda: conda install -c conda-forge glpk
  - Pyomo solver name: `glpk`
  - PuLP solver backend: GLPK_CMD (requires glpsol on PATH)

- CBC (COIN-OR Branch and Cut)
  - Good open-source MIP solver with better MILP performance than GLPK.
  - Install:
    - conda: conda install -c conda-forge coincbc
    - Debian/Ubuntu: apt-get packages may vary; prefer conda or build from source.
  - Pyomo solver name: `cbc` (or use `SolverFactory('cbc')`)
  - PuLP default solver is CBC (PULP_CBC_CMD)

Commercial solvers (recommended for large/complex experiments; academic licenses often available):
- Gurobi
  - High-performance commercial solver. Free academic licenses are available for qualified users.
  - Install from Gurobi website; ensure license is activated and `gurobi` executable is on PATH or configured in the solver interface.
  - Pyomo solver name: `gurobi`
- CPLEX
  - High-performance commercial solver from IBM with academic licensing available.
  - Install from IBM, set environment variables and/or add to PATH.
  - Pyomo solver name: `cplex`

Configuring solvers in Pyomo:
- Example: pyo.SolverFactory('gurobi'). For GLPK use `glpk`, CBC use `cbc`.
- Ensure solver executables are on PATH or provide the executable path when creating the solver object.

Which solver to choose?
- Development & reproducibility: GLPK or CBC (open-source).
- Large-scale/production experiments: Gurobi or CPLEX for speed and robustness.

---

## Optional / development dependencies

These packages are not required to run experiments but are useful for development, testing, and reproducibility:

| Package | Purpose |
|--------:|---------|
| jupyterlab | Interactive notebooks for exploration |
| ipython | REPL convenience |
| pytest | Unit testing |
| black | Code formatting |
| mypy | Static type checking |
| sphinx | Documentation generation |
| tox | Multi-environment testing |

Install them with:
- pip install --upgrade jupyterlab pytest black mypy sphinx tox
or add them to a `dev-requirements.txt` or `extras_require` in your packaging.

---

## Verification

A. Quick import & solver availability check (one-liner)
- After activating your environment and installing requirements, run:

```bash
python - <<'PY'
import pyomo.environ as pyo, pulp, sys
print("pyomo:", getattr(pyo, '__version__', 'unknown'))
print("pulp:", getattr(pulp, '__version__', 'unknown'))
for s in ("glpk","cbc","gurobi","cplex"):
    try:
        avail = pyo.SolverFactory(s).available(exception_flag=False)
    except Exception as e:
        avail = False
    print(f"Solver {s}: available={avail}")
PY
```

B. Minimal PuLP sanity test
- Create and run a short script to check modeling + solver end-to-end using PuLP + CBC:

```python
# save as verify_pulp.py and run: python verify_pulp.py
import pulp
x = pulp.LpVariable('x', lowBound=0)
y = pulp.LpVariable('y', lowBound=0)
prob = pulp.LpProblem('simple', pulp.LpMaximize)
prob += 3*x + 4*y
prob += 2*x + y <= 20
prob += x + 2*y <= 20
prob.solve(pulp.PULP_CBC_CMD(msg=False))
print("Status:", pulp.LpStatus[prob.status])
print("Objective:", pulp.value(prob.objective))
print("x,y:", x.value(), y.value())
```

C. Minimal Pyomo sanity test
- Create and run a short Pyomo script to check solver handshake:

```python
# save as verify_pyomo.py and run: python verify_pyomo.py
from pyomo.environ import ConcreteModel, Var, Objective, Constraint, NonNegativeReals, SolverFactory
m = ConcreteModel()
m.x = Var(within=NonNegativeReals)
m.y = Var(within=NonNegativeReals)
m.obj = Objective(expr=3*m.x + 4*m.y, sense=1)  # maximize
m.c1 = Constraint(expr=2*m.x + m.y <= 20)
m.c2 = Constraint(expr=m.x + 2*m.y <= 20)

# choose solver: "cbc" or "glpk" (or "gurobi" if available)
solver = SolverFactory('cbc')
res = solver.solve(m, tee=False)
print(res.solver.termination_condition)
print("Objective:", m.obj())
print("x,y:", m.x(), m.y())
```

If the scripts run and report a feasible solution and solver availability is True for at least one solver, your installation is functional.

---

## Notes for reproducible research
- Record your environment (python version, pip freeze or conda list) alongside experiment results. Example:
  - pip freeze > requirements-frozen.txt
  - conda list --explicit > conda-environment.txt
- For experiments using commercial solvers, record solver version and license type.
- Consider adding a Dockerfile or conda environment.yml for fully reproducible runs.

---

If you'd like, I can:
- Commit these files to the repository on a new branch and open a pull request, or
- Add a small verification script (verify_env.py) to the `scripts/` directory and commit it.
