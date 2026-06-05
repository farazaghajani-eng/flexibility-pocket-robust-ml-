"""
stackelberg_mpc.py

Stackelberg upper-level planner (leader) that chooses per-generator small investments
(ΔRU, ΔPmax) anticipating the follower MPC response.

Requirements:
- mpc_control.py must be in the same folder and expose:
    - build_default_params()
    - MPCController

Behavior:
- Leader evaluates candidate investments by running the MPC (follower) and computing:
    leader_obj = invest_cost + operational_cost + M_penalty * total_slack
- Optimization of the leader decision is performed with a derivative-free optimizer
  (scipy.optimize.minimize Nelder-Mead/Powell if available, otherwise random search).
"""
import copy
import time
import math
import numpy as np
import matplotlib.pyplot as plt

# Try to import SciPy; if not available fallback to random search
try:
    from scipy.optimize import minimize
    SCIPY_AVAILABLE = True
except Exception:
    SCIPY_AVAILABLE = False

# Import the MPC controller and parameter builder from mpc_control.py
try:
    from mpc_control import build_default_params, MPCController
except Exception:
    raise ImportError("Could not import mpc_control. Make sure mpc_control.py (from the MPC implementation) "
                      "is in the same directory and defines build_default_params() and MPCController.")


class StackelbergPlanner:
    def __init__(self, base_params, Cost_inv_RU=None, Cost_inv_Pmax=None,
                 Max_Delta_RU=None, Max_Delta_Pmax=None, leader_M_penalty=None,
                 mpc_solver_name='CBC', verbosity=False):
        """
        base_params: dict from build_default_params() describing the system (G,S,Pmin,Pmax,RU,...)
        Cost_inv_RU / Cost_inv_Pmax: dicts cost per MW/MW/hr for investments (if None uses defaults)
        Max_Delta_RU / Max_Delta_Pmax: dicts for upper bounds on leader decisions
        leader_M_penalty: penalty multiplier for slack usage in the leader objective (overrides MPC's M_penalty)
        """
        self.base_params = copy.deepcopy(base_params)
        self.G = base_params['G']
        self.nG = len(self.G)

        # investment costs
        if Cost_inv_RU is None:
            Cost_inv_RU = {g: 50.0 for g in self.G}
        if Cost_inv_Pmax is None:
            Cost_inv_Pmax = {g: 10.0 for g in self.G}
        self.Cost_inv_RU = Cost_inv_RU
        self.Cost_inv_Pmax = Cost_inv_Pmax

        # max allowed deltas
        if Max_Delta_RU is None:
            Max_Delta_RU = {g: 5.0 for g in self.G}
        if Max_Delta_Pmax is None:
            Max_Delta_Pmax = {g: 50.0 for g in self.G}
        self.Max_Delta_RU = Max_Delta_RU
        self.Max_Delta_Pmax = Max_Delta_Pmax

        # leader slack penalty (on top of MPC slacks; used for leader objective)
        if leader_M_penalty is None:
            leader_M_penalty = 1e5
        self.leader_M_penalty = leader_M_penalty

        self.mpc_solver_name = mpc_solver_name
        self.verbosity = verbosity

        # cache for (delta_RU, delta_Pmax) -> evaluation results to speed up
        self._eval_cache = {}

        # default MPC settings (we instantiate an MPC controller inside leader eval)
        self.mpc_kwargs = {
            'solver_name': mpc_solver_name,
            'verbosity': False
        }

    def _flatten_decision_vec(self, x):
        """
        interpret vector x of length 2*nG as [delta_RU_g1..gN, delta_Pmax_g1..gN]
        """
        x = np.asarray(x, dtype=float).flatten()
        if x.size != 2 * self.nG:
            raise ValueError("Decision vector length mismatch.")
        delta_RU = {g: float(x[i]) for i, g in enumerate(self.G)}
        delta_Pmax = {g: float(x[self.nG + i]) for i, g in enumerate(self.G)}
        return delta_RU, delta_Pmax

    def _clip_decisions(self, delta_RU, delta_Pmax):
        """
        Ensure decisions are within [0, MaxDelta]
        """
        delta_RU_c = {g: max(0.0, min(self.Max_Delta_RU[g], delta_RU[g])) for g in self.G}
        delta_Pmax_c = {g: max(0.0, min(self.Max_Delta_Pmax[g], delta_Pmax[g])) for g in self.G}
        return delta_RU_c, delta_Pmax_c

    def _build_upgraded_params(self, delta_RU, delta_Pmax):
        """
        Return a new params dict where Pmax and RU are upgraded according to deltas.
        """
        p = copy.deepcopy(self.base_params)
        # modify RU and Pmax
        for g in self.G:
            p['Pmax'][g] = float(p['Pmax'][g]) + float(delta_Pmax[g])
            p['RU'][g] = float(p['RU'][g]) + float(delta_RU[g])
            p['RD'][g] = float(p['RD'][g]) + float(delta_RU[g])  # assume symmetric improvement
        return p

    def _compute_operational_cost_and_slack(self, params_upgraded, run_visualize=False):
        """
        Runs the MPC as follower with upgraded params and returns:
          - total_fuel_cost (sum over steps of fuel costs)
          - total_slack (sum over steps of slacks)
          - history (the MPC history dict)
        """
        # instantiate fresh MPCController with upgraded params
        mpc = MPCController(params_upgraded, solver_name=self.mpc_solver_name, verbosity=False)
        # align MPC leader penalty with leader_M_penalty so mp controller objective uses big M for slacks:
        mpc.M_penalty = max(mpc.M_penalty, self.leader_M_penalty)

        start_time = time.time()
        hist = mpc.run_receding_horizon(auto_enhance=True, visualize=False)
        elapsed = time.time() - start_time
        if self.verbosity:
            print(f"  MPC run complete (took {elapsed:.2f}s).")

        # compute fuel cost from recorded dispatch
        total_fuel_cost = 0.0
        DeltaT_hours = params_upgraded['DeltaT'] / 60.0
        for t, pdict in hist['dispatch'].items():
            for g, pval in pdict.items():
                total_fuel_cost += params_upgraded['C_inc'][g] * float(pval) * DeltaT_hours

        total_slack = sum(hist['slacks'])

        return total_fuel_cost, total_slack, hist

    def leader_evaluate(self, x):
        """
        Given flat decision vector x, return leader objective and diagnostics:
          obj = investment_cost + operational_cost + leader_M_penalty * total_slack
        The function caches evaluations keyed by clipped decision vectors for speed.
        Returns (obj, info_dict)
        """
        delta_RU_raw, delta_Pmax_raw = self._flatten_decision_vec(x)
        # clip to feasible range
        delta_RU, delta_Pmax = self._clip_decisions(delta_RU_raw, delta_Pmax_raw)

        # make a cache key (rounded to a few decimals to merge near-equals)
        key = tuple([round(delta_RU[g], 6) for g in self.G] + [round(delta_Pmax[g], 6) for g in self.G])
        if key in self._eval_cache:
            if self.verbosity:
                print("  Using cached evaluation.")
            return self._eval_cache[key]['obj'], self._eval_cache[key]['info']

        # Build upgraded params and run follower (MPC)
        params_up = self._build_upgraded_params(delta_RU, delta_Pmax)
        op_cost, total_slack, hist = self._compute_operational_cost_and_slack(params_up)

        # investment cost
        invest_cost = 0.0
        for g in self.G:
            invest_cost += self.Cost_inv_RU[g] * delta_RU[g] + self.Cost_inv_Pmax[g] * delta_Pmax[g]

        # leader objective
        obj = invest_cost + op_cost + (self.leader_M_penalty * total_slack)

        info = {
            'delta_RU': delta_RU,
            'delta_Pmax': delta_Pmax,
            'invest_cost': invest_cost,
            'operational_cost': op_cost,
            'total_slack': total_slack,
            'history': hist
        }

        # cache and return
        self._eval_cache[key] = {'obj': obj, 'info': info}
        if self.verbosity:
            print(f"  Eval: invest={invest_cost:.2f}, op={op_cost:.2f}, slack={total_slack:.4f}, obj={obj:.2f}")
        return obj, info

    def optimize(self, x0=None, method='Nelder-Mead', maxiter=50, random_search_iters=200):
        """
        Optimize leader decision vector.
        - x0: initial guess 1D array length 2*nG. If None, starts at zeros.
        - method: 'Nelder-Mead' or 'Powell' (requires SciPy), otherwise random-search fallback.
        - Returns best_info dict with keys: 'x', 'obj', 'info'.
        """
        if x0 is None:
            x0 = np.zeros(2 * self.nG)

        # wrap objective for scipy (only if scipy available)
        def _obj_wrapped(x):
            # clip and evaluate
            obj, _ = self.leader_evaluate(x)
            return float(obj)

        best_result = {'x': None, 'obj': float('inf'), 'info': None}

        if SCIPY_AVAILABLE and method in ['Nelder-Mead', 'Powell']:
            if self.verbosity:
                print("Starting SciPy minimize (derivative-free) outer optimization...")
            res = minimize(_obj_wrapped, x0, method=method,
                           options={'maxiter': maxiter, 'disp': self.verbosity})
            x_opt = res.x
            obj_opt, info_opt = self.leader_evaluate(x_opt)
            best_result.update({'x': x_opt, 'obj': obj_opt, 'info': info_opt})
            return best_result
        else:
            # Randomized search fallback + local improvement: sample around x0
            if self.verbosity:
                print("SciPy not available or unsupported method. Running randomized search fallback.")

            # start with initial guess
            cur_x = np.array(x0, dtype=float)
            cur_obj, cur_info = self.leader_evaluate(cur_x)
            best_result.update({'x': cur_x, 'obj': cur_obj, 'info': cur_info})

            rng = np.random.RandomState(0)
            for it in range(random_search_iters):
                # sample perturbation scaled by max bounds
                cand = cur_x.copy()
                # generate gaussian noise scaled by max values
                for i, g in enumerate(self.G):
                    # delta RU
                    ru_idx = i
                    p_idx = self.nG + i
                    ru_scale = self.Max_Delta_RU[g]
                    p_scale = self.Max_Delta_Pmax[g]
                    cand[ru_idx] += rng.normal(scale=0.3 * ru_scale)
                    cand[p_idx] += rng.normal(scale=0.3 * p_scale)
                # clip
                # convert to delta maps and clip
                dRU_c, dP_c = self._flatten_decision_vec(cand)
                dRU_c, dP_c = self._clip_decisions(dRU_c, dP_c)
                # back to vector
                cand_vec = np.array([dRU_c[g] for g in self.G] + [dP_c[g] for g in self.G], dtype=float)

                cand_obj, cand_info = self.leader_evaluate(cand_vec)
                if cand_obj < best_result['obj'] - 1e-6:
                    best_result.update({'x': cand_vec, 'obj': cand_obj, 'info': cand_info})
                    cur_x = cand_vec
                    cur_obj = cand_obj
                    if self.verbosity:
                        print(f" Random search iter {it}: new best obj = {cand_obj:.2f}")

            return best_result


# ------------------------------------------------------------
# Demo / example usage
# ------------------------------------------------------------
def demo_stackelberg_run(verbosity=True):
    # Build base params (demo network)
    base_params = build_default_params()

    # Add investment cost params to base_params (so MPC can pick them up if needed)
    # We'll define them here separately for the leader
    Cost_inv_RU = {g: 50.0 for g in base_params['G']}
    Cost_inv_Pmax = {g: 10.0 for g in base_params['G']}
    Max_Delta_RU = {g: 5.0 for g in base_params['G']}
    Max_Delta_Pmax = {g: 40.0 for g in base_params['G']}

    # Instantiate planner
    planner = StackelbergPlanner(base_params,
                                 Cost_inv_RU=Cost_inv_RU,
                                 Cost_inv_Pmax=Cost_inv_Pmax,
                                 Max_Delta_RU=Max_Delta_RU,
                                 Max_Delta_Pmax=Max_Delta_Pmax,
                                 leader_M_penalty=1e5,
                                 mpc_solver_name='CBC',
                                 verbosity=verbosity)

    # Baseline run (no investments)
    x0 = np.zeros(2 * len(base_params['G']))
    base_obj, base_info = planner.leader_evaluate(x0)
    print("\nBaseline (no investment):")
    print(f"  Obj = {base_obj:.2f}, invest={base_info['invest_cost']:.2f}, op={base_info['operational_cost']:.2f}, slack={base_info['total_slack']:.4f}")

    # Optimize leader decisions
    opt = planner.optimize(x0=x0, method='Nelder-Mead', maxiter=30, random_search_iters=300)
    print("\nOptimization result:")
    print(f"  Best obj = {opt['obj']:.2f}")
    best_delta_RU = opt['info']['delta_RU']
    best_delta_Pmax = opt['info']['delta_Pmax']
    print("  Best ΔRU per gen:", best_delta_RU)
    print("  Best ΔPmax per gen:", best_delta_Pmax)
    print("  Invest cost:", opt['info']['invest_cost'])
    print("  Operational cost after invest:", opt['info']['operational_cost'])
    print("  Total slack after invest:", opt['info']['total_slack'])

    # Compare baseline vs after-investment dispatch visually (use hist from info)
    hist_base = base_info['history']
    hist_opt = opt['info']['history']
    times = sorted(hist_base['dispatch'].keys())
    total_base = [sum(hist_base['dispatch'][t].values()) for t in times]
    total_opt = [sum(hist_opt['dispatch'][t].values()) for t in times]
    demand = [base_params['D'][t] for t in times]
    plt.figure(figsize=(10, 4))
    plt.plot(times, demand, '-k', label='Demand')
    plt.plot(times, total_base, '--C0', label='Baseline dispatch')
    plt.plot(times, total_opt, '-C1', label='After-invest dispatch')
    plt.xlabel('Time')
    plt.ylabel('MW')
    plt.legend()
    plt.title('Dispatch: baseline vs after Stackelberg investments')
    plt.tight_layout()
    plt.show()

    # Plot total slacks
    plt.figure(figsize=(8, 3))
    plt.plot(times, hist_base['slacks'], '--C0', label='Baseline slacks')
    plt.plot(times, hist_opt['slacks'], '-C1', label='After-invest slacks')
    plt.xlabel('Time')
    plt.ylabel('Slack (aggregate)')
    plt.legend()
    plt.title('Aggregate slack usage (baseline vs after invest)')
    plt.tight_layout()
    plt.show()

    return opt


if __name__ == "__main__":
    demo_stackelberg_run(verbosity=True)
