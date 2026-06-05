"""
mpc_control.py

Deterministic receding-horizon MPC scheduler (PuLP) for thermal units + storage.

Core features:
- Rolling horizon finite-horizon MILP solved at each time step.
- Ramping dynamics, per-time reserve constraints, battery/H2 storage, and slacks.
- Automatic enhancement strategies (slack penalties, temporary RU boosts) when infeasible.
- Diagnostics to identify binding constraints and flexibility bottlenecks.
- Visualization utilities.

Usage:
- Integrate with your two-stage planning outputs by passing upgraded Pmax/RU dicts to MPCController.
- Run a demo with the builtin small test case by calling: python mpc_control.py

Note: This implementation uses PuLP and CBC to keep compatibility with your repo.
For large-scale problems, consider Pyomo + Gurobi/CPLEX for better performance and warm-starts.
"""
import math
import copy
import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import pulp as pl

# ------------------------------------------------------------
# Parameter helpers (adapt these to read your planning outputs)
# ------------------------------------------------------------
def build_default_params():
    """
    Build a small test system. Replace or feed in results from the two-stage planning model.
    Returns a dict with generator + storage + horizon params.
    """
    params = {}
    G = list(range(1, 6))    # 5 thermal units for demo
    T_total = 24             # total simulation length (hours)
    H = 6                    # MPC horizon (hours)
    S = [1, 2]               # 2 storage units (could represent battery & H2)

    DeltaT = 60.0            # minutes (use 60 to make RU units per hour consistent)
    # Generator data
    Pmin = {g: 20 for g in G}
    Pmax = {g: 100 for g in G}     # these should come from investment stage; demo uses base Pmax
    RU = {g: 20 for g in G}        # MW per hour ramp rate (use consistent time units)
    RD = {g: 20 for g in G}
    C_inc = {g: 8 + g for g in G}  # fuel cost ($/MWh)
    # If you use investment results, pass upgraded Pmax and RU dicts here.

    # Storage data
    eta_ch = {1: 0.95, 2: 0.90}
    eta_dis = {1: 0.95, 2: 0.90}
    E_max = {1: 200.0, 2: 500.0}
    P_ch_max = {1: 50.0, 2: 80.0}
    P_dis_max = {1: 50.0, 2: 80.0}
    SOC0 = {1: 100.0, 2: 200.0}

    # Demand & reserves (deterministic forecast)
    D = {t: 400 + 80.0 * math.sin(2.0 * math.pi * (t-1)/24.0) for t in range(1, T_total+1)}
    R_up_req = {t: 50.0 for t in range(1, T_total+1)}
    R_down_req = {t: 50.0 for t in range(1, T_total+1)}

    # Initial dispatch (p0) - start at Pmin
    p0 = {g: float(Pmin[g]) for g in G}

    params.update({
        'G': G, 'S': S, 'T_total': T_total, 'H': H, 'DeltaT': DeltaT,
        'Pmin': Pmin, 'Pmax': Pmax, 'RU': RU, 'RD': RD, 'C_inc': C_inc,
        'eta_ch': eta_ch, 'eta_dis': eta_dis, 'E_max': E_max, 'P_ch_max': P_ch_max,
        'P_dis_max': P_dis_max, 'SOC0': SOC0,
        'D': D, 'R_up_req': R_up_req, 'R_down_req': R_down_req,
        'p0': p0
    })
    return params

# ------------------------------------------------------------
# MPC Controller
# ------------------------------------------------------------
class MPCController:
    def __init__(self, params, solver_name='CBC', verbosity=False):
        """
        params: dict returned by build_default_params or constructed from planning outputs.
        solver_name: 'CBC' (PuLP) by default. For other solvers adapt the solver factory call.
        """
        self.params = copy.deepcopy(params)
        self.G = params['G']
        self.S = params['S']
        self.T_total = params['T_total']
        self.H = params['H']
        self.DeltaT = params['DeltaT']  # minutes per step (we assume RU in MW/hour so careful unit conversion)
        # Convert RU per hour to RU per step if DeltaT is in minutes:
        # RU_per_step = RU_per_hour * (DeltaT / 60)
        self.Pmin = params['Pmin']
        self.Pmax = params['Pmax']
        self.RU = params['RU']
        self.RD = params['RD']
        self.C_inc = params['C_inc']
        self.eta_ch = params['eta_ch']
        self.eta_dis = params['eta_dis']
        self.E_max = params['E_max']
        self.P_ch_max = params['P_ch_max']
        self.P_dis_max = params['P_dis_max']
        self.SOC0 = params['SOC0']
        self.D = params['D']
        self.R_up_req = params['R_up_req']
        self.R_down_req = params['R_down_req']
        self.p0 = params['p0']
        self.solver_name = solver_name
        self.verbosity = verbosity

        # Costs and penalties for enhancements
        self.M_penalty = 1e5       # penalty for balance/reserve slacks (large)
        self.operational_RU_cost = {g: 100.0 for g in self.G}  # $ per MW/hr for temporary RU boost
        # limits on temporary RU boost
        self.max_temp_RU_boost = {g: 5.0 for g in self.G}  # MW/hr

        # Data storage for timeline
        self.history = {
            'dispatch': {},    # dispatch per time step for each g
            'pch': {}, 'pdis': {}, 'SOC': {},
            'cost': [], 'slacks': []
        }

    # -----------------------------
    # Problem builder for horizon
    # -----------------------------
    def build_mpc_problem(self, t0, H, state, enhancements=None, use_slacks=True, allow_temp_RU_boost=False):
        """
        Build a PuLP LpProblem for times t = t0 .. t0+H-1 (1-indexed time in params['D']).
        state: dict with keys 'p0' (per-generator current MW) and 'SOC' (per-storage current SOC).
        enhancements: dict to control enhancement options (not used heavily here).
        use_slacks: if True include balance/reserve slacks with penalty; otherwise standard constraints (may be infeasible).
        allow_temp_RU_boost: if True include temporary RU boost variables (penalized) to restore feasibility.
        Returns (prob, var_dict) where var_dict contains references to variables for diagnostics.
        """
        prob = pl.LpProblem("MPC_Horizon", pl.LpMinimize)

        # index sets
        times = [t for t in range(t0, min(t0+H, self.T_total+1))]  # inclusive times

        # Variables
        p = pl.LpVariable.dicts('p', (self.G, times), lowBound=0, cat='Continuous')  # dispatch MW
        pch = pl.LpVariable.dicts('pch', (self.S, times), lowBound=0, cat='Continuous')
        pdis = pl.LpVariable.dicts('pdis', (self.S, times), lowBound=0, cat='Continuous')
        SOC = pl.LpVariable.dicts('SOC', (self.S, times), lowBound=0, cat='Continuous')

        # Ramp-up available variables (to model reserve and ramp availability)
        rup_avail = pl.LpVariable.dicts('rup_avail', (self.G, times), lowBound=0, cat='Continuous')
        rdown_avail = pl.LpVariable.dicts('rdown_avail', (self.G, times), lowBound=0, cat='Continuous')

        # Slack variables
        slack_bal_pos = pl.LpVariable.dicts('slack_bal_pos', times, lowBound=0, cat='Continuous') if use_slacks else {}
        slack_bal_neg = pl.LpVariable.dicts('slack_bal_neg', times, lowBound=0, cat='Continuous') if use_slacks else {}
        slack_res_up = pl.LpVariable.dicts('slack_res_up', times, lowBound=0, cat='Continuous') if use_slacks else {}
        slack_res_down = pl.LpVariable.dicts('slack_res_down', times, lowBound=0, cat='Continuous') if use_slacks else {}

        # Optional temporary RU boost
        temp_RU = None
        if allow_temp_RU_boost:
            temp_RU = pl.LpVariable.dicts('temp_RU', (self.G, times), lowBound=0, cat='Continuous')

        # Objective: fuel cost + optional RU boost cost + slack penalties
        fuel_cost = pl.lpSum([self.C_inc[g] * p[g][t] * (self.DeltaT / 60.0) for g in self.G for t in times])
        obj = fuel_cost
        if allow_temp_RU_boost:
            obj += pl.lpSum([self.operational_RU_cost[g] * temp_RU[g][t] for g in self.G for t in times])
        if use_slacks:
            obj += self.M_penalty * pl.lpSum([slack_bal_pos[t] + slack_bal_neg[t] + slack_res_up[t] + slack_res_down[t] for t in times])

        prob += obj

        # Constraints
        # 1) generator bounds
        for g in self.G:
            for t in times:
                prob += p[g][t] >= self.Pmin[g], f"pmin_g{g}_t{t}"
                prob += p[g][t] <= self.Pmax[g], f"pmax_g{g}_t{t}"

        # 2) storage bounds
        for s in self.S:
            for t in times:
                prob += pch[s][t] <= self.P_ch_max[s], f"pch_max_s{s}_t{t}"
                prob += pdis[s][t] <= self.P_dis_max[s], f"pdis_max_s{s}_t{t}"
                prob += SOC[s][t] <= self.E_max[s], f"SOC_max_s{s}_t{t}"

        # 3) SOC dynamics
        for s in self.S:
            for idx, t in enumerate(times):
                if idx == 0:
                    soc_prev = state['SOC'][s]
                else:
                    soc_prev = SOC[s][times[idx-1]]
                prob += SOC[s][t] == soc_prev + (self.eta_ch[s] * pch[s][t] - pdis[s][t] / max(1e-9, self.eta_dis[s])) * (self.DeltaT / 60.0), f"SOC_dyn_s{s}_t{t}"

        # 4) Power balance per time
        for t in times:
            gen_sum = pl.lpSum([p[g][t] for g in self.G])
            storage_net = pl.lpSum([pdis[s][t] - pch[s][t] for s in self.S])
            if use_slacks:
                prob += gen_sum + storage_net + slack_bal_pos[t] - slack_bal_neg[t] == self.D[t], f"Power_balance_t{t}"
            else:
                prob += gen_sum + storage_net == self.D[t], f"Power_balance_t{t}"

        # 5) Ramp availability modeling and reserve constraints
        # Model rup_avail[g,t] <= RU_per_step and rup_avail[g,t] <= Pmax - p[g,t]
        RU_step = {g: self.RU[g] * (self.DeltaT / 60.0) for g in self.G}
        RD_step = {g: self.RD[g] * (self.DeltaT / 60.0) for g in self.G}
        for g in self.G:
            for t in times:
                prob += rup_avail[g][t] <= RU_step[g] + (temp_RU[g][t] if allow_temp_RU_boost else 0), f"rup_cap_g{g}_t{t}"
                prob += rup_avail[g][t] <= self.Pmax[g] - p[g][t] + 1e6 * 0, f"rup_headroom1_g{g}_t{t}"  # linear: rup_avail <= Pmax - p
                prob += rdown_avail[g][t] <= RD_step[g], f"rdown_cap_g{g}_t{t}"
                prob += rdown_avail[g][t] <= p[g][t] - self.Pmin[g] + 1e6 * 0, f"rdown_headroom_g{g}_t{t}"

                if allow_temp_RU_boost:
                    prob += temp_RU[g][t] <= self.max_temp_RU_boost[g], f"temp_RU_max_g{g}_t{t}"

        # Reserve requirements (sum of rup_avail >= R_up_req)
        for t in times:
            if use_slacks:
                prob += pl.lpSum([rup_avail[g][t] for g in self.G]) + slack_res_up[t] >= self.R_up_req[t], f"Reserve_up_req_t{t}"
                prob += pl.lpSum([rdown_avail[g][t] for g in self.G]) + slack_res_down[t] >= self.R_down_req[t], f"Reserve_down_req_t{t}"
            else:
                prob += pl.lpSum([rup_avail[g][t] for g in self.G]) >= self.R_up_req[t], f"Reserve_up_req_t{t}"
                prob += pl.lpSum([rdown_avail[g][t] for g in self.G]) >= self.R_down_req[t], f"Reserve_down_req_t{t}"

        # 6) ramping constraints between consecutive steps (enforce p[t] - p[t-1] <= RU_step)
        for g in self.G:
            for idx, t in enumerate(times):
                if idx == 0:
                    prev_p = state['p0'][g]
                else:
                    prev_p = p[g][times[idx-1]]
                prob += p[g][t] - prev_p <= RU_step[g] + (temp_RU[g][t] if allow_temp_RU_boost else 0), f"ramp_up_g{g}_t{t}"
                prob += prev_p - p[g][t] <= RD_step[g], f"ramp_down_g{g}_t{t}"

        # Return problem and variable dicts for postprocessing
        var_dict = {
            'p': p, 'pch': pch, 'pdis': pdis, 'SOC': SOC,
            'rup_avail': rup_avail, 'rdown_avail': rdown_avail,
            'slack_bal_pos': slack_bal_pos, 'slack_bal_neg': slack_bal_neg,
            'slack_res_up': slack_res_up, 'slack_res_down': slack_res_down,
            'temp_RU': temp_RU
        }
        return prob, var_dict, times

    # -----------------------------
    # Solve & extract results
    # -----------------------------
    def solve_and_extract(self, prob, var_dict, times, time_offset=0):
        """
        Solve the PuLP problem and return a solution dictionary and diagnostics.
        """
        solver = pl.PULP_CBC_CMD(msg=self.verbosity, timeLimit=300)
        result_status = prob.solve(solver)
        status = pl.LpStatus[prob.status]
        if self.verbosity:
            print("Solver status:", status)

        sol = {}
        sol['status'] = status
        sol['objective'] = pl.value(prob.objective) if status == 'Optimal' or status == 'Not Solved' else None

        # Extract variables
        vals = {}
        for k, d in var_dict.items():
            if not d:
                vals[k] = {}
                continue
            vals[k] = {}
            # d is nested dict
            if isinstance(next(iter(d.values())), dict):  # e.g., p[g][t] structure
                for i in d:
                    vals[k][i] = {}
                    for t in d[i]:
                        try:
                            vals[k][i][t] = pl.value(d[i][t])
                        except Exception:
                            vals[k][i][t] = None
            else:
                # d[t] structure
                for t in d:
                    try:
                        vals[k][t] = pl.value(d[t])
                    except Exception:
                        vals[k][t] = None

        sol['vars'] = vals
        sol['times'] = times
        return sol

    # -----------------------------
    # Diagnostics: infeasibility and binding constraints
    # -----------------------------
    def analyze_solution(self, sol):
        """
        Analyze solution to find slacks used and binding limits (approx).
        Returns a report dict.
        """
        report = {}
        status = sol['status']
        report['status'] = status
        slacks_used = []
        if 'slack_bal_pos' in sol['vars']:
            for t, v in sol['vars']['slack_bal_pos'].items():
                if v is not None and v > 1e-6:
                    slacks_used.append(('bal_pos', t, v))
        if 'slack_bal_neg' in sol['vars']:
            for t, v in sol['vars']['slack_bal_neg'].items():
                if v is not None and v > 1e-6:
                    slacks_used.append(('bal_neg', t, v))
        if 'slack_res_up' in sol['vars']:
            for t, v in sol['vars']['slack_res_up'].items():
                if v is not None and v > 1e-6:
                    slacks_used.append(('res_up', t, v))
        if 'slack_res_down' in sol['vars']:
            for t, v in sol['vars']['slack_res_down'].items():
                if v is not None and v > 1e-6:
                    slacks_used.append(('res_down', t, v))

        # Binding ramp/reserve indicators:
        binding = []
        # If variable rup_avail[g][t] is very close to both RU_step and Pmax-p, then it's binding
        rup = sol['vars'].get('rup_avail', {})
        pvals = sol['vars'].get('p', {})
        for g in rup:
            for t in rup[g]:
                rupv = rup[g][t]
                pgv = pvals.get(g, {}).get(t, None)
                if pgv is None or rupv is None:
                    continue
                RU_step = self.RU[g] * (self.DeltaT / 60.0)
                headroom = self.Pmax[g] - pgv
                # check closeness
                eps = 1e-3
                if abs(rupv - RU_step) <= max(1e-2, 1e-3*abs(RU_step)):
                    binding.append(('rup_rate_binding', g, t, RU_step, rupv))
                if abs(rupv - headroom) <= max(1e-2, 1e-3*abs(headroom)):
                    binding.append(('rup_headroom_binding', g, t, headroom, rupv))
        report['slacks_used'] = slacks_used
        report['binding'] = binding
        return report

    # -----------------------------
    # Run receding horizon loop
    # -----------------------------
    def run_receding_horizon(self, auto_enhance=True, visualize=False):
        """
        Runs the MPC from t=1 to T_total.
        auto_enhance: if True, when base solve is infeasible, try enhancements (slacks + temp_RU) and re-solve.
        visualize: if True produce plots at the end.
        """
        # initialize state
        state = {
            'p0': copy.deepcopy(self.p0),
            'SOC': copy.deepcopy(self.SOC0)
        }

        # time loop
        for t in range(1, self.T_total+1):
            # Build base problem (no temporary RU boost, but with slacks)
            prob, var_dict, times = self.build_mpc_problem(t0=t, H=self.H, state=state, use_slacks=True, allow_temp_RU_boost=False)
            sol = self.solve_and_extract(prob, var_dict, times)
            report = self.analyze_solution(sol)

            # If infeasible or slacks used (or solver status not optimal), attempt enhancement if allowed
            if sol['status'] != 'Optimal' or len(report['slacks_used']) > 0:
                if self.verbosity:
                    print(f"At time {t}: base MPC not optimal or used slacks: status={sol['status']}, slacks={report['slacks_used']}")
                if auto_enhance:
                    # Try allowing temporary RU boost (penalized) to restore feasibility or reduce slacks
                    prob_e, var_dict_e, times_e = self.build_mpc_problem(t0=t, H=self.H, state=state, use_slacks=True, allow_temp_RU_boost=True)
                    sol_e = self.solve_and_extract(prob_e, var_dict_e, times_e)
                    report_e = self.analyze_solution(sol_e)
                    if self.verbosity:
                        print(f" Enhancement attempt status: {sol_e['status']}, slacks after enhancement: {report_e['slacks_used']}")
                    # Choose the feasible solution with lower total slack or lower objective
                    chosen = sol_e if sol_e['status'] == 'Optimal' else sol
                    chosen_report = report_e if sol_e['status'] == 'Optimal' else report
                    sol = chosen
                    report = chosen_report
                else:
                    # keep base but warn
                    if self.verbosity:
                        print("Auto-enhance disabled: continuing with base (may use slacks).")

            # Apply first-step actions (first time in sol['times'])
            if sol['status'] not in ['Optimal', 'Not Solved', '']:
                # No feasible solution found
                if self.verbosity:
                    print(f"Time {t}: no feasible solution found by solver. Attempting to apply fallback: keep p0 and only discharge storage if possible.")
                # fallback: keep p = p0, no storage change
                action_p = copy.deepcopy(state['p0'])
                action_pch = {s: 0.0 for s in self.S}
                action_pdis = {s: 0.0 for s in self.S}
                action_SOC = copy.deepcopy(state['SOC'])
                step_cost = None
                slacks_val = None
            else:
                # extract first-step
                times_list = sol['times']
                tfirst = times_list[0]
                action_p = {g: sol['vars']['p'][g][tfirst] for g in self.G}
                action_pch = {s: sol['vars']['pch'][s][tfirst] for s in self.S}
                action_pdis = {s: sol['vars']['pdis'][s][tfirst] for s in self.S}
                action_SOC = {s: sol['vars']['SOC'][s][tfirst] for s in self.S}
                step_cost = sol['objective']
                slacks_val = [(k, v) for k, v in sol['vars'].items() if 'slack' in k]

            # store history
            self.history['dispatch'][t] = action_p
            self.history['pch'][t] = action_pch
            self.history['pdis'][t] = action_pdis
            self.history['SOC'][t] = action_SOC
            self.history['cost'].append(step_cost)
            # total slack amount aggregated
            slack_total = 0.0
            for key in ['slack_bal_pos', 'slack_bal_neg', 'slack_res_up', 'slack_res_down']:
                if sol['vars'].get(key):
                    for val in sol['vars'][key].values():
                        if val:
                            slack_total += max(0.0, val)
            self.history['slacks'].append(slack_total)

            if self.verbosity:
                print(f"Time {t}: applied first-step dispatch (generators): {action_p}")
                print(f"         SOC after action: {action_SOC}, slacks total={slack_total}")

            # Update state for next step:
            # For p0, next step p0_g = action_p[g] (we assume perfect tracking)
            state['p0'] = {g: float(action_p[g]) for g in self.G}
            # For SOC, use action pch/pdis and dynamics
            for s in self.S:
                soc_prev = state['SOC'][s]
                soc_new = soc_prev + (self.eta_ch[s] * action_pch[s] - action_pdis[s] / max(1e-9, self.eta_dis[s])) * (self.DeltaT / 60.0)
                # enforce bounds
                soc_new = max(0.0, min(self.E_max[s], soc_new))
                state['SOC'][s] = soc_new

        if visualize:
            self.plot_results()
        return self.history

    # -----------------------------
    # Visualization & reporting
    # -----------------------------
    def plot_results(self):
        sns.set_style('whitegrid')
        times = sorted(self.history['dispatch'].keys())
        # aggregate dispatch
        total_disp = [sum(self.history['dispatch'][t].values()) for t in times]
        demand = [self.D[t] for t in times]
        fig, ax = plt.subplots(3, 1, figsize=(12, 10))
        ax[0].plot(times, demand, label='Demand', color='black', linewidth=2)
        ax[0].plot(times, total_disp, label='Total Dispatch', color='tab:blue')
        ax[0].set_ylabel('MW')
        ax[0].legend()
        # plot SOC for each storage
        for s in self.S:
            soc_series = [self.history['SOC'][t][s] for t in times]
            ax[1].plot(times, soc_series, label=f'SOC_{s}')
        ax[1].set_ylabel('Energy (MWh)')
        ax[1].legend()
        # slacks and cost
        ax[2].plot(times, self.history['slacks'], label='Total Slack', color='tab:red')
        ax2 = ax[2].twinx()
        ax2.plot(times, [c if c else 0.0 for c in self.history['cost']], label='Objective', color='tab:green')
        ax[2].set_ylabel('Slack')
        ax2.set_ylabel('Objective ($)')
        ax[2].legend(loc='upper left')
        ax2.legend(loc='upper right')
        plt.xlabel('Time')
        plt.tight_layout()
        plt.show()

# ------------------------------------------------------------
# README section (string to include in repository README or docs)
# ------------------------------------------------------------
README_MPC_TEXT = """
MPC extension (mpc_control.py)
------------------------------
This module implements a deterministic receding-horizon Model Predictive Control (MPC)
scheduler for operational dispatch of thermal generators and storage resources.

How to use:
- Import MPCController and build_default_params() or pass in parameters read from the GEP planning stage.
- Instantiate controller and call run_receding_horizon(auto_enhance=True, visualize=True)
- The controller expects investment-stage parameters (Pmax, RU, RD, ...).
- Enhancement strategies: the code automatically attempts a penalized temporary RU boost when base MPC is infeasible
  and keeps slack variables in the objective with a large penalty.

Design notes:
- PuLP + CBC is used for compatibility with the existing repo. For larger problems prefer Pyomo + commercial solvers.
- The controller is modular: the problem builder and enhancement blocks are separable to allow further extensions,
  e.g., adding market purchases, flexible demand, or hierarchical leader-follower interactions.

Outputs:
- history (dictionary) containing per-time dispatch, storage actions, SOC, objective, and slack usage.
- Visualization helpers to inspect dispatch, SOC, slacks, and costs.

Next steps to integrate with the two-stage stochastic MILP:
- Export the investment-stage decisions (upgraded Pmax, RU) from the GEP model as JSON or Python dicts.
- Pass these upgraded parameter dicts to MPCController instead of the defaults.
- Extend the enhancement strategies to include short-term purchases or pre-emption of investment budgets (operational "call-on" of rehabilitation capacity).
"""

# ------------------------------------------------------------
# Main demo
# ------------------------------------------------------------
if __name__ == "__main__":
    params
