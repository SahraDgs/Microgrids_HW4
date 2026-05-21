from pyomo.environ import SolverFactory, SolverStatus
from datetime import timedelta
import pandas as pd
import numpy as np
from param import *

def check_res(res):
    eps = 1e-3
    errors = []

    for t in range(len(res.t)):

        # 1. Power balance
        lhs = res.P_pv[t] + res.P_gen[t] + res.P_imp[t]
        rhs = res.P_exp[t] + res.P_load[t]
        if hasattr(res, 'P_bss'):
            P_ch_bss  = max( res.P_bss[t], 0)   # positive = charging
            P_dis_bss = max(-res.P_bss[t], 0)   # positive = discharging
            lhs += P_dis_bss
            rhs += P_ch_bss
        if hasattr(res, 'P_ev'):
            P_ch_ev  = max( res.P_ev[t], 0)
            P_dis_ev = max(-res.P_ev[t], 0)
            lhs += P_dis_ev
            rhs += P_ch_ev
        if hasattr(res, 'P_hp_hot'):
            rhs += res.P_hp_hot[t] + res.P_hp_cold[t]
        if abs(lhs - rhs) > eps:
            errors.append(f"[t={t}] Power balance violated: {lhs:.4f} != {rhs:.4f} (diff={abs(lhs-rhs):.4f})")

        # 2. PV limits
        if res.P_pv[t] < -eps:
            errors.append(f"[t={t}] P_pv negative: {res.P_pv[t]:.4f}")
        if res.P_pv[t] > res.P_pv_max[t] + eps:
            errors.append(f"[t={t}] P_pv exceeds MPP: {res.P_pv[t]:.4f} > {res.P_pv_max[t]:.4f}")

        # 3. Battery SOC bounds
        if hasattr(res, 'SOC_bss') and hasattr(res, 'C_bss') and res.C_bss > 0:
            if res.SOC_bss[t] < SOC_min_bss * res.C_bss - eps:
                errors.append(f"[t={t}] SOC_bss below min: {res.SOC_bss[t]:.4f} < {SOC_min_bss * res.C_bss:.4f}")
            if res.SOC_bss[t] > SOC_max_bss * res.C_bss + eps:
                errors.append(f"[t={t}] SOC_bss above max: {res.SOC_bss[t]:.4f} > {SOC_max_bss * res.C_bss:.4f}")

        # 4. EV SOC bounds (only when connected)
        if hasattr(res, 'SOC_ev') and res.EV_connected[t]:
            if res.SOC_ev[t] < SOC_min_ev * C_ev - eps:
                errors.append(f"[t={t}] SOC_ev below min: {res.SOC_ev[t]:.4f} < {SOC_min_ev * C_ev:.4f}")
            if res.SOC_ev[t] > SOC_max_ev * C_ev + eps:
                errors.append(f"[t={t}] SOC_ev above max: {res.SOC_ev[t]:.4f} > {SOC_max_ev * C_ev:.4f}")

        # 5. HP temperature bounds
        if hasattr(res, 'T_hp'):
            if res.T_hp[t] < res.T_set[t] - delta_T_max - eps:
                errors.append(f"[t={t}] T_hp below comfort: {res.T_hp[t]:.2f} < {res.T_set[t] - delta_T_max:.2f}")
            if res.T_hp[t] > res.T_set[t] + delta_T_max + eps:
                errors.append(f"[t={t}] T_hp above comfort: {res.T_hp[t]:.2f} > {res.T_set[t] + delta_T_max:.2f}")

        # 6. No negative powers
        for name, arr in [("P_imp", res.P_imp), ("P_exp", res.P_exp), ("P_gen", res.P_gen)]:
            if arr[t] < -eps:
                errors.append(f"[t={t}] {name} is negative: {arr[t]:.4f}")

    # Report
    if not errors:
        print(f"check_res passed — all constraints satisfied ({len(res.t)} timesteps checked)")
    else:
        print(f"check_res found {len(errors)} violation(s):")
        for e in errors[:20]:     # show at most 20 to avoid flooding the console
            print(f"   {e}")
        if len(errors) > 20:
            print(f"   ... and {len(errors) - 20} more.")
    return errors


def print_res(res):
    dt = delta_t   # [h]

    # Energy totals [kWh]
    E_pv      =  res.P_pv.sum()  * dt
    E_gen     =  res.P_gen.sum() * dt
    E_imp     =  res.P_imp.sum() * dt
    E_exp     =  res.P_exp.sum() * dt
    E_load    =  res.P_load.sum()* dt

    # Costs [EUR]
    cost_imp  =  E_imp  * PI_imp
    cost_gen  =  E_gen  * PI_gen
    revenue   =  E_exp  * PI_exp
    total_cost = cost_imp + cost_gen - revenue

    # Self-consumption & self-sufficiency
    E_pv_used      = E_pv - E_exp                        # PV consumed locally
    self_consumption  = 100 * E_pv_used / E_pv           if E_pv  > 0 else 0
    self_sufficiency  = 100 * (E_pv_used + E_gen) / E_load if E_load > 0 else 0

    print("=" * 52)
    print("          OPERATIONAL PLANNING — RESULTS")
    print("=" * 52)
    print(f"  Simulation period     : {res.n_days} days")
    print(f"  Time step             : {dt*60:.0f} min  ({res.t_s} steps)")
    print("-" * 52)
    print(f"  Energy produced (PV)  : {E_pv:>10.1f} kWh")
    print(f"  Energy produced (gen) : {E_gen:>10.1f} kWh")
    print(f"  Energy imported       : {E_imp:>10.1f} kWh")
    print(f"  Energy exported       : {E_exp:>10.1f} kWh")
    print(f"  Energy consumed       : {E_load:>10.1f} kWh")
    print("-" * 52)
    print(f"  Cost   — import       : {cost_imp:>10.2f} EUR")
    print(f"  Cost   — generator    : {cost_gen:>10.2f} EUR")
    print(f"  Revenue— export       : {revenue:>10.2f} EUR")
    print(f"  ► Total OPEX          : {total_cost:>10.2f} EUR")
    if hasattr(res, 'objective') and res.objective is not None:
        print(f"  ► Objective (solver)  : {res.objective:>10.2f} EUR")
    print("-" * 52)
    print(f"  Self-consumption      : {self_consumption:>9.1f} %")
    print(f"  Self-sufficiency      : {self_sufficiency:>9.1f} %")
    print("=" * 52)


def print_sizing_results(res):
    print("=" * 52)
    print("             SIZING — OPTIMAL ASSET SIZES")
    print("=" * 52)

    # Retrieve sizes (set by save_sizing_results)
    C_pv_opt     = getattr(res, 'C_pv',     None)
    P_nom_pv_opt = getattr(res, 'P_nom_pv', None)
    C_bss_opt    = getattr(res, 'C_bss',    None)
    P_nom_bss_opt= getattr(res, 'P_nom_bss',None)
    P_max_gen_opt= getattr(res, 'P_max_gen',None)

    if C_pv_opt      is not None: print(f"  PV system size        : {C_pv_opt:>8.2f} kWp")
    if P_nom_pv_opt  is not None: print(f"  PV inverter           : {P_nom_pv_opt:>8.2f} kW")
    if C_bss_opt     is not None: print(f"  Battery capacity      : {C_bss_opt:>8.2f} kWh")
    if P_nom_bss_opt is not None: print(f"  Battery inverter      : {P_nom_bss_opt:>8.2f} kW")
    if P_max_gen_opt is not None: print(f"  Diesel genset         : {P_max_gen_opt:>8.2f} kW")

    # CAPEX breakdown (part 2)
    print("-" * 52)
    capex_total = 0
    if C_pv_opt      is not None:
        c = C_pv_opt     * PI_c_pv;   capex_total += c
        print(f"  CAPEX PV              : {c:>8.0f} EUR  ({PI_c_pv} EUR/kWp)")
    if P_nom_pv_opt  is not None:
        c = P_nom_pv_opt * PI_c_inv;  capex_total += c
        print(f"  CAPEX PV inverter     : {c:>8.0f} EUR  ({PI_c_inv} EUR/kW)")
    if C_bss_opt     is not None:
        c = C_bss_opt    * PI_c_bss;  capex_total += c
        print(f"  CAPEX battery         : {c:>8.0f} EUR  ({PI_c_bss} EUR/kWh)")
    if P_nom_bss_opt is not None:
        c = P_nom_bss_opt* PI_c_inv;  capex_total += c
        print(f"  CAPEX BSS inverter    : {c:>8.0f} EUR  ({PI_c_inv} EUR/kW)")
    if P_max_gen_opt is not None:
        c = P_max_gen_opt* PI_c_gen;  capex_total += c
        print(f"  CAPEX genset          : {c:>8.0f} EUR  ({PI_c_gen} EUR/kW)")

    print(f"  ► Total CAPEX         : {capex_total:>8.0f} EUR")
    if inv_hor > 0:
        print(f"  ► Annualised CAPEX    : {capex_total/inv_hor:>8.0f} EUR/year  (/{inv_hor} years)")

    # OPEX reminder
    if hasattr(res, 'objective') and res.objective is not None:
        print("-" * 52)
        opex = res.objective - capex_total / inv_hor  if inv_hor > 0 else res.objective
        print(f"  OPEX (yearly)         : {opex:>8.0f} EUR/year")
        print(f"  ► Total cost/year     : {res.objective:>8.0f} EUR/year")
    print("=" * 52)



def plot_res(res):
    #TODO: Make a nice looking plot function
    return

def solve_model(m, res):
    # Solve the optimization problem
    solver = SolverFactory('gurobi')
    output = solver.solve(m, tee=True)  # Parameter 'tee=True' prints the solver output

    # Print elapsed time
    status = output.solver.status

    # Check the solution status
    if status == SolverStatus.ok:
        print("Simulation completed")
        res = save_results(res, m)
        res.save_sizing_results(m)
        return m, res
    elif status == SolverStatus.warning:
        print("Solver finished with a warning.")
    elif status == SolverStatus.error:
        print("Solver encountered an error and did not converge.")
    elif status == SolverStatus.aborted:
        print("Solver was aborted before completing the optimization.")
    else:
        print("Solver status unknown.")
    return None, None

def update_model(model, res, SOC_0_bss, SOC_0_ev, T_0_hp):
    for t in res.t:
        model.P_load[t] = res.P_load[t]
        model.P_pv_max[t] = res.P_pv_max[t]
        model.EV_connected[t] = res.EV_connected[t]
        model.t_arr[t] = res.t_arr[t]
        model.t_dep[t] = res.t_dep[t]
        model.SOC_i_ev[t] = res.SOC_i_ev[t]
        model.T_set[t] = res.T_set[t]
        model.P_loss[t] = res.P_loss[t]
    model.SOC_0_bss = SOC_0_bss
    model.SOC_0_ev = SOC_0_ev
    model.T_0_hp = T_0_hp

    return model

class Results:
    def __init__(self, start_time, n_days, yearly_kwh, yearly_km):
        self.start_time = start_time 
        self.t_s = int(n_days*24/delta_t)                # Total number of discrete time steps in the simulation
        self.n_days = n_days
        self.yearly_kwh = yearly_kwh
        self.yearly_km = yearly_km
        self.t = np.arange(0,self.t_s)

        self.P_pv = np.zeros(self.t_s)
        self.P_bss = np.zeros(self.t_s)
        self.P_ev = np.zeros(self.t_s)
        self.P_gen = np.zeros(self.t_s)
        self.P_imp = np.zeros(self.t_s)
        self.P_exp = np.zeros(self.t_s)

        self.SOC_ev = np.zeros(self.t_s)
        self.SOC_bss = np.zeros(self.t_s)

        # Initialize SOCs
        self.SOC_bss_i = 0.5          

        # Load data from CSV files into pandas DataFrames
        self.df = pd.read_csv('HW2.csv', delimiter=';', index_col="DateTime", parse_dates=True, date_format='%Y-%m-%d %H:%M:%S')#, date_parser=lambda x: datetime.strptime(x, '%Y-%m-%d %H:%M:%S'))
        self.P_load = np.array([self.df.loc[self.start_time + timedelta(hours=t*delta_t)]["Load"].clip(min=0) * self.yearly_kwh for t in self.t])
        self.P_pv_max = np.array([self.df.loc[self.start_time + timedelta(hours=t*delta_t)]["PV"].clip(min=0) for t in self.t])
        self.EV_connected = np.array([self.df.loc[self.start_time + timedelta(hours=t*delta_t)]["EV"] for t in self.t])
        self.P_loss = np.array([self.df.loc[self.start_time + timedelta(hours=t*delta_t)]["P_loss"] for t in self.t])
        self.T_set = np.array([self.df.loc[self.start_time + timedelta(hours=t*delta_t)]["T_set"] for t in self.t])
        self.datetime = [self.start_time + timedelta(hours=t*delta_t) for t in self.t]


        self.SOC_i_ev = np.array([SOC_target_ev*C_ev - (self.EV_connected[t]*self.yearly_km) / (5e6) if self.EV_connected[t] > 0 and (t == 0 or self.EV_connected[t-1] == 0) else 0 for t in range(self.t_s)])
        self.t_arr = np.array([1 if self.EV_connected[t] > 0 and (t == 0 or self.EV_connected[t-1] == 0) else 0 for t in range(self.t_s)])
        self.t_dep = np.array([1 if self.EV_connected[t] == 0 and (t > 0 and self.EV_connected[t-1] > 0) else 0  for t in range(self.t_s)])
        if self.EV_connected[-1] > 0: self.t_dep[-1] = 1
        self.EV_connected = np.array([1 if self.EV_connected[t] > 0 else 0 for t in range(self.t_s)])

    def save_sizing_results(self, m):
        self.C_bss = m.C_bss.value
        self.P_nom_bss = m.P_nom_bss.value
        self.C_pv = m.C_pv.value
        self.P_nom_pv = m.P_nom_pv.value
        self.C_ev = m.C_ev.value
        self.P_nom_ev = m.P_nom_ev.value
        self.P_max_gen = m.P_max_gen.value


def save_results(res, m):
    res.P_imp = np.array([m.P_imp[t].value for t in m.periods])
    res.P_exp = np.array([m.P_exp[t].value for t in m.periods])
    res.P_pv = np.array([m.P_pv[t].value for t in m.periods])
    res.P_bss = np.array([m.P_charge_bss[t].value - m.P_discharge_bss[t].value for t in m.periods])
    res.P_ev = np.array([m.P_charge_ev[t].value - m.P_discharge_ev[t].value for t in m.periods])
    res.P_gen = np.array([m.P_gen[t].value for t in m.periods])
    res.P_hp_hot = np.array([m.P_hp_hot[t].value for t in m.periods])
    res.P_hp_cold = np.array([m.P_hp_cold[t].value for t in m.periods])
    res.T_hp = np.array([m.T_hp[t].value for t in m.periods])
    res.SOC_ev = np.array([m.SOC_ev[t].value for t in m.periods])
    res.SOC_bss = np.array([m.SOC_bss[t].value for t in m.periods])
    res.objective = m.objective()
    return res

