from pyomo.environ import SolverFactory, SolverStatus
from datetime import timedelta
import pandas as pd
import numpy as np
from param import *

import matplotlib.pyplot as plt

def check_res(res):
    eps = 1e-3
    n_errors = 0

    for t in range(len(res.t)):

        # 1. Power balance
        into_bus = res.P_pv[t] + res.P_gen[t] + res.P_imp[t]

        out_of_bus = (res.P_load[t] + res.P_exp[t]
                      + res.P_hp_hot[t] + res.P_hp_cold[t]
                      + res.P_bss[t] + res.P_ev[t])
        
        if abs(into_bus - out_of_bus) > eps:  #error if the power balance is not respected
            print(f"[t={t}] Power balance not respected: {into_bus:f} != {out_of_bus:f}")
            n_errors += 1

        # 2. PV limits
        if res.P_pv[t] > res.P_pv_max[t] + eps:    # error if we are above the max value
            print(f"[t={t}] P_pv {res.P_pv[t]:f} above the maximal value {res.P_pv_max[t]:f}")
            n_errors += 1

        # 3. Battery SOC bounds
        if res.SOC_bss[t] < SOC_min_bss * res.C_bss - eps:
            print(f"[t={t}] SOC_bss below the minimal value: {res.SOC_bss[t]:f}")
            n_errors += 1
        if res.SOC_bss[t] > SOC_max_bss * res.C_bss + eps:
            print(f"[t={t}] SOC_bss above the maximal value: {res.SOC_bss[t]:f}")
            n_errors += 1

        # 4. EV SOC bounds (must be respected only when connected)
        if res.EV_connected[t]:
            if res.SOC_ev[t] < SOC_min_ev * C_ev - eps:
                print(f"[t={t}] SOC_ev below the minimal value: {res.SOC_ev[t]:f} < {SOC_min_ev * C_ev:f}")
                n_errors += 1
            if res.SOC_ev[t] > SOC_max_ev * C_ev + eps:
                print(f"[t={t}] SOC_ev above the maximal value: {res.SOC_ev[t]:f} > {SOC_max_ev * C_ev:f}")
                n_errors += 1

        # 5. House temperature comfort bounds
        if res.T_hp[t] < res.T_set[t] - delta_T_max - eps:
            print(f"[t={t}] T_hp below comfort: {res.T_hp[t]:f} < {res.T_set[t] - delta_T_max:f}")
            n_errors += 1
        if res.T_hp[t] > res.T_set[t] + delta_T_max + eps:
            print(f"[t={t}] T_hp above comfort: {res.T_hp[t]:f} > {res.T_set[t] + delta_T_max:f}")
            n_errors += 1


    # Report
    if n_errors == 0:
        print(f"check passed: all constraints are satisfied")
    else:
        print(f"check FAILED: {n_errors} constraint violation(s) found")
    return


def print_res(res):
    dt = delta_t   # [h]

    # Energy totals [kWh]
    E_pv = res.P_pv.sum() * dt
    E_gen = res.P_gen.sum() * dt
    E_imp = res.P_imp.sum() * dt
    E_exp = res.P_exp.sum() * dt
    E_load = res.P_load.sum() * dt

    # Costs [EUR]
    cost_imp = E_imp * PI_imp
    cost_gen = E_gen * PI_gen
    revenue = E_exp * PI_exp
    total_cost = cost_imp + cost_gen - revenue

    # Self-consumption & self-sufficiency
    E_pv_used = E_pv - E_exp                        # PV consumed locally
    if E_pv > 0:                # to avoid dividing by 0
        self_consumption = 100 * E_pv_used / E_pv
    else:
        self_consumption = 0

    if E_load > 0:
        self_sufficiency = 100 * (E_pv_used + E_gen) / E_load
    else:
        self_sufficiency = 0

    print()
    print("----- OPERATIONAL PLANNING RESULTS -----")
    print(f"Simulation period : {res.n_days} days ({res.t_s} steps of {dt*60:.0f} min)")
    print()
    print(f"PV produced    : {E_pv:.1f} kWh")
    print(f"Gen produced   : {E_gen:.1f} kWh")
    print(f"Imported       : {E_imp:.1f} kWh")
    print(f"Exported       : {E_exp:.1f} kWh")
    print(f"Load consumed  : {E_load:.1f} kWh")
    print()
    print(f"Import cost     : {cost_imp:.2f} EUR")
    print(f"Generator cost  : {cost_gen:.2f} EUR")
    print(f"Export revenue  : {revenue:.2f} EUR")
    print(f"Total OPEX      : {total_cost:.2f} EUR")
    print(f"Solver objective: {res.objective:.2f} EUR")
    print()
    print(f"Self-consumption : {self_consumption:.1f} %")
    print(f"Self-sufficiency : {self_sufficiency:.1f} %")
    return 

def print_sizing_results(res):


    # TO CHANGE WHEN WE DO PHASE 2 !!!!

    # Optimal asset sizes (decided by the optimizer)
    C_pv      = res.C_pv
    P_nom_pv  = res.P_nom_pv
    C_bss     = res.C_bss
    P_nom_bss = res.P_nom_bss
    P_max_gen = res.P_max_gen

    print("----- SIZING: OPTIMAL ASSET SIZES -----")
    print(f"PV system     : {C_pv:.2f} kWp")
    print(f"PV inverter   : {P_nom_pv:.2f} kW")
    print(f"Battery       : {C_bss:.2f} kWh")
    print(f"BSS inverter  : {P_nom_bss:.2f} kW")
    print(f"Diesel genset : {P_max_gen:.2f} kW")
    print()

    # CAPEX: size x unit price, for each technology
    capex_pv      = C_pv      * PI_c_pv
    capex_pv_inv  = P_nom_pv  * PI_c_inv
    capex_bss     = C_bss     * PI_c_bss
    capex_bss_inv = P_nom_bss * PI_c_inv
    capex_gen     = P_max_gen * PI_c_gen
    capex_total   = capex_pv + capex_pv_inv + capex_bss + capex_bss_inv + capex_gen

    print(f"CAPEX PV          : {capex_pv:.0f} EUR")
    print(f"CAPEX PV inverter : {capex_pv_inv:.0f} EUR")
    print(f"CAPEX battery     : {capex_bss:.0f} EUR")
    print(f"CAPEX BSS inverter: {capex_bss_inv:.0f} EUR")
    print(f"CAPEX genset      : {capex_gen:.0f} EUR")
    print(f"Total CAPEX       : {capex_total:.0f} EUR")
    print()

    # CAPEX is spread over the investment horizon (annualised)
    annual_capex = capex_total / inv_hor
    opex = res.objective - annual_capex   # objective = annual CAPEX + OPEX

    print(f"Annualised CAPEX : {annual_capex:.0f} EUR/year")
    print(f"OPEX             : {opex:.0f} EUR/year")
    print(f"Total cost/year  : {res.objective:.0f} EUR/year")

    return


def plot_res_day(res):
    dt = delta_t
    steps_per_day = int(24 / dt)

    # day to plot 
    day = 220                              
    start = day * steps_per_day
    end   = start + steps_per_day
    time_x = np.arange(steps_per_day) * dt # x-axis: 0 to 24 h
   

    # battery SOC in percent of capacity
    SOC_bss_pct = 100 * res.SOC_bss[start:end] / res.C_bss
    SOC_ev_pct  = 100 * res.SOC_ev[start:end]  / res.C_ev
    SOC_ev_pct = np.where(res.EV_connected[start:end] == 1, SOC_ev_pct, np.nan)

    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(11, 10), sharex=True)

    # Graph 1: PV production + grid exchange
    ax1.plot(time_x, res.P_pv[start:end],  label='PV production')
    ax1.plot(time_x, res.P_imp[start:end] - res.P_exp[start:end], label='Import/export')
    ax1.plot(time_x, -res.P_ev[start:end], label='EV discharge/charge')
    ax1.plot(time_x, -res.P_bss[start:end], label='battery discharge/charge')

    ax1.set_ylabel('Power [kW]')
    ax1.legend(title=f'Production and grid exchange')
    ax1.set_title(f'Day {day}')
    ax1.grid(True)

    # Graph 2: consumption breakdown
    ax2.plot(time_x, res.P_load[start:end], label='Load')
    ax2.plot(time_x, res.P_hp_hot[start:end], label='HP heating')
    ax2.plot(time_x, res.P_hp_cold[start:end], label='HP cooling')

    ax2.set_ylabel('Power [kW]')
    ax2.legend(title='Consumption')
    ax2.grid(True)

    # Graph 3: battery SOC (%)
    ax3.plot(time_x, SOC_bss_pct, label='Battery SOC')
    ax3.plot(time_x, SOC_ev_pct, label='EV SOC')
    ax3.axhline(100 * SOC_min_bss, color='red', linestyle='--', label='SOC min')
    ax3.axhline(100 * SOC_max_bss, color='green', linestyle='--', label='SOC max')
    ax3.set_ylabel('SOC [%]')
    ax3.set_xlabel('Time [h]')
    ax3.legend(title='Battery state of charge')
    ax3.grid(True)

    plt.tight_layout()
    plt.savefig(f'dispatch_day_{day}.png', dpi=150, bbox_inches='tight')
    plt.show()


# The following function is not used in the main code as explained in the report

def plot_res_week(res):
    dt = delta_t
    steps_per_day = int(24 / dt)

    # choose the week
    start_day = 29 
    start = start_day * steps_per_day
    end   = start + 7 * steps_per_day
    time_x  = np.arange(7 * steps_per_day) * dt / 24

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8), sharex=True)

    # Graph 1: PV production + grid exchange
    ax1.plot(time_x, res.P_pv[start:end],  label='PV production')
    ax1.plot(time_x, res.P_imp[start:end] - res.P_exp[start:end], label='Net grid (import − export)')
    ax1.plot(time_x, - res.P_ev[start:end], label='EV (discharge + / charge −)')
    ax1.plot(time_x, - res.P_bss[start:end], label='Battery (discharge + / charge −)')

    ax1.set_ylabel('Power [kW]')
    ax1.legend(title=f'Generation, grid and storage')
    ax1.set_title(f'Week from day {start_day}')
    ax1.grid(True)

    # Graph 2: consumption breakdown
    ax2.plot(time_x, res.P_load[start:end], label='Load')
    ax2.plot(time_x, res.P_hp_hot[start:end], label='HP heating')
    ax2.plot(time_x, res.P_hp_cold[start:end], label='HP cooling')
    
    ax2.set_ylabel('Power [kW]')
    ax2.legend(title='Loads (consumption)')
    ax2.grid(True)

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

