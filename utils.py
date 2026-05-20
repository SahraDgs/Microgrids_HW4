from pyomo.environ import SolverFactory, SolverStatus
from datetime import timedelta
import pandas as pd
import numpy as np
from param import *

def check_res(res):
    eps = 1e-3  # 1 W or .1% error tolerance
    for t in range(len(res.t)):
        0=0
        # TODO: Make a function that test if the physical constraitns are indead respected
    return

def print_res(res):
    # TODO: Make a function to print a summary of the results
    return

def print_sizing_results(res):
    # TODO: Add a print with additional info specific to part 2
    return



def plot_res(res):
    #TODO: Make a nice looking plot function
    return

def solve_model(m, res):
    # Solve the optimization problem
    solver = SolverFactory('gurobi')
    output = solver.solve(m)#, tee=True)  # Parameter 'tee=True' prints the solver output

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

