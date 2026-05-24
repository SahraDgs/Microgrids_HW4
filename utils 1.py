from pyomo.environ import SolverFactory, SolverStatus, TerminationCondition, value
from datetime import timedelta
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from param import *


def _split_net_power(P_net):
    """
    In this project:
    res.P_bss = P_charge_bss - P_discharge_bss
    res.P_ev  = P_charge_ev  - P_discharge_ev

    Positive value  => charging
    Negative value  => discharging
    """
    P_net = np.array(P_net)

    P_charge = np.maximum(P_net, 0)
    P_discharge = np.maximum(-P_net, 0)

    return P_charge, P_discharge


def check_res(res):
    """
    Basic feasibility checks.
    These checks are only for debugging/reporting.
    The real feasibility is enforced by the optimization constraints.
    """

    eps = 1e-3

    print("\n========== Checking Results ==========")

    P_charge_bss, P_discharge_bss = _split_net_power(res.P_bss)
    P_charge_ev, P_discharge_ev = _split_net_power(res.P_ev)

    # Power balance check
    supply = (
        res.P_pv
        + res.P_gen
        + P_discharge_bss
        + P_discharge_ev
        + res.P_imp
    )

    demand = (
        res.P_load
        + P_charge_bss
        + P_charge_ev
        + res.P_hp_hot
        + res.P_hp_cold
        + res.P_exp
    )

    balance_error = supply - demand
    max_balance_error = np.max(np.abs(balance_error))

    print(f"Max power balance error: {max_balance_error:.6f} kW")

    if max_balance_error <= eps:
        print("Power balance: OK")
    else:
        print("WARNING: Power balance violation detected.")

    # BSS SOC bounds
    if hasattr(res, "C_bss"):
        bss_min = SOC_min_bss * res.C_bss
        bss_max = SOC_max_bss * res.C_bss

        min_soc_bss = np.min(res.SOC_bss)
        max_soc_bss = np.max(res.SOC_bss)

        print(f"BSS SOC range: [{min_soc_bss:.3f}, {max_soc_bss:.3f}] kWh")
        print(f"BSS SOC allowed: [{bss_min:.3f}, {bss_max:.3f}] kWh")

        if min_soc_bss >= bss_min - eps and max_soc_bss <= bss_max + eps:
            print("BSS SOC bounds: OK")
        else:
            print("WARNING: BSS SOC bounds violation detected.")

    # EV SOC bounds
    ev_min = SOC_min_ev * C_ev
    ev_max = SOC_max_ev * C_ev

    min_soc_ev = np.min(res.SOC_ev)
    max_soc_ev = np.max(res.SOC_ev)

    print(f"EV SOC range: [{min_soc_ev:.3f}, {max_soc_ev:.3f}] kWh")
    print(f"EV SOC allowed: [{ev_min:.3f}, {ev_max:.3f}] kWh")

    if min_soc_ev >= ev_min - eps and max_soc_ev <= ev_max + eps:
        print("EV SOC bounds: OK")
    else:
        print("WARNING: EV SOC bounds violation detected.")

    # EV departure target
    dep_indices = np.where(res.t_dep == 1)[0]

    if len(dep_indices) > 0:
        target_energy = SOC_target_ev * C_ev
        min_soc_departure = np.min(res.SOC_ev[dep_indices])

        print(f"EV target at departure: {target_energy:.3f} kWh")
        print(f"Minimum EV SOC at departure: {min_soc_departure:.3f} kWh")

        if min_soc_departure >= target_energy - eps:
            print("EV departure target: OK")
        else:
            print("WARNING: EV target not reached at some departure.")

    # Temperature comfort
    temp_low = res.T_set - delta_T_max
    temp_high = res.T_set + delta_T_max

    min_temp_violation = np.min(res.T_hp - temp_low)
    max_temp_violation = np.max(res.T_hp - temp_high)

    print(f"House temperature range: [{np.min(res.T_hp):.3f}, {np.max(res.T_hp):.3f}] degC")

    if min_temp_violation >= -eps and max_temp_violation <= eps:
        print("Temperature comfort bounds: OK")
    else:
        print("WARNING: Temperature comfort violation detected.")

    print("======================================\n")

    return


def print_res(res):
    """
    Print a summary of the operational planning results.
    """

    print("\n========== Operational Planning Results ==========")

    E_load = np.sum(res.P_load) * delta_t
    E_pv = np.sum(res.P_pv) * delta_t
    E_gen = np.sum(res.P_gen) * delta_t
    E_imp = np.sum(res.P_imp) * delta_t
    E_exp = np.sum(res.P_exp) * delta_t

    P_charge_bss, P_discharge_bss = _split_net_power(res.P_bss)
    P_charge_ev, P_discharge_ev = _split_net_power(res.P_ev)

    E_bss_ch = np.sum(P_charge_bss) * delta_t
    E_bss_dis = np.sum(P_discharge_bss) * delta_t

    E_ev_ch = np.sum(P_charge_ev) * delta_t
    E_ev_dis = np.sum(P_discharge_ev) * delta_t

    E_hp_hot = np.sum(res.P_hp_hot) * delta_t
    E_hp_cold = np.sum(res.P_hp_cold) * delta_t

    cost_gen = PI_gen * E_gen
    cost_imp = PI_imp * E_imp
    revenue_exp = PI_exp * E_exp

    opex = cost_gen + cost_imp - revenue_exp

    print(f"Simulation horizon: {res.n_days} days")
    print(f"Number of time steps: {res.t_s}")
    print(f"Time step: {delta_t} h")
    print("")
    print(f"Objective value from solver: {res.objective:.2f} EUR")
    print(f"Recomputed OPEX: {opex:.2f} EUR")

    print("\n----- Energy summary [kWh] -----")
    print(f"Load energy:              {E_load:.2f}")
    print(f"PV used:                  {E_pv:.2f}")
    print(f"Generator energy:         {E_gen:.2f}")
    print(f"Grid import:              {E_imp:.2f}")
    print(f"Grid export:              {E_exp:.2f}")
    print(f"BSS charged:              {E_bss_ch:.2f}")
    print(f"BSS discharged:           {E_bss_dis:.2f}")
    print(f"EV charged:               {E_ev_ch:.2f}")
    print(f"EV discharged:            {E_ev_dis:.2f}")
    print(f"HP heating energy:        {E_hp_hot:.2f}")
    print(f"HP cooling energy:        {E_hp_cold:.2f}")

    if hasattr(res, "C_pv"):
        P_pv_available = res.C_pv * res.P_pv_max
        E_pv_available = np.sum(P_pv_available) * delta_t
        E_pv_curtailed = np.sum(np.maximum(P_pv_available - res.P_pv, 0)) * delta_t

        print("\n----- PV summary [kWh] -----")
        print(f"PV available:             {E_pv_available:.2f}")
        print(f"PV used:                  {E_pv:.2f}")
        print(f"PV curtailed:             {E_pv_curtailed:.2f}")

    print("\n----- Cost summary [EUR] -----")
    print(f"Generator cost:           {cost_gen:.2f}")
    print(f"Import cost:              {cost_imp:.2f}")
    print(f"Export revenue:           {revenue_exp:.2f}")
    print(f"OPEX:                     {opex:.2f}")

    print("\n----- Peaks [kW] -----")
    print(f"Peak import:              {np.max(res.P_imp):.2f}")
    print(f"Peak export:              {np.max(res.P_exp):.2f}")
    print(f"Peak generator:           {np.max(res.P_gen):.2f}")
    print(f"Peak PV used:             {np.max(res.P_pv):.2f}")
    print(f"Peak BSS charge:          {np.max(P_charge_bss):.2f}")
    print(f"Peak BSS discharge:       {np.max(P_discharge_bss):.2f}")
    print(f"Peak EV charge:           {np.max(P_charge_ev):.2f}")
    print(f"Peak EV discharge:        {np.max(P_discharge_ev):.2f}")

    print("=================================================\n")

    return


def print_sizing_results(res):
    """
    Print additional sizing information.
    This is mostly useful for Part 2.
    """

    print("\n========== Sizing Results ==========")

    attrs = [
        "C_pv",
        "P_nom_pv",
        "C_bss",
        "P_nom_bss",
        "P_max_gen",
        "C_ev",
        "P_nom_ev",
    ]

    for attr in attrs:
        if hasattr(res, attr):
            print(f"{attr}: {getattr(res, attr):.3f}")

    print("====================================\n")

    return


def plot_res(res):
    """
    Plot the main operational planning results.
    If the horizon is long, only the first 7 days are plotted.
    """

    if len(res.t) == 0:
        print("No results to plot.")
        return

    max_days_to_plot = 7
    n_plot = min(len(res.t), int(max_days_to_plot * 24 / delta_t))

    time = res.datetime[:n_plot]

    P_charge_bss, P_discharge_bss = _split_net_power(res.P_bss)
    P_charge_ev, P_discharge_ev = _split_net_power(res.P_ev)

    # ------------------------------------------------------------------
    # Plot 1: Main power profiles
    # ------------------------------------------------------------------
    plt.figure(figsize=(14, 5))
    plt.plot(time, res.P_load[:n_plot], label="Load")
    plt.plot(time, res.P_pv[:n_plot], label="PV used")
    plt.plot(time, res.P_gen[:n_plot], label="Generator")
    plt.plot(time, res.P_imp[:n_plot], label="Grid import")
    plt.plot(time, res.P_exp[:n_plot], label="Grid export")
    plt.title("Main power profiles")
    plt.ylabel("Power [kW]")
    plt.xlabel("Time")
    plt.xticks(rotation=30)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ------------------------------------------------------------------
    # Plot 2: Battery and EV net power
    # ------------------------------------------------------------------
    plt.figure(figsize=(14, 5))
    plt.plot(time, res.P_bss[:n_plot], label="BSS net power (+ charge, - discharge)")
    plt.plot(time, res.P_ev[:n_plot], label="EV net power (+ charge, - discharge)")
    plt.axhline(0, linewidth=0.8)
    plt.title("Battery and EV net power")
    plt.ylabel("Power [kW]")
    plt.xlabel("Time")
    plt.xticks(rotation=30)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ------------------------------------------------------------------
    # Plot 3: State of charge
    # ------------------------------------------------------------------
    plt.figure(figsize=(14, 5))
    plt.plot(time, res.SOC_bss[:n_plot], label="BSS SOC")
    plt.plot(time, res.SOC_ev[:n_plot], label="EV SOC")
    plt.title("State of charge")
    plt.ylabel("Energy [kWh]")
    plt.xlabel("Time")
    plt.xticks(rotation=30)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ------------------------------------------------------------------
    # Plot 4: Heat pump temperature
    # ------------------------------------------------------------------
    plt.figure(figsize=(14, 5))
    plt.plot(time, res.T_hp[:n_plot], label="House temperature")
    plt.plot(time, res.T_set[:n_plot], label="Temperature setpoint")
    plt.plot(time, res.T_set[:n_plot] + delta_T_max, "--", label="Upper comfort bound")
    plt.plot(time, res.T_set[:n_plot] - delta_T_max, "--", label="Lower comfort bound")
    plt.title("Thermal comfort")
    plt.ylabel("Temperature [degC]")
    plt.xlabel("Time")
    plt.xticks(rotation=30)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    # ------------------------------------------------------------------
    # Plot 5: Heat pump electrical power
    # ------------------------------------------------------------------
    plt.figure(figsize=(14, 5))
    plt.plot(time, res.P_hp_hot[:n_plot], label="HP heating power")
    plt.plot(time, res.P_hp_cold[:n_plot], label="HP cooling power")
    plt.title("Heat pump power")
    plt.ylabel("Power [kW]")
    plt.xlabel("Time")
    plt.xticks(rotation=30)
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()

    return


def solve_model(m, res):
    """
    Solve the optimization problem with Gurobi.
    """

    solver = SolverFactory("gurobi")
    output = solver.solve(m, tee=True)

    status = output.solver.status
    termination = output.solver.termination_condition

    print("Solver status:", status)
    print("Termination condition:", termination)

    if status == SolverStatus.ok and termination == TerminationCondition.optimal:
        print("Simulation completed")
        res = save_results(res, m)
        res.save_sizing_results(m)
        return m, res

    elif status == SolverStatus.ok:
        print("Solver finished, but termination was not optimal.")
        print("Termination condition:", termination)

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
    """
    Update mutable parameters for rolling horizon or repeated simulations.
    """

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
        self.t_s = int(n_days * 24 / delta_t)
        self.n_days = n_days
        self.yearly_kwh = yearly_kwh
        self.yearly_km = yearly_km
        self.t = np.arange(0, self.t_s)

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
        self.df = pd.read_csv(
            "HW2.csv",
            delimiter=";",
            index_col="DateTime",
            parse_dates=True,
            date_format="%Y-%m-%d %H:%M:%S",
        )

        self.P_load = np.array(
            [
                self.df.loc[self.start_time + timedelta(hours=t * delta_t)]["Load"].clip(min=0)
                * self.yearly_kwh
                for t in self.t
            ]
        )

        self.P_pv_max = np.array(
            [
                self.df.loc[self.start_time + timedelta(hours=t * delta_t)]["PV"].clip(min=0)
                for t in self.t
            ]
        )

        self.EV_connected = np.array(
            [
                self.df.loc[self.start_time + timedelta(hours=t * delta_t)]["EV"]
                for t in self.t
            ]
        )

        self.P_loss = np.array(
            [
                self.df.loc[self.start_time + timedelta(hours=t * delta_t)]["P_loss"]
                for t in self.t
            ]
        )

        self.T_set = np.array(
            [
                self.df.loc[self.start_time + timedelta(hours=t * delta_t)]["T_set"]
                for t in self.t
            ]
        )

        self.datetime = [
            self.start_time + timedelta(hours=t * delta_t)
            for t in self.t
        ]

        self.SOC_i_ev = np.array(
            [
                SOC_target_ev * C_ev - (self.EV_connected[t] * self.yearly_km) / (5e6)
                if self.EV_connected[t] > 0 and (t == 0 or self.EV_connected[t - 1] == 0)
                else 0
                for t in range(self.t_s)
            ]
        )

        self.t_arr = np.array(
            [
                1
                if self.EV_connected[t] > 0 and (t == 0 or self.EV_connected[t - 1] == 0)
                else 0
                for t in range(self.t_s)
            ]
        )

        self.t_dep = np.array(
            [
                1
                if self.EV_connected[t] == 0 and (t > 0 and self.EV_connected[t - 1] > 0)
                else 0
                for t in range(self.t_s)
            ]
        )

        if self.EV_connected[-1] > 0:
            self.t_dep[-1] = 1

        self.EV_connected = np.array(
            [
                1 if self.EV_connected[t] > 0 else 0
                for t in range(self.t_s)
            ]
        )

    def save_sizing_results(self, m):
        self.C_bss = value(m.C_bss)
        self.P_nom_bss = value(m.P_nom_bss)
        self.C_pv = value(m.C_pv)
        self.P_nom_pv = value(m.P_nom_pv)
        self.C_ev = value(m.C_ev)
        self.P_nom_ev = value(m.P_nom_ev)
        self.P_max_gen = value(m.P_max_gen)


def save_results(res, m):
    res.P_imp = np.array([value(m.P_imp[t]) for t in m.periods])
    res.P_exp = np.array([value(m.P_exp[t]) for t in m.periods])
    res.P_pv = np.array([value(m.P_pv[t]) for t in m.periods])

    res.P_bss = np.array(
        [
            value(m.P_charge_bss[t]) - value(m.P_discharge_bss[t])
            for t in m.periods
        ]
    )

    res.P_ev = np.array(
        [
            value(m.P_charge_ev[t]) - value(m.P_discharge_ev[t])
            for t in m.periods
        ]
    )

    res.P_gen = np.array([value(m.P_gen[t]) for t in m.periods])

    res.P_hp_hot = np.array([value(m.P_hp_hot[t]) for t in m.periods])
    res.P_hp_cold = np.array([value(m.P_hp_cold[t]) for t in m.periods])
    res.T_hp = np.array([value(m.T_hp[t]) for t in m.periods])

    res.SOC_ev = np.array([value(m.SOC_ev[t]) for t in m.periods])
    res.SOC_bss = np.array([value(m.SOC_bss[t]) for t in m.periods])

    res.objective = value(m.objective)

    return res