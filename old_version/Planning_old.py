from old_version.param import delta_t
from old_version.param import SOC_min_bss, SOC_max_bss, eff_bss
from old_version.param import C_ev, P_nom_ev, eff_ev, SOC_min_ev, SOC_max_ev, SOC_target_ev
from old_version.param import PI_gen, PI_imp, PI_exp
from old_version.param import P_max_hp, COP_hp, delta_T_max, C_hp

from pyomo.environ import (
    ConcreteModel,
    Param,
    Var,
    Objective,
    Constraint,
    NonNegativeReals,
    minimize,
    value,
)

from datetime import datetime
import old_version.utils as utils


# =============================================================================
# Constraint rules
# =============================================================================

# -----------------------------------------------------------------------------
# 1. General power balance
# -----------------------------------------------------------------------------
def constraint_rule_power_balance(model, t):
    return (
        model.P_pv[t]
        + model.P_gen[t]
        + model.P_discharge_bss[t]
        + model.P_discharge_ev[t]
        + model.P_imp[t]
        ==
        model.P_load[t]
        + model.P_charge_bss[t]
        + model.P_charge_ev[t]
        + model.P_hp_hot[t]
        + model.P_hp_cold[t]
        + model.P_exp[t]
    )


# -----------------------------------------------------------------------------
# 2. PV constraints
# -----------------------------------------------------------------------------
def constraint_rule_pv_available(model, t):
    # IMPORTANT:
    # P_pv_max[t] is an MPP profile/ratio.
    # Therefore available PV = C_pv * P_pv_max[t]
    return model.P_pv[t] <= model.C_pv * model.P_pv_max[t]


def constraint_rule_pv_inverter(model, t):
    return model.P_pv[t] <= model.P_nom_pv


# -----------------------------------------------------------------------------
# 3. Generator constraints
# -----------------------------------------------------------------------------
def constraint_rule_gen_max(model, t):
    return model.P_gen[t] <= model.P_max_gen


# -----------------------------------------------------------------------------
# 4. Battery storage constraints
# -----------------------------------------------------------------------------
def constraint_rule_bss_charge_power(model, t):
    return model.P_charge_bss[t] <= model.P_nom_bss


def constraint_rule_bss_discharge_power(model, t):
    return model.P_discharge_bss[t] <= model.P_nom_bss


def constraint_rule_bss_soc_min(model, t):
    return model.SOC_bss[t] >= SOC_min_bss * model.C_bss


def constraint_rule_bss_soc_max(model, t):
    return model.SOC_bss[t] <= SOC_max_bss * model.C_bss


def constraint_rule_bss_soc_dynamics(model, t):
    if t == 0:
        return (
            model.SOC_bss[t]
            ==
            model.SOC_0_bss
            + eff_bss * model.P_charge_bss[t] * delta_t
            - (model.P_discharge_bss[t] / eff_bss) * delta_t
        )

    return (
        model.SOC_bss[t]
        ==
        model.SOC_bss[t - 1]
        + eff_bss * model.P_charge_bss[t] * delta_t
        - (model.P_discharge_bss[t] / eff_bss) * delta_t
    )


def constraint_rule_bss_final_soc(model, t):
    # Apply only on last time step
    if t != model.last_period:
        return Constraint.Skip

    # Avoid ending the horizon with an artificially empty battery
    return model.SOC_bss[t] == model.SOC_0_bss


# -----------------------------------------------------------------------------
# 5. EV constraints
# -----------------------------------------------------------------------------
def constraint_rule_ev_charge_power(model, t):
    # If EV_connected[t] = 0 => P_charge_ev[t] <= 0
    return model.P_charge_ev[t] <= model.P_nom_ev * model.EV_connected[t]


def constraint_rule_ev_discharge_power(model, t):
    # If you do not want V2G/V2H, add a separate constraint fixing this to zero.
    return model.P_discharge_ev[t] <= model.P_nom_ev * model.EV_connected[t]


def constraint_rule_ev_soc_min(model, t):
    return model.SOC_ev[t] >= SOC_min_ev * model.C_ev


def constraint_rule_ev_soc_max(model, t):
    return model.SOC_ev[t] <= SOC_max_ev * model.C_ev


def constraint_rule_ev_soc_dynamics(model, t):
    # In utils.py, SOC_i_ev[t] is already in kWh.
    # If EV arrives at time t, its SOC is reset to arrival SOC.

    if t == 0:
        if value(model.t_arr[t]) == 1:
            initial_energy = model.SOC_i_ev[t]
        else:
            initial_energy = model.SOC_0_ev

        return (
            model.SOC_ev[t]
            ==
            initial_energy
            + eff_ev * model.P_charge_ev[t] * delta_t
            - (model.P_discharge_ev[t] / eff_ev) * delta_t
        )

    if value(model.t_arr[t]) == 1:
        return (
            model.SOC_ev[t]
            ==
            model.SOC_i_ev[t]
            + eff_ev * model.P_charge_ev[t] * delta_t
            - (model.P_discharge_ev[t] / eff_ev) * delta_t
        )

    return (
        model.SOC_ev[t]
        ==
        model.SOC_ev[t - 1]
        + eff_ev * model.P_charge_ev[t] * delta_t
        - (model.P_discharge_ev[t] / eff_ev) * delta_t
    )


def constraint_rule_ev_target_departure(model, t):
    # If EV does not leave at this time, skip.
    if value(model.t_dep[t]) == 0:
        return Constraint.Skip

    # At departure, EV should be at least target SOC.
    # >= is better than == because more than target is acceptable.
    return model.SOC_ev[t] >= SOC_target_ev * model.C_ev


def constraint_rule_no_ev_discharge(model, t):
    # Uncomment this constraint in create_model if EV should only charge.
    return model.P_discharge_ev[t] == 0


# -----------------------------------------------------------------------------
# 6. Heat pump and thermal comfort constraints
# -----------------------------------------------------------------------------
def constraint_rule_hp_hot_max(model, t):
    return model.P_hp_hot[t] <= P_max_hp


def constraint_rule_hp_cold_max(model, t):
    return model.P_hp_cold[t] <= P_max_hp


def constraint_rule_temperature_min(model, t):
    return model.T_hp[t] >= model.T_set[t] - delta_T_max


def constraint_rule_temperature_max(model, t):
    return model.T_hp[t] <= model.T_set[t] + delta_T_max


def constraint_rule_temperature_dynamics(model, t):
    if t == 0:
        return (
            model.T_hp[t]
            ==
            model.T_0_hp
            + (
                COP_hp * model.P_hp_hot[t]
                - COP_hp * model.P_hp_cold[t]
                - model.P_loss[t]
            )
            * delta_t
            / C_hp
        )

    return (
        model.T_hp[t]
        ==
        model.T_hp[t - 1]
        + (
            COP_hp * model.P_hp_hot[t]
            - COP_hp * model.P_hp_cold[t]
            - model.P_loss[t]
        )
        * delta_t
        / C_hp
    )


# =============================================================================
# Model creation
# =============================================================================
def create_model(res, C_pv, C_bss, P_nom_bss, P_nom_pv, P_max_gen):
    model = ConcreteModel()

    model.periods = range(res.t_s)
    model.last_period = res.t_s - 1

    # -------------------------------------------------------------------------
    # Parameters from Results object
    # -------------------------------------------------------------------------
    model.P_load = Param(
        model.periods,
        initialize=[res.P_load[t] for t in model.periods],
        mutable=True,
    )

    model.P_pv_max = Param(
        model.periods,
        initialize=[res.P_pv_max[t] for t in model.periods],
        mutable=True,
    )

    model.EV_connected = Param(
        model.periods,
        initialize=[res.EV_connected[t] for t in model.periods],
        mutable=True,
    )

    model.t_arr = Param(
        model.periods,
        initialize=[res.t_arr[t] for t in model.periods],
        mutable=True,
    )

    model.t_dep = Param(
        model.periods,
        initialize=[res.t_dep[t] for t in model.periods],
        mutable=True,
    )

    model.SOC_i_ev = Param(
        model.periods,
        initialize=[res.SOC_i_ev[t] for t in model.periods],
        mutable=True,
    )

    model.T_set = Param(
        model.periods,
        initialize=[res.T_set[t] for t in model.periods],
        mutable=True,
    )

    model.P_loss = Param(
        model.periods,
        initialize=[res.P_loss[t] for t in model.periods],
        mutable=True,
    )

    model.SOC_0_bss = Param(initialize=0.5 * C_bss, mutable=True)
    model.SOC_0_ev = Param(initialize=0.5 * C_ev, mutable=True)
    model.T_0_hp = Param(initialize=res.T_set[0], mutable=True)

    # -------------------------------------------------------------------------
    # Fixed asset sizes for operational planning
    # -------------------------------------------------------------------------
    model.P_nom_pv = Param(initialize=P_nom_pv)
    model.C_bss = Param(initialize=C_bss)
    model.C_pv = Param(initialize=C_pv)
    model.P_nom_bss = Param(initialize=P_nom_bss)
    model.P_nom_ev = Param(initialize=P_nom_ev)
    model.C_ev = Param(initialize=C_ev)
    model.P_max_gen = Param(initialize=P_max_gen)

    # -------------------------------------------------------------------------
    # Variables
    # -------------------------------------------------------------------------
    model.P_imp = Var(model.periods, within=NonNegativeReals)
    model.P_exp = Var(model.periods, within=NonNegativeReals)

    model.P_pv = Var(model.periods, within=NonNegativeReals)
    model.P_gen = Var(model.periods, within=NonNegativeReals)

    model.P_charge_bss = Var(model.periods, within=NonNegativeReals)
    model.P_discharge_bss = Var(model.periods, within=NonNegativeReals)

    model.P_charge_ev = Var(model.periods, within=NonNegativeReals)
    model.P_discharge_ev = Var(model.periods, within=NonNegativeReals)

    model.P_hp_hot = Var(model.periods, within=NonNegativeReals)
    model.P_hp_cold = Var(model.periods, within=NonNegativeReals)

    model.T_hp = Var(model.periods, within=NonNegativeReals)

    model.SOC_bss = Var(model.periods, within=NonNegativeReals)
    model.SOC_ev = Var(model.periods, within=NonNegativeReals)

    # -------------------------------------------------------------------------
    # Objective function
    # -------------------------------------------------------------------------
    eps = 1e-6

    model.objective = Objective(
        sense=minimize,
        expr=sum(
            delta_t
            * (
                PI_gen * model.P_gen[t]
                + PI_imp * model.P_imp[t]
                - PI_exp * model.P_exp[t]
                + eps
                * (
                    model.P_charge_bss[t]
                    + model.P_discharge_bss[t]
                    + model.P_charge_ev[t]
                    + model.P_discharge_ev[t]
                    + model.P_hp_hot[t]
                    + model.P_hp_cold[t]
                )
            )
            for t in model.periods
        ),
    )

    # -------------------------------------------------------------------------
    # Constraints
    # -------------------------------------------------------------------------

    # General
    model.constraint_power_balance = Constraint(
        model.periods,
        rule=constraint_rule_power_balance,
    )

    # PV
    model.constraint_pv_available = Constraint(
        model.periods,
        rule=constraint_rule_pv_available,
    )

    model.constraint_pv_inverter = Constraint(
        model.periods,
        rule=constraint_rule_pv_inverter,
    )

    # Generator
    model.constraint_gen_max = Constraint(
        model.periods,
        rule=constraint_rule_gen_max,
    )

    # BSS
    model.constraint_bss_charge_power = Constraint(
        model.periods,
        rule=constraint_rule_bss_charge_power,
    )

    model.constraint_bss_discharge_power = Constraint(
        model.periods,
        rule=constraint_rule_bss_discharge_power,
    )

    model.constraint_bss_soc_min = Constraint(
        model.periods,
        rule=constraint_rule_bss_soc_min,
    )

    model.constraint_bss_soc_max = Constraint(
        model.periods,
        rule=constraint_rule_bss_soc_max,
    )

    model.constraint_bss_soc_dynamics = Constraint(
        model.periods,
        rule=constraint_rule_bss_soc_dynamics,
    )

    model.constraint_bss_final_soc = Constraint(
        model.periods,
        rule=constraint_rule_bss_final_soc,
    )

    # EV
    model.constraint_ev_charge_power = Constraint(
        model.periods,
        rule=constraint_rule_ev_charge_power,
    )

    model.constraint_ev_discharge_power = Constraint(
        model.periods,
        rule=constraint_rule_ev_discharge_power,
    )

    model.constraint_ev_soc_min = Constraint(
        model.periods,
        rule=constraint_rule_ev_soc_min,
    )

    model.constraint_ev_soc_max = Constraint(
        model.periods,
        rule=constraint_rule_ev_soc_max,
    )

    model.constraint_ev_soc_dynamics = Constraint(
        model.periods,
        rule=constraint_rule_ev_soc_dynamics,
    )

    model.constraint_ev_target_departure = Constraint(
        model.periods,
        rule=constraint_rule_ev_target_departure,
    )

    # If EV discharge is not allowed, uncomment this:
    # model.constraint_no_ev_discharge = Constraint(
    #     model.periods,
    #     rule=constraint_rule_no_ev_discharge,
    # )

    # Heat pump
    model.constraint_hp_hot_max = Constraint(
        model.periods,
        rule=constraint_rule_hp_hot_max,
    )

    model.constraint_hp_cold_max = Constraint(
        model.periods,
        rule=constraint_rule_hp_cold_max,
    )

    model.constraint_temperature_min = Constraint(
        model.periods,
        rule=constraint_rule_temperature_min,
    )

    model.constraint_temperature_max = Constraint(
        model.periods,
        rule=constraint_rule_temperature_max,
    )

    model.constraint_temperature_dynamics = Constraint(
        model.periods,
        rule=constraint_rule_temperature_dynamics,
    )

    return model


def run(model, results):
    model, results = utils.solve_model(model, results)

    if model and results:
        utils.check_res(results)
        utils.print_res(results)
        utils.plot_res(results)

    return results


if __name__ == "__main__":
    start_time = datetime(2021, 1, 1, 0, 0, 0)

    # For debugging: 2 or 7
    # For final result: 365
    n_days = 365

    # Given quantities for the system sizes
    C_pv = 10
    C_bss = 40
    P_nom_bss = 10
    P_nom_pv = 10
    P_max_gen = 10

    # Do not keep these equal to zero.
    yearly_kwh = 3500
    yearly_km = 15000

    results = utils.Results(
        start_time,
        n_days,
        yearly_kwh=yearly_kwh,
        yearly_km=yearly_km,
    )

    model = create_model(
        results,
        C_pv,
        C_bss,
        P_nom_bss,
        P_nom_pv,
        P_max_gen,
    )

    run(model, results)