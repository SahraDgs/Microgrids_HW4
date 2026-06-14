from param import delta_t, C_pv_max, C_bss_max
from param import SOC_min_bss, SOC_max_bss, eff_bss
from param import C_ev, P_nom_ev, eff_ev, SOC_min_ev, SOC_max_ev, SOC_target_ev
from param import PI_gen, PI_imp, PI_exp
from param import P_max_hp, COP_hp, delta_T_max, C_hp
from param import PI_c_pv, PI_c_bss, PI_c_inv, PI_c_gen, inv_hor
from param import isolated_microgrids, unpaid_exp       # point 3 for sizing

from pyomo.environ import ConcreteModel, Param, Var, Objective, Constraint, NonNegativeReals, minimize, value
from datetime import datetime
import utils as utils
import numpy as np

# HOW TO USE THIS CODE
#
# - Scenarios (point 3):
#   Set isolated_microgrids or unpaid_exp to True in utils.py to switch scenario.
#
# - Plot analyses (points 4 to 6):
#   Uncomment the corresponding line in the __main__ section at the
#   bottom of this file to produce each plot used in the report.



# Constraint Rules definitions

# General -----------------------------------------------

def constraint_rule_Pbalance(model, i): 
    return (model.P_pv[i] + model.P_gen[i] + model.P_imp[i] + model.P_discharge_bss[i] + model.P_discharge_ev[i] == model.P_exp[i] + model.P_charge_bss[i] + model.P_charge_ev[i] + model.P_hp_hot[i] + model.P_hp_cold[i] + model.P_load[i])

# Power generation (PV pannels)---------------------------
def constraint_rule_pv_max(model, i):
    return (model.P_pv[i] <= model.P_pv_max[i] * model.C_pv)

def constraint_rule_nom_pv(model, i):
    return (model.P_pv[i] <= model.P_nom_pv)


# Energy storage (Battery)--------------------------------

def constraint_rule_SOC_max_bss(model, i):
    return (model.SOC_bss[i] <= SOC_max_bss * model.C_bss)

def constraint_rule_SOC_min_bss(model, i):
    return (model.SOC_bss[i] >= SOC_min_bss * model.C_bss)

def constraint_rule_eff_bss(model, i):
    if i == model.periods[-1]:   # on the last step: avoid errors when the time i+1 does not exist in the data
        return Constraint.Skip
    
    return model.SOC_bss[i+1] == model.SOC_bss[i] + eff_bss * model.P_charge_bss[i] * delta_t - model.P_discharge_bss[i] * delta_t
     #consider efficiency only on charging (efficiency on discharging = 1) see slide 6

def constraint_rule_Pbss_nom_charge(model, i):
    return (model.P_charge_bss[i] <= model.P_nom_bss)   # output of the inverter = input or output of the battery (discharge or charge) 

def constraint_rule_Pbss_nom_discharge(model, i):
    return (model.P_discharge_bss[i] <= model.P_nom_bss)

def constraint_rule_SOC_bss_init(model, i):
    if i != 0:   # because this constraint apply only to the first step
        return Constraint.Skip
    
    return model.SOC_bss[i] == model.SOC_0_bss * model.C_bss # multiply here because as C_bss is now a variable we 
                                                             # cannot multiply directly  in the initialization of SOC_0_bss


# Eletric Vehicle -----------------------------------------

def constraint_rule_SOC_max_ev(model, i):
    return (model.SOC_ev[i] <= SOC_max_ev * model.C_ev)

def constraint_rule_SOC_min_ev(model, i):
    return (model.SOC_ev[i] >= SOC_min_ev * model.C_ev)

def constraint_rule_eff_ev(model, i):
    if i == model.periods[-1]:   # on the last step: avoid errors when the time i+1 does not exist in the data
        return Constraint.Skip
    
    if value(model.EV_connected[i]) == 0 or value(model.EV_connected[i+1]) == 0:    # if the EV is not connected, the SOC is not tracked
        return Constraint.Skip
    
    return model.SOC_ev[i+1] == model.SOC_ev[i] + eff_ev * model.P_charge_ev[i] * delta_t - (1/eff_ev) * model.P_discharge_ev[i] * delta_t 
    #consider efficiency on both charging and  discharging -> see slide 7 (this time no specified -> for both)

def constraint_rule_SOC_target_leaving(model, i):
    if i == model.periods[0]:   # on the first step: avoid errors when the time i-1 does not exist in the data
        return Constraint.Skip
    if value(model.t_dep[i]) == 0:   # not leaving -> no constraint rule
        return Constraint.Skip

    return (model.SOC_ev[i-1] >= SOC_target_ev *model.C_ev)  # leaving -> must have the target SOC at the previous step   CHANGE : wa ==

def constraint_rule_Pev_nom_charge(model, i):
    if value(model.EV_connected[i]) == 0:    #if not connected -> impossible to charge
        return (model.P_charge_ev[i] == 0)
    
    return (model.P_charge_ev[i] <= model.P_nom_ev)   # charge possible only if EV is connected 

def constraint_rule_Pev_nom_discharge(model, i):
    if value(model.EV_connected[i]) == 0:    #if not connected -> impossible to discharge in our microgrid
        return (model.P_discharge_ev[i] == 0)
    
    return (model.P_discharge_ev[i] <= model.P_nom_ev)


def constraint_rule_SOC_ev_init(model, i):
    if i != 0:   # because this constraint apply only to the first step (init)
        return Constraint.Skip
    if value(model.SOC_i_ev[i]) != 0:   # because if we are plug-in at start, we must use the "Initial SOC of the EV connected"
        return Constraint.Skip
    
    # if the car is away at start we want to give an initial value (to start somewhere)
    return (model.SOC_ev[i] == model.SOC_0_ev)


def constraint_rule_SOC_plug_in(model, i):  
    if value(model.SOC_i_ev[i]) == 0:
        return Constraint.Skip
    
    return (model.SOC_ev[i] == model.SOC_i_ev[i])


# Heat pump ---------------------------------

# thermal dynamics 
def constraint_rule_hp_dynamics(model, i):
    if i == model.periods[-1]: # Do not treat the last step to avoid overflow error
        return Constraint.Skip
    else:
        #thermal balance
        return (C_hp * model.T_hp[i+1] == C_hp * model.T_hp[i]
        + delta_t * (COP_hp * model.P_hp_hot[i] - COP_hp * model.P_hp_cold[i] - model.P_loss[i]))
        # thermal energy of the house at t+1 = thermal energy at t 
        # + heat entering (thermal energy added) - heat removed (thermal energy removed)  - losses) 

# thermal initial temperature
def constraint_rule_hp_init(model, i):
    if i != 0:
        return Constraint.Skip
    return model.T_hp[i] == model.T_0_hp

# Max temperature
def constraint_rule_T_max(model, i):
    return model.T_hp[i] <= model.T_set[i] + model.delta_T_max

# Min temperature
def constraint_rule_T_min(model, i):
    return model.T_hp[i] >= model.T_set[i] - model.delta_T_max

# Max P HP hot
def constraint_rule_hp_hot_max(model, i):
    return model.P_hp_hot[i] <= P_max_hp

# max P HP cold
def constraint_rule_hp_cold_max(model, i):
    return model.P_hp_cold[i] <= P_max_hp

# Controllable generation ---------------------------------------

def constraint_rule_gen_max(model, i):
    return model.P_gen[i] <= model.P_max_gen

# sizing phase:

# Let the battery in the same state as we found it (not borrow energy fron another year)
def constraint_rule_SOC_bss_same_state(model):
    return model.SOC_bss[model.periods[-1]] == model.SOC_bss[0]

# Sizing phase: isolated microgrids

def constraint_rule_no_imp(model, i):
    if not isolated_microgrids:
        return Constraint.Skip
    
    return model.P_imp[i] == 0

def constraint_rule_no_exp(model, i):
    if not isolated_microgrids:
        return Constraint.Skip
    
    return model.P_exp[i] == 0

def constraint_rule_budget_limit(model):
    return model.budget >= (PI_c_pv * model.C_pv + PI_c_bss * model.C_bss + PI_c_inv *model.P_nom_bss + PI_c_inv *model.P_nom_pv + PI_c_gen* model.P_max_gen)



def create_model(res, budget = None, delta_T_max_pt5 = None, price_growth = 0.0):
    # Create a concrete model
    model = ConcreteModel()
    
    model.periods = range(res.t_s)
    # model.connections = range(len(res.t_arr)) # A set is created with length equal to the number of EV connections

    # Access all parameters present in the res object.
    # These exists for each t in model.t:
    #  - model.P_load[t] = Load power at time t
    #  - model.P_pv_max[t] = Max available PV power at time t
    #  - model.EV_connected[t] = EV connected at time t
    #  - model.t_arr[t] = Bollean connexion of the EV
    #  - model.t_dep[t] = Boolean disconnexion of the EV
    #  - model.SOC_i_ev[t] = Initial SOC of the EV connected at time t (only non zero if t_arr[t] is true)
    model.P_load = Param(model.periods, initialize=[res.P_load[t] for t in model.periods], mutable=True)
    model.P_pv_max = Param(model.periods, initialize=[res.P_pv_max[t] for t in  model.periods], mutable=True)
    model.EV_connected = Param(model.periods, initialize=[res.EV_connected[t] for t in  model.periods], mutable=True)
    model.t_arr = Param(model.periods, initialize=[res.t_arr[t] for t in model.periods], mutable=True)
    model.t_dep = Param(model.periods, initialize=[res.t_dep[t] for t in model.periods], mutable=True)
    model.SOC_i_ev = Param(model.periods, initialize=[res.SOC_i_ev[t] for t in model.periods], mutable=True)
    model.T_set = Param(model.periods, initialize=[res.T_set[t] for t in model.periods], mutable=True)
    model.P_loss = Param(model.periods, initialize=[res.P_loss[t] for t in model.periods], mutable=True)
    model.SOC_0_bss = Param(initialize=0.5, mutable=True)
    model.SOC_0_ev = Param(initialize=0.5*C_ev, mutable=True)
    model.T_0_hp = Param(initialize=res.T_set[0], mutable=True)


    # Asset sizes
    model.P_nom_pv = Var(within=NonNegativeReals)                           # Nominal power for PV inverter
    model.C_bss = Var(within=[0,C_bss_max])                                 # Battery capacity
    model.C_pv = Var(within=[0,C_pv_max])                                   # PV system size
    model.P_nom_bss = Var(within=NonNegativeReals)                          # Battery inverter nominal power
    model.P_max_gen = Var(within=NonNegativeReals)                          # Maximum generator power
    model.P_nom_ev = Param(initialize=P_nom_ev)                             # EV charger nominal power
    model.C_ev = Param(initialize=C_ev)                                     # EV capacity
    model.P_nom_hp = Param(initialize=P_max_hp)                                 # HP max power

    # Variables
    model.P_imp = Var(model.periods, within=NonNegativeReals)              # Imported power
    model.P_exp = Var(model.periods, within=NonNegativeReals)              # Exported power
    model.P_pv = Var(model.periods, within=NonNegativeReals)               # PV power output 
    model.P_gen = Var(model.periods, within=NonNegativeReals)              # Generator power output 
    model.P_charge_bss = Var(model.periods, within=NonNegativeReals)       # Battery charging power 
    model.P_discharge_bss = Var(model.periods, within=NonNegativeReals)    # Battery discharging power 
    model.P_charge_ev = Var(model.periods, within=NonNegativeReals)        # EV charging power 
    model.P_discharge_ev = Var(model.periods, within=NonNegativeReals)     # EV discharging power 
    model.P_hp_hot = Var(model.periods, within=NonNegativeReals) 
    model.P_hp_cold = Var(model.periods, within=NonNegativeReals) 
    model.T_hp = Var(model.periods, within=NonNegativeReals)
    model.T_dev = Var(model.periods, within=NonNegativeReals)   # was not in the planning phase ??????


    # Energy storage variables for battery and EV
    model.SOC_bss = Var(model.periods, within=NonNegativeReals)            # Bss state of charge [kWh]
    model.SOC_ev = Var(model.periods, within=NonNegativeReals)             # EV state of charge [kWh]

    # to make the HP load vary (sizing phase: point 4)
    if not delta_T_max_pt5 is None:
        model.delta_T_max = Param(initialize=delta_T_max_pt5, mutable=True)
    else:
        model.delta_T_max = Param(initialize=delta_T_max, mutable=True) 
    
    # Define the objective function ----------------------------------------------------------------------------
    #  cost of import of electricity - cost of export of electricity + cost of Generator
    if unpaid_exp:
        PI_exp_case = 0
    else:
        PI_exp_case = PI_exp

    # average factor over the horizon(1 if price_growth=0)
    price_factor_avg = np.mean([(1 + price_growth)**y for y in range(inv_hor)])    

    opex_year = sum(delta_t * (PI_imp * model.P_imp[i] - PI_exp_case * model.P_exp[i] + PI_gen * model.P_gen[i]) for i in model.periods)

    capex_annual = (PI_c_pv * model.C_pv + PI_c_bss * model.C_bss + PI_c_inv *model.P_nom_bss + PI_c_inv *model.P_nom_pv + PI_c_gen* model.P_max_gen)/inv_hor

    model.objective = Objective(sense=minimize, expr = price_factor_avg * opex_year + capex_annual)
    
    
    #Constraints ---------------------------------------------------------------------------------------------------------------------------
    #General
    model.constraint_Pbalance = Constraint(model.periods, rule = constraint_rule_Pbalance)

    # Power generation (PV pannels) 
    model.constraint_pv_max = Constraint(model.periods, rule = constraint_rule_pv_max)
    model.constraint_nom_pv = Constraint(model.periods, rule = constraint_rule_nom_pv)

    # Energy storage (Battery)
    model.constraint_SOC_max_bss = Constraint(model.periods, rule = constraint_rule_SOC_max_bss)
    model.constraint_SOC_min_bss = Constraint(model.periods, rule = constraint_rule_SOC_min_bss)
    model.constraint_eff_bss = Constraint(model.periods, rule = constraint_rule_eff_bss)
    model.constraint_Pbss_nom_discharge = Constraint(model.periods, rule = constraint_rule_Pbss_nom_discharge)
    model.constraint_Pbss_nom_charge = Constraint(model.periods, rule = constraint_rule_Pbss_nom_charge)
    model.constraint_SOC_bss_init = Constraint(model.periods, rule = constraint_rule_SOC_bss_init)

    # Eletric Vehicle
    model.constraint_SOC_max_ev = Constraint(model.periods, rule = constraint_rule_SOC_max_ev)
    model.constraint_SOC_min_ev = Constraint(model.periods, rule = constraint_rule_SOC_min_ev)
    model.constraint_eff_ev = Constraint(model.periods, rule = constraint_rule_eff_ev)
    model.constraint_Pev_nom_charge = Constraint(model.periods, rule = constraint_rule_Pev_nom_charge)
    model.constraint_Pev_nom_discharge = Constraint(model.periods, rule = constraint_rule_Pev_nom_discharge)
    model.constraint_SOC_target_leaving = Constraint(model.periods, rule = constraint_rule_SOC_target_leaving)
    model.constraint_SOC_ev_init = Constraint(model.periods, rule = constraint_rule_SOC_ev_init)
    model.constraint_SOC_plug_in = Constraint(model.periods, rule = constraint_rule_SOC_plug_in)

    # Fixed load and Heat pump
    model.constraint_hp_dynamics = Constraint(model.periods, rule=constraint_rule_hp_dynamics)
    model.constraint_hp_init = Constraint(model.periods, rule=constraint_rule_hp_init)
    model.constraint_T_max = Constraint(model.periods, rule=constraint_rule_T_max)
    model.constraint_T_min = Constraint(model.periods, rule=constraint_rule_T_min)
    model.constraint_hp_hot_max = Constraint(model.periods, rule=constraint_rule_hp_hot_max)
    model.constraint_hp_cold_max = Constraint(model.periods, rule=constraint_rule_hp_cold_max)

    # Controllable generation
    model.constraint_gen_max = Constraint(model.periods, rule=constraint_rule_gen_max)

    # Isolated microgrids (sizing phase: point 3)
    model.constraint_no_imp = Constraint(model.periods, rule=constraint_rule_no_imp)
    model.constraint_no_exp = Constraint(model.periods, rule=constraint_rule_no_exp)

    # Battery (sizing phase)
    model.constraint_SOC_bss_same_state = Constraint(rule=constraint_rule_SOC_bss_same_state)

    # to set a budget limit (sizing phase: point 4)
    if not budget is None:
        model.budget = Param(initialize=budget)
        model.constraint_budget_limit = Constraint(rule=constraint_rule_budget_limit)
    
    

    return model

def run(model, results):
    model, results = utils.solve_model(model, results)
    if model and results:
        utils.check_res(results)
        utils.print_sizing_results(results)
        return results         # modif for budget part
    return None             # modif for budget part

#Phase 2: point 4 budget ----------------------
def budget_dependance(n_days, start_time):
    start_time = datetime(2021, 1, 1, 0, 0, 0)                                  
    n_days = 365                                                                

    results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=14770)

    budget_list = list(range(5000, 40000, 3000)) 
    PV_system_sizes = []
    PV_inverter_sizes = []
    Battery_sizes = []
    BSS_inverter_sizes = []
    Diesel_genset_sizes = []
    kept_budgets = []
    Total = []
    for budget_i in budget_list: 
        model = create_model(results, budget=budget_i)
        run_res = run(model, results)
        if run_res is None:
            print(f"Budget {budget_i}: infeasible")
            continue

        #save results
        kept_budgets.append(budget_i)
        PV_system_sizes.append(results.C_pv)
        PV_inverter_sizes.append(results.P_nom_pv)
        Battery_sizes.append(results.C_bss)
        BSS_inverter_sizes.append(results.P_nom_bss)
        Diesel_genset_sizes.append(results.P_max_gen)
        Total.append(results.objective)

    utils.plot_sizing_budget(kept_budgets, PV_system_sizes, PV_inverter_sizes, Battery_sizes, BSS_inverter_sizes, Diesel_genset_sizes, Total)

def HP_dependance(n_days, start_time):

    temp_range_list = list(np.arange(0, 15, 1)) 
    PV_system_sizes = []
    PV_inverter_sizes = []
    Battery_sizes = []
    BSS_inverter_sizes = []
    Diesel_genset_sizes = []
    kept_temp_range = []
    Total = []
    for temp_range_i in temp_range_list:
        results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=14770)
        model = create_model(results, delta_T_max_pt5=temp_range_i)
        run_res = run(model, results)
        if run_res is None:
            print(f"comfort temperature {temp_range_i}: infeasible")
            continue

        #save results
        kept_temp_range.append(temp_range_i)
        PV_system_sizes.append(results.C_pv)
        PV_inverter_sizes.append(results.P_nom_pv)
        Battery_sizes.append(results.C_bss)
        BSS_inverter_sizes.append(results.P_nom_bss)
        Diesel_genset_sizes.append(results.P_max_gen)
        Total.append(results.objective)

    utils.plot_sizing(kept_temp_range, PV_system_sizes, PV_inverter_sizes, Battery_sizes, BSS_inverter_sizes, Diesel_genset_sizes, "HP")

    

def EV_dependance(n_days, start_time):
                                                                

    yearly_km_list = list(range(0, 30000, 2000)) 
    PV_system_sizes = []
    PV_inverter_sizes = []
    Battery_sizes = []
    BSS_inverter_sizes = []
    Diesel_genset_sizes = []
    kept_yearly_km = []
    Total = []
    for yearly_km_i in yearly_km_list:
        results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=yearly_km_i)
        model = create_model(results)
        run_res = run(model, results)
        if run_res is None:
            print(f"yearly_km {yearly_km_i}: infeasible")
            continue

        #save results
        kept_yearly_km.append(yearly_km_i)
        PV_system_sizes.append(results.C_pv)
        PV_inverter_sizes.append(results.P_nom_pv)
        Battery_sizes.append(results.C_bss)
        BSS_inverter_sizes.append(results.P_nom_bss)
        Diesel_genset_sizes.append(results.P_max_gen)
        Total.append(results.objective)

    utils.plot_sizing(kept_yearly_km, PV_system_sizes, PV_inverter_sizes, Battery_sizes, BSS_inverter_sizes, Diesel_genset_sizes, "EV")

def base_load_dependance(n_days, start_time):

    yearly_kwh_list = list(range(0, 7000, 500)) 
    PV_system_sizes = []
    PV_inverter_sizes = []
    Battery_sizes = []
    BSS_inverter_sizes = []
    Diesel_genset_sizes = []
    kept_yearly_kwh = []
    Total = []
    for yearly_kwh_i in yearly_kwh_list:
        results = utils.Results(start_time, n_days, yearly_kwh=yearly_kwh_i, yearly_km=14770)
        model = create_model(results)
        run_res = run(model, results)
        if run_res is None:
            print(f"yearly_kwh {yearly_kwh_i}: infeasible")
            continue

        #save results
        kept_yearly_kwh.append(yearly_kwh_i)
        PV_system_sizes.append(results.C_pv)
        PV_inverter_sizes.append(results.P_nom_pv)
        Battery_sizes.append(results.C_bss)
        BSS_inverter_sizes.append(results.P_nom_bss)
        Diesel_genset_sizes.append(results.P_max_gen)
        Total.append(results.objective)

    utils.plot_sizing(kept_yearly_kwh, PV_system_sizes, PV_inverter_sizes, Battery_sizes, BSS_inverter_sizes, Diesel_genset_sizes, "baseload")

def cost_dependance_horizon(n_days, start_time):

    price_growth_list = [-0.08, -0.04, 0.0, 0.04, 0.08]
    PV_system_sizes = []
    PV_inverter_sizes = []
    Battery_sizes = []
    BSS_inverter_sizes = []
    Diesel_genset_sizes = []
    Total = []
    for price_growth_i in price_growth_list:
        results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=14770)
        model = create_model(results, price_growth = price_growth_i)
        run(model, results)

        #save results
        PV_system_sizes.append(results.C_pv)
        PV_inverter_sizes.append(results.P_nom_pv)
        Battery_sizes.append(results.C_bss)
        BSS_inverter_sizes.append(results.P_nom_bss)
        Diesel_genset_sizes.append(results.P_max_gen)
        Total.append(results.objective)

    utils.plot_sizing(price_growth_list, PV_system_sizes, PV_inverter_sizes, Battery_sizes, BSS_inverter_sizes, Diesel_genset_sizes, "Price_growth")


if __name__ == "__main__":
    start_time = datetime(2021, 1, 1, 0, 0, 0)                                  # Start time of the simulation [YYYY, MM, DD, HH, MM, SS]
    n_days = 365                                                                 # Number of days to simulate

    results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=14770)      # Initialize results object with start time and number of days, yearly consumption and km driven
    model = create_model(results)
    run(model, results)

    # Analysis plot: to uncomment if needed
    #budget_dependance(n_days, start_time)
    #HP_dependance(n_days, start_time)
    #EV_dependance(n_days, start_time)
    #base_load_dependance(n_days, start_time)
    #cost_dependance_horizon(n_days, start_time)

