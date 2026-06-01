from param import delta_t
from param import SOC_min_bss, SOC_max_bss, eff_bss
from param import C_ev, P_nom_ev, eff_ev, SOC_min_ev, SOC_max_ev, SOC_target_ev
from param import PI_gen, PI_imp, PI_exp
from param import P_max_hp, COP_hp, delta_T_max, C_hp

from pyomo.environ import ConcreteModel, Param, Var, Objective, Constraint, NonNegativeReals, minimize, value
from datetime import datetime
import utils


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
    
    return model.SOC_bss[i] == model.SOC_0_bss


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

    return (model.SOC_ev[i-1] == SOC_target_ev *model.C_ev)  # leaving -> must have the target SOC at the previous step

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
    return model.T_hp[i] <= model.T_set[i] + delta_T_max

# Min temperature
def constraint_rule_T_min(model, i):
    return model.T_hp[i] >= model.T_set[i] - delta_T_max

# Max P HP hot
def constraint_rule_hp_hot_max(model, i):
    return model.P_hp_hot[i] <= P_max_hp

# max P HP cold
def constraint_rule_hp_cold_max(model, i):
    return model.P_hp_cold[i] <= P_max_hp

# Controllable generation ---------------------------------------

def constraint_rule_gen_max(model, i):
    return model.P_gen[i] <= model.P_max_gen   



# -----------------------------------------------------------------------------
def create_model(res,C_pv,C_bss,P_nom_bss, P_nom_pv, P_max_gen):
    # Create a concrete model
    model = ConcreteModel()
    
    model.periods = range(res.t_s)   # integer from 0 to N-1 representing all time slots
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
    model.SOC_0_bss = Param(initialize=0.5*C_bss, mutable=True)
    model.SOC_0_ev = Param(initialize=0.5*C_ev, mutable=True)
    model.T_0_hp = Param(initialize=res.T_set[0], mutable=True)

    # Operation planning asset sizes
    model.P_nom_pv = Param(initialize=P_nom_pv)                            # Nominal power for PV inverter
    model.C_bss = Param(initialize=C_bss)                                  # Battery capacity
    model.C_pv = Param(initialize=C_pv)                                    # PV system size
    model.P_nom_bss = Param(initialize=P_nom_bss)                          # Battery inverter nominal power
    model.P_nom_ev = Param(initialize=P_nom_ev)                            # EV inverter nominal power
    model.C_ev = Param(initialize=C_ev)                                    # EV capacity
    model.P_max_gen = Param(initialize=P_max_gen)                          # Maximum generator power

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


    # Energy storage variables for battery and EV
    model.SOC_bss = Var(model.periods, within=NonNegativeReals)            # Bss state of charge [kWh]
    model.SOC_ev = Var(model.periods, within=NonNegativeReals)             # EV state of charge [kWh]
    
    # Define the objective function ----------------------------------------------------------------------------
    model.objective = Objective(sense=minimize, 
    expr=sum(delta_t * (PI_imp * model.P_imp[i] - PI_exp * model.P_exp[i] + PI_gen * model.P_gen[i]) #  cost of import of electricity - cost of export of electricity + cost of Generator
    for i in model.periods))
    
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

   
    return model


def run(model, results):
    model, results = utils.solve_model(model, results)
    if model and results:
        utils.check_res(results)
        utils.print_res(results)
        utils.plot_res_day(results)
        #utils.plot_res_week(results)   
    return results


if __name__ == "__main__":
    start_time = datetime(2021, 1, 1, 0, 0, 0)                                  # Start time of the simulation [YYYY, MM, DD, HH, MM, SS]
    n_days = 365                                                                 # Number of days to simulate

    # Given quantities for the system sizes
    C_pv = 10                            # PV system size [kWp]
    C_bss = 40                           # Battery capacity [kWh]	
    P_nom_bss = 10                       # Battery inverter nominal power [kW]
    P_nom_pv = 10                        # PV inverter nominal power [kW]
    P_max_gen = 10                       # Maximum generator power [kW]


    results = utils.Results(start_time, n_days, yearly_kwh=3900, yearly_km=14770)      # Initialize results object with start time and number of days, yearly consumption and km driven
    model = create_model(results,C_pv,C_bss,P_nom_bss, P_nom_pv, P_max_gen)
    run(model, results)

# Report: source = https://pyomo.readthedocs.io/en/6.7.3/pyomo_overview/simple_examples.html
