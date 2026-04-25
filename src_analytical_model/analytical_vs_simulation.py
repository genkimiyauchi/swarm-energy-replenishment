#from sympy import *
#from IPython.display import Markdown as md
import numpy as np
import calculate_functions as cf
import random   

CASE1 = 1 # charger fully charged, workers rest
CASE2 = 2 # charger fully charged, charger rests
CASE3 = 3 # workers fully charged, workers rest
CASE4 = 4 # workers fully charged, charger rests

# distance = 500.0 # (cm) distance between charging and work locations
# speed = 12.0 #(cm/s) robot speed
#eta = 2.0
#nu_work_value = eta * nu_commute_value # rate of consuming energy when working
nu_min_value = 0.005 # how much a robot consumes when idle

# maximum values used from https://www.mdpi.com/1996-1073/15/3/674#B103-energies-15-00674 and https://lirias.kuleuven.be/retrieve/804774
# Capbot has 2 in parallel: 120F, 3V, 23g --> In Parallel: 240F, 3V, 46g --> E = 0.5 * C * V^2 = 0.5 * 240F * 3V^2 = 1080J --> 1080J --> 0.3Wh --> energy density 0.3 Wh/0.046kg = 6.52 Wh/kg
# fully charges in 16 second, runs for 51 minutes on top speed

## Values for capacitor: take duty cycle of capbot which is at 99%, rest of the information used from https://www.mdpi.com/1996-1073/15/3/674#B103-energies-15-00674
q_cap_value = 3060.0 # with 1/s when moving, can move for 51 minutes at full speed
nu_commute_value = 1.0 # rate of consuming energy when travelling per time unit
nu_charge_cap_value= q_cap_value / 16.0 # can be fully charged in 16 seconds
xi_cap = 0.5


## Values for battery depending on the capacitor values
q_bat_value = 10800 # E-puck 2: 3h autonomy
nu_charge_bat_value = q_bat_value / (2.5*60*60) # E-puck 2 charges in 2.5 hours
xi_bat = 0.99

n_c_value = 1
tau_values = [2, 3, 4, 5, 6, 7, 8, 9, 10]
zeta_values = [2, 3, 4, 5, 6, 7, 8, 9, 10]
eta_values = [0.5, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10]#list(np.arange(0.1, 20.1, 0.1))#
delta_commute_values_cap = list(range(10,501,10))# last three are too high for the capacity
delta_commute_values_bat = list(range(10,501,10)) #[700, 800, 900]#[35, 40, 45]#[5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60] #[30, 32, 34, 36, 38, 40, 42, 44, 46, 48, 50] # break point for low capacitor
capacity_values = [q_cap_value, q_bat_value]
nu_charge_values = [nu_charge_cap_value, nu_charge_bat_value]
xi_values = [xi_cap, xi_bat]


cases_CS_variable_cap = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values_cap), len(eta_values)))
EE_CS_variable_cap = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values_cap), len(eta_values)))
DC_CS_variable_cap = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values_cap), len(eta_values)))
work_CS_variable_cap = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values_cap), len(eta_values)))
cycle_duration_CS_variable_cap = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values_cap), len(eta_values)))
case_1_list_cap = []
case_2_list_cap = []
case_3_list_cap = []
case_4_list_cap = []

## Calculate the cases for various configurations for the supercap
for i in range(len(tau_values)):
    for j in range(len(zeta_values)):
        for k in range(len(delta_commute_values_cap)):
            case1_met = False
            case2_met = False
            case3_met = False
            case4_met = False
            for l in range(len(eta_values)):
                nu_work_value = eta_values[l] * nu_commute_value
                cases_CS_variable_cap[i, j, k, l], EE_CS_variable_cap[i, j, k, l], DC_CS_variable_cap[i, j, k, l], _, _ = cf.calculate_CS_case_differentiation(q_w_max=q_cap_value,
                                                                delta_commute=delta_commute_values_cap[k],
                                                                nu_work=nu_work_value,
                                                                nu_move=nu_commute_value,
                                                                nu_min=nu_min_value,
                                                                nu_charge=nu_charge_cap_value,
                                                                nu_transfer=nu_charge_cap_value,
                                                                xi=xi_cap,
                                                                tau=tau_values[i],
                                                                zeta=zeta_values[j],
                                                                n_c=n_c_value)
                if cases_CS_variable_cap[i, j, k, l] == CASE1:
                    case1_met = True
                    case_1_list_cap.append((tau_values[i], zeta_values[j], eta_values[l], delta_commute_values_cap[k]))
                elif cases_CS_variable_cap[i, j, k, l] == CASE2:
                    case2_met = True
                    case_2_list_cap.append((tau_values[i], zeta_values[j], eta_values[l], delta_commute_values_cap[k]))
                elif cases_CS_variable_cap[i, j, k, l] == CASE3:
                    case3_met = True
                    case_3_list_cap.append((tau_values[i], zeta_values[j], eta_values[l], delta_commute_values_cap[k]))
                elif cases_CS_variable_cap[i, j, k, l] == CASE4:
                    case4_met = True
                    case_4_list_cap.append((tau_values[i], zeta_values[j], eta_values[l], delta_commute_values_cap[k]))
                

flattened = cases_CS_variable_cap.flatten()

#print(f"Case 1 samples: {random.sample(case_1_list_cap, 1)}") 
#print(f"Case 2 samples: {random.sample(case_2_list_cap, 1)}") 
#print(f"Case 3 samples: {random.sample(case_3_list_cap, 1)}") 
#print(f"Case 4 samples: {random.sample(case_4_list_cap, 1)}") 

values, counts = np.unique(flattened, return_counts=True)
freqs = dict(zip(values, counts))
#print(freqs)



#### Compare the simulation results with the analytical results
cf.calculate_and_plot_scenario_with_log_file(q_w_max=q_cap_value, delta_commute=40, nu_work=1*nu_commute_value, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=5, zeta=5, n_c=1, simulation_result_log_file="results_2025-11-20/mobile_supercap_tau5_zeta5_eta1.00_dcomm40_001/log_data.pb")
cf.calculate_and_plot_scenario_with_log_file(q_w_max=q_bat_value, delta_commute=40, nu_work=1*nu_commute_value, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_bat_value, nu_transfer=nu_charge_bat_value, xi=0.99, tau=5, zeta=5, n_c=1, simulation_result_log_file="results_2025-11-20/mobile_battery_tau5_zeta5_eta1.00_dcomm40_001/log_data.pb")
cf.calculate_and_plot_scenario_with_log_file(q_w_max=q_cap_value, delta_commute=190, nu_work=8*nu_commute_value, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=3, zeta=4, n_c=1, simulation_result_log_file="results_2025-11-20/mobile_supercap_tau3_zeta4_eta8.00_dcomm190_001/log_data.pb")
cf.calculate_and_plot_scenario_with_log_file(q_w_max=q_cap_value, delta_commute=290, nu_work=5*nu_commute_value, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=7, zeta=3, n_c=1, simulation_result_log_file="results_2025-11-20/mobile_supercap_tau7_zeta3_eta5.00_dcomm290_001/log_data.pb")
cf.calculate_and_plot_scenario_with_log_file(q_w_max=q_cap_value, delta_commute=140, nu_work=7*nu_commute_value, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=8, zeta=2, n_c=1, simulation_result_log_file="results_2025-11-20/mobile_supercap_tau8_zeta2_eta7.00_dcomm140_001/log_data.pb")
cf.calculate_and_plot_scenario_with_multiple_log_files_test(q_w_max_list=[q_cap_value, q_bat_value],
    delta_commute_list=[40, 40], nu_work_list=[1*nu_commute_value, 1*nu_commute_value], nu_move_list=[nu_commute_value, nu_commute_value],
    nu_min_list=[nu_min_value, nu_min_value], nu_charge_list=[nu_charge_cap_value, nu_charge_bat_value], nu_transfer_list=[nu_charge_cap_value, nu_charge_bat_value],
    xi_list=[0.5, 0.99], tau_list=[5, 5], zeta_list=[5, 5], n_c_list=[1, 1],
    simulation_result_log_files=["results_2025-11-20/mobile_supercap_tau5_zeta5_eta1.00_dcomm40_001/log_data.pb",
                                 "results_2025-11-20/mobile_battery_tau5_zeta5_eta1.00_dcomm40_001/log_data.pb"], labels=["Supercap.", "Battery"])

# ----------------------------
# Example plots
# ----------------------------

#cf.plot_CS_polygon(q_w_max=q_cap_value, delta_commute=190, nu_work=8*1, nu_move=1, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=3, zeta=4, n_c=1)
#cf.plot_CS_polygon(q_w_max=q_cap_value, delta_commute=40, nu_work=1*1, nu_move=1, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=5, zeta=5, n_c=1)
#cf.plot_CS_polygon(q_w_max=q_cap_value, delta_commute=290, nu_work=5*1, nu_move=1, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=7, zeta=3, n_c=1)
#cf.plot_CS_polygon(q_w_max=q_cap_value, delta_commute=140, nu_work=7*1, nu_move=1, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=0.5, tau=8, zeta=2, n_c=1)
##plt.show()
param_list = [
    (q_cap_value, 190, 8*1, 1, nu_min_value, nu_charge_cap_value, nu_charge_cap_value, 0.5, 3, 4, 1),
    (q_cap_value, 40, 1*1, 1, nu_min_value, nu_charge_cap_value, nu_charge_cap_value, 0.5, 5, 5, 1),
    (q_cap_value, 290, 5*1, 1, nu_min_value, nu_charge_cap_value, nu_charge_cap_value, 0.5, 7, 3, 1),
    (q_cap_value, 140, 7*1, 1, nu_min_value, nu_charge_cap_value, nu_charge_cap_value, 0.5, 8, 2, 1)
]
cf.plot_CS_polygon_grid(param_list)