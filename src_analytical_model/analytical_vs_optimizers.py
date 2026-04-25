import datetime
import sys
import numpy as np
import scipy.optimize as opt
from casadi import *
import calculate_functions as cf
import time
import matplotlib.pyplot as plt

OPTIMIZE = False # if set to true, the optimizations will be run again, otherwise, the timestamps of the already existing results will be used to load the results from the .npz files. 

delta_c_charge_SX = SX.sym('delta_c_charge')
delta_transfer_SX = SX.sym('delta_transfer')
delta_w_work_SX = SX.sym('delta_w_work')
delta_w_rest_SX = SX.sym('delta_w_rest')
delta_c_rest_SX = SX.sym('delta_c_rest')
q_c_charged_SX = SX.sym('q_c_charged')
q_w_charged_SX = SX.sym('q_w_charged')

def calculate_CS_feasible(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):
    help = (zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min)
    c_m_min = (2*delta_commute*(nu_move + nu_min + nu_min * help)) / (1 - nu_min / (nu_charge - nu_min) * help)
    c_w_min = (2*delta_commute + c_m_min/(nu_charge-nu_min))*nu_min
    return c_m_min <= tau * q_w_max and c_w_min <= q_w_max


def calculate_CS_all_variables_timed(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):
    c_m_max = tau * q_w_max # chargers capacity is tau times the workers capacity
    n_w = zeta * n_c 

    start = time.time()
    # extract delta_c_rest for case 2
    delta_c_rest_case_2 = np.maximum(0, q_w_max/(nu_work+nu_min)*(1-nu_min/nu_charge)-2*delta_commute*(1+nu_move/nu_charge) - q_w_max/nu_charge * (zeta/xi * nu_transfer + nu_min) / (nu_transfer - nu_min))
    # either maximum capacity, or case 2 where it can supply all workers with full capacity even when having less than the maximum capacity, also limit to 0 in the (very unlikely) case that nu_min is way bigger than nu_transfer 
    q_c_charged = np.maximum(0, np.minimum(c_m_max, 2 * (nu_move + nu_min) * delta_commute + q_w_max * (zeta/xi * nu_transfer + nu_min) / (nu_transfer - nu_min) + nu_min * delta_c_rest_case_2))
    # Directly following
    delta_c_charge = q_c_charged/(nu_charge-nu_min)
    # extract delta_c_rest for case 1
    delta_c_rest_case_1 = np.maximum(0, (q_c_charged - 2 * (nu_move + nu_min) * delta_commute)/nu_min - (q_c_charged/(nu_charge - nu_min) * nu_charge - 2 * delta_commute * nu_move) / (nu_min + nu_min**2/(nu_work + nu_min) * (nu_transfer - nu_min)/(zeta/xi * nu_transfer + nu_min)))
    delta_transfer = np.maximum(0, np.minimum(q_w_max /(nu_transfer - nu_min), (q_c_charged - 2 * (nu_move + nu_min) * delta_commute)/(zeta/xi * nu_transfer + nu_min) - nu_min/(zeta/xi * nu_transfer + nu_min) * delta_c_rest_case_1)) # Either the time the charger needs to give the worker its remaining capacity, or the time it needs to charge all workers to the maximum capacity (considering that there is a energy loss) 
    q_w_charged = (nu_transfer - nu_min) * delta_transfer # the max boundary for q_w_charged is already encoded in delta_transfer, so we can just multiply it with the rate at which the charger can transfer energy to the worker
    delta_c_rest = np.maximum(0, (q_c_charged - 2*delta_commute * (nu_move + nu_min))/nu_min - delta_transfer * (zeta/xi * nu_transfer + nu_min)/nu_min)
    # The maximum of delta_work should already be encapsulated in q_w_charged (delta_transfer, respectively)
    delta_w_work = np.maximum(0, (q_w_charged - (delta_c_charge + 2*delta_commute + delta_c_rest) * nu_min) / nu_work) # either the remaining capacity divided by the wpork rate when substracting the time the charger needs to get new energy or the time the worker needs to use all its charge for working
    delta_w_rest = np.maximum(0, (q_w_charged - delta_w_work*(nu_work+nu_min))/nu_min)
    
    end = time.time()    
    
    
    cycle_duration = delta_c_charge + 2 * delta_commute + delta_transfer + delta_c_rest
    energy_efficiency_2 = (n_w * delta_w_work * nu_work)/(n_c * (q_c_charged + delta_c_charge*nu_min)) # same as 1
    duty_cycle = delta_w_work/cycle_duration 
    work_2 = n_w * duty_cycle

    # Check bounds and constraints for analytical solution (same as for optimizers: 0 tolerance for bounds, 1e-8 tolerance for equality constraints)
    tolerance = 0
    success = 1

    if q_c_charged > tau * q_w_max + tolerance or q_c_charged < -tolerance or q_w_charged > q_w_max + tolerance:
        success = 0
        print("Warning: q_c_charged exceeded in analytical solution: ", q_c_charged, " (max: ", tau * q_w_max, ")")
    if q_w_charged < -tolerance or q_w_charged > q_w_max + tolerance:
        success = 0
        print("Warning: q_c_charged exceeded in analytical solution: ", q_w_charged, " (max: ", tau * q_w_max, ")")
    if delta_c_charge < -tolerance or delta_transfer < -tolerance or delta_w_work < -tolerance or delta_w_rest < -tolerance or delta_c_rest < -tolerance:
        success = 0
        print("Warning: negative time variable in analytical solution: ", delta_c_charge, delta_transfer, delta_w_work, delta_w_rest, delta_c_rest)
    
    # check constraints
    con = [
            # con_1
            q_c_charged - (nu_charge - nu_min) * delta_c_charge,
            # con_2
            delta_transfer - (q_c_charged - 2*delta_commute*(nu_move+nu_min)-delta_c_rest*nu_min) / (zeta/xi*nu_transfer + nu_min),
            # con_3
            delta_w_work + delta_w_rest + delta_transfer - (delta_c_charge + 2*delta_commute + delta_transfer + delta_c_rest),
            # con_4
            q_w_charged - (nu_transfer - nu_min) * delta_transfer,
            # con_5
            q_w_charged - (delta_w_work * (nu_work+nu_min) + delta_w_rest * nu_min)
        ]

    tolerance = 1e-8
    # constraints must be == 0
    for j, c in enumerate(con):
        if c < -tolerance or c > tolerance:   # allow tolerance
            success = 0
            print(f"Warning: constraint violation for constraint {j}: ", c)

    return [success, end-start, duty_cycle, energy_efficiency_2]


def calculate_CS_all_variables_scipy(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c, x0=None):
    # Initial guess
    if x0 is None:
        x0 = np.array([
            tau * q_w_max/(nu_charge - nu_min),  # delta_c_charge
            q_w_max /(nu_transfer - nu_min),  # delta_transfer
            q_w_max/(nu_work + nu_min),  # delta_w_work
            0.0,  # delta_w_rest
            0.0,  # delta_c_rest
            tau * q_w_max,  # q_c_charged
            q_w_max  # q_w_charged
        ])

    # Define bounds for variables
    bounds = [
        (0, tau * q_w_max/(nu_charge - nu_min)),   # delta_c_charge ≥ 0
        (0, q_w_max /(nu_transfer - nu_min)),   # delta_transfer ≥ 0
        (0, q_w_max/(nu_work + nu_min)),   # delta_w_work ≥ 0
        (0, q_w_max/nu_min),   # delta_w_rest ≥ 0
        (0, tau * q_w_max/nu_min),   # delta_c_rest ≥ 0
        (0, tau * q_w_max),  # q_c_charged between 0 and tau * q_w_max
        (0, q_w_max)  # q_w_charged between 0 and tau * q_w_max
    ]
    

    def objective(x):
        delta_c_charge, delta_transfer, delta_w_work, delta_w_rest, delta_c_rest, q_c_charged, q_w_charged = x
        # Maximize duty cycle => minimize negative
        duty_cycle = delta_w_work / (delta_w_work + delta_w_rest + delta_transfer)
        return -duty_cycle  # negative because scipy only minimizes

    def constraints(x):
        delta_c_charge, delta_transfer, delta_w_work, delta_w_rest, delta_c_rest, q_c_charged, q_w_charged = x
                
        return [
            # con_1
            q_c_charged - (nu_charge - nu_min) * delta_c_charge,
            # con_2
            delta_transfer - (q_c_charged - 2*delta_commute*(nu_move+nu_min)-delta_c_rest*nu_min) / (zeta/xi*nu_transfer + nu_min),
            # con_3
            (delta_w_work + delta_w_rest + delta_transfer - (delta_c_charge + 2*delta_commute + delta_transfer + delta_c_rest)) / q_w_max,
            # con_4
            q_w_charged - (nu_transfer - nu_min) * delta_transfer,
            # con_5
            q_w_charged - (delta_w_work * (nu_work+nu_min) + delta_w_rest * nu_min)
        ]

    # Build constraint dictionary for scipy
    cons = [{'type': 'eq', 'fun': lambda x, i=i: constraints(x)[i]} for i in range(5)]

    # Solve
    start = time.time()
    res = opt.minimize(objective, x0, method='trust-constr', bounds=bounds, constraints=cons)
    end = time.time()

    # Extract results
    delta_c_charge, delta_transfer, delta_w_work, delta_w_rest, delta_c_rest, q_c_charged, q_w_charged = res.x
    duty_cycle = delta_w_work / (delta_w_work + delta_w_rest + delta_transfer)
    energy_efficiency_2 = (zeta * n_c * delta_w_work * nu_work)/(n_c * (q_c_charged + delta_c_charge*nu_min))
    return [res.success, end-start, duty_cycle, energy_efficiency_2, res.status, res.nit]
  

IPOPT_STATUS_MAP = {
    # Successful / Acceptable
    "Solve_Succeeded": 1,
    "Solved_To_Acceptable_Level": 2,
    "Feasible_Point_Found": 3,

    # Problematic / Numerical Issues
    "Maximum_Iterations_Exceeded": 10,
    "Maximum_WallTime_Exceeded": 11,
    "Maximum_CpuTime_Exceeded": 12,
    "Search_Direction_Becomes_Too_Small": 13,
    "Diverging_Iterates": 14,
    "Restoration_Failed": 15,
    "Error_In_Step_Computation": 16,

    # Rare / Fatal / Modeling or system errors
    "User_Requested_Stop": 20,
    "Invalid_Option": 21,
    "Not_Enough_Degrees_Of_Freedom": 22,
    "Invalid_Problem_Definition": 23,
    "Unrecoverable_Exception": 24,
    "NonIpopt_Exception_Thrown": 25,
    "Insufficient_Memory": 26,
    "Internal_Error": 27,
}

def calculate_CS_all_variables_casadi(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c, x0=None):
    if x0 is None:
        x0 = vertcat(
            tau * q_w_max/(nu_charge - nu_min),  # delta_c_charge
            q_w_max /(nu_transfer - nu_min),  # delta_transfer
            q_w_max/(nu_work + nu_min),  # delta_w_work
            0.0,  # delta_w_rest
            0.0,  # delta_c_rest
            tau * q_w_max,  # q_c_charged
            q_w_max  # q_w_charged
            )

    delta_c_charge = delta_c_charge_SX
    delta_transfer = delta_transfer_SX
    delta_w_work = delta_w_work_SX
    delta_w_rest = delta_w_rest_SX
    delta_c_rest = delta_c_rest_SX
    q_c_charged = q_c_charged_SX
    q_w_charged = q_w_charged_SX
    
    # bounds for opimization variables
    lower_bounds = vertcat(0, 0, 0, 0, 0, 0, 0)
    upper_bounds = vertcat(tau * q_w_max/(nu_charge - nu_min), q_w_max /(nu_transfer - nu_min), q_w_max/(nu_work + nu_min), q_w_max/nu_min, tau * q_w_max/nu_min, tau * q_w_max, q_w_max)
    # delta_c_charge ≥ 0
    # delta_transfer ≥ 0
    # delta_w_work ≥ 0
    # delta_w_rest ≥ 0
    # delta_c_rest ≥ 0
    # q_c_charged between 0 and tau * q_w_max
    # q_w_charged between 0 and tau * q_w_max
    

    # constraints
    constraints = vertcat(q_c_charged - (nu_charge - nu_min) * delta_c_charge,# con_1
            # con_2
            delta_transfer - (q_c_charged - 2*delta_commute*(nu_move+nu_min)-delta_c_rest*nu_min) / (zeta/xi*nu_transfer + nu_min),
            # con_3
            delta_w_work + delta_w_rest + delta_transfer - (delta_c_charge + 2*delta_commute + delta_transfer + delta_c_rest),
            # con_4
            q_w_charged - (nu_transfer - nu_min) * delta_transfer,
            # con_5
            q_w_charged - (delta_w_work * (nu_work+nu_min) + delta_w_rest * nu_min))
    
    # objective
    duty_cycle = delta_w_work / (delta_w_work + delta_w_rest + delta_transfer)
        

    nlp = {'x':vertcat(delta_c_charge, delta_transfer, delta_w_work, delta_w_rest, delta_c_rest, q_c_charged, q_w_charged),
            'f':-duty_cycle, 
            'g':constraints} # f is minimized
    S = nlpsol('S', 'ipopt', nlp, {
        'ipopt.print_level': 0,
        'ipopt.sb': 'yes',
        'ipopt.bound_relax_factor': 0,
        'print_time': False # this avoids printing time, but also prevents the timing statistics from being recorded
    })
    start = time.time()
    r = S(x0=x0, lbx=lower_bounds, ubx=upper_bounds, lbg=0, ubg=0)
    end = time.time()
    x_opt = r['x']
    stats = S.stats()

    delta_c_charge_res = x_opt[0]
    delta_transfer_res = x_opt[1]
    delta_w_work_res = x_opt[2]
    delta_w_rest_res = x_opt[3]
    delta_c_rest_res = x_opt[4]
    q_c_charged_res = x_opt[5]
    q_w_charged_res = x_opt[6]
    duty_cycle_res = delta_w_work_res / (delta_w_work_res + delta_w_rest_res + delta_transfer_res)
    energy_efficiency_2_res = (zeta * n_c * delta_w_work_res * nu_work)/(n_c * (q_c_charged_res + delta_c_charge_res*nu_min))
    return [stats["success"], end-start, duty_cycle_res, energy_efficiency_2_res, IPOPT_STATUS_MAP.get(stats["return_status"], -1), stats["iter_count"]]


nu_min_value = 0.005 # how much a robot consumes when idle

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
tau_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
zeta_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
eta_values = [0.5, 1, 2, 3, 4, 5]
capacity_values = [q_cap_value, q_bat_value]
nu_charge_values = [nu_charge_cap_value, nu_charge_bat_value]
xi_values = [xi_cap, xi_bat]
delta_commute_values = list(np.concatenate([
    np.arange(5, 100, 5),    
    np.arange(100, 1000, 50),    
    np.arange(1000, 10500, 500),  
]))

analytical_results = np.zeros((len(tau_values), len(zeta_values), len(eta_values), len(delta_commute_values), 5)) # last dimension has feasible, success, time, duty cycle, energy efficiency
scipy_results = np.zeros((len(tau_values), len(zeta_values), len(eta_values), len(delta_commute_values), 7)) # last dimension has feasible, success, time, duty cycle, energy efficiency
ipopt_results = np.zeros((len(tau_values), len(zeta_values), len(eta_values), len(delta_commute_values), 7)) # last dimension has feasible, success, time, duty cycle, energy efficiency
cases = np.zeros((len(tau_values), len(zeta_values), len(eta_values), len(delta_commute_values), 5)) # last dimension has feasible, success, time, duty cycle, energy efficiency
x0_guesses = np.zeros((len(tau_values), len(zeta_values), len(eta_values), len(delta_commute_values), 7)) # last dimension has the 7 variables


if OPTIMIZE:
    for k in range(len(tau_values)):
        tau = tau_values[k]
        print("Calculating for tau=", tau)
        for l in range(len(zeta_values)):
            zeta = zeta_values[l]
            print("Calculating for tau=", tau, ", zeta=", zeta)
            for m in range(len(eta_values)):
                nu_work = eta_values[m] * nu_commute_value
                for i in range(len(delta_commute_values)):
                    delta_commute = delta_commute_values[i]
                    #print("Calculating for tau=", tau, ", zeta=", zeta, ", eta=", eta_values[m], ", delta_commute=", delta_commute)
                    
                    # check if the problem is even feasible with given parameters
                    if not calculate_CS_feasible(q_w_max=q_cap_value, delta_commute=delta_commute,  nu_work=nu_work, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=xi_cap, tau=tau, zeta=zeta, n_c=n_c_value):
                        #print("  Not feasible, skipping...")
                        analytical_results[k, l, m, i, 0] = 0
                        scipy_results[k, l, m, i, 0] = 0
                        ipopt_results[k, l, m, i, 0] = 0
                        continue
                    else:
                        #print("  Feasible.")
                        analytical_results[k, l, m, i, 0] = 1
                        scipy_results[k, l, m, i, 0] = 1
                        ipopt_results[k, l, m, i, 0] = 1

                    # Analytical
                    analytical_result = calculate_CS_all_variables_timed(q_w_max=q_cap_value, delta_commute=delta_commute, nu_work=nu_work, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=xi_cap, tau=tau, zeta=zeta, n_c=n_c_value)
                    
                    analytical_results[k, l, m, i, 1] = analytical_result[0] 
                    analytical_results[k, l, m, i, 2] = analytical_result[1]
                    analytical_results[k, l, m, i, 3] = analytical_result[2]
                    analytical_results[k, l, m, i, 4] = analytical_result[3]

                    cases[k, l, m, i, :] = cf.calculate_CS_case_differentiation(q_w_max=q_cap_value, delta_commute=delta_commute, nu_work=nu_work, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=xi_cap, tau=tau, zeta=zeta, n_c=n_c_value)

                    # create an initial guess: this is a rather good initial guess as we know delta_w_work should be maximized.
                    x0_guesses[k, l, m, i, :] = np.array([
                        tau * q_cap_value/(nu_charge_cap_value - nu_min_value),  # delta_c_charge
                        q_cap_value /(nu_charge_cap_value - nu_min_value),  # delta_transfer
                        q_cap_value/(nu_work + nu_min_value),  # delta_w_work
                        0,  # delta_w_rest
                        0,  # delta_c_rest
                        tau * q_cap_value,  # q_c_charged
                        q_cap_value  # q_w_charged
                    ])
                    # Uncomment this if delta_w_work should start at 0.
                    # x0_guesses[k, l, m, i, :] = np.array([
                    #     tau * q_cap_value/(nu_charge_cap_value - nu_min_value),  # delta_c_charge
                    #     q_cap_value /(nu_charge_cap_value - nu_min_value),  # delta_transfer
                    #     0, #q_cap_value/(nu_work + nu_min_value),  # delta_w_work
                    #     0,  # delta_w_rest
                    #     0,  # delta_c_rest
                    #     tau * q_cap_value,  # q_c_charged
                    #     q_cap_value  # q_w_charged
                    # ])
                    
                    # Scipy
                    scipy_result = calculate_CS_all_variables_scipy(q_w_max=q_cap_value, delta_commute=delta_commute, nu_work=nu_work, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=xi_cap, tau=tau, zeta=zeta, n_c=n_c_value, x0=x0_guesses[k, l, m, i, :])
                    scipy_results[k, l, m, i, 1] = scipy_result[0]
                    scipy_results[k, l, m, i, 2] = scipy_result[1]
                    scipy_results[k, l, m, i, 3] = scipy_result[2]
                    scipy_results[k, l, m, i, 4] = scipy_result[3]
                    scipy_results[k, l, m, i, 5] = scipy_result[4]
                    scipy_results[k, l, m, i, 6] = scipy_result[5]

                    # IPOPT via Casadi
                    ipopt_result = calculate_CS_all_variables_casadi(q_w_max=q_cap_value, delta_commute=delta_commute, nu_work=nu_work, nu_move=nu_commute_value, nu_min=nu_min_value, nu_charge=nu_charge_cap_value, nu_transfer=nu_charge_cap_value, xi=xi_cap, tau=tau, zeta=zeta, n_c=n_c_value, x0=x0_guesses[k, l, m, i, :])
                    ipopt_results[k, l, m, i, 1] = ipopt_result[0]
                    ipopt_results[k, l, m, i, 2] = ipopt_result[1]
                    ipopt_results[k, l, m, i, 3] = ipopt_result[2]
                    ipopt_results[k, l, m, i, 4] = ipopt_result[3]
                    ipopt_results[k, l, m, i, 5] = ipopt_result[4]
                    ipopt_results[k, l, m, i, 6] = ipopt_result[5]


    timestamp = datetime.datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
    np.savez_compressed(f"optimizer_results/analytical_results_{timestamp}.npz", analytical_results=analytical_results)
    np.savez_compressed(f"optimizer_results/scipy_results_{timestamp}.npz", scipy_results=scipy_results)
    np.savez_compressed(f"optimizer_results/ipopt_results_{timestamp}.npz", ipopt_results=ipopt_results)
    np.savez_compressed(f"optimizer_results/cases_{timestamp}.npz", cases=cases)
    np.savez_compressed(f"optimizer_results/x0_guesses_{timestamp}.npz", x0_guesses=x0_guesses)

    print("Results saved with timestamp: ", timestamp)

else:
    timestamp = "2026-04-23_04-54-24" # delta_work set to 0 as initial guess
    #timestamp = "2026-04-23_00-02-36" # delta_work set to max as initial guess

analytical_results = np.load(f"optimizer_results/analytical_results_{timestamp}.npz")["analytical_results"]
scipy_results      = np.load(f"optimizer_results/scipy_results_{timestamp}.npz")["scipy_results"]
ipopt_results      = np.load(f"optimizer_results/ipopt_results_{timestamp}.npz")["ipopt_results"]
cases              = np.load(f"optimizer_results/cases_{timestamp}.npz")["cases"]

# ----------------------------
# Indices for clarity
FEASIBLE = 0
SUCCESS  = 1
TIME     = 2
DUTY     = 3
EFF      = 4
STATUS   = 5
ITER     = 6

# ---- FEASIBILITY MASK ----
feasible_mask = analytical_results[..., FEASIBLE] == 1

# ---- SUCCESS MASKS ----
analytical_success_mask = analytical_results[..., SUCCESS] == 1 # should be all true, but we check anyway
if not np.all(analytical_success_mask[feasible_mask]):
    print("Warning: Not all feasible cases are successful in the analytical results.")
scipy_success_mask = scipy_results[..., SUCCESS] == 1
ipopt_success_mask = ipopt_results[..., SUCCESS] == 1


# Combined success mask (SciPy OR IPOPT)
any_success_mask = feasible_mask & (scipy_success_mask | ipopt_success_mask)

# ----------------------------
# Count failures vs successes
# ----------------------------

total_feasible = feasible_mask.sum()

scipy_success_count = (feasible_mask & scipy_success_mask).sum()
ipopt_success_count = (feasible_mask & ipopt_success_mask).sum()

scipy_fail_count = total_feasible - scipy_success_count
ipopt_fail_count = total_feasible - ipopt_success_count

# Set duty cycle and energy efficiency to 0 for cases where optimization failed but the configuration was feasible
scipy_results[feasible_mask & ~scipy_success_mask, DUTY] = 0
scipy_results[feasible_mask & ~scipy_success_mask, EFF] = 0
ipopt_results[feasible_mask & ~ipopt_success_mask, DUTY] = 0
ipopt_results[feasible_mask & ~ipopt_success_mask, EFF] = 0

# ----------------------------
# Percentage deviations vs analytical
# ----------------------------

# Extract analytical reference data
analytical_time = analytical_results[..., TIME]
analytical_duty = analytical_results[..., DUTY]
analytical_eff  = analytical_results[..., EFF]

# Set non feasible cases to 1 to avoid division by zero 
analytical_time[~feasible_mask] = 1
analytical_duty[~feasible_mask] = 1
analytical_eff[~feasible_mask] = 1

# SciPy
scipy_time_factor = scipy_results[..., TIME] / analytical_time 
scipy_rel_duty = 100 * scipy_results[..., DUTY] / analytical_duty
scipy_rel_eff  = 100 * scipy_results[..., EFF] / analytical_eff


# IPOPT
ipopt_time_factor = ipopt_results[..., TIME] / analytical_time
ipopt_rel_duty = 100 * ipopt_results[..., DUTY] / analytical_duty
ipopt_rel_eff  = 100 * ipopt_results[..., EFF] / analytical_eff

# Only consider successful cases
scipy_time_factor = scipy_time_factor[feasible_mask & scipy_success_mask]
scipy_rel_duty = scipy_rel_duty[feasible_mask & scipy_success_mask]
scipy_rel_eff  = scipy_rel_eff[feasible_mask & scipy_success_mask]

ipopt_time_factor = ipopt_time_factor[feasible_mask & ipopt_success_mask]
ipopt_rel_duty = ipopt_rel_duty[feasible_mask & ipopt_success_mask]
ipopt_rel_eff  = ipopt_rel_eff[feasible_mask & ipopt_success_mask]

# ----------------------------
# Print summary statistics
# ----------------------------

def summarize(name, rel_time, rel_duty, rel_eff):
    print(f"========== {name} Deviation of analytical solution==========")
    print(f"Time factor optimizer/analytical: mean = {rel_time.mean():.3f}, median = {np.median(rel_time):.3f}%")
    print(f"Duty cycle deviation: mean = {rel_duty.mean():.3f}%, median = {np.median(rel_duty):.3f}%")
    print(f"Energy efficiency deviation: mean = {rel_eff.mean():.3f}%, median = {np.median(rel_eff):.3f}%")
    print()

summarize("SciPy", scipy_time_factor, scipy_rel_duty, scipy_rel_eff)
summarize("IPOPT", ipopt_time_factor, ipopt_rel_duty, ipopt_rel_eff)


print("========== Optimization Success/Failure ==========")
print(f"Total feasible cases: {total_feasible}")
print(f"SciPy successes: {scipy_success_count}, failures: {scipy_fail_count}")
print(f"IPOPT successes: {ipopt_success_count}, failures: {ipopt_fail_count}")
print()

print("========== Absolute Duty Cycle Statistics ==========")
print("Analytical Duty Cycle Mean: ", analytical_results[..., DUTY][feasible_mask].mean(), "Median: ", np.median(analytical_results[..., DUTY][feasible_mask]), "Varianz: ", analytical_results[..., DUTY][feasible_mask].std())
print("Scipy Duty Cycle Mean: ", scipy_results[..., DUTY][feasible_mask].mean(), "Median: ", np.median(scipy_results[..., DUTY][feasible_mask]), "Varianz: ", scipy_results[..., DUTY][feasible_mask].std())
print("IPOPT Duty Cycle Mean: ", ipopt_results[..., DUTY][feasible_mask].mean(), "Median: ", np.median(ipopt_results[..., DUTY][feasible_mask]), "Varianz: ", ipopt_results[..., DUTY][feasible_mask].std())
print()

print("========== Absolute Energy Efficiency Statistics ==========")
print("Analytical Energy Efficiency Mean: ", analytical_results[..., EFF][feasible_mask].mean(), "Median: ", np.median(analytical_results[..., EFF][feasible_mask]), "Varianz: ", analytical_results[..., EFF][feasible_mask].std())
print("Scipy Energy Efficiency Mean: ", scipy_results[..., EFF][feasible_mask].mean(), "Median: ", np.median(scipy_results[..., EFF][feasible_mask]), "Varianz: ", scipy_results[..., EFF][feasible_mask].std())
print("IPOPT Energy Efficiency Mean: ", ipopt_results[..., EFF][feasible_mask].mean(), "Median: ", np.median(ipopt_results[..., EFF][feasible_mask]), "Varianz: ", ipopt_results[..., EFF][feasible_mask].std())
print()

print("========== Absolute Time Statistics (seconds) ==========")
print("Analytical Time Mean: ", analytical_results[..., TIME][feasible_mask].mean(), "Median: ", np.median(analytical_results[..., TIME][feasible_mask]), "Varianz: ", analytical_results[..., TIME][feasible_mask].std())
print("Scipy Time Mean: ", scipy_results[..., TIME][feasible_mask & scipy_success_mask].mean(), "Median: ", np.median(scipy_results[..., TIME][feasible_mask & scipy_success_mask]), "Varianz: ", scipy_results[..., TIME][feasible_mask & scipy_success_mask].std())
print("IPOPT Time Mean: ", ipopt_results[..., TIME][feasible_mask & ipopt_success_mask].mean(), "Median: ", np.median(ipopt_results[..., TIME][feasible_mask & ipopt_success_mask]), "Varianz: ", ipopt_results[..., TIME][feasible_mask & ipopt_success_mask].std())
print()



