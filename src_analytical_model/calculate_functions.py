import numpy as np
import matplotlib.pyplot as plt
from read_protobuf_experiment.sim_data import SimData

def calculate_IS(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):

    # IS for comparison, take the same values for the workers regarding moving and charging as the charger
    delta_work = np.maximum(0, (q_w_max - 2 * (nu_move + nu_min) * delta_commute)/(nu_work + nu_min))
    n_w = zeta * n_c
    n_w_IS = n_w + tau * n_c # number of workers in IS, match the summed capacity that CS posesses

    # IS for comparison, take the same values for the workers regarding moving and charging as the charger
    delta_w_charge = q_w_max/(nu_charge-nu_min)

    cycle_duration = delta_work + delta_w_charge + 2 * delta_commute

    duty_cycle = n_w_IS * delta_work/(n_w_IS*(delta_work + 2 * delta_commute + delta_w_charge))

    energy_used_for_work = nu_work*delta_work

    energy_withdrawn = q_w_max + nu_min*delta_w_charge

    energy_efficiency =  n_w_IS * energy_used_for_work/(n_w_IS * energy_withdrawn) # Energy Used For Work/Total Energy

    # amount of work
    work = n_w_IS * duty_cycle

    return (energy_efficiency, duty_cycle, work, cycle_duration)


def calculate_CS(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):
    results = calculate_CS_all_variables(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c)
    energy_efficiency = results["energy_efficiency"]
    duty_cycle = results["duty_cycle"]
    work = results["work"]
    cycle_duration = results["cycle_duration"]

    return (energy_efficiency, duty_cycle, work, cycle_duration)



CASE1 = 1 # charger fully charged, workers rest
CASE2 = 2 # charger fully charged, charger rests
CASE3 = 3 # workers fully charged, workers rest
CASE4 = 4 # workers fully charged, charger rests

def calculate_CS_case_differentiation(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):

    results = calculate_CS_all_variables(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c)
    #print("Results: " + str(results))
    energy_efficiency = results["energy_efficiency"]
    duty_cycle = results["duty_cycle"]
    work = results["work"]
    cycle_duration = results["cycle_duration"]
    q_c_charged = round(results["q_c_charged"],5) # round so we dont have problems with numerical inaccuracies like e-12>0
    q_w_charged = round(results["q_w_charged"],5) # round so we dont have problems with numerical inaccuracies like e-12>0
    delta_c_charge = results["delta_c_charge"]
    delta_transfer = results["delta_transfer"]
    delta_w_work = round(results["delta_w_work"],5) # round so we dont have problems with numerical inaccuracies like e-12>0
    delta_w_rest = round(results["delta_w_rest"],5) # round so we dont have problems with numerical inaccuracies like e-12>0
    delta_c_rest = round(results["delta_c_rest"],5) # round so we dont have problems with numerical inaccuracies like e-12>0
    q_c_max = tau * q_w_max # chargers capacity is tau times the workers capacity
    replenishment_case = 0
    
    if q_c_charged == q_c_max and q_w_charged <= q_w_max and delta_w_rest >= 0 and delta_c_rest == 0:
        replenishment_case = CASE1
    elif q_c_charged == q_c_max and q_w_charged <= q_w_max and delta_w_rest == 0 and delta_c_rest > 0:
        replenishment_case = CASE2
    elif q_c_charged < q_c_max and q_w_charged == q_w_max and delta_w_rest >= 0 and delta_c_rest == 0:
        replenishment_case = CASE3
    elif q_c_charged < q_c_max and q_w_charged == q_w_max and delta_w_rest == 0 and delta_c_rest > 0:
        replenishment_case = CASE4
    else:
        replenishment_case = -1 # infeasible
    
    return (replenishment_case, energy_efficiency, duty_cycle, work, cycle_duration)


def calculate_CS_all_variables(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):
    q_c_max = tau * q_w_max # chargers capacity is tau times the workers capacity

    # extract delta_c_rest for case 2
    delta_c_rest_case_2 = np.maximum(0, q_w_max/(nu_work+nu_min)*(1-nu_min/nu_charge)-2*delta_commute*(1+nu_move/nu_charge) - q_w_max/nu_charge * (zeta/xi * nu_transfer + nu_min) / (nu_transfer - nu_min))
    # either maximum capacity, or case 2 where it can supply all workers with full capacity even when having less than the maximum capacity, also limit to 0 in the (very unlikely) case that nu_min is way bigger than nu_transfer 
    q_c_charged = np.maximum(0, np.minimum(q_c_max, 2 * (nu_move + nu_min) * delta_commute + q_w_max * (zeta/xi * nu_transfer + nu_min) / (nu_transfer - nu_min) + nu_min * delta_c_rest_case_2))
    # Directly following
    delta_c_charge = q_c_charged/(nu_charge-nu_min)

    delta_c_charge_different = 2 * delta_commute * (nu_move/nu_charge) + q_w_max/nu_charge *((zeta * nu_transfer + nu_min)/(xi * nu_transfer - nu_min) + nu_min/(nu_work + nu_min))
 
    q_c_charged_different = delta_c_charge_different * (nu_charge - nu_min)

    # extract delta_c_rest for case 1
    delta_c_rest_case_1 = np.maximum(0, (q_c_charged - 2 * (nu_move + nu_min) * delta_commute)/nu_min - (q_c_charged/(nu_charge - nu_min) * nu_charge - 2 * delta_commute * nu_move) / (nu_min + nu_min**2/(nu_work + nu_min) * (nu_transfer - nu_min)/(zeta/xi * nu_transfer + nu_min)))
    
    delta_transfer = np.maximum(0, np.minimum(q_w_max /(nu_transfer - nu_min), (q_c_charged - 2 * (nu_move + nu_min) * delta_commute)/(zeta/xi * nu_transfer + nu_min) - nu_min/(zeta/xi * nu_transfer + nu_min) * delta_c_rest_case_1)) # Either the time the charger needs to give the worker its remaining capacity, or the time it needs to charge all workers to the maximum capacity (considering that there is a energy loss) 
    
    q_w_charged = (nu_transfer - nu_min) * delta_transfer # the max boundary for q_w_charged is already encoded in delta_transfer, so we can just multiply it with the rate at which the charger can transfer energy to the worker

    n_w = zeta * n_c 

    delta_c_rest = np.maximum(0, (q_c_charged - 2*delta_commute * (nu_move + nu_min))/nu_min - delta_transfer * (zeta/xi * nu_transfer + nu_min)/nu_min)
    
    # The maximum of delta_work should already be encapsulated in q_w_charged (delta_transfer, respectively)
    delta_w_work = np.maximum(0, (q_w_charged - (delta_c_charge + 2*delta_commute + delta_c_rest) * nu_min) / nu_work) # either the remaining capacity divided by the wpork rate when substracting the time the charger needs to get new energy or the time the worker needs to use all its charge for working
    
    # Different cases should be already encapsulated in delta_transfer and q_c_charged, so we can calculate it more generally as
    delta_w_rest = np.maximum(0, (q_w_charged - delta_w_work*(nu_work+nu_min))/nu_min)
     
    cycle_duration = delta_c_charge + 2 * delta_commute + delta_transfer + delta_c_rest
    
    energy_efficiency = (n_w * delta_w_work * nu_work)/(n_c * (q_c_charged + delta_c_charge*nu_min)) 

    duty_cycle = delta_w_work/cycle_duration # divide by the cycle of a charger in the case that the workers cycle is not adding up due to a drained battery
  
    work = n_w * duty_cycle

    
    return {
        "energy_efficiency": energy_efficiency,
        "duty_cycle": duty_cycle,
        "work": work,
        "cycle_duration": cycle_duration,
        "q_c_charged": q_c_charged,
        "delta_c_charge": delta_c_charge,
        "delta_transfer": delta_transfer,
        "q_w_charged": q_w_charged,
        "delta_w_work": delta_w_work,
        "delta_w_rest": delta_w_rest,
        "delta_c_rest": delta_c_rest,
    }

def calculate_and_plot_scenario_with_log_file(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c, simulation_result_log_file):
    results = calculate_CS_all_variables(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c)
    n_w = zeta * n_c

    sim = SimData(simulation_result_log_file)    
    #overall_duration = int(sim.totalTime / 10) 
    overall_duration = int(sim.totalTime)
    simulation_result = sim.totalPoints
    simulation_result_per_time = [0]
    print(f"Overall duration from log file: {overall_duration}, total work from log file: {simulation_result}")
    for time in range(1, overall_duration + 1):
        simulation_result_per_time.append(sim[time*10]['log'].points)

    work_percentage = results["work"]
    cycle_duration = results["cycle_duration"]
    n_cycles = overall_duration / cycle_duration
    #total_work = work_percentage * overall_duration


    def work_over_time(time: float) -> float:
        n_cycles = int(time / cycle_duration)

        work_results = n_cycles * cycle_duration * work_percentage
        if time - n_cycles * cycle_duration <= results["delta_w_work"]:
            work_results += n_w *(time - n_cycles * cycle_duration) # < delta_work means working for this part of the cycle
        else:
            work_results += n_w *results["delta_w_work"] # worked full work part of the cycle
        return work_results
        
    
    total_work = work_over_time(overall_duration)
    print(f"Model prediction: Total work done in {overall_duration} time units: {total_work}")
    print(f"Difference to simulation result: {total_work - simulation_result}, meaning that the simulation reaches {simulation_result/total_work *100}% of work of the analytical model, yielding an error of {(total_work - simulation_result)/total_work * 100}%")
    
    time_points = np.linspace(0, overall_duration, overall_duration + 1)
    plt.plot(time_points, [work_over_time(time) for time in time_points], label='Analytical model')
    plt.plot(time_points, simulation_result_per_time, label='Simulation')
    plt.xlabel('Time in seconds')
    plt.ylabel('Cumulative Work')
    plt.grid(True)
    plt.legend()


def calculate_and_plot_scenario_with_multiple_log_files(q_w_max_list, delta_commute_list, nu_work_list, nu_move_list, nu_min_list, nu_charge_list, nu_transfer_list, xi_list, tau_list, zeta_list, n_c_list, simulation_result_log_files, labels=None):
    assert len(q_w_max_list) == len(simulation_result_log_files), "All parameter lists must have the same length"

    colors = ["#004488", "#ddaa33", "#56B4E9", "#D55E00",
              "#CC79A7", "#009E73", "#F0E442", "#000000"]
    plt.figure(figsize=(12, 3))  # narrower plot

    for i in range(len(q_w_max_list)):
        label = labels[i] if labels and i < len(labels) else f"Run {i+1}"
        color = colors[i % len(colors)]

        # Unpack parameters for current run
        q_w_max = q_w_max_list[i]
        delta_commute = delta_commute_list[i]
        nu_work = nu_work_list[i]
        nu_move = nu_move_list[i]
        nu_min = nu_min_list[i]
        nu_charge = nu_charge_list[i]
        nu_transfer = nu_transfer_list[i]
        xi = xi_list[i]
        tau = tau_list[i]
        zeta = zeta_list[i]
        n_c = n_c_list[i]
        simulation_result_log_file = simulation_result_log_files[i]

        # Calculate analytical model
        results = calculate_CS_all_variables(
            q_w_max, delta_commute, nu_work, nu_move,
            nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c
        )
        n_w = zeta * n_c

        # Load simulation data
        sim = SimData(simulation_result_log_file)    
        #overall_duration = int(sim.totalTime / 10)
        overall_duration = int(sim.totalTime)
        simulation_result = sim.totalPoints
        simulation_result_per_time = [0]
        for time in range(1, overall_duration + 1):
            simulation_result_per_time.append(sim[time*10]['log'].points)

        # Extract analytical results
        work_percentage = results["work"]
        cycle_duration = results["cycle_duration"]

        def work_over_time(time: float) -> float:
            n_cycles = int(time / cycle_duration)
            work_results = n_cycles * cycle_duration * work_percentage
            if time - n_cycles * cycle_duration <= results["delta_w_work"]:
                work_results += n_w * (time - n_cycles * cycle_duration)
            else:
                work_results += n_w * results["delta_w_work"]
            return work_results

        time_points = np.linspace(0, overall_duration, overall_duration + 1)

        # Plot analytical and simulation data
        label_prefix = f"Run {i+1}"
        plt.plot(time_points, [work_over_time(t) for t in time_points], color=colors[i],
                 label=f"{label} – Analytical")
        plt.plot(time_points, simulation_result_per_time, color=colors[i],
                 linestyle='--', label=f"{label} – Simulation")

    plt.xlabel('Time (s)',fontsize=16)
    plt.ylabel('Cumulative Work',fontsize=16)
    plt.xticks(fontsize=14)
    plt.yticks(fontsize=14)
    plt.grid(True, linestyle=':', linewidth=0.8)
    plt.legend(fontsize=14)
    #plt.xscale('log')
    #plt.yscale('log')
    #plt.xlim(left=0)  # ensures plot starts exactly at 0 and ends at the final duration
    plt.margins(x=0.02)  # adds a bit of margin around the plot  
    plt.tight_layout()
    plt.savefig("cumulative_work_comparison_model_simulation.png", dpi=300)
    plt.show()


def calculate_and_plot_scenario_with_multiple_log_files_test(
    q_w_max_list, delta_commute_list, nu_work_list, nu_move_list,
    nu_min_list, nu_charge_list, nu_transfer_list, xi_list, tau_list,
    zeta_list, n_c_list, simulation_result_log_files, labels=None):

    assert len(q_w_max_list) == len(simulation_result_log_files), "All parameter lists must have the same length"

    colors = ["#004488", "#ddaa33", "#56B4E9", "#D55E00",
              "#CC79A7", "#009E73", "#F0E442", "#000000"]

    # Two equally sized subplots
    fig, ( ax_full, ax_zoom) = plt.subplots(1, 2, figsize=(12, 3), sharey=False)
    
    # Define zoom window
    zoom_start = 10380
    zoom_end = 10700
    #zoom_start = 600000
    #zoom_end = 604800
    zoom_range = zoom_end - zoom_start

    zoom_ymin, zoom_ymax = float('inf'), -float('inf')

    for i in range(len(q_w_max_list)):
        label = labels[i] if labels and i < len(labels) else f"Run {i+1}"
        color = colors[i % len(colors)]

        # Unpack parameters
        q_w_max = q_w_max_list[i]
        delta_commute = delta_commute_list[i]
        nu_work = nu_work_list[i]
        nu_move = nu_move_list[i]
        nu_min = nu_min_list[i]
        nu_charge = nu_charge_list[i]
        nu_transfer = nu_transfer_list[i]
        xi = xi_list[i]
        tau = tau_list[i]
        zeta = zeta_list[i]
        n_c = n_c_list[i]
        simulation_result_log_file = simulation_result_log_files[i]

        # Analytical model
        results = calculate_CS_all_variables(
            q_w_max, delta_commute, nu_work, nu_move,
            nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c
        )
        n_w = zeta * n_c
        work_percentage = results["work"]
        cycle_duration = results["cycle_duration"]

        def work_over_time(time: float) -> float:
            n_cycles = int(time / cycle_duration)
            work_results = n_cycles * cycle_duration * work_percentage
            if time - n_cycles * cycle_duration <= results["delta_w_work"]:
                work_results += n_w * (time - n_cycles * cycle_duration)
            else:
                work_results += n_w * results["delta_w_work"]
            return work_results

        # Simulation data
        sim = SimData(simulation_result_log_file)
        overall_duration = int(sim.totalTime)
        time_points = np.arange(0, overall_duration + 1)

        # Start manually with zero since sim[0] doesn’t exist
        sim_points = [0]
        for time in range(1, overall_duration + 1):
            sim_points.append(sim[time * 10]['log'].points)

        analytical = [work_over_time(t) for t in time_points]

        # Plot full data
        ax_full.plot(time_points, analytical, color=color, label=f"{label} – Analytical")
        ax_full.plot(time_points, sim_points, color=color, linestyle='--', label=f"{label} – Simulation")

        # Zoomed subset
        mask = (time_points >= zoom_start) & (time_points <= zoom_end)
        zoom_t = time_points[mask]
        zoom_a = np.array(analytical)[mask]
        zoom_s = np.array(sim_points)[mask]

        ax_zoom.plot(zoom_t, zoom_a, color=color)
        ax_zoom.plot(zoom_t, zoom_s, color=color, linestyle='--')

        # Track y-range for zoom box
        zoom_ymin = min(zoom_ymin, np.min(zoom_a), np.min(zoom_s))
        zoom_ymax = max(zoom_ymax, np.max(zoom_a), np.max(zoom_s))

    # Format zoom plot
    ypad = 0.05 * (zoom_ymax - zoom_ymin)
    ax_zoom.set_xlim(zoom_start, zoom_end)
    ax_zoom.set_ylim(zoom_ymin - ypad, zoom_ymax + ypad)
    ax_zoom.tick_params(axis='both', which='major', labelsize=12)
    ax_full.tick_params(axis='both', which='major', labelsize=12)
    #ax_zoom.set_title(f"Zoomed region ({zoom_start}–{zoom_end} s)", fontsize=12)
    ax_zoom.set_xlabel("Time (s)", fontsize=14)
    ax_full.set_ylabel("Cumulative Work", fontsize=14)
    
    
    ax_zoom.grid(True, linestyle=':', linewidth=0.7)

    # Format full plot
    ax_full.set_xlabel("Time (s)", fontsize=14)
    ax_full.grid(True, linestyle=':', linewidth=0.7)
    ax_full.margins(x=0.02)
    #ax_full.set_title("Cumulative Work Over Time", fontsize=14)

    # Legend above both
    fig.legend(fontsize=14, loc='upper center', ncol=len(labels)*2, bbox_to_anchor=(0.5, 1.04), frameon=False)

    # Rectangle marking zoom region
    # rect = Rectangle((zoom_start, zoom_ymin - ypad),
    #                  width=zoom_range,
    #                  height=(zoom_ymax - zoom_ymin + 2 * ypad),
    #                  linewidth=1, edgecolor='gray', facecolor='none', linestyle='--')
    # ax_full.add_patch(rect)

    # Connect zoom box to zoom subplot
    # con_top = ConnectionPatch(
    #     xyA=(0, 1), coordsA="axes fraction", axesA=ax_zoom,
    #     xyB=(zoom_end, zoom_ymax + ypad), coordsB="data", axesB=ax_full,
    #     linestyle="--", color="gray", linewidth=0.9)
    # con_bot = ConnectionPatch(
    #     xyA=(0, 0), coordsA="axes fraction", axesA=ax_zoom,
    #     xyB=(zoom_end, zoom_ymin - ypad), coordsB="data", axesB=ax_full,
    #     linestyle="--", color="gray", linewidth=0.9)
    # fig.add_artist(con_top)
    # fig.add_artist(con_bot)

    plt.tight_layout(rect=[0, 0, 1, 0.96])
    #plt.savefig("cumulative_work_comparison_model_simulation.png", dpi=300, bbox_inches='tight')
    plt.show()


def plot_CS_polygon(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c):
    optimal_solution = calculate_CS_all_variables(q_w_max, delta_commute, nu_work, nu_move, nu_min, nu_charge, nu_transfer, xi, tau, zeta, n_c)


    delta_work = np.linspace(0, q_w_max/(nu_work + nu_min), 10000)
    plot_y_axis_limit = 7000

    # all constraints are seen from delta_w_rest, so constraint=delta_w_rest
    # constraint 1: delta_w_rest >= 0
    constraint_1 = np.zeros_like(delta_work)
    # constraint 2: delta_c_rest >= 0
    constraint_2 = (2 * delta_commute * (1 + nu_move/nu_charge) - delta_work * (1 - (nu_min + (nu_work + nu_min)*(zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/nu_charge)) / (1 - (nu_min + nu_min * (zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/nu_charge)
    # constraint 3: delta_w_work >= 0
    constraint_3 = [0, plot_y_axis_limit*1.5]  # vertical line at delta_w_work = 0

    # constraint 4: q_w_charged <= q_w_max
    constraint_4 = (q_w_max - delta_work * (nu_work + nu_min))/nu_min
    # constraint 3: q_c_charged <= tau * q_w_max
    constraint_5 = (tau * q_w_max - (2 * delta_commute * nu_move)/(nu_charge/(nu_charge-nu_min)) - delta_work * (nu_min + (nu_work + nu_min)*(zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/(nu_charge/(nu_charge-nu_min))) / ((nu_min + nu_min * (zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/(nu_charge/(nu_charge-nu_min)))
    
    # constraint 6: q_w_charged >= 0 (also over delta_transfer >=0)
    #constraint_6 = (0 - delta_work * (nu_work + nu_min))/nu_min
    # constraint 7: q_c_charged >= 0 (also over delta_c_charge >=0)
    #constraint_7 = (0 - (2 * delta_commute * nu_move)/(nu_charge/(nu_charge-nu_min)) - delta_work * (nu_min + (nu_work + nu_min)*(zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/(nu_charge/(nu_charge-nu_min))) / ((nu_min + nu_min * (zeta/xi * nu_transfer + nu_min)/(nu_transfer - nu_min))/(nu_charge/(nu_charge-nu_min)))
    
    # Pointwise envelope
    lb = np.max(np.vstack([constraint_1, constraint_2,]), axis=0)   # lower boundary curve
    ub = np.min(np.vstack([constraint_4, constraint_5]), axis=0)   # upper boundary curve

    # Mask where region is feasible (lb <= ub)
    feasible = lb <= ub

    if plt.gcf().axes == []:
        plt.figure(figsize=(8, 6))
    plt.plot(delta_work, constraint_1, label=r'$\Delta^\text{w}_{\text{rest}} \geq 0$',linewidth=4.5,zorder=1, color="blue")
    plt.plot(delta_work, constraint_2, label=r'$\Delta^\text{c}_{\text{rest}} \geq 0$',linewidth=4.5, color="orange")
    plt.plot(np.zeros(2), constraint_3, label=r'$\Delta^\text{w}_{\text{work}} \geq 0$',linewidth=4.5, color="green")
    plt.plot(delta_work, constraint_4, label=r'$q^\text{w}_{\text{charged}} \leq q^\text{w}_{\text{max}}$',linewidth=4.5, color="purple")
    plt.plot(delta_work, constraint_5, label=r'$q^\text{c}_{\text{charged}} \leq q^\text{c}_{\text{max}}$',linewidth=4.5, color="red")
    #plt.plot(delta_work, constraint_6, label=r'$q^\text{w}_{\text{charged}} \geq 0$') # already covered by delta_w_rest >=0
    #plt.plot(delta_work, constraint_7, label=r'$q^\text{c}_{\text{charged}} \geq 0$') # already covered by delta_w_rest >=0
    #plt.plot(optimal_solution["delta_w_work"], optimal_solution["delta_w_rest"], 'bo', label='Optimal solution',zorder=5)

    # Shade feasible region
    plt.fill_between(
    delta_work,
    lb,
    ub,
    where=feasible,
    alpha=0.25,
    color='gray',
    label='Feasible region'
    )

    ax = plt.gca()
    ax.spines['bottom'].set_zorder(0)
    plt.legend(
    loc='upper center',
    bbox_to_anchor=(0.5, 1.18),
    ncol=3,
    frameon=False,
    fontsize=14
    )
    plt.xlabel(r'$\Delta^\text{w}_{\text{work}}$', fontsize=18, labelpad=1)
    plt.ylabel(r'$\Delta^\text{w}_{\text{rest}}$', fontsize=18, labelpad=1)
    plt.ylim(0, plot_y_axis_limit)
    plt.xticks([0])
    plt.yticks([0])
    #plt.title('Feasible Region for CS')
    #plt.show()


def plot_CS_polygon_grid(param_list):
    fig, axes = plt.subplots(1, 4, figsize=(12, 4), sharey=True)

    for i, (ax, params) in enumerate(zip(axes, param_list)):
        plt.sca(ax)  # set current axis → your function will plot here
        plot_CS_polygon(*params)

        # Remove duplicate y-labels
        if i != 0:
            ax.set_ylabel("")

    # Get legend from first axis
    handles, labels = axes[0].get_legend_handles_labels()

    # Remove legends from individual plots
    for ax in axes:
        ax.get_legend().remove()

    # Shared legend
    fig.legend(
        handles,
        labels,
        loc='upper center',
        bbox_to_anchor=(0.5, 1.0),
        ncol=6,
        frameon=False,
        fontsize=14
    )

    plt.tight_layout()
    plt.subplots_adjust(top=0.85)
    #plt.savefig("all_cases_figure.png", dpi=300, bbox_inches='tight')
    plt.show()