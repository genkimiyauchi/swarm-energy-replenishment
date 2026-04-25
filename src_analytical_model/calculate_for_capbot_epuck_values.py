#from sympy import *
#from IPython.display import Markdown as md
import numpy as np
import matplotlib.pyplot as plt
import calculate_functions as cf
from matplotlib.colors import Normalize
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime

nu_min_value = 0.005 # how much a robot consumes when idle

## Values for capacitor: take duty cycle of capbot which is at 99%
q_cap_value = 3060.0 # with 1/s when moving, can move for 51 minutes at full speed
nu_commute_value = 1.0 # rate of consuming energy when travelling per time unit
nu_charge_cap_value= q_cap_value / 16.0 # can be fully charged in 16 seconds
xi_cap = 0.5

## Values for battery depending on the capacitor values
q_bat_value = 10800 # E-puck 2: 3h autonomy
nu_charge_bat_value = q_bat_value / (2.5*60*60) # E-puck 2 charges in 2.5 hours
xi_bat = 0.99

# Define parameter ranges
n_c_value = 1
tau_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
zeta_values = [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]
eta_values = [0.5, 1, 2]
delta_commute_values = [1200, 1300, 1400]
capacity_values = [q_cap_value, q_bat_value]
nu_charge_values = [nu_charge_cap_value, nu_charge_bat_value]
xi_values = [xi_cap, xi_bat]

# Define result storage arrays
n_w_IS_variable = np.zeros((len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))

EE_IS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
DC_IS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
work_IS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
cycle_duration_IS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
EE_CS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
DC_CS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
work_CS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))
cycle_duration_CS_variable = np.zeros((len(capacity_values), len(tau_values), len(zeta_values), len(delta_commute_values), len(eta_values)))



# function that performs calculations for a single delta_commute value
def calculate_IS_for_delta_commute_value_index(k):
    results = {}
    for h in range(len(capacity_values)):
        for i in range(len(tau_values)):
            for j in range(len(zeta_values)):
                for l in range(len(eta_values)):
                    results[(h,i,j,k,l)] =  cf.calculate_IS(q_w_max=capacity_values[h],
                    delta_commute=delta_commute_values[k],
                    nu_work=eta_values[l] * nu_commute_value,
                    nu_move=nu_commute_value,
                    nu_min=nu_min_value,
                    nu_charge=nu_charge_values[h],
                    nu_transfer=nu_charge_values[h],
                    xi=xi_values[h],
                    tau= tau_values[i],
                    zeta= zeta_values[j],
                    n_c= n_c_value)
    return results

# Start parallelization
with ThreadPoolExecutor(max_workers=16) as executor:
    futures = [executor.submit(calculate_IS_for_delta_commute_value_index, k) for k in range(len(delta_commute_values))]
    for future in futures:
        results = future.result()
        for key, val in results.items():
            h, i, j, k, l = key
            EE_IS_variable[h,i,j,k,l], DC_IS_variable[h,i,j,k,l], work_IS_variable[h,i,j,k,l], cycle_duration_IS_variable[h,i,j,k,l] = val


# function that performs calculations for a single delta_commute value
def calculate_CS_for_delta_commute_value_index(k):
    results = {}
    for h in range(len(capacity_values)):
        for i in range(len(tau_values)):
            for j in range(len(zeta_values)):
                for l in range(len(eta_values)):
                    results[(h,i,j,k,l)] =  cf.calculate_CS(q_w_max=capacity_values[h],
                    delta_commute=delta_commute_values[k],
                    nu_work=eta_values[l] * nu_commute_value,
                    nu_move=nu_commute_value,
                    nu_min=nu_min_value,
                    nu_charge=nu_charge_values[h],
                    nu_transfer=nu_charge_values[h],
                    xi=xi_values[h],
                    tau= tau_values[i],
                    zeta= zeta_values[j],
                    n_c= n_c_value)
    return results

# Start parallelization
with ThreadPoolExecutor(max_workers=16) as executor:
    futures = [executor.submit(calculate_CS_for_delta_commute_value_index, k) for k in range(len(delta_commute_values))]
    for future in futures:
        results = future.result()
        for key, val in results.items():
            h, i, j, k, l = key
            EE_CS_variable[h,i,j,k,l], DC_CS_variable[h,i,j,k,l], work_CS_variable[h,i,j,k,l], cycle_duration_CS_variable[h,i,j,k,l] = val


# change to true if you want to save the results
if True:
    # Save results to a compressed .npz file with timestamp
    t = datetime.now()
    np.savez_compressed(f'script_values_{t.strftime("%Y_%m_%d_%H_%M_%S")}.npz', capacity_values=capacity_values, tau_values=tau_values, zeta_values=zeta_values, delta_commute_values=delta_commute_values, eta_values=eta_values, n_w_IS_variable=n_w_IS_variable,
                        EE_IS_variable=EE_IS_variable, DC_IS_variable=DC_IS_variable, work_IS_variable=work_IS_variable, cycle_duration_IS_variable=cycle_duration_IS_variable, EE_CS_variable=EE_CS_variable, DC_CS_variable=DC_CS_variable, work_CS_variable=work_CS_variable, cycle_duration_CS_variable=cycle_duration_CS_variable)

#### Comparison of Tau and Zeta: Work ####
# Create a figure
work_ratio_variable = work_CS_variable / work_IS_variable
# Set up figure and axes
# --- Plot setup ---
figsize = (10, 4)
figure, axis = plt.subplots(
    len(delta_commute_values),
    len(eta_values),
    figsize=figsize,
    sharex=True,
    sharey=True
)

axis = np.atleast_2d(axis)

for k in range(len(delta_commute_values)):
    for l in range(len(eta_values)):
        X, Y = np.meshgrid(zeta_values, tau_values)
        cmap = plt.get_cmap('coolwarm').copy()
        cmap.set_bad(color='cyan')

        pcm = axis[len(delta_commute_values) - k - 1, l].pcolormesh(
            X, Y,
            work_ratio_variable[0, :, :, k, l],
            cmap=cmap,
            shading='auto',
            norm=Normalize(vmin=0, vmax=2)
        )

        ax = axis[len(delta_commute_values) - k - 1, l]

        # τ label only on left column
        if l == 0:
            ax.set_ylabel(r'$\tau$', fontsize=12, labelpad=-2)

# Identify the bottom row and add ζ + η labels there
bottom_row = axis[-1, :]  # last row always bottom for sharex=True
for l, ax in enumerate(bottom_row):
    ax.set_xlabel(r'$\zeta$', fontsize=12, labelpad=-2)
    ax.tick_params(labelbottom=True)         # ensure tick labels visible
    ax.xaxis.set_visible(True)               # force x-axis visibility

# Δ_commute labels on left of each row
plt.subplots_adjust(wspace=0.1, hspace=0.1, left=0.1, right=1.05, top=0.9, bottom=0.22)
for k, delta in enumerate(reversed(delta_commute_values)):
    ax_left = axis[k, 0]
    pos = ax_left.get_position()
    figure.text(
        pos.x0 - 0.07,
        pos.y0 + pos.height / 2,
        rf'$\Delta_{{\text{{commute}}}}$' + '\n' + rf'$= {delta}$',
        ha='center', va='center', rotation=90, fontsize=12
    )
for k, eta in enumerate(eta_values):
    ax_bottom = axis[len(delta_commute_values) - 1, k]
    pos = ax_bottom.get_position()
    ax_bottom.text(
        0.5, -0.6,  # relative to the axis
        rf'$\eta = {eta}$',
        ha='center', va='top',
        transform=ax_bottom.transAxes,
        fontsize=12
    )

# Shared colorbar
cbar = figure.colorbar(pcm, ax=axis, orientation='vertical', pad=0.015)
cbar.set_label('Ratio of Work (CS. / IS.)', fontsize=12)

#plt.savefig("ratio_plot_stretched.png", dpi=300, bbox_inches='tight', pad_inches=0.05)
plt.show()


############ Comparison of delta_commute ############
delta_commute_values_big_range = list(range(0, 20001, 5))
delta_commute_values_big_range = list(np.concatenate([
    np.arange(0, 100, 1),    
    np.arange(100, 1000, 5),    
    np.arange(1000, 10000, 10),  
    np.arange(10000, 30001, 50)  
]))
ee_IS_cap = np.zeros(len(delta_commute_values_big_range))
ee_CS_cap = np.zeros(len(delta_commute_values_big_range))
ee_IS_bat = np.zeros(len(delta_commute_values_big_range))
ee_CS_bat = np.zeros(len(delta_commute_values_big_range))
work_IS_cap = np.zeros(len(delta_commute_values_big_range))
work_CS_cap = np.zeros(len(delta_commute_values_big_range))
work_IS_bat = np.zeros(len(delta_commute_values_big_range))
work_CS_bat = np.zeros(len(delta_commute_values_big_range))
for i in range(len(delta_commute_values_big_range)):
    ee_IS_cap[i], _, work_IS_cap[i], _ = cf.calculate_IS(q_w_max=q_cap_value,
                        delta_commute=delta_commute_values_big_range[i],
                        nu_work=1 * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_cap_value,
                        nu_transfer=nu_charge_cap_value,
                        xi=xi_cap,
                        tau= 5,
                        zeta= 5,
                        n_c= n_c_value)
    ee_CS_cap[i], _, work_CS_cap[i], _ = cf.calculate_CS(q_w_max=q_cap_value,
                        delta_commute=delta_commute_values_big_range[i],
                        nu_work=1 * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_cap_value,
                        nu_transfer=nu_charge_cap_value,
                        xi=xi_cap,
                        tau= 5,
                        zeta= 5,
                        n_c= n_c_value) 
    ee_IS_bat[i], _, work_IS_bat[i], _ = cf.calculate_IS(q_w_max=q_bat_value,
                        delta_commute=delta_commute_values_big_range[i],
                        nu_work=1 * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_bat_value,
                        nu_transfer=nu_charge_bat_value,
                        xi=xi_bat,
                        tau= 5,
                        zeta= 5,
                        n_c= n_c_value)
    ee_CS_bat[i], _, work_CS_bat[i], _ = cf.calculate_CS(q_w_max=q_bat_value,
                        delta_commute=delta_commute_values_big_range[i],
                        nu_work=1 * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_bat_value,
                        nu_transfer=nu_charge_bat_value,
                        xi=xi_bat,
                        tau= 5,
                        zeta= 5,
                        n_c= n_c_value)

work_diff_cap = work_CS_cap - work_IS_cap
work_diff_bat = work_CS_bat - work_IS_bat


fig, axes = plt.subplots(1, 2, figsize=(12, 4), sharex=True)
ax1, ax2 = axes

# --- Plot 1: Amount of Work ---
ax1.plot(delta_commute_values_big_range, work_IS_cap, label='Supercap. IS.', color='#117733', linestyle='dotted', linewidth=3)
ax1.plot(delta_commute_values_big_range, work_CS_cap, label='Supercap. CS.', color='#117733', linestyle='dashed', linewidth=3)
ax1.plot(delta_commute_values_big_range, work_IS_bat, label='Battery IS.', color='#88ccee', linestyle='dotted', linewidth=3)
ax1.plot(delta_commute_values_big_range, work_CS_bat, label='Battery CS.', color='#88ccee', linestyle='dashed', linewidth=3)
ax1.set_xlabel(r'$\Delta_{\text{commute}}$', fontsize=16)
ax1.set_ylabel('Amount of Work', fontsize=16)
ax1.set_xscale('log')
ax1.tick_params(axis='both', labelsize=14)

# --- Plot 2: Energy Efficiency ---
ax2.plot(delta_commute_values_big_range, ee_IS_cap, color='#117733', linestyle='dotted', linewidth=3)
ax2.plot(delta_commute_values_big_range, ee_CS_cap, color='#117733', linestyle='dashed', linewidth=3)
ax2.plot(delta_commute_values_big_range, ee_IS_bat, color="#88ccee", linestyle='dotted', linewidth=3)
ax2.plot(delta_commute_values_big_range, ee_CS_bat, color="#88ccee", linestyle='dashed', linewidth=3)
ax2.set_xlabel(r'$\Delta_{\text{commute}}$', fontsize=16)
ax2.set_ylabel('Energy Efficiency', fontsize=16)
ax2.set_xscale('log')
ax2.tick_params(axis='both', labelsize=14)


handles, labels = ax1.get_legend_handles_labels()
fig.legend(handles, labels, loc='upper center', ncol=4, fontsize=14, frameon=False)

plt.tight_layout(rect=[0.01, 0, 0.99, 0.9])  
#plt.savefig("combined_plots.png", dpi=300, bbox_inches='tight', pad_inches=0.05)
plt.show()




############ Alpha Beta plots ############
delta_commute_used_alpha_beta = 700
eta_used_alpha_beta = 1
tau_used_alpha_beta = 5
zeta_used_alpha_beta = 5
alpha_values = np.concatenate([
    np.arange(1, 10, 0.05),     
    np.arange(10, 100, 0.5),    
    np.arange(100, 1000, 5),    
    np.arange(1000, 10001, 50)  
])
beta_values = np.concatenate([
    np.arange(1, 10, 0.05),     
    np.arange(10, 100, 0.5),    
    np.arange(100, 1000, 5),    
    np.arange(1000, 10001, 50),  
    np.arange(10000, 19999, 500)
])
IS_cap = cf.calculate_IS(q_w_max=q_cap_value,
                        delta_commute=delta_commute_used_alpha_beta,
                        nu_work=eta_used_alpha_beta * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_cap_value,
                        nu_transfer=nu_charge_cap_value,
                        xi=xi_cap,
                        tau= tau_used_alpha_beta,
                        zeta= zeta_used_alpha_beta,
                        n_c= n_c_value)
CS_cap = cf.calculate_CS(q_w_max=q_cap_value,
                        delta_commute=delta_commute_used_alpha_beta,
                        nu_work=eta_used_alpha_beta * nu_commute_value,
                        nu_move=nu_commute_value,
                        nu_min=nu_min_value,
                        nu_charge=nu_charge_cap_value,
                        nu_transfer=nu_charge_cap_value,
                        xi=xi_cap,
                        tau= tau_used_alpha_beta,
                        zeta= zeta_used_alpha_beta,
                        n_c= n_c_value) 

diff_ee_cap = CS_cap[0] - IS_cap[0]
diff_dq_cap = CS_cap[1] - IS_cap[1]
diff_work_cap = CS_cap[2] - IS_cap[2]

alpha_beta_IS_bat = np.zeros((len(alpha_values),len(beta_values),4))
alpha_beta_CS_bat = np.zeros((len(alpha_values),len(beta_values),4))
alpha_beta_ee_diff_bat = np.zeros((len(alpha_values),len(beta_values)))
alpha_beta_dc_diff_bat = np.zeros((len(alpha_values),len(beta_values)))
alpha_beta_work_diff_bat = np.zeros((len(alpha_values),len(beta_values)))
for a in range(len(alpha_values)):
    for b in range(len(beta_values)):
        q_bat_value_beta = alpha_values[a] * q_cap_value
        nu_charge_bat_value_beta = nu_charge_cap_value / beta_values[b]
        alpha_beta_IS_bat[a, b, :] = cf.calculate_IS(q_w_max=q_bat_value_beta,
                            delta_commute=delta_commute_used_alpha_beta,
                            nu_work=eta_used_alpha_beta * nu_commute_value,
                            nu_move=nu_commute_value,
                            nu_min=nu_min_value,
                            nu_charge=nu_charge_bat_value_beta,
                            nu_transfer=nu_charge_bat_value_beta,
                            xi=xi_bat,
                            tau= tau_used_alpha_beta,
                            zeta= zeta_used_alpha_beta,
                            n_c= n_c_value)
        alpha_beta_CS_bat[a, b, :] = cf.calculate_CS(q_w_max=q_bat_value_beta,
                            delta_commute=delta_commute_used_alpha_beta,
                            nu_work=eta_used_alpha_beta * nu_commute_value,
                            nu_move=nu_commute_value,
                            nu_min=nu_min_value,
                            nu_charge=nu_charge_bat_value_beta,
                            nu_transfer=nu_charge_bat_value_beta,
                            xi=xi_bat,
                            tau= tau_used_alpha_beta,
                            zeta= zeta_used_alpha_beta,
                            n_c= n_c_value)

alpha_beta_ee_diff_bat = alpha_beta_CS_bat[0] - alpha_beta_IS_bat[0]
alpha_beta_dc_diff_bat = alpha_beta_CS_bat[1] - alpha_beta_IS_bat[1]
alpha_beta_work_diff_bat = alpha_beta_CS_bat[2] - alpha_beta_IS_bat[2]

# compare both strategies for battery in one plot with regions where DC and EE are positive or negative
X, Y = np.meshgrid(alpha_values, beta_values)

dc_IS = alpha_beta_IS_bat[:, :, 1].T - IS_cap[1]
ee_IS = alpha_beta_IS_bat[:, :, 0].T - IS_cap[0]

dc_CS = alpha_beta_CS_bat[:, :, 1].T - CS_cap[1]
ee_CS = alpha_beta_CS_bat[:, :, 0].T - CS_cap[0]

fig, axes = plt.subplots(1, 2, figsize=(9, 3.5), sharex=True, sharey=True)  
ax1, ax2 = axes

for ax, dc, ee, title in zip(
    axes, 
    [dc_IS, dc_CS], 
    [ee_IS, ee_CS], 
    ["Individual Strategy", "Cooperative Strategy"]
):
    # calculate regions
    regions = (dc > 0).astype(int) + 2 * (ee > 0).astype(int)

    # colormap for regions
    cmap = {
        0: "#004488",   # both negative
        2: "#BB5566",      # EE positive, DC negative
        3: "#DDAA33",   # both positive
    }

    ax.contourf(
        X, Y, regions,
        levels=[-0.5, 0.5, 1.5, 2.5, 3.5],
        colors=[cmap.get(k, "none") for k in range(4)], alpha=0.5
    )

    ax.contour(X, Y, dc, levels=[0], colors="white", linewidths=2)
    ax.contour(X, Y, ee, levels=[0], colors="white", linewidths=2)
    ax.plot([q_bat_value/q_cap_value], [nu_charge_cap_value/nu_charge_bat_value], marker='*', color='#000000', markersize=10, label=rf'Previously used configuration ($\alpha$ ≈ {q_bat_value/q_cap_value:.2f}, $\beta$ ≈ {nu_charge_cap_value/nu_charge_bat_value:.2f})')
    ax.set_xscale("log")
    ax.set_yscale("log")
    ax.set_xlabel(r"$\alpha$", fontsize=13)
    ax.tick_params(axis="both", labelsize=10)
    ax.set_title(title, fontsize=13)

ax1.set_ylabel(r"$\beta$", fontsize=13)
ax2.set_ylabel("")               

handles = [
    plt.Rectangle((0, 0), 1, 1, color="#004488", alpha=0.5,
                  label='Battery less productive and less efficient'),
    plt.Rectangle((0, 0), 1, 1, color="#BB5566", alpha=0.5,
                  label='Battery less productive but more efficient'),
    plt.Rectangle((0, 0), 1, 1, color="#DDAA33", alpha=0.5,
                  label='Battery more productive and more efficient'),
    plt.Line2D([0], [0], marker='*', color='#000000', markersize=10, linestyle='None',
           label=rf'Previously used configuration ($\alpha$ ≈ {q_bat_value/q_cap_value:.2f}, $\beta$ ≈ {nu_charge_cap_value/nu_charge_bat_value:.2f})')
]

fig.legend(
    handles=handles,
    loc="upper center",
    ncol=2,              
    fontsize=11,         
    frameon=False,
    bbox_to_anchor=(0.5, 0.98)
)


plt.tight_layout(rect=[0, 0, 1, 0.83])  
#plt.savefig("combined_alpha_beta_regions_compact.png", dpi=300, bbox_inches='tight', pad_inches=0.05)
plt.show()



