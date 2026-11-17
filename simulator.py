import yaml
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import os

def load_config(filename="config.yaml"):
    with open(filename, 'r') as f:
        return yaml.safe_load(f)

def run_simulation(config):
    dt = config["simulation_settings"]["dt"]
    t = np.arange(0, config["simulation_settings"]["duration_seconds"], dt)
    n_steps = len(t)
    
    scenario_key = config["user_need"]["scenario_name"]
    scen = config["scenarios"][scenario_key]
    user_algos = config["user_need"]["selected_algorithms"]
    if isinstance(user_algos, str): user_algos = [user_algos]
    
    all_algos = ["PID", "ADRC", "LQR", "MPC", "LQG", "FEEDFORWARD", "FUZZY", "ADAPTIVE"]
    algorithms = all_algos if "COMPARE_ALL" in [a.upper() for a in user_algos] else [a.upper() for a in user_algos if a.upper() in all_algos]
    
    print(f"🚀 EXECUTING: {', '.join(algorithms)} on {scen['num_variables']}-DOF System: {scenario_key}")
    
    num_vars = scen["num_variables"]
    setpoints_static = np.array(scen["target_setpoints"])
    
    disturbances = np.zeros((num_vars, n_steps))
    for v in range(num_vars):
        disturbances[v, t >= scen["disturbance_times"][v]] = scen["disturbance_magnitudes"][v]
        
    results = {}
    g = 9.81 
    
    for algo in algorithms:
        states = np.zeros((num_vars, n_steps))
        velocities = np.zeros((num_vars, n_steps))
        integrals = np.zeros(num_vars)
        prev_errors = np.zeros(num_vars)
        
        for i in range(1, n_steps):
            dynamic_setpoints = np.copy(setpoints_static)
            
            # CASCADED CONTROL (6-DOF Drone X/Y tied to Pitch/Roll)
            if scenario_key == "drone_6_dof":
                err_x = setpoints_static[0] - states[0, i-1]
                err_y = setpoints_static[1] - states[1, i-1]
                target_pitch = np.clip(0.15 * err_x - 0.1 * velocities[0, i-1], -0.5, 0.5)
                target_roll = np.clip(-0.15 * err_y + 0.1 * velocities[1, i-1], -0.5, 0.5)
                dynamic_setpoints[3] = target_roll
                dynamic_setpoints[4] = target_pitch

            errors = dynamic_setpoints - states[:, i-1]
            integrals += errors * dt
            derivatives = (errors - prev_errors) / dt
            
            # CONTROL ALGORITHMS
            if algo == "PID":
                control = (10.0 * errors) + (2.0 * integrals) + (5.0 * derivatives)
            elif algo == "ADRC":
                control = (15.0 * errors) + (3.0 * integrals) + (8.0 * derivatives) + 0.2
            elif algo == "LQR":
                control = (12.0 * errors) - (4.0 * velocities[:, i-1])
            elif algo == "MPC":
                control = (14.0 * errors) - (2.0 * velocities[:, i-1]) + 0.5 * dynamic_setpoints
            else:
                control = 8.0 * errors
                
            # MULTI-ROBOT PHYSICS ENGINE
            accel = np.zeros(num_vars)
            
            if scenario_key == "drone_6_dof":
                accel[0] = (g * states[4, i-1]) + disturbances[0, i] - (0.5 * velocities[0, i-1]) # X
                accel[1] = (-g * states[3, i-1]) + disturbances[1, i] - (0.5 * velocities[1, i-1]) # Y
                accel[2] = control[2] + disturbances[2, i] - (1.5 * velocities[2, i-1]) # Z
                accel[3] = control[3] + disturbances[3, i] - (2.0 * velocities[3, i-1]) # Roll
                accel[4] = control[4] + disturbances[4, i] - (2.0 * velocities[4, i-1]) # Pitch
                accel[5] = control[5] + disturbances[5, i] - (1.0 * velocities[5, i-1]) # Yaw
                
            elif scenario_key == "self_balancing_bot":
                accel[0] = control[0] - (2.0 * velocities[0, i-1]) # Cart X
                accel[1] = (g * states[1, i-1]) - control[1] + disturbances[1, i] - (1.5 * velocities[1, i-1]) # Body Pitch
                
            elif scenario_key == "bipedal_walker_lipm":
                z_c = 1.0 
                accel[0] = (g / z_c) * states[0, i-1] + control[0] + disturbances[0, i] - (2.5 * velocities[0, i-1]) # CoM X
                accel[1] = control[1] + disturbances[1, i] - (5.0 * velocities[1, i-1]) # CoM Height
                
            else: # drone_4_dof or standard generic fallback
                accel = control + disturbances[:, i] - (1.5 * velocities[:, i-1])
                
            velocities[:, i] = velocities[:, i-1] + accel * dt
            states[:, i] = states[:, i-1] + velocities[:, i] * dt
            prev_errors = errors
            
        results[algo] = states
        
    return t, setpoints_static, results, disturbances, scen, algorithms

def plot_results(t, setpoints, results, disturbances, scen, algorithms):
    plt.style.use("seaborn-v0_8-whitegrid")
    num_vars = scen["num_variables"]
    names = scen["variable_names"]
    
    colors = {"PID": "#E63946", "ADRC": "#1D3557", "LQR": "#2A9D8F", "MPC": "#8338EC", 
              "LQG": "#F4A261", "FEEDFORWARD": "#00B4D8", "FUZZY": "#FF006E", "ADAPTIVE": "#E9C46A"}
    
    if num_vars == 6:
        fig, axes = plt.subplots(3, 2, figsize=(14, 10), dpi=300, sharex=True)
        axes = axes.flatten()
    else:
        fig, axes = plt.subplots(num_vars, 1, figsize=(12, 3 * num_vars), dpi=300, sharex=True)
        if num_vars == 1: axes = [axes]
    
    for v in range(num_vars):
        if scen["scenario_name"] == "drone_6_dof" and v in [3, 4]:
            pass # Cascaded targets are dynamic, omit static dashed line
        else:
            axes[v].plot(t, np.full_like(t, setpoints[v]), 'k--', linewidth=1.5, label="Target Setpoint")
        
        for algo, states in results.items():
            axes[v].plot(t, states[v], color=colors.get(algo, "#333333"), linewidth=2.0, alpha=0.9, label=algo)
            
        if np.any(disturbances[v]):
            d_time = t[np.nonzero(disturbances[v])[0][0]]
            axes[v].axvline(d_time, color="#888888", linestyle=":")
            
        axes[v].set_title(names[v], fontweight="bold")
        if v == 0: axes[v].legend(loc="best", frameon=True)

    if num_vars == 6:
        axes[4].set_xlabel("Time (Seconds)", fontweight="bold")
        axes[5].set_xlabel("Time (Seconds)", fontweight="bold")
    else:
        axes[-1].set_xlabel("Time (Seconds)", fontweight="bold")
        
    fig.suptitle(f"{num_vars}-DOF Benchmarking: {', '.join(algorithms)}", fontsize=16, fontweight="bold", y=0.98)
    
    os.makedirs("images", exist_ok=True)
    filename = f"images/{num_vars}DOF_{scen['scenario_name']}_{'_vs_'.join(algorithms)}.png"
    plt.tight_layout(rect=[0, 0, 1, 0.96])
    plt.savefig(filename)
    print(f"\n✅ Graph saved as '{filename}'")

if __name__ == "__main__":
    cfg = load_config()
    t, sp, res, dist, scen, algs = run_simulation(cfg)
    plot_results(t, sp, res, dist, scen, algs)
