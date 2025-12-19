# Physical AI Locomotion - System Identification

A ROS2-based system identification platform for determining inertial parameters of robotic leg components through trial-and-error methodology. This project uses pendulum-based experiments to iteratively identify mass, center of mass, and inertia tensor values by comparing simulated and real-world dynamic behavior.

## Overview

This project implements a **system identification workflow** to find accurate inertial parameters (mass, center of mass, moment of inertia) for individual leg joints. The methodology involves:

1. **Physical Experiments**: Free-fall pendulum tests on real hardware (hip, knee, ankle links)
2. **Simulation Modeling**: Gazebo physics simulation with parametric inertia values
3. **Comparison Analysis**: Automated trajectory alignment and error metrics (RMSE, correlation)
4. **Iterative Refinement**: Trial-and-error tuning of simulation parameters until sim matches real data

The goal is to obtain physically accurate inertia tensors that enable high-fidelity simulation for locomotion control development.

## Project Structure

```
physical_ai_locomotion_team/
├── src/
│   ├── system_identification_description/  # Individual link URDF models (hip, knee, ankle)
│   ├── system_identification_simulation/   # Pendulum test simulations and data logging
│   ├── test_station_description/           # Full test station URDF (integrated system)
│   ├── test_station_simulation/            # Full system simulation
│   ├── test_station/                       # Kinematics and control algorithms
│   ├── controller/                         # Hardware interface layer
│   ├── pos_raw_signal/                     # Real hardware experimental data (CSV)
│   └── sim_signal/                         # Reference simulation baseline
├── sim_signal/                             # Timestamped simulation trial outputs
├── results/                                # Comparison plots and analysis results
├── compare_sim.py                          # Sim vs real comparison and alignment tool
└── Diagnose.py                             # Data validation utility
```

## Features

- **System Identification Workflow**: Trial-and-error inertia parameter tuning methodology
- **Isolated Pendulum Tests**: Individual hip, knee, and ankle link experiments
- **Automated Comparison**: Position-based trajectory alignment and error metrics (RMSE, correlation)
- **High-Frequency Data Logging**: 1000+ Hz joint state capture (position, velocity, effort)
- **Parametric Simulation**: Easily adjustable inertia tensors, damping, and friction
- **Physics-Based Validation**: Gazebo simulation with ROS2 control integration
- **Multi-Angle Testing**: Support for experiments at various initial positions

## Requirements

- ROS2 Humble
- Gazebo Classic
- Python 3.8+
- Required Python packages: `pandas`, `numpy`, `matplotlib`

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/physical_ai_locomotion_team.git
cd physical_ai_locomotion_team
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install pandas numpy matplotlib
```

3. Build the workspace:
```bash
colcon build
source install/setup.bash
```

## Usage

### System Identification Workflow

**Step 1: Run Physical Experiment**
- Perform free-fall pendulum test on real hardware
- Record joint state data (position, velocity)
- Save to `src/pos_raw_signal/`

**Step 2: Run Simulation with Initial Guess**
```bash
# Knee link example
ros2 launch system_identification_simulation knee_link_only_sim.py
```

**Step 3: Compare Sim vs Real**
```bash
python3 src/compare_sim.py
```

**Step 4: Adjust Parameters**
- Edit inertia values in URDF: `src/system_identification_description/robot/visual/knee_link_only.xacro`
- Modify: `knee_ixx`, `knee_iyy`, `knee_izz` (moment of inertia)
- Modify: `mass_knee`, `knee_inertial_x/y/z` (mass and COM)
- Modify: `joint_damping`, `joint_friction`

**Step 5: Iterate**
- Rebuild: `colcon build --packages-select system_identification_description`
- Repeat Steps 2-4 until RMSE is minimized

### Running Individual Link Simulations

**Knee Link Pendulum:**
```bash
ros2 launch system_identification_simulation knee_link_only_sim.py
```

**Hip Link Pendulum:**
```bash
ros2 launch system_identification_simulation hip_link_only_sim.py
```

**Ankle Link Pendulum:**
```bash
ros2 launch system_identification_simulation ankle_link_only_sim.py
```

### Manual Data Logging

To log simulation data with custom parameters:
```bash
ros2 run system_identification_simulation log_data.py --ros-args \
  -p joint_name:=world_to_knee \
  -p duration:=5.0 \
  -p output_dir:=sim_signal
```

### Comparing Simulation vs Real Data

```bash
python3 src/compare_sim.py
```

This generates comparison plots showing:
- Position trajectories
- Velocity profiles
- Phase portraits
- Error metrics (RMSE, correlation)

## Key Components

### 1. Parametric Robot Models

Inertial parameters defined in xacro files (tunable for system identification):
- **Hip Link**: 0.425 kg, Izz = 0.00050 kg·m² (example values)
- **Knee Link**: 0.091 kg, Izz = 0.000175 kg·m² (example values)
- **Ankle Link**: 0.721 kg, Izz = 0.00280 kg·m² (example values)
- Center-of-mass positions and full inertia tensors

### 2. Free-Fall Pendulum Simulation

- Minimal damping and friction for inertia identification
- Gazebo physics engine with ROS2 control
- High-frequency state logging (1000+ Hz)
- Effort controllers @ 100 Hz for controlled experiments

### 3. Comparison and Alignment Tools

- **Position-based alignment**: Automatically trims real data to match simulation starting position
- **Signal interpolation**: Resamples data to common time grid
- **Error metrics**: RMSE, correlation coefficient, peak analysis
- **Visualization**: Position/velocity trajectories, phase portraits

## Configuration

### Tuning Inertial Parameters (System Identification)

Edit physical properties in URDF files:

**Knee Link:**
- File: `src/system_identification_description/robot/visual/knee_link_only.xacro`
- Parameters: `mass_knee`, `knee_ixx/iyy/izz`, `knee_inertial_x/y/z`

**Hip Link:**
- File: `src/system_identification_description/robot/visual/hip_link_only.xacro`
- Parameters: `mass_hip`, `hip_ixx/iyy/izz`, `hip_inertial_x/y/z`

**Ankle Link:**
- File: `src/system_identification_description/robot/visual/ankle_link_only.xacro`
- Parameters: `mass_ankle`, `ankle_ixx/iyy/izz`, `ankle_inertial_x/y/z`

### Joint Dynamics

Modify damping and friction coefficients:
- `joint_damping` - Viscous damping (N·m·s/rad)
- `joint_friction` - Coulomb friction (N·m)

### Controller Configuration

Controller settings in:
- `src/system_identification_simulation/config/knee_pendulum_controller.yaml`
- `src/system_identification_simulation/config/hip_pendulum_controller.yaml`
- `src/system_identification_simulation/config/ankle_pendulum_controller.yaml`

### Comparison Settings

Adjust alignment and filtering in:
- `src/compare_sim.py` (lines 23-42 for physical parameters)

## Data Format

All CSV files contain:
- `sim_time_sec`: Simulation time (reset to 0 at logging start)
- `position_rad`: Joint position in radians
- `velocity_rad_s`: Joint velocity in rad/s
- `effort_N_m`: Applied joint effort in N·m

## Contributing

This is a research project. For questions or collaboration opportunities, please open an issue.

## License

[Add your license here]

## Methodology Notes

### Trial-and-Error System Identification Process

This project uses an empirical approach to find inertial parameters:

1. Start with estimated or CAD-derived parameters
2. Run physical experiment (free-fall pendulum test)
3. Simulate with current parameter guess
4. Compare trajectories and compute error metrics
5. Manually adjust parameters based on error patterns:
   - If sim swings faster → increase inertia (Izz)
   - If sim amplitude decays faster → reduce damping
   - If sim phase leads/lags → adjust center of mass
6. Repeat until RMSE converges to acceptable threshold

**Advantages:**
- No complex optimization algorithms required
- Intuitive physical understanding of parameter effects
- Works well for low-DOF systems (single pendulums)

**Limitations:**
- Manual iteration can be time-consuming
- May not find global optimum
- Requires experience to interpret error patterns

## Acknowledgments

Developed for physical AI locomotion research with focus on accurate inertial parameter identification for high-fidelity simulation.
