# Physical AI Locomotion - System Identification

A ROS2-based system identification platform for determining inertial parameters of robotic leg components through trial-and-error methodology. This project uses pendulum-based experiments to iteratively identify mass, center of mass, and inertia tensor values by comparing simulated and real-world dynamic behavior.

## Overview

This project implements a **system identification workflow** to find accurate inertial parameters (mass, center of mass, moment of inertia) for individual leg joints. The methodology involves:

1. **Physical Experiments**: Free-fall pendulum tests on real hardware (hip, knee, ankle links)
2. **Simulation Modeling**: Gazebo physics simulation with parametric inertia values
3. **Comparison Analysis**: Automated trajectory alignment and error metrics (RMSE, correlation)
4. **Iterative Refinement**: Trial-and-error tuning of simulation parameters until sim matches real data

**Goal: Position tracking error < 10%** - Obtain physically accurate inertia tensors that enable high-fidelity simulation for locomotion control development.

## Project Structure

```
physical_ai_locomotion_team/
├── src/                                    # ROS2 source packages
│   ├── system_identification_description/  # Individual link URDF models (hip, knee, ankle)
│   ├── system_identification_simulation/   # Pendulum test simulations and data logging
│   ├── test_station_description/           # Full test station URDF (integrated 7-DOF system)
│   ├── test_station_simulation/            # Full system simulation environment
│   ├── test_station/                       # Kinematics and control algorithms (skeleton)
│   └── pos_raw_signal/                     # Real hardware experimental data (CSV)
├── sim_signal/                             # Timestamped simulation trial outputs (generated)
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
- Gazebo Classic (Ignition Gazebo)
- Python 3.8+
- Required Python packages: `pandas`, `numpy`, `matplotlib`, `scipy`
- ROS2 packages: `ros_gz_sim`, `ros_gz_bridge`, `robot_state_publisher`, `controller_manager`

## Installation

1. Clone the repository:
```bash
git clone https://github.com/yourusername/physical_ai_locomotion_team.git
cd physical_ai_locomotion_team
```

2. Install dependencies:
```bash
rosdep install --from-paths src --ignore-src -r -y
pip3 install pandas numpy matplotlib scipy
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

**Step 3: Configure File Paths for Comparison**
- Edit `compare_sim.py` (lines 589-592)
- Update `SIM_FILE` to point to your latest simulation output:
  ```python
  SIM_FILE = "/home/prime/physical_ai_locomotion_team/sim_signal/world_to_hip_sim_YYYYMMDD_HHMMSS.csv"
  ```
- Update `REAL_FILE_OPTIONS` to point to your real experiment data:
  ```python
  REAL_FILE_OPTIONS = [
      "/home/prime/physical_ai_locomotion_team/src/pos_raw_signal/hip_inertia_0deg.csv"
  ]
  ```

**Step 4: Run Comparison**
```bash
python3 compare_sim.py
```

**Step 5: Adjust Parameters**
- Edit inertia values in URDF: `src/system_identification_description/robot/visual/knee_link_only.xacro`
- Modify: `knee_ixx`, `knee_iyy`, `knee_izz` (moment of inertia)
- Modify: `mass_knee`, `knee_inertial_x/y/z` (mass and COM)
- Modify: `joint_damping`, `joint_friction`

**Step 6: Iterate**
- Rebuild: `colcon build --packages-select system_identification_description`
- Repeat Steps 2-5 until position tracking error < 10%

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

**1. Configure File Paths**

Edit `compare_sim.py` (lines 589-596) to set:
```python
# File paths
SIM_FILE = "/home/prime/physical_ai_locomotion_team/sim_signal/world_to_hip_sim_20251219_162153.csv"
REAL_FILE_OPTIONS = [
    "/home/prime/physical_ai_locomotion_team/src/pos_raw_signal/hip_inertia_0deg.csv"
]

# Transformation settings (usually keep at default)
OFFSET_RAD = 0.0       # Position offset in radians
INVERT_PHASE = False    # Phase inversion flag
```

**File Path Guidelines:**
- `SIM_FILE`: Use the latest timestamped CSV from `sim_signal/` directory
- `REAL_FILE_OPTIONS`: Match the link you're testing (hip/knee/ankle)
  - Hip: `src/pos_raw_signal/hip_inertia_0deg.csv`
  - Knee: `src/pos_raw_signal/trim_link2_inertia_0deg.csv`
  - Ankle: `src/pos_raw_signal/ankle_link_inertia_m180deg.csv`

**2. Run Comparison**

```bash
python3 compare_sim.py
```

This generates comparison plots showing:
- Position trajectories
- Velocity profiles
- Phase portraits
- Error metrics (RMSE, correlation)

## Package Architecture

The project is organized into two main systems:

### System Identification Packages (Individual Link Testing)
- **system_identification_description**: URDF/XACRO models for isolated hip, knee, and ankle links
  - Contains individual link mesh files and inertial parameters
  - Single-link pendulum models suspended from world joint
  - Primary location for inertial parameter tuning
- **system_identification_simulation**: Simulation environment for pendulum experiments
  - Launch files for hip, knee, and ankle pendulum tests
  - High-frequency data logger (`log_data.py`)
  - Controller configurations for effort control
  - Gravity enabler and release controller scripts

### Test Station Packages (Full Integrated System)
- **test_station_description**: Complete 7-DOF robot model
  - Kinematic chain: base → linear stages → hip → knee → ankle → end effector
  - Full system URDF with actuators and mechanical constraints
  - Used only for full system simulations (not individual link testing)
- **test_station_simulation**: Full system simulation environment
  - Provides gravity enabler and pendulum release controller
  - Simulation configs for full robot
- **test_station**: Control and kinematics library (skeleton implementations)
  - Forward/inverse kinematics
  - PID controllers (position and velocity)
  - Trajectory planning utilities
  - **Note**: These are placeholder implementations for future development

**Package Separation**: Individual link models and full system models are kept separate to enable focused parameter tuning on isolated components before integration testing.

## Key Components

### 1. Parametric Robot Models

Inertial parameters defined in xacro files (tunable for system identification):

**Hip Link** (`hip_link_only.xacro`):
- Mass: 0.425 kg
- Inertia: Izz = 0.00050 kg·m²
- Joint: world_to_hip (continuous, damping=0.000325 N·m·s/rad, friction=0.00270 N·m)

**Knee Link** (`knee_link_only.xacro`):
- Mass: 0.091 kg
- Inertia: Izz = 0.000175 kg·m²
- Joint: world_to_knee (continuous, damping=0.0004 N·m·s/rad, friction=0.0008 N·m)

**Ankle Link** (`ankle_link_only.xacro`):
- Mass: 0.721 kg
- Inertia: Izz = 0.00280 kg·m²
- Joint: world_to_ankle (continuous, damping=0.00130 N·m·s/rad, friction=0.0140 N·m)

All models include full 3x3 inertia tensors and center-of-mass positions for accurate dynamics.

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
- `compare_sim.py` (lines 23-42 for physical parameters)

## Data Format

**Simulation CSV files** (`sim_signal/`) contain:
- `timestamp`: Wall clock time
- `sim_time_sec`: Simulation time (reset to 0 at logging start)
- `position_rad`: Joint position in radians
- `velocity_rad_s`: Joint velocity in rad/s
- `effort_Nm`: Applied joint effort in N·m
- `acceleration_rad_s2`: Computed acceleration in rad/s²

**Real hardware CSV files** (`src/pos_raw_signal/`) contain:
- `date`, `time`: Timestamp information
- `timestamp`: Milliseconds since epoch
- `position_rad`: Joint position in radians
- `velocity_rad_s`: Joint velocity in rad/s
- `acceleration_rad_s2`: Joint acceleration in rad/s²

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

## Recent Updates

**December 2025:**
- Cleaned up redundant files between `test_station` and `system_identification` packages
- Consolidated individual link XACRO files in `system_identification_description`
- Updated launch files to reference correct package locations
- Fixed mesh file references to use `system_identification_description` package
- Removed duplicate launch files and dummy placeholder files
- Updated CMakeLists.txt files to reflect current package structure

**Current Focus:**
- Hip link parameter tuning in progress (31 simulation trials on 2025-12-19)
- Iterative RMSE minimization through manual parameter adjustment
- **Goal: Position tracking error < 10%** between simulation and real hardware

## Acknowledgments

Developed for physical AI locomotion research with focus on accurate inertial parameter identification for high-fidelity simulation.
