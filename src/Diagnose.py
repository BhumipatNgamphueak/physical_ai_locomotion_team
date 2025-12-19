#!/usr/bin/env python3
"""
Diagnostic Script - Analyze Sim and Real Data
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

print("="*60)
print("DATA DIAGNOSTIC TOOL")
print("="*60)

# Load simulation data
sim_file = "/tmp/sim_joint_states_20251123_224914.csv"
print(f"\n📊 Loading simulation data: {sim_file}")
sim_df = pd.read_csv(sim_file)

print(f"\nSimulation data shape: {sim_df.shape}")
print(f"Columns: {list(sim_df.columns)}")
print(f"\nFirst 5 rows:")
print(sim_df.head())
print(f"\nLast 5 rows:")
print(sim_df.tail())

# Statistics
print(f"\n📈 Simulation Statistics:")
print(f"  Time range: {sim_df['sim_time_sec'].min():.3f} to {sim_df['sim_time_sec'].max():.3f} seconds")
print(f"  Duration: {sim_df['sim_time_sec'].max() - sim_df['sim_time_sec'].min():.3f} seconds")
print(f"  Samples: {len(sim_df)}")
print(f"  Sample rate: {len(sim_df) / (sim_df['sim_time_sec'].max() - sim_df['sim_time_sec'].min()):.1f} Hz")
print(f"\n  Position range: {np.degrees(sim_df['position_rad'].min()):.1f}° to {np.degrees(sim_df['position_rad'].max()):.1f}°")
print(f"  Initial position: {np.degrees(sim_df['position_rad'].iloc[0]):.3f}°")
print(f"  Final position: {np.degrees(sim_df['position_rad'].iloc[-1]):.3f}°")
print(f"\n  Velocity range: {sim_df['velocity_rad_s'].min():.3f} to {sim_df['velocity_rad_s'].max():.3f} rad/s")
print(f"  Initial velocity: {sim_df['velocity_rad_s'].iloc[0]:.3f} rad/s")

# Load real data
real_file = "/home/prime/physical_ai_locomotion_team/src/inertia_m180deg.csv"
print(f"\n\n📊 Loading real data: {real_file}")
real_df = pd.read_csv(real_file)

print(f"\nReal data shape: {real_df.shape}")
print(f"Columns: {list(real_df.columns)}")
print(f"\nFirst 10 rows:")
print(real_df.head(10))
print(f"\nLast 5 rows:")
print(real_df.tail())

# Check for NaN values
print(f"\n🔍 Checking for NaN/invalid values:")
print(f"  NaN in timestamp: {real_df['timestamp'].isna().sum()}")
print(f"  NaN in position: {real_df['position_rad'].isna().sum()}")
print(f"  NaN in velocity: {real_df['velocity_rad_s'].isna().sum()}")

# Remove NaN values
real_df_clean = real_df.dropna()
print(f"\nAfter removing NaN: {len(real_df_clean)} samples (removed {len(real_df) - len(real_df_clean)})")

# Parse time from date and time columns
# Format: "2025-11-13" and "04:15:29.638:"
print(f"\n🔍 Parsing timestamps:")

try:
    # Combine date and time, remove trailing colon
    datetime_str = real_df_clean['date'] + ' ' + real_df_clean['time'].str.rstrip(':')
    
    # Parse to datetime
    timestamps = pd.to_datetime(datetime_str, format='%Y-%m-%d %H:%M:%S.%f')
    
    # Convert to seconds relative to first timestamp
    time_s = (timestamps - timestamps.iloc[0]).dt.total_seconds().values
    
    print(f"   ✓ Successfully parsed from date/time columns")
    print(f"   Start: {real_df_clean['date'].iloc[0]} {real_df_clean['time'].iloc[0]}")
    print(f"   End:   {real_df_clean['date'].iloc[-1]} {real_df_clean['time'].iloc[-1]}")
    timestamp_type = "parsed from date/time columns"
    
except Exception as e:
    print(f"   ⚠️  Error parsing date/time: {e}")
    print(f"   Falling back to timestamp column...")
    
    # Fallback: Try to interpret timestamp
    time_raw = real_df_clean['timestamp'].values

    # Check if timestamps look like milliseconds (large numbers > 1000)
    if time_raw[0] > 1000:
        # Assume milliseconds
        time_s = (time_raw - time_raw[0]) / 1000.0
        timestamp_type = "milliseconds"
    else:
        # Probably sample numbers or fractional seconds - use sample number as time
        # Assume 1000 Hz sampling (adjust if you know actual rate)
        print(f"\n⚠️  WARNING: Timestamp values are too small ({time_raw[0]:.1f} to {time_raw[-1]:.1f})")
        print(f"    These don't look like milliseconds!")
        print(f"    Assuming sample numbers with 1000 Hz sampling rate...")
        time_s = np.arange(len(time_raw)) / 1000.0  # Assume 1000 Hz
        timestamp_type = "sample numbers (assumed 1000 Hz)"

print(f"\n📈 Real Data Statistics:")
print(f"  Timestamp interpretation: {timestamp_type}")
print(f"  Time range: {time_s[0]:.3f} to {time_s[-1]:.3f} seconds")
print(f"  Duration: {time_s[-1] - time_s[0]:.3f} seconds")
print(f"  Samples: {len(real_df_clean)}")
if time_s[-1] - time_s[0] > 0:
    print(f"  Sample rate: {len(real_df_clean) / (time_s[-1] - time_s[0]):.1f} Hz")
else:
    print(f"  Sample rate: Cannot calculate (zero duration)")
print(f"\n  Position range: {np.degrees(real_df_clean['position_rad'].min()):.1f}° to {np.degrees(real_df_clean['position_rad'].max()):.1f}°")
print(f"  Initial position: {np.degrees(real_df_clean['position_rad'].iloc[0]):.3f}°")
print(f"  Final position: {np.degrees(real_df_clean['position_rad'].iloc[-1]):.3f}°")
print(f"\n  Velocity range: {real_df_clean['velocity_rad_s'].min():.3f} to {real_df_clean['velocity_rad_s'].max():.3f} rad/s")
print(f"  Initial velocity: {real_df_clean['velocity_rad_s'].iloc[0]:.3f} rad/s")

# Convert to numpy arrays for later use
sim_time = sim_df['sim_time_sec'].values
sim_pos = sim_df['position_rad'].values
sim_vel = sim_df['velocity_rad_s'].values

real_pos = real_df_clean['position_rad'].values
real_vel = real_df_clean['velocity_rad_s'].values

# Find when motion starts (release point)
print(f"\n🎯 Detecting Release Points:")
sim_vel_arr = sim_df['velocity_rad_s'].values
sim_moving = np.abs(sim_vel_arr) > 0.01
if np.any(sim_moving):
    sim_release_idx = np.argmax(sim_moving)
    print(f"  Sim release at index {sim_release_idx}, t={sim_time[sim_release_idx]:.3f}s")
    print(f"    Position: {np.degrees(sim_pos[sim_release_idx]):.3f}°")
    print(f"    Velocity: {sim_vel_arr[sim_release_idx]:.3f} rad/s")

real_moving = np.abs(real_vel) > 0.01
if np.any(real_moving):
    real_release_idx = np.argmax(real_moving)
    print(f"  Real release at index {real_release_idx}, t={time_s[real_release_idx]:.3f}s")
    print(f"    Position: {np.degrees(real_pos[real_release_idx]):.3f}°")
    print(f"    Velocity: {real_vel[real_release_idx]:.3f} rad/s")

# Create diagnostic plots
print(f"\n📊 Creating diagnostic plots...")

fig, axes = plt.subplots(2, 2, figsize=(14, 10))
fig.suptitle('Data Diagnostic - Sim vs Real', fontsize=16, fontweight='bold')

# Plot 1: Position over time
ax = axes[0, 0]
ax.plot(sim_time, np.degrees(sim_pos), 'b-', label='Sim', linewidth=2)
ax.plot(time_s, np.degrees(real_pos), 'r-', label='Real', linewidth=2, alpha=0.7)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (degrees)')
ax.set_title('Raw Position Data')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 2: Velocity over time
ax = axes[0, 1]
ax.plot(sim_time, sim_vel, 'b-', label='Sim', linewidth=2)
ax.plot(time_s, real_vel, 'r-', label='Real', linewidth=2, alpha=0.7)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Velocity (rad/s)')
ax.set_title('Raw Velocity Data')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 3: Phase portrait
ax = axes[1, 0]
ax.plot(np.degrees(sim_pos), sim_vel, 'b-', label='Sim', linewidth=2, alpha=0.7)
ax.plot(np.degrees(real_pos), real_vel, 'r-', label='Real', linewidth=2, alpha=0.7)
ax.set_xlabel('Position (degrees)')
ax.set_ylabel('Velocity (rad/s)')
ax.set_title('Phase Portrait (Full Data)')
ax.legend()
ax.grid(True, alpha=0.3)

# Plot 4: First 2 seconds comparison
ax = axes[1, 1]
sim_mask = sim_time <= 2.0
real_mask = time_s <= 2.0
ax.plot(sim_time[sim_mask], np.degrees(sim_pos[sim_mask]), 'b-', label='Sim', linewidth=2)
ax.plot(time_s[real_mask], np.degrees(real_pos[real_mask]), 'r-', label='Real', linewidth=2, alpha=0.7)
ax.set_xlabel('Time (s)')
ax.set_ylabel('Position (degrees)')
ax.set_title('First 2 Seconds - Position')
ax.legend()
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('/home/prime/physical_ai_locomotion_team/results/diagnostic_plot.png', dpi=150, bbox_inches='tight')
print(f"✅ Diagnostic plot saved to: /home/prime/physical_ai_locomotion_team/results/diagnostic_plot.png")

print("\n" + "="*60)
print("DIAGNOSTIC COMPLETE")
print("="*60)