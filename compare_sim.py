#!/usr/bin/env python3
"""
Simulation vs Real Experiment Comparison - WITH 180° OFFSET
Offsets real data by +180° to align with simulation starting position
"""

import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import signal, interpolate
import os
import shutil


# ========================================
# 📁 OUTPUT PATH
# ========================================
OUTPUT_DIR = os.path.expanduser("~/physical_ai_locomotion_team/results")
os.makedirs(OUTPUT_DIR, exist_ok=True)


class SimRealComparison:
    def __init__(self, sim_file, real_file, output_dir, offset_rad=0.0, invert_phase=False):
        self.sim_file = sim_file
        self.real_file = real_file
        self.output_dir = output_dir
        self.offset_rad = offset_rad
        self.invert_phase = invert_phase

        print(f"📐 Configuration:")
        print(f"   Offset: +{self.offset_rad:.4f} rad ({np.degrees(self.offset_rad):.2f}°)")
        print(f"   Invert phase: {'Yes (*-1)' if self.invert_phase else 'No (*1)'}")
        
        # Load data
        print("Loading simulation data...")
        self.sim_data = self.load_sim_data(sim_file)
        
        print("Loading real experiment data...")
        self.real_data = self.load_real_data(real_file)
        
        print(f"Sim data: {len(self.sim_data)} points")
        print(f"Real data: {len(self.real_data)} points")

    def load_sim_data(self, filename):
        """Load simulation data from CSV"""
        df = pd.read_csv(filename)

        # Apply offset and phase inversion to simulation data
        position_original = df['position_rad'].values
        position_offset = position_original + self.offset_rad

        # Apply phase inversion if requested
        phase_multiplier = -1.0 if self.invert_phase else 1.0
        position_final = position_offset * phase_multiplier

        print(f"\n📐 Applying transformations to simulation data:")
        print(f"   Original range: {position_original.min():.4f} to {position_original.max():.4f} rad")
        print(f"   Offset: +{self.offset_rad:.4f} rad")
        print(f"   Phase multiplier: {phase_multiplier:.1f} ({'inverted' if self.invert_phase else 'original'})")
        print(f"   Final range: {position_final.min():.4f} to {position_final.max():.4f} rad")

        # Get time data
        time_s = df['sim_time_sec'].values - df['sim_time_sec'].values[0]

        # Compute velocity from position (same method as real data)
        velocity_final = np.zeros_like(position_final)
        dt = np.diff(time_s)
        velocity_final[:-1] = np.diff(position_final) / dt
        velocity_final[-1] = velocity_final[-2]

        # Apply Savitzky-Golay filter to smooth velocity (same as real data)
        window_length = min(51, len(velocity_final) if len(velocity_final) % 2 == 1 else len(velocity_final) - 1)
        if window_length >= 5:
            velocity_final = signal.savgol_filter(velocity_final, window_length=window_length, polyorder=3)
            print(f"   ✓ Velocity recomputed from position using numerical differentiation")
            print(f"   ✓ Applied Savitzky-Golay filter (window={window_length}, poly=3) to smooth velocity")
        else:
            print(f"   ✓ Velocity recomputed from position using numerical differentiation")

        print(f"   Velocity range: {velocity_final.min():.4f} to {velocity_final.max():.4f} rad/s")

        # Convert to numpy arrays
        df_clean = pd.DataFrame({
            'time': time_s,
            'position': position_final,
            'velocity': velocity_final,
            'acceleration': np.zeros_like(velocity_final)  # Not used for comparison
        })

        return df_clean

    def load_real_data(self, filename):
        """Load real experiment data from CSV using date/time columns"""
        df = pd.read_csv(filename)
        
        # Remove NaN values
        df_clean = df.dropna()
        
        if len(df_clean) == 0:
            raise ValueError("No valid data in real experiment file!")
        
        # Parse time from date and time columns
        print(f"\n🔍 Parsing timestamps from date/time columns:")
        
        try:
            # Combine date and time, remove trailing colon
            datetime_str = df_clean['date'] + ' ' + df_clean['time'].str.rstrip(':')
            
            # Parse to datetime
            timestamps = pd.to_datetime(datetime_str, format='%Y-%m-%d %H:%M:%S.%f')
            
            # Convert to seconds relative to first timestamp
            time_s = (timestamps - timestamps.iloc[0]).dt.total_seconds().values
            
            print(f"   ✓ Successfully parsed date/time columns")
            print(f"   Start: {df_clean['date'].iloc[0]} {df_clean['time'].iloc[0]}")
            print(f"   End:   {df_clean['date'].iloc[-1]} {df_clean['time'].iloc[-1]}")
            print(f"   Duration: {time_s[-1]:.3f} seconds")
            
        except Exception as e:
            print(f"   ⚠️  Error parsing date/time: {e}")
            print(f"   Falling back to sample numbers with 1000 Hz...")
            time_s = np.arange(len(df_clean)) / 1000.0
        
        # Use real data - compute velocity from position
        position_data = df_clean['position_rad'].values

        # Check for duplicate or invalid time values
        dt = np.diff(time_s)
        if np.any(dt <= 0):
            print(f"   ⚠️  Warning: Found {np.sum(dt <= 0)} duplicate or non-monotonic time values")
            print(f"   Cleaning time data...")
            # Remove duplicate times
            unique_mask = np.concatenate([[True], dt > 0])
            time_s = time_s[unique_mask]
            position_data = position_data[unique_mask]
            print(f"   ✓ Removed {np.sum(~unique_mask)} duplicate samples")
            print(f"   New data length: {len(time_s)}")

        # Compute velocity from position using numerical differentiation
        # Using simple forward difference for robustness
        velocity_data = np.zeros_like(position_data)
        dt = np.diff(time_s)
        velocity_data[:-1] = np.diff(position_data) / dt
        # Use last value for the final point
        velocity_data[-1] = velocity_data[-2]

        # Apply Savitzky-Golay filter to smooth velocity (removes noise from differentiation)
        # Window length must be odd and >= polynomial order + 2
        window_length = min(51, len(velocity_data) if len(velocity_data) % 2 == 1 else len(velocity_data) - 1)
        if window_length >= 5:  # Minimum window size
            velocity_data = signal.savgol_filter(velocity_data, window_length=window_length, polyorder=3)
            print(f"   ✓ Applied Savitzky-Golay filter (window={window_length}, poly=3) to smooth velocity")

        # Optionally load acceleration (or compute from velocity if needed)
        if 'acceleration_rad_s2' in df_clean.columns:
            acceleration_data = df_clean['acceleration_rad_s2'].values[unique_mask if 'unique_mask' in locals() else slice(None)]
        else:
            acceleration_data = np.zeros_like(velocity_data)
            acceleration_data[:-1] = np.diff(velocity_data) / dt
            acceleration_data[-1] = acceleration_data[-2]

        print(f"\n📐 Real data:")
        print(f"   Position range: {position_data.min():.4f} to {position_data.max():.4f} rad")
        print(f"   ✓ Velocity computed from position using numerical differentiation")
        print(f"   Velocity range: {velocity_data.min():.4f} to {velocity_data.max():.4f} rad/s")

        # Create clean dataframe
        df_clean = pd.DataFrame({
            'time': time_s,
            'position': position_data,
            'velocity': velocity_data,
            'acceleration': acceleration_data
        })
        
        return df_clean

    def find_release_point(self, data, threshold_vel=0.01, threshold_pos=0.1):
        """
        Find when the pendulum is released
        Uses POSITION change as primary indicator (more reliable than velocity)
        """
        position = data['position'].values
        velocity = data['velocity'].values
        
        # Method 1: Position-based detection (more reliable)
        # Find when position changes by more than threshold from initial
        initial_pos = position[0]
        pos_change = np.abs(position - initial_pos)
        
        # Find first point where position changed by more than threshold (5 degrees)
        pos_threshold_rad = np.radians(5.0)  # 5 degrees
        significant_motion = pos_change > pos_threshold_rad
        
        if np.any(significant_motion):
            release_idx_pos = np.argmax(significant_motion)
        else:
            release_idx_pos = 0
        
        # Method 2: Velocity-based detection (backup)
        # Use higher threshold to avoid noise
        high_vel = np.abs(velocity) > 0.5  # Increased from 0.01 to 0.5 rad/s
        
        if np.any(high_vel):
            release_idx_vel = np.argmax(high_vel)
        else:
            release_idx_vel = 0
        
        # Use position-based detection (more reliable)
        release_idx = release_idx_pos
        
        print(f"    Position-based detection: index {release_idx_pos}")
        print(f"    Velocity-based detection: index {release_idx_vel}")
        print(f"    Using: index {release_idx} (position-based)")
        
        return release_idx

    def align_trajectories(self, align_method='release'):
        """
        Align simulation and real data
        """

        if align_method == 'position':
            # Position-based alignment: trim real data to start at same position as sim
            sim_start_pos = self.sim_data['position'].iloc[0]
            real_positions = self.real_data['position'].values

            print(f"\n📍 Position-Based Alignment:")
            print(f"  Sim starting position: {sim_start_pos:.4f} rad ({np.degrees(sim_start_pos):.2f}°)")
            print(f"  Real data position range: {real_positions.min():.4f} to {real_positions.max():.4f} rad")

            # Find first index in real data where position matches sim start (within tolerance)
            tolerance = 0.05  # 0.05 rad (~3 degrees) tolerance
            position_diff = np.abs(real_positions - sim_start_pos)
            matching_indices = np.where(position_diff < tolerance)[0]

            if len(matching_indices) > 0:
                real_start_idx = matching_indices[0]
                print(f"  Found matching position at real data index {real_start_idx}")
                print(f"    Real position at match: {real_positions[real_start_idx]:.4f} rad")
                print(f"    Position difference: {position_diff[real_start_idx]:.4f} rad ({np.degrees(position_diff[real_start_idx]):.2f}°)")
            else:
                print(f"  ⚠️  No exact match found within {tolerance} rad tolerance")
                print(f"  Using closest position instead...")
                real_start_idx = np.argmin(position_diff)
                print(f"    Closest match at index {real_start_idx}")
                print(f"    Real position: {real_positions[real_start_idx]:.4f} rad")
                print(f"    Position difference: {position_diff[real_start_idx]:.4f} rad ({np.degrees(position_diff[real_start_idx]):.2f}°)")

            # Trim data
            self.sim_aligned = self.sim_data.copy()
            self.real_aligned = self.real_data.iloc[real_start_idx:].copy()

            print(f"\n⏱️  Time Alignment:")
            print(f"  Before alignment:")
            print(f"    - Sim:  t={self.sim_aligned['time'].values[0]:.3f}s to t={self.sim_aligned['time'].values[-1]:.3f}s")
            print(f"    - Real: t={self.real_aligned['time'].values[0]:.3f}s to t={self.real_aligned['time'].values[-1]:.3f}s")
            print(f"    - Real data trimmed: removed {real_start_idx} samples from beginning")

            # Reset time to 0
            self.sim_aligned['time'] = self.sim_aligned['time'].values - self.sim_aligned['time'].values[0]
            self.real_aligned['time'] = self.real_aligned['time'].values - self.real_aligned['time'].values[0]

            print(f"  After alignment (both reset to t=0):")
            print(f"    - Sim:  t={self.sim_aligned['time'].values[0]:.3f}s to t={self.sim_aligned['time'].values[-1]:.3f}s")
            print(f"    - Real: t={self.real_aligned['time'].values[0]:.3f}s to t={self.real_aligned['time'].values[-1]:.3f}s")
            print(f"  ✓ Both datasets now start at same position and t=0.000s")

        elif align_method == 'release':
            # Find release points
            sim_release = self.find_release_point(self.sim_data)
            real_release = self.find_release_point(self.real_data)

            print(f"\n📍 Release Point Detection:")
            print(f"  Sim:")
            print(f"    - Release at index {sim_release}")
            print(f"    - Original time: {self.sim_data['time'].iloc[sim_release]:.3f}s")
            print(f"    - Position: {self.sim_data['position'].iloc[sim_release]:.4f} rad")
            print(f"  Real:")
            print(f"    - Release at index {real_release}")
            print(f"    - Original time: {self.real_data['time'].iloc[real_release]:.3f}s")
            print(f"    - Position: {self.real_data['position'].iloc[real_release]:.4f} rad (after offset)")

            # Trim data before release
            self.sim_aligned = self.sim_data.iloc[sim_release:].copy()
            self.real_aligned = self.real_data.iloc[real_release:].copy()

            print(f"\n⏱️  Time Alignment:")
            print(f"  Before alignment:")
            print(f"    - Sim:  t={self.sim_aligned['time'].values[0]:.3f}s to t={self.sim_aligned['time'].values[-1]:.3f}s")
            print(f"    - Real: t={self.real_aligned['time'].values[0]:.3f}s to t={self.real_aligned['time'].values[-1]:.3f}s")

            # Reset time to 0 at release
            self.sim_aligned['time'] = self.sim_aligned['time'].values - self.sim_aligned['time'].values[0]
            self.real_aligned['time'] = self.real_aligned['time'].values - self.real_aligned['time'].values[0]

            print(f"  After alignment (both reset to t=0):")
            print(f"    - Sim:  t={self.sim_aligned['time'].values[0]:.3f}s to t={self.sim_aligned['time'].values[-1]:.3f}s")
            print(f"    - Real: t={self.real_aligned['time'].values[0]:.3f}s to t={self.real_aligned['time'].values[-1]:.3f}s")
            print(f"  ✓ Both datasets now start at t=0.000s (synchronized)")

        print(f"\nAligned data lengths: Sim={len(self.sim_aligned)}, Real={len(self.real_aligned)}")

    def interpolate_to_common_time(self):
        """
        Interpolate both datasets to a common time vector
        """
        # Create common time vector
        max_time = min(self.sim_aligned['time'].max(), self.real_aligned['time'].max())
        self.common_time = np.linspace(0, max_time, 1000)
        
        # Interpolate sim data
        sim_pos_interp = np.interp(self.common_time, 
                                    self.sim_aligned['time'].values, 
                                    self.sim_aligned['position'].values)
        sim_vel_interp = np.interp(self.common_time, 
                                    self.sim_aligned['time'].values, 
                                    self.sim_aligned['velocity'].values)
        
        # Interpolate real data
        real_pos_interp = np.interp(self.common_time, 
                                     self.real_aligned['time'].values, 
                                     self.real_aligned['position'].values)
        real_vel_interp = np.interp(self.common_time, 
                                     self.real_aligned['time'].values, 
                                     self.real_aligned['velocity'].values)
        
        self.sim_interp = pd.DataFrame({
            'time': self.common_time,
            'position': sim_pos_interp,
            'velocity': sim_vel_interp
        })
        
        self.real_interp = pd.DataFrame({
            'time': self.common_time,
            'position': real_pos_interp,
            'velocity': real_vel_interp
        })

    def calculate_errors(self):
        """Calculate error metrics"""
        
        # Position errors
        pos_error = self.sim_interp['position'].values - self.real_interp['position'].values
        self.pos_rmse = np.sqrt(np.mean(pos_error**2))
        self.pos_max_error = np.max(np.abs(pos_error))
        self.pos_mean_error = np.mean(pos_error)
        self.pos_std_error = np.std(pos_error)
        
        # Velocity errors
        vel_error = self.sim_interp['velocity'].values - self.real_interp['velocity'].values
        self.vel_rmse = np.sqrt(np.mean(vel_error**2))
        self.vel_max_error = np.max(np.abs(vel_error))
        self.vel_mean_error = np.mean(vel_error)
        self.vel_std_error = np.std(vel_error)
        
        # Percentage errors (relative to range)
        pos_range = self.real_interp['position'].max() - self.real_interp['position'].min()
        self.pos_percent_error = (self.pos_rmse / pos_range) * 100 if pos_range > 0 else 0
        
        vel_range = self.real_interp['velocity'].max() - self.real_interp['velocity'].min()
        self.vel_percent_error = (self.vel_rmse / vel_range) * 100 if vel_range > 0 else 0

    def print_metrics(self):
        """Print comparison metrics"""
        print("\n" + "="*60)
        print("COMPARISON METRICS - KNEE LINK (RADIANS)")
        print("="*60)

        # Calculate oscillation characteristics
        sim_pos = self.sim_aligned['position'].values
        real_pos = self.real_aligned['position'].values
        sim_vel = self.sim_aligned['velocity'].values
        real_vel = self.real_aligned['velocity'].values

        # Find peaks to estimate frequency
        from scipy.signal import find_peaks
        sim_peaks, _ = find_peaks(sim_pos, distance=20)
        real_peaks, _ = find_peaks(real_pos, distance=20)

        sim_freq = 0
        real_freq = 0
        if len(sim_peaks) > 1:
            sim_period = np.mean(np.diff(self.sim_aligned['time'].values[sim_peaks]))
            sim_freq = 1.0 / sim_period if sim_period > 0 else 0
        if len(real_peaks) > 1:
            real_period = np.mean(np.diff(self.real_aligned['time'].values[real_peaks]))
            real_freq = 1.0 / real_period if real_period > 0 else 0

        print("\nOSCILLATION DYNAMICS:")
        print(f"  Sim frequency:  {sim_freq:.3f} Hz (period: {1/sim_freq if sim_freq > 0 else 0:.3f} s)")
        print(f"  Real frequency: {real_freq:.3f} Hz (period: {1/real_freq if real_freq > 0 else 0:.3f} s)")
        print(f"  Frequency ratio: {real_freq/sim_freq if sim_freq > 0 else 0:.2f}× (real/sim)")
        print(f"  Sim velocity peak: {np.max(np.abs(sim_vel)):.2f} rad/s")
        print(f"  Real velocity peak: {np.max(np.abs(real_vel)):.2f} rad/s")
        print(f"  Velocity ratio: {np.max(np.abs(real_vel))/np.max(np.abs(sim_vel)) if np.max(np.abs(sim_vel)) > 0 else 0:.2f}× (real/sim)")

        print("\nPOSITION ERRORS:")
        print(f"  RMSE:       {self.pos_rmse:.6f} rad")
        print(f"  Max Error:  {self.pos_max_error:.6f} rad")
        print(f"  Mean Error: {self.pos_mean_error:.6f} rad")
        print(f"  Std Dev:    {self.pos_std_error:.6f} rad")
        print(f"  Percent:    {self.pos_percent_error:.2f}%")

        print("\nVELOCITY ERRORS:")
        print(f"  RMSE:       {self.vel_rmse:.6f} rad/s")
        print(f"  Max Error:  {self.vel_max_error:.6f} rad/s")
        print(f"  Mean Error: {self.vel_mean_error:.6f} rad/s")
        print(f"  Std Dev:    {self.vel_std_error:.6f} rad/s")
        print(f"  Percent:    {self.vel_percent_error:.2f}%")

        print("\n" + "="*60)

        # Goal check (< 10% error)
        print("\nGOAL CHECK (< 10% error for Sim-to-Real transfer):")
        if self.pos_percent_error < 10.0:
            print(f"  ✓ Position error: {self.pos_percent_error:.2f}% < 10% PASS!")
        else:
            print(f"  ✗ Position error: {self.pos_percent_error:.2f}% > 10% FAIL!")

        if self.vel_percent_error < 10.0:
            print(f"  ✓ Velocity error: {self.vel_percent_error:.2f}% < 10% PASS!")
        else:
            print(f"  ✗ Velocity error: {self.vel_percent_error:.2f}% > 10% FAIL!")

        print("="*60)

    def plot_comparison(self):
        """Create comparison plots"""
        
        fig, axes = plt.subplots(3, 2, figsize=(14, 12))

        # Build subtitle based on transformations
        subtitle_parts = []
        if self.offset_rad != 0:
            subtitle_parts.append(f"offset {self.offset_rad:+.3f} rad")
        if self.invert_phase:
            subtitle_parts.append("inverted phase")
        subtitle = f"(Sim data: {', '.join(subtitle_parts)})" if subtitle_parts else "(No transformations)"

        fig.suptitle(f'Simulation vs Real Experiment Comparison - Knee Link\n{subtitle}',
                     fontsize=16, fontweight='bold')
        
        # Convert to numpy arrays
        sim_time = self.sim_aligned['time'].values
        sim_pos = self.sim_aligned['position'].values
        sim_vel = self.sim_aligned['velocity'].values
        
        real_time = self.real_aligned['time'].values
        real_pos = self.real_aligned['position'].values
        real_vel = self.real_aligned['velocity'].values
        
        sim_interp_pos = self.sim_interp['position'].values
        sim_interp_vel = self.sim_interp['velocity'].values
        real_interp_pos = self.real_interp['position'].values
        real_interp_vel = self.real_interp['velocity'].values

        # Show full graph without trimming
        max_plot_time = max(sim_time[-1], real_time[-1])

        # Position comparison
        ax = axes[0, 0]
        ax.plot(sim_time, sim_pos, 'b-', label='Simulation', linewidth=2)
        ax.plot(real_time, real_pos, 'r--', label='Real', linewidth=2, alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position (rad)')
        ax.set_title('Position Trajectory (Full Data)')
        ax.legend()
        ax.grid(True, alpha=0.3)

        # Velocity comparison
        ax = axes[1, 0]
        ax.plot(sim_time, sim_vel, 'b-', label='Simulation', linewidth=2)
        ax.plot(real_time, real_vel, 'r--', label='Real', linewidth=2, alpha=0.7)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title('Velocity Trajectory (Full Data)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Position error
        ax = axes[0, 1]
        pos_error = sim_interp_pos - real_interp_pos
        ax.plot(self.common_time, pos_error, 'k-', linewidth=1.5)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax.fill_between(self.common_time, 0, pos_error, alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Position Error (rad)')
        ax.set_title(f'Position Error (RMSE: {self.pos_rmse:.4f} rad)')
        ax.grid(True, alpha=0.3)
        
        # Velocity error
        ax = axes[1, 1]
        vel_error = sim_interp_vel - real_interp_vel
        ax.plot(self.common_time, vel_error, 'k-', linewidth=1.5)
        ax.axhline(y=0, color='gray', linestyle='--', alpha=0.5)
        ax.fill_between(self.common_time, 0, vel_error, alpha=0.3)
        ax.set_xlabel('Time (s)')
        ax.set_ylabel('Velocity Error (rad/s)')
        ax.set_title(f'Velocity Error (RMSE: {self.vel_rmse:.4f} rad/s)')
        ax.grid(True, alpha=0.3)
        
        # Phase plot
        ax = axes[2, 0]
        ax.plot(sim_pos, sim_vel, 'b-', label='Simulation', linewidth=2, alpha=0.7)
        ax.plot(real_pos, real_vel, 'r-', label='Real', linewidth=2, alpha=0.7)
        ax.set_xlabel('Position (rad)')
        ax.set_ylabel('Velocity (rad/s)')
        ax.set_title('Phase Portrait (Full Data)')
        ax.legend()
        ax.grid(True, alpha=0.3)
        
        # Error summary
        ax = axes[2, 1]
        ax.axis('off')
        summary_text = f"""
ERROR METRICS:

Position RMSE: {self.pos_rmse:.4f} rad
Position Max:  {self.pos_max_error:.4f} rad
Position %:    {self.pos_percent_error:.2f}%

Velocity RMSE: {self.vel_rmse:.4f} rad/s
Velocity Max:  {self.vel_max_error:.4f} rad/s
Velocity %:    {self.vel_percent_error:.2f}%

Goal: < 10% error
Position: {"✓ PASS" if self.pos_percent_error < 10 else "✗ FAIL"}
Velocity: {"✓ PASS" if self.vel_percent_error < 10 else "✗ FAIL"}

Transformations:
  Offset: {self.offset_rad:+.3f} rad
  Invert: {'Yes' if self.invert_phase else 'No'}

Sim start:  {sim_pos[0]:.4f} rad
Real start: {real_pos[0]:.4f} rad
        """
        ax.text(0.1, 0.5, summary_text, fontsize=11, family='monospace',
                verticalalignment='center')
        
        plt.tight_layout()
        
        # Save figure
        output_file = os.path.join(self.output_dir, 'sim_vs_real_comparison_offset.png')
        plt.savefig(output_file, dpi=150, bbox_inches='tight')
        print(f"\n📊 Plot saved to: {output_file}")

    def run_analysis(self):
        """Run complete comparison analysis"""
        print("\n" + "="*60)
        print("STARTING COMPARISON ANALYSIS - KNEE LINK (RADIANS)")
        print("="*60)

        print("\n1. Aligning trajectories by matching positions...")
        # Align both datasets by matching starting positions (trim real data)
        self.align_trajectories(align_method='position')

        print("\n2. Interpolating to common time grid...")
        self.interpolate_to_common_time()

        print("\n3. Calculating error metrics...")
        self.calculate_errors()

        print("\n4. Generating comparison plots...")
        self.plot_comparison()

        self.print_metrics()

        return {
            'pos_rmse': self.pos_rmse,
            'pos_percent': self.pos_percent_error,
            'vel_rmse': self.vel_rmse,
            'vel_percent': self.vel_percent_error
        }


def main():
    """Main function"""

    # ========================================
    # 🔧 CONFIGURATION - ADJUST THESE VALUES
    # ========================================

    # File paths
    SIM_FILE = "/home/prime/physical_ai_locomotion_team/sim_signal/world_to_hip_sim_20251221_141131.csv"
    REAL_FILE_OPTIONS = [
        "/home/prime/physical_ai_locomotion_team/src/pos_raw_signal/hip_inertia_0deg.csv"
    ]

    # Transformation settings
    OFFSET_RAD = 0.0       # Position offset in radians (should be 0 now that coordinate frames match)
    INVERT_PHASE = False    # Set to False - coordinate frames now match (0=horizontal, -π/2=vertical)

    # ========================================

    print("="*60)
    print("SIMULATION VS REAL COMPARISON - HIP LINK (RADIANS)")
    print("="*60)
    print()
    
    # Check files
    print("Checking files...")
    if not os.path.exists(SIM_FILE):
        print(f"❌ ERROR: Simulation file not found: {SIM_FILE}")
        return
    else:
        print(f"✓ Simulation file: {SIM_FILE}")
    
    REAL_FILE = None
    for path in REAL_FILE_OPTIONS:
        if os.path.exists(path):
            REAL_FILE = path
            print(f"✓ Real experiment file: {REAL_FILE}")
            break
    
    if REAL_FILE is None:
        print(f"❌ ERROR: Real experiment file not found!")
        return
    
    print()

    # Run comparison with configured settings
    try:
        comparison = SimRealComparison(SIM_FILE, REAL_FILE, OUTPUT_DIR,
                                      offset_rad=OFFSET_RAD,
                                      invert_phase=INVERT_PHASE)
        metrics = comparison.run_analysis()
        
        print("\n✅ Analysis complete!")
        print()
        print("Output files:")
        print(f"  📊 Plot: {OUTPUT_DIR}/sim_vs_real_comparison_offset.png")
        print()
        print("View plot:")
        print(f"  xdg-open {OUTPUT_DIR}/sim_vs_real_comparison_offset.png")
        print()
        
        return metrics
        
    except Exception as e:
        print(f"\n❌ ERROR during comparison:")
        print(f"   {str(e)}")
        import traceback
        traceback.print_exc()
        return None


if __name__ == '__main__':
    main()