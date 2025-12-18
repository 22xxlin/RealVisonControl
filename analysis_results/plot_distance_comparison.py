#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Plot visual vs actual distance comparison for vehicles and balls in VSWARM10 dataset
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import glob
import os

def load_all_csv_files(directory):
    """Load all CSV files in the directory"""
    csv_files = glob.glob(os.path.join(directory, "log_sysID_*.csv"))
    print(f"Found {len(csv_files)} CSV files:")
    for f in csv_files:
        print(f"  - {os.path.basename(f)}")

    all_data = []
    for csv_file in csv_files:
        try:
            df = pd.read_csv(csv_file)
            df['source_file'] = os.path.basename(csv_file)
            all_data.append(df)
            print(f"Loaded {os.path.basename(csv_file)}: {len(df)} records")
        except Exception as e:
            print(f"Error loading {csv_file}: {e}")

    if all_data:
        combined_df = pd.concat(all_data, ignore_index=True)
        print(f"\nTotal records loaded: {len(combined_df)}")
        return combined_df
    else:
        return None

def plot_distance_comparison(data, target_type, title, filename):
    """Plot distance comparison chart with separate rows for each CSV-Camera combination"""
    filtered_data = data[data['Target_Type'].str.strip() == target_type].copy()

    if len(filtered_data) == 0:
        print(f"Warning: No data found for target type '{target_type}'")
        return

    csv_files = sorted(filtered_data['source_file'].unique())
    cameras = sorted(filtered_data['Cam_ID'].unique())

    combinations = []
    for csv_file in csv_files:
        for cam_id in cameras:
            subset = filtered_data[(filtered_data['source_file'] == csv_file) &
                                  (filtered_data['Cam_ID'] == cam_id)]
            if len(subset) > 0:
                combinations.append((csv_file, cam_id, subset))

    n_rows = len(combinations)
    fig, axes = plt.subplots(n_rows, 2, figsize=(16, 4*n_rows))
    if n_rows == 1:
        axes = axes.reshape(1, -1)

    colors = ['blue', 'green', 'orange', 'purple', 'red', 'brown']

    for idx, (csv_file, cam_id, subset) in enumerate(combinations):
        print(f"{csv_file} - Cam{cam_id} - {target_type}: {len(subset)} records")

        axes[idx, 0].scatter(subset['Vicon_Dist'], subset['Vision_Dist'],
                            alpha=0.4, s=15, c=colors[idx % len(colors)])

        min_d = min(subset['Vicon_Dist'].min(), subset['Vision_Dist'].min())
        max_d = max(subset['Vicon_Dist'].max(), subset['Vision_Dist'].max())
        axes[idx, 0].plot([min_d, max_d], [min_d, max_d], 'r--', linewidth=2)
        axes[idx, 0].set_xlabel('Actual Distance [m]', fontsize=10)
        axes[idx, 0].set_ylabel('Vision Distance [m]', fontsize=10)
        axes[idx, 0].set_title(f'{csv_file} - Cam{cam_id}', fontsize=11, fontweight='bold')
        axes[idx, 0].grid(True, alpha=0.3)
        axes[idx, 0].set_aspect('equal', adjustable='box')

        time_norm = subset['Time'] - subset['Time'].min()
        axes[idx, 1].plot(time_norm, subset['Vision_Dist'], 'b-', alpha=0.6, linewidth=0.8, label='Vision')
        axes[idx, 1].plot(time_norm, subset['Vicon_Dist'], 'r-', alpha=0.6, linewidth=0.8, label='Actual')
        axes[idx, 1].set_xlabel('Time [s]', fontsize=10)
        axes[idx, 1].set_ylabel('Distance [m]', fontsize=10)
        axes[idx, 1].set_title(f'{csv_file} - Cam{cam_id}', fontsize=11, fontweight='bold')
        axes[idx, 1].legend(fontsize=8)
        axes[idx, 1].grid(True, alpha=0.3)

    plt.suptitle(title, fontsize=16, fontweight='bold', y=0.998)
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"\nPlot saved: {filename}")
    plt.show()

def main():
    script_dir = os.path.dirname(os.path.abspath(__file__))
    os.chdir(script_dir)

    print("=" * 80)
    print("VSWARM10 Distance Comparison Analysis (Separated by CSV and Camera)")
    print("=" * 80)

    data = load_all_csv_files(script_dir)

    if data is None or len(data) == 0:
        print("Error: No data loaded")
        return

    print(f"\nTarget types: {data['Target_Type'].unique()}")
    print(f"CSV files: {sorted(data['source_file'].unique())}")
    print(f"Camera IDs: {sorted(data['Cam_ID'].unique())}")

    plot_distance_comparison(data, 'CAR',
                            'Vehicles: Vision vs Actual Distance (by CSV & Camera)',
                            'car_distance_comparison.png')

    plot_distance_comparison(data, 'BALL',
                            'Balls: Vision vs Actual Distance (by CSV & Camera)',
                            'ball_distance_comparison.png')

    print("\n" + "=" * 80)
    print("Analysis Complete!")
    print("=" * 80)

if __name__ == "__main__":
    main()

