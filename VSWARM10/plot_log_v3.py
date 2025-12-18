#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("log_v3_20251218_234632.csv")
df['source_file'] = 'log_v3_20251218_234632.csv'

def plot_target(data, target_type, filename):
    filtered = data[data['Target_Type'].str.strip() == target_type].copy()
    if len(filtered) == 0:
        return
    
    cameras = sorted(filtered['Cam_ID'].unique())
    n_rows = len(cameras)
    
    fig, axes = plt.subplots(n_rows, 2, figsize=(16, 4*n_rows))
    if n_rows == 1:
        axes = axes.reshape(1, -1)
    
    colors = ['blue', 'green', 'orange']
    
    for idx, cam_id in enumerate(cameras):
        subset = filtered[filtered['Cam_ID'] == cam_id]
        print(f"Cam{cam_id} - {target_type}: {len(subset)} records")
        
        # Scatter
        axes[idx, 0].scatter(subset['Vicon_Dist'], subset['Vision_Dist'], 
                            alpha=0.4, s=15, c=colors[idx % 3])
        min_d = min(subset['Vicon_Dist'].min(), subset['Vision_Dist'].min())
        max_d = max(subset['Vicon_Dist'].max(), subset['Vision_Dist'].max())
        axes[idx, 0].plot([min_d, max_d], [min_d, max_d], 'r--', linewidth=2)
        axes[idx, 0].set_xlabel('Actual Distance [m]', fontsize=10)
        axes[idx, 0].set_ylabel('Vision Distance [m]', fontsize=10)
        axes[idx, 0].set_title(f'Cam{cam_id} - Scatter', fontsize=11, fontweight='bold')
        axes[idx, 0].grid(True, alpha=0.3)
        axes[idx, 0].set_aspect('equal', adjustable='box')
        
        # Time series
        time_norm = subset['Time'] - subset['Time'].min()
        axes[idx, 1].plot(time_norm, subset['Vision_Dist'], 'b-', alpha=0.6, linewidth=0.8, label='Vision')
        axes[idx, 1].plot(time_norm, subset['Vicon_Dist'], 'r-', alpha=0.6, linewidth=0.8, label='Actual')
        axes[idx, 1].set_xlabel('Time [s]', fontsize=10)
        axes[idx, 1].set_ylabel('Distance [m]', fontsize=10)
        axes[idx, 1].set_title(f'Cam{cam_id} - Time Series', fontsize=11, fontweight='bold')
        axes[idx, 1].legend(fontsize=8)
        axes[idx, 1].grid(True, alpha=0.3)
    
    plt.suptitle(f'{target_type}: Vision vs Actual Distance (by Camera)', fontsize=16, fontweight='bold', y=0.998)
    plt.tight_layout()
    plt.savefig(filename, dpi=300, bbox_inches='tight')
    print(f"Saved: {filename}\n")
    plt.show()

print("="*60)
plot_target(df, 'CAR', 'log_v3_car_comparison.png')
plot_target(df, 'BALL', 'log_v3_ball_comparison.png')
print("="*60)

