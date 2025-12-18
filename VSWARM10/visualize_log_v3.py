#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Visualization for log_v3_20251218_234632.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data
df = pd.read_csv("log_v3_20251218_234632.csv")

# Create comprehensive visualization
fig = plt.figure(figsize=(18, 12))

# 1. Distance comparison scatter plot
ax1 = plt.subplot(3, 3, 1)
car_data = df[df['Target_Type'].str.strip() == 'CAR']
ball_data = df[df['Target_Type'].str.strip() == 'BALL']
ax1.scatter(car_data['Vicon_Dist'], car_data['Vision_Dist'], alpha=0.4, s=20, c='blue', label='CAR')
ax1.scatter(ball_data['Vicon_Dist'], ball_data['Vision_Dist'], alpha=0.4, s=20, c='red', label='BALL')
max_d = max(df['Vicon_Dist'].max(), df['Vision_Dist'].max())
ax1.plot([0, max_d], [0, max_d], 'k--', linewidth=1, alpha=0.5)
ax1.set_xlabel('Vicon Distance [m]')
ax1.set_ylabel('Vision Distance [m]')
ax1.set_title('Distance: Vision vs Vicon')
ax1.legend()
ax1.grid(True, alpha=0.3)

# 2. Distance error distribution
ax2 = plt.subplot(3, 3, 2)
ax2.hist(car_data['Dist_Err'], bins=30, alpha=0.6, color='blue', label='CAR')
ax2.hist(ball_data['Dist_Err'], bins=30, alpha=0.6, color='red', label='BALL')
ax2.axvline(0, color='k', linestyle='--', linewidth=1)
ax2.set_xlabel('Distance Error [m]')
ax2.set_ylabel('Count')
ax2.set_title('Distance Error Distribution')
ax2.legend()
ax2.grid(True, alpha=0.3)

# 3. Angle error distribution
ax3 = plt.subplot(3, 3, 3)
ax3.hist(car_data['Ang_Err'], bins=30, alpha=0.6, color='blue', label='CAR')
ax3.hist(ball_data['Ang_Err'], bins=30, alpha=0.6, color='red', label='BALL')
ax3.axvline(0, color='k', linestyle='--', linewidth=1)
ax3.set_xlabel('Angle Error [deg]')
ax3.set_ylabel('Count')
ax3.set_title('Angle Error Distribution')
ax3.legend()
ax3.grid(True, alpha=0.3)

# 4. Distance error vs actual distance
ax4 = plt.subplot(3, 3, 4)
ax4.scatter(car_data['Vicon_Dist'], car_data['Dist_Err'], alpha=0.4, s=20, c='blue', label='CAR')
ax4.scatter(ball_data['Vicon_Dist'], ball_data['Dist_Err'], alpha=0.4, s=20, c='red', label='BALL')
ax4.axhline(0, color='k', linestyle='--', linewidth=1)
ax4.set_xlabel('Vicon Distance [m]')
ax4.set_ylabel('Distance Error [m]')
ax4.set_title('Error vs Distance')
ax4.legend()
ax4.grid(True, alpha=0.3)

# 5. Distance error vs robot speed
ax5 = plt.subplot(3, 3, 5)
ax5.scatter(car_data['Robot_Speed'], car_data['Dist_Err'], alpha=0.4, s=20, c='blue', label='CAR')
ax5.scatter(ball_data['Robot_Speed'], ball_data['Dist_Err'], alpha=0.4, s=20, c='red', label='BALL')
ax5.axhline(0, color='k', linestyle='--', linewidth=1)
ax5.set_xlabel('Robot Speed [m/s]')
ax5.set_ylabel('Distance Error [m]')
ax5.set_title('Error vs Robot Speed')
ax5.legend()
ax5.grid(True, alpha=0.3)

# 6. Time series - distance
ax6 = plt.subplot(3, 3, 6)
time_norm = df['Time'] - df['Time'].min()
ax6.plot(time_norm, df['Vision_Dist'], 'b.', alpha=0.3, markersize=2, label='Vision')
ax6.plot(time_norm, df['Vicon_Dist'], 'r.', alpha=0.3, markersize=2, label='Vicon')
ax6.set_xlabel('Time [s]')
ax6.set_ylabel('Distance [m]')
ax6.set_title('Distance Time Series')
ax6.legend()
ax6.grid(True, alpha=0.3)

# 7. Camera distribution pie chart
ax7 = plt.subplot(3, 3, 7)
cam_counts = df['Cam_ID'].value_counts()
ax7.pie(cam_counts.values, labels=[f'Cam {c}' for c in cam_counts.index], autopct='%1.1f%%')
ax7.set_title('Camera Distribution')

# 8. Target type by camera
ax8 = plt.subplot(3, 3, 8)
cam_target = df.groupby(['Cam_ID', 'Target_Type']).size().unstack(fill_value=0)
cam_target.plot(kind='bar', ax=ax8, color=['blue', 'red'])
ax8.set_xlabel('Camera ID')
ax8.set_ylabel('Count')
ax8.set_title('Target Type by Camera')
ax8.legend(['CAR', 'BALL'])
ax8.grid(True, alpha=0.3, axis='y')

# 9. Vision class distribution
ax9 = plt.subplot(3, 3, 9)
class_counts = df['Vision_Class'].value_counts().sort_index()
ax9.bar(class_counts.index, class_counts.values, color=['red', 'orange', 'green'])
ax9.set_xlabel('Vision Class')
ax9.set_ylabel('Count')
ax9.set_title('Vision Class Distribution')
ax9.grid(True, alpha=0.3, axis='y')

plt.suptitle('log_v3_20251218_234632.csv - Comprehensive Analysis', fontsize=16, fontweight='bold')
plt.tight_layout()
plt.savefig('log_v3_analysis.png', dpi=300, bbox_inches='tight')
print("Visualization saved: log_v3_analysis.png")
plt.show()

