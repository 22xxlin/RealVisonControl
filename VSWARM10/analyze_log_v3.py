#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Comprehensive analysis for log_v3_20251218_234632.csv
"""

import pandas as pd
import matplotlib.pyplot as plt
import numpy as np

# Load data
csv_file = "log_v3_20251218_234632.csv"
df = pd.read_csv(csv_file)

print("=" * 80)
print(f"Analysis of {csv_file}")
print("=" * 80)

# Basic statistics
print(f"\nTotal records: {len(df)}")
print(f"Time span: {df['Time'].max() - df['Time'].min():.2f} seconds")
print(f"\nColumns: {list(df.columns)}")

# Target type distribution
print("\n" + "=" * 80)
print("TARGET TYPE DISTRIBUTION")
print("=" * 80)
target_counts = df['Target_Type'].value_counts()
for target, count in target_counts.items():
    print(f"{target.strip()}: {count} records ({count/len(df)*100:.1f}%)")

# Camera distribution
print("\n" + "=" * 80)
print("CAMERA DISTRIBUTION")
print("=" * 80)
cam_counts = df['Cam_ID'].value_counts().sort_index()
for cam, count in cam_counts.items():
    print(f"Camera {cam}: {count} records ({count/len(df)*100:.1f}%)")

# Vision class distribution
print("\n" + "=" * 80)
print("VISION CLASS DISTRIBUTION")
print("=" * 80)
class_counts = df['Vision_Class'].value_counts().sort_index()
for cls, count in class_counts.items():
    print(f"Class {cls}: {count} records ({count/len(df)*100:.1f}%)")

# Distance error analysis
print("\n" + "=" * 80)
print("DISTANCE ERROR ANALYSIS")
print("=" * 80)
for target in df['Target_Type'].unique():
    target_data = df[df['Target_Type'] == target]
    print(f"\n{target.strip()}:")
    print(f"  Vision Distance: {target_data['Vision_Dist'].mean():.3f} ± {target_data['Vision_Dist'].std():.3f} m")
    print(f"  Vicon Distance:  {target_data['Vicon_Dist'].mean():.3f} ± {target_data['Vicon_Dist'].std():.3f} m")
    print(f"  Distance Error:  {target_data['Dist_Err'].mean():.3f} ± {target_data['Dist_Err'].std():.3f} m")
    print(f"  Error range: [{target_data['Dist_Err'].min():.3f}, {target_data['Dist_Err'].max():.3f}] m")

# Angle error analysis
print("\n" + "=" * 80)
print("ANGLE ERROR ANALYSIS")
print("=" * 80)
for target in df['Target_Type'].unique():
    target_data = df[df['Target_Type'] == target]
    print(f"\n{target.strip()}:")
    print(f"  Angle Error: {target_data['Ang_Err'].mean():.2f} ± {target_data['Ang_Err'].std():.2f} deg")
    print(f"  Error range: [{target_data['Ang_Err'].min():.2f}, {target_data['Ang_Err'].max():.2f}] deg")

# Robot speed analysis
print("\n" + "=" * 80)
print("ROBOT SPEED ANALYSIS")
print("=" * 80)
print(f"Speed range: [{df['Robot_Speed'].min():.3f}, {df['Robot_Speed'].max():.3f}] m/s")
print(f"Mean speed: {df['Robot_Speed'].mean():.3f} m/s")
print(f"Speed > 0.1 m/s: {len(df[df['Robot_Speed'] > 0.1])} records ({len(df[df['Robot_Speed'] > 0.1])/len(df)*100:.1f}%)")

# Camera-Target breakdown
print("\n" + "=" * 80)
print("CAMERA-TARGET BREAKDOWN")
print("=" * 80)
for cam in sorted(df['Cam_ID'].unique()):
    print(f"\nCamera {cam}:")
    cam_data = df[df['Cam_ID'] == cam]
    for target in cam_data['Target_Type'].unique():
        count = len(cam_data[cam_data['Target_Type'] == target])
        print(f"  {target.strip()}: {count} records")

print("\n" + "=" * 80)
print("Analysis Complete!")
print("=" * 80)

