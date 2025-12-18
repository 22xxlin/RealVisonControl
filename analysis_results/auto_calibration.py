import pandas as pd
import numpy as np
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# 读取数据
df = pd.read_csv('log_sysID_20251218_173613.csv')
car_data = df[df['Target_Type'].str.strip() == 'CAR'].copy()
ball_data = df[df['Target_Type'].str.strip() == 'BALL'].copy()

print("="*80)
print("自动标定程序 - 基于VICON真实值")
print("="*80)

# 标定函数: Distance = k / Width
def distance_model(width, k):
    return k / width

# 为每个相机标定小车参数
print("\n【小车标定】")
print("-"*80)

car_calibration = {}
for cam_id in sorted(car_data['Cam_ID'].unique()):
    cam_data = car_data[car_data['Cam_ID'] == cam_id]
    
    # 过滤异常值
    cam_data = cam_data[cam_data['Vision_Width'] > 0.5]
    cam_data = cam_data[cam_data['Vicon_Dist'] < 5.0]
    
    widths = cam_data['Vision_Width'].values
    distances = cam_data['Vicon_Dist'].values
    
    # 拟合 k = fx × real_width
    try:
        popt, pcov = curve_fit(distance_model, widths, distances)
        k_optimal = popt[0]
        
        # 计算误差
        predicted = distance_model(widths, k_optimal)
        errors = distances - predicted
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        
        # 假设 real_width = 0.31m，反推 fx
        real_width_car = 0.31
        fx_optimal = k_optimal / real_width_car
        
        car_calibration[cam_id] = {
            'k': k_optimal,
            'fx': fx_optimal,
            'rmse': rmse,
            'mae': mae,
            'n_samples': len(cam_data)
        }
        
        print(f"\nCam {cam_id} (n={len(cam_data)}):")
        print(f"  最优 k = {k_optimal:.4f}")
        print(f"  推算 fx = {fx_optimal:.2f} (假设 real_width=0.31m)")
        print(f"  RMSE = {rmse:.4f} m")
        print(f"  MAE = {mae:.4f} m")
        print(f"  当前fx=498.0的修正系数 = {fx_optimal/498.0:.4f}")
        
    except Exception as e:
        print(f"\nCam {cam_id}: 标定失败 - {e}")

# 为每个相机标定球参数
print("\n" + "="*80)
print("【球标定】")
print("-"*80)

ball_calibration = {}
for cam_id in sorted(ball_data['Cam_ID'].unique()):
    cam_data = ball_data[ball_data['Cam_ID'] == cam_id]
    
    # 过滤异常值
    cam_data = cam_data[cam_data['Vision_Width'] > 0.4]
    cam_data = cam_data[cam_data['Vicon_Dist'] < 2.0]
    
    widths = cam_data['Vision_Width'].values
    distances = cam_data['Vicon_Dist'].values
    
    try:
        popt, pcov = curve_fit(distance_model, widths, distances)
        k_optimal = popt[0]
        
        predicted = distance_model(widths, k_optimal)
        errors = distances - predicted
        rmse = np.sqrt(np.mean(errors**2))
        mae = np.mean(np.abs(errors))
        
        # 假设 real_width = 0.23m，反推 fx
        real_width_ball = 0.23
        fx_optimal = k_optimal / real_width_ball
        
        ball_calibration[cam_id] = {
            'k': k_optimal,
            'fx': fx_optimal,
            'rmse': rmse,
            'mae': mae,
            'n_samples': len(cam_data)
        }
        
        print(f"\nCam {cam_id} (n={len(cam_data)}):")
        print(f"  最优 k = {k_optimal:.4f}")
        print(f"  推算 fx = {fx_optimal:.2f} (假设 real_width=0.23m)")
        print(f"  RMSE = {rmse:.4f} m")
        print(f"  MAE = {mae:.4f} m")
        print(f"  当前fx=498.0的修正系数 = {fx_optimal/498.0:.4f}")
        
    except Exception as e:
        print(f"\nCam {cam_id}: 标定失败 - {e}")

# 生成配置代码
print("\n" + "="*80)
print("【生成的配置代码】")
print("="*80)
print("\n# 方案A: 简单修正系数")
print("CAMERA_CORRECTION_FACTORS = {")
for cam_id in sorted(set(list(car_calibration.keys()) + list(ball_calibration.keys()))):
    car_factor = car_calibration.get(cam_id, {}).get('fx', 498.0) / 498.0
    ball_factor = ball_calibration.get(cam_id, {}).get('fx', 498.0) / 498.0
    print(f"    {cam_id}: {{'car': {car_factor:.4f}, 'ball': {ball_factor:.4f}}},")
print("}")

print("\n# 方案B: 每个相机独立内参")
print("CAM_INTRINSICS_PER_CAMERA = {")
for cam_id in sorted(car_calibration.keys()):
    fx = car_calibration[cam_id]['fx']
    print(f"    {cam_id}: {{'fx': {fx:.2f}, 'fy': {fx:.2f}, 'cx': 331.28, 'cy': 156.14}},")
print("}")

print("\n# 方案C: 每个相机每个目标的k值")
print("DISTANCE_K_VALUES = {")
for cam_id in sorted(set(list(car_calibration.keys()) + list(ball_calibration.keys()))):
    print(f"    {cam_id}: {{")
    if cam_id in car_calibration:
        print(f"        'car': {car_calibration[cam_id]['k']:.4f},")
    if cam_id in ball_calibration:
        print(f"        'ball': {ball_calibration[cam_id]['k']:.4f},")
    print(f"    }},")
print("}")

# 可视化标定结果
fig, axes = plt.subplots(2, 3, figsize=(18, 10))

for idx, cam_id in enumerate(sorted(car_calibration.keys())):
    if idx >= 3: break
    
    cam_data = car_data[car_data['Cam_ID'] == cam_id]
    cam_data = cam_data[cam_data['Vision_Width'] > 0.5]
    cam_data = cam_data[cam_data['Vicon_Dist'] < 5.0]
    
    widths = cam_data['Vision_Width'].values
    distances_vicon = cam_data['Vicon_Dist'].values
    distances_vision = cam_data['Vision_Dist'].values
    
    k_optimal = car_calibration[cam_id]['k']
    distances_calibrated = distance_model(widths, k_optimal)
    
    # 上排: 小车
    axes[0, idx].scatter(widths, distances_vision, alpha=0.5, s=10, label='Vision (原始)', color='red')
    axes[0, idx].scatter(widths, distances_vicon, alpha=0.5, s=10, label='VICON (真实)', color='blue')
    axes[0, idx].scatter(widths, distances_calibrated, alpha=0.5, s=10, label='标定后', color='green')
    
    # 绘制拟合曲线
    w_range = np.linspace(widths.min(), widths.max(), 100)
    axes[0, idx].plot(w_range, distance_model(w_range, k_optimal), 'g--', linewidth=2, label='拟合曲线')
    
    axes[0, idx].set_xlabel('Vision_Width')
    axes[0, idx].set_ylabel('Distance (m)')
    axes[0, idx].set_title(f'CAR - Cam {cam_id}\nRMSE={car_calibration[cam_id]["rmse"]:.3f}m')
    axes[0, idx].legend(fontsize=8)
    axes[0, idx].grid(True, alpha=0.3)

# 下排: 球
for idx, cam_id in enumerate(sorted(ball_calibration.keys())):
    if idx >= 3: break
    
    cam_data = ball_data[ball_data['Cam_ID'] == cam_id]
    cam_data = cam_data[cam_data['Vision_Width'] > 0.4]
    cam_data = cam_data[cam_data['Vicon_Dist'] < 2.0]
    
    widths = cam_data['Vision_Width'].values
    distances_vicon = cam_data['Vicon_Dist'].values
    distances_vision = cam_data['Vision_Dist'].values
    
    k_optimal = ball_calibration[cam_id]['k']
    distances_calibrated = distance_model(widths, k_optimal)
    
    axes[1, idx].scatter(widths, distances_vision, alpha=0.5, s=10, label='Vision (原始)', color='red')
    axes[1, idx].scatter(widths, distances_vicon, alpha=0.5, s=10, label='VICON (真实)', color='blue')
    axes[1, idx].scatter(widths, distances_calibrated, alpha=0.5, s=10, label='标定后', color='green')
    
    w_range = np.linspace(widths.min(), widths.max(), 100)
    axes[1, idx].plot(w_range, distance_model(w_range, k_optimal), 'g--', linewidth=2, label='拟合曲线')
    
    axes[1, idx].set_xlabel('Vision_Width')
    axes[1, idx].set_ylabel('Distance (m)')
    axes[1, idx].set_title(f'BALL - Cam {cam_id}\nRMSE={ball_calibration[cam_id]["rmse"]:.3f}m')
    axes[1, idx].legend(fontsize=8)
    axes[1, idx].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('calibration_results.png', dpi=150, bbox_inches='tight')
print("\n标定可视化已保存为 calibration_results.png")

