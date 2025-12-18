import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy import stats
import glob

# 读取所有数据
csv_files = sorted(glob.glob('log_sysID_*.csv'))
df_all = pd.concat([pd.read_csv(f) for f in csv_files], ignore_index=True)
car_data = df_all[df_all['Target_Type'].str.strip() == 'CAR'].copy()
ball_data = df_all[df_all['Target_Type'].str.strip() == 'BALL'].copy()

print("="*80)
print("线性修正分析 - 寻找简单有效的修正方法")
print("="*80)

# 分析1: Vision_Dist vs Vicon_Dist 的线性关系
print("\n【分析1】Vision_Dist 与 Vicon_Dist 的线性关系")
print("-"*80)

for target_type, data in [('小车', car_data), ('球', ball_data)]:
    print(f"\n{target_type}:")
    
    # 线性拟合: Vicon = a * Vision + b
    slope, intercept, r_value, p_value, std_err = stats.linregress(
        data['Vision_Dist'], data['Vicon_Dist']
    )
    
    print(f"  线性关系: Vicon_Dist = {slope:.4f} × Vision_Dist + {intercept:.4f}")
    print(f"  相关系数 R² = {r_value**2:.4f}")
    print(f"  ✓ 修正公式: Vision_Dist_corrected = Vision_Dist × {slope:.4f} + {intercept:.4f}")
    
    # 计算修正后的误差
    corrected = data['Vision_Dist'] * slope + intercept
    original_rmse = np.sqrt(np.mean(data['Dist_Err']**2))
    corrected_rmse = np.sqrt(np.mean((data['Vicon_Dist'] - corrected)**2))
    improvement = (1 - corrected_rmse/original_rmse) * 100
    
    print(f"  原始RMSE: {original_rmse:.4f}m")
    print(f"  修正后RMSE: {corrected_rmse:.4f}m")
    print(f"  改进: {improvement:.1f}%")

# 分析2: 按相机分别线性修正
print("\n" + "="*80)
print("【分析2】按相机ID分别线性修正")
print("-"*80)

cam_corrections = {}
for cam_id in sorted(car_data['Cam_ID'].unique()):
    print(f"\n=== Cam {cam_id} ===")
    
    for target_type, data in [('小车', car_data), ('球', ball_data)]:
        cam_data = data[data['Cam_ID'] == cam_id]
        if len(cam_data) < 10:
            continue
            
        slope, intercept, r_value, _, _ = stats.linregress(
            cam_data['Vision_Dist'], cam_data['Vicon_Dist']
        )
        
        corrected = cam_data['Vision_Dist'] * slope + intercept
        original_rmse = np.sqrt(np.mean(cam_data['Dist_Err']**2))
        corrected_rmse = np.sqrt(np.mean((cam_data['Vicon_Dist'] - corrected)**2))
        improvement = (1 - corrected_rmse/original_rmse) * 100
        
        print(f"{target_type}: y = {slope:.3f}x + {intercept:.3f}, R²={r_value**2:.3f}, 改进{improvement:.0f}%")
        
        if cam_id not in cam_corrections:
            cam_corrections[cam_id] = {}
        cam_corrections[cam_id][target_type] = {'slope': slope, 'intercept': intercept}

# 分析3: Vision_Width的线性修正
print("\n" + "="*80)
print("【分析3】Vision_Width 是否需要修正")
print("-"*80)

for target_type, data in [('小车', car_data), ('球', ball_data)]:
    print(f"\n{target_type}:")
    
    # 理论: Vicon_Dist = K / Vision_Width
    # 即: Vicon_Dist × Vision_Width = K (应该是常数)
    product = data['Vicon_Dist'] * data['Vision_Width']
    
    print(f"  Vicon_Dist × Vision_Width:")
    print(f"    均值: {product.mean():.4f}")
    print(f"    标准差: {product.std():.4f}")
    print(f"    变异系数: {product.std()/product.mean()*100:.1f}%")
    
    if product.std()/product.mean() < 0.3:
        print(f"  ✓ Vision_Width测量准确，无需修正")
    else:
        print(f"  ⚠️ Vision_Width测量不稳定")

# 分析4: 角度的线性修正
print("\n" + "="*80)
print("【分析4】角度误差的线性修正")
print("-"*80)

for target_type, data in [('小车', car_data), ('球', ball_data)]:
    print(f"\n{target_type}:")
    
    for cam_id in sorted(data['Cam_ID'].unique()):
        cam_data = data[data['Cam_ID'] == cam_id]
        if len(cam_data) < 10:
            continue
        
        mean_err = cam_data['Ang_Err'].mean()
        std_err = cam_data['Ang_Err'].std()
        
        # 检查是否是系统性偏差（均值远大于标准差）
        if abs(mean_err) > std_err:
            print(f"  Cam {cam_id}: 系统性偏差 {mean_err:+.2f}° (std={std_err:.2f}°) ⚠️ 需要修正")
        else:
            print(f"  Cam {cam_id}: 随机误差 {mean_err:+.2f}° (std={std_err:.2f}°) ✓ 无需修正")

# 生成修正代码
print("\n" + "="*80)
print("【推荐修正方案】最简单有效的线性修正")
print("="*80)

print("\n# 方案1: 全局线性修正（最简单）")
print("def correct_distance(vision_dist, target_type):")
for target_type, data in [('小车', car_data), ('球', ball_data)]:
    slope, intercept, r_value, _, _ = stats.linregress(data['Vision_Dist'], data['Vicon_Dist'])
    target_key = 'car' if target_type == '小车' else 'ball'
    print(f"    if target_type == '{target_key}':")
    print(f"        return vision_dist * {slope:.4f} + {intercept:.4f}  # R²={r_value**2:.3f}")

print("\n# 方案2: 按相机分别修正（更精确）")
print("DISTANCE_CORRECTION = {")
for cam_id in sorted(cam_corrections.keys()):
    print(f"    {cam_id}: {{")
    for target_type, params in cam_corrections[cam_id].items():
        target_key = 'car' if target_type == '小车' else 'ball'
        print(f"        '{target_key}': ({params['slope']:.4f}, {params['intercept']:.4f}),")
    print(f"    }},")
print("}")
print("\ndef correct_distance(vision_dist, cam_id, target_type):")
print("    slope, intercept = DISTANCE_CORRECTION[cam_id][target_type]")
print("    return vision_dist * slope + intercept")

print("\n# 方案3: 角度修正（固定偏移）")
print("ANGLE_CORRECTION = {")
for target_type, data in [('小车', car_data), ('球', ball_data)]:
    target_key = 'car' if target_type == '小车' else 'ball'
    print(f"    '{target_key}': {{")
    for cam_id in sorted(data['Cam_ID'].unique()):
        cam_data = data[data['Cam_ID'] == cam_id]
        if len(cam_data) >= 10:
            mean_err = cam_data['Ang_Err'].mean()
            print(f"        {cam_id}: {-mean_err:.2f},  # 修正 {mean_err:+.2f}° 偏差")
    print(f"    }},")
print("}")

# 可视化
fig, axes = plt.subplots(2, 3, figsize=(18, 10))

# 小车线性关系
ax = axes[0, 0]
ax.scatter(car_data['Vision_Dist'], car_data['Vicon_Dist'], alpha=0.3, s=10)
slope, intercept, r_value, _, _ = stats.linregress(car_data['Vision_Dist'], car_data['Vicon_Dist'])
x_line = np.linspace(car_data['Vision_Dist'].min(), car_data['Vision_Dist'].max(), 100)
ax.plot(x_line, slope * x_line + intercept, 'r-', linewidth=2, label=f'y={slope:.2f}x+{intercept:.2f}')
ax.plot([0, 2], [0, 2], 'k--', alpha=0.5, label='Ideal')
ax.set_xlabel('Vision Distance (m)')
ax.set_ylabel('VICON Distance (m)')
ax.set_title(f'CAR: Linear Fit (R²={r_value**2:.3f})')
ax.legend()
ax.grid(True, alpha=0.3)

# 球线性关系
ax = axes[0, 1]
ax.scatter(ball_data['Vision_Dist'], ball_data['Vicon_Dist'], alpha=0.3, s=10, color='orange')
slope, intercept, r_value, _, _ = stats.linregress(ball_data['Vision_Dist'], ball_data['Vicon_Dist'])
x_line = np.linspace(ball_data['Vision_Dist'].min(), ball_data['Vision_Dist'].max(), 100)
ax.plot(x_line, slope * x_line + intercept, 'r-', linewidth=2, label=f'y={slope:.2f}x+{intercept:.2f}')
ax.plot([0, 2], [0, 2], 'k--', alpha=0.5, label='Ideal')
ax.set_xlabel('Vision Distance (m)')
ax.set_ylabel('VICON Distance (m)')
ax.set_title(f'BALL: Linear Fit (R²={r_value**2:.3f})')
ax.legend()
ax.grid(True, alpha=0.3)

# 小车修正前后对比
ax = axes[0, 2]
slope, intercept, _, _, _ = stats.linregress(car_data['Vision_Dist'], car_data['Vicon_Dist'])
corrected = car_data['Vision_Dist'] * slope + intercept
ax.scatter(car_data['Vicon_Dist'], car_data['Vision_Dist'], alpha=0.3, s=10, label='Original')
ax.scatter(car_data['Vicon_Dist'], corrected, alpha=0.3, s=10, label='Corrected')
ax.plot([0, 2], [0, 2], 'r--', linewidth=2, label='Ideal')
ax.set_xlabel('VICON Distance (m)')
ax.set_ylabel('Vision Distance (m)')
ax.set_title('CAR: Before/After Correction')
ax.legend()
ax.grid(True, alpha=0.3)

# 按相机的线性关系（小车）
ax = axes[1, 0]
for cam_id in sorted(car_data['Cam_ID'].unique()):
    cam_subset = car_data[car_data['Cam_ID'] == cam_id]
    ax.scatter(cam_subset['Vision_Dist'], cam_subset['Vicon_Dist'], alpha=0.4, s=10, label=f'Cam {cam_id}')
    slope, intercept, _, _, _ = stats.linregress(cam_subset['Vision_Dist'], cam_subset['Vicon_Dist'])
    x_line = np.linspace(cam_subset['Vision_Dist'].min(), cam_subset['Vision_Dist'].max(), 50)
    ax.plot(x_line, slope * x_line + intercept, '--', linewidth=2)
ax.plot([0, 2], [0, 2], 'k--', alpha=0.5)
ax.set_xlabel('Vision Distance (m)')
ax.set_ylabel('VICON Distance (m)')
ax.set_title('CAR: Linear Fit by Camera')
ax.legend()
ax.grid(True, alpha=0.3)

# 按相机的线性关系（球）
ax = axes[1, 1]
for cam_id in sorted(ball_data['Cam_ID'].unique()):
    cam_subset = ball_data[ball_data['Cam_ID'] == cam_id]
    if len(cam_subset) < 10:
        continue
    ax.scatter(cam_subset['Vision_Dist'], cam_subset['Vicon_Dist'], alpha=0.4, s=10, label=f'Cam {cam_id}')
    slope, intercept, _, _, _ = stats.linregress(cam_subset['Vision_Dist'], cam_subset['Vicon_Dist'])
    x_line = np.linspace(cam_subset['Vision_Dist'].min(), cam_subset['Vision_Dist'].max(), 50)
    ax.plot(x_line, slope * x_line + intercept, '--', linewidth=2)
ax.plot([0, 2], [0, 2], 'k--', alpha=0.5)
ax.set_xlabel('Vision Distance (m)')
ax.set_ylabel('VICON Distance (m)')
ax.set_title('BALL: Linear Fit by Camera')
ax.legend()
ax.grid(True, alpha=0.3)

# 角度误差分布
ax = axes[1, 2]
for cam_id in sorted(car_data['Cam_ID'].unique()):
    cam_subset = car_data[car_data['Cam_ID'] == cam_id]
    mean_err = cam_subset['Ang_Err'].mean()
    ax.axvline(mean_err, linestyle='--', linewidth=2, label=f'Cam {cam_id}: {mean_err:+.1f}°')
ax.hist(car_data['Ang_Err'], bins=50, alpha=0.3, color='gray')
ax.axvline(0, color='r', linestyle='-', linewidth=2, label='Target')
ax.set_xlabel('Angle Error (deg)')
ax.set_ylabel('Frequency')
ax.set_title('CAR: Angle Error Distribution')
ax.legend(fontsize=8)
ax.grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig('linear_correction_analysis.png', dpi=150, bbox_inches='tight')
print("\n✓ 可视化已保存: linear_correction_analysis.png")

