#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
文件: data_analysis.py (智能诊断版)
功能: 
  1. 读取 CSV 日志。
  2. 像医生一样分析“病情” (线性误差、系统偏差)。
  3. 直接给出 vision_pub.py 的参数修改建议 (处方)。
"""

import pandas as pd
import numpy as np
import glob
import os
import sys

# === 配置当前使用的参数 (用于反推建议值) ===
CURRENT_PARAMS = {
    'basketball_width': 0.23,  # 当前代码里的球直径
    'car_width': 0.31,         # 当前代码里的车宽
    'cam_height': 0.15         # 当前代码里的相机高度
}

def get_latest_log():
    list_of_files = glob.glob('/home/nvidia/Downloads/Ros/pseudo_ros_architecture/log_full_20251217_223950.csv') 
    if not list_of_files:
        print("❌ 未找到日志文件")
        return None
    return max(list_of_files, key=os.path.getctime)

def print_section(title):
    print("\n" + "="*60)
    print(f" {title}")
    print("="*60)

def diagnose_width_method(df, target_name, current_width_param):
    """
    诊断宽度测距法：检查是否存在线性缩放误差
    """
    print_section(f"🕵️  诊断报告: {target_name} (宽度法分析)")
    
    if df.empty:
        print("   ⚠️ 无数据，跳过。")
        return

    # 1. 核心计算：比例因子
    # 过滤掉过近的数据(<0.5m)防止分母过小造成噪声
    valid_df = df[df['Vicon_Dist'] > 0.1]
    
    if len(valid_df) < 10:
        print("   ⚠️ 有效数据太少 (<10帧)，无法精确诊断。")
        return

    # Ratio > 1.0 表示视觉测远了 (Over-estimate)
    # Ratio < 1.0 表示视觉测近了 (Under-estimate)
    ratios = valid_df['Vision_Width'] / valid_df['Vicon_Dist']
    mean_ratio = ratios.mean()
    std_ratio = ratios.std()
    
    # 2. 输出现象
    print(f"【现象】 线性缩放一致性检查")
    print(f"   - 样本数: {len(valid_df)} 帧")
    print(f"   - 平均比例 (Vision/Truth): {mean_ratio:.4f}")
    print(f"   - 波动程度 (Std): {std_ratio:.4f}")
    
    # 3. 判定病情
    error_percent = (mean_ratio - 1.0) * 100
    print("-" * 40)
    
    if abs(error_percent) < 2.0:
        print(f"✅ [健康] 误差在 2% 以内 ({error_percent:+.2f}%)，无需修改参数。")
    else:
        status = "偏大 (测远了)" if error_percent > 0 else "偏小 (测近了)"
        print(f"❌ [异常] 视觉测量系统性 {status} {abs(error_percent):.1f}%")
        
        # 4. 开处方
        # 原理: D_vis = (f * W_param) / px
        # 如果 D_vis 是 D_real 的 1.1倍，说明 W_param 设大了 1.1倍
        suggested_width = current_width_param / mean_ratio
        
        print(f"   ----------------------------------------")
        print(f"   📝 [处方] 修改 vision_pub.py 参数:")
        print(f"   原参数: CLASS_REAL_WIDTHS['...'] = {current_width_param} m")
        print(f"   新建议: CLASS_REAL_WIDTHS['...'] = {suggested_width:.4f} m")
        print(f"   ----------------------------------------")

def diagnose_fusion_method(df):
    """
    诊断融合法：对比几何法 vs 宽度法 vs 融合结果
    """
    print_section(f"⚖️  诊断报告: 小车 (多传感器融合分析)")
    
    if df.empty:
        print("   ⚠️ 无小车数据。")
        return

    valid_df = df[df['Vicon_Dist'] > 0.5]
    
    # 计算三种方法的误差绝对值 (MAE)
    mae_width = np.mean(np.abs(valid_df['Vision_Width'] - valid_df['Vicon_Dist']))
    mae_geo   = np.mean(np.abs(valid_df['Vision_Geo'] - valid_df['Vicon_Dist']))
    mae_fused = np.mean(np.abs(valid_df['Vision_Dist'] - valid_df['Vicon_Dist']))

    print(f"【现象】 三种方法的平均误差 (MAE)")
    print(f"   1. 纯宽度法 (Width): {mae_width:.3f} m")
    print(f"   2. 纯几何法 (Geo):   {mae_geo:.3f} m")
    print(f"   3. 当前融合 (Fused): {mae_fused:.3f} m")
    
    print("-" * 40)
    
    # 比较逻辑
    best_method = min([("Width", mae_width), ("Geo", mae_geo), ("Fused", mae_fused)], key=lambda x: x[1])
    
    print(f"🏆 [结论] 表现最好的是: 【{best_method[0]}】")
    
    print("\n🧐 [深度分析]")
    if mae_geo > mae_width * 1.5:
        print("   ❌ 几何法 (Geo) 误差显著大于宽度法！")
        print("   可能原因: ")
        print("     1. 相机高度 (CAM_HEIGHT) 测量不准。")
        print("     2. 相机有俯仰角 (Pitch)，但代码假设为0。")
        print("     3. 远处(>3m) 几何法本身由于像素量化误差导致发散。")
        print("   💡 建议: 降低融合权重中 Geo 的占比，或者仅在近距离(<2m)使用 Geo。")
    
    elif mae_width > mae_geo * 1.5:
        print("   ❌ 宽度法 (Width) 误差显著大于几何法！")
        print("   可能原因: 小车真实宽度参数不准，或者 YOLO 检测框抖动大。")
        print("   💡 建议: 按照上面的【宽度法诊断】修正车宽参数。")
        
    else:
        print("   ✅ 两种方法误差接近，当前的融合逻辑运行良好。")

def main():
    log_file = get_latest_log()
    if not log_file: return
    
    print(f"📂 正在分析日志: {log_file}")
    
    try:
        df = pd.read_csv(log_file)
    except Exception as e:
        print(f"无法读取文件: {e}")
        return

    # 分离数据
    # 注意：Target_Type 列是在最新的 data_logger.py 中加入的
    # 如果是旧日志，可能没有这一列，需要做个兼容
    if 'Target_Type' in df.columns:
        df_ball = df[df['Target_Type'].str.contains("BALL", na=False)]
        df_car  = df[df['Target_Type'].str.contains("CAR", na=False)]
    else:
        # 兼容旧日志：尝试通过 Class_ID 判断
        df_ball = df[df['Vision_Class'] == 6]
        df_car  = df[df['Vision_Class'] <= 5]

    # === 1. 诊断篮球 (纯宽度) ===
    diagnose_width_method(df_ball, "篮球 (Ball)", CURRENT_PARAMS['basketball_width'])

    # === 2. 诊断小车 (宽度) ===
    # 先单独看小车的宽度准不准，这决定了融合的基础
    diagnose_width_method(df_car, "小车 (Car-Width)", CURRENT_PARAMS['car_width'])

    # === 3. 诊断融合逻辑 ===
    diagnose_fusion_method(df_car)

    print("\n" + "="*60)
    print("🏁 诊断结束")

if __name__ == "__main__":
    main()