# 队形锁定问题分析与修复方案

## 问题描述
Follower 和 Leader 可能挤在球的同一侧，形成"V字形"而非"三角形包围"。

## 根本原因
当前算法只比较角度误差，未验证选择的方向是否真正包围球。

## 几何约束（先验知识）
- **+120°**: Leader 必须在球的**左侧**（从 Follower 视角，theta_ball_to_leader > 0）
- **-120°**: Leader 必须在球的**右侧**（从 Follower 视角，theta_ball_to_leader < 0）

## 修复代码（插入到 combined_transport_node.py 第209-220行）

```python
else:
    # 自动择优 + 包围验证
    theta_leader = math.atan2(yl - yt, xl - xt)
    diff_rad = math.radians(FORMATION_ANGLE_DIFF)
    err_pos = wrap_rad_pi((theta_leader + diff_rad) - theta_robot)
    err_neg = wrap_rad_pi((theta_leader - diff_rad) - theta_robot)

    # 关键修复：验证选择的方向是否能包围球
    # +120° 要求 Leader 在球左侧 (theta_leader > 0)
    # -120° 要求 Leader 在球右侧 (theta_leader < 0)
    theta_ball_to_leader = math.atan2(yl - yt, xl - xt)
    
    valid_pos = theta_ball_to_leader > 0  # Leader 在球左侧，+120° 有效
    valid_neg = theta_ball_to_leader < 0  # Leader 在球右侧，-120° 有效
    
    # 优先选择有效且误差小的
    if valid_pos and (not valid_neg or abs(err_pos) < abs(err_neg)):
        final_err_rad, side_char = err_pos, "+"
    elif valid_neg:
        final_err_rad, side_char = err_neg, "-"
    else:
        # 都无效时（极端情况），选误差小的
        final_err_rad, side_char = (err_pos, "+") if abs(err_pos) < abs(err_neg) else (err_neg, "-")
    
    # 记录意图
    self.latest_side_intent = side_char

    angle_err_deg = math.degrees(final_err_rad)
    if self.is_in_deadband and abs(angle_err_deg) < 5.0: v_tan = 0.0
    else: v_tan = max(-0.3, min(0.3, KP_THETA * final_err_rad * curr_dist))
```

## 为什么 test_formation_lock_vision_ekf.py 没问题？
可能的原因：
1. 测试场景中初始位置恰好满足约束
2. 迟滞锁定机制更快，减少了错误选择的机会
3. 实际上也有问题，但未被观察到

## 验证方法
修复后测试以下场景：
- Follower 在球右侧，Leader 在球左侧 → 应选 +120°
- Follower 在球左侧，Leader 在球右侧 → 应选 -120°
- Follower 在球后方，Leader 在球前方 → 根据相对位置选择

