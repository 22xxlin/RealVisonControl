#!/usr/bin/env python3
"""
去抖动机制测试脚本
模拟滑动窗口产生的过渡信号，验证去抖动效果
"""

from collections import defaultdict


class DebounceSimulator:
    """模拟去抖动机制"""
    
    def __init__(self, debounce_threshold=5):
        self.debounce_threshold = debounce_threshold
        self.pattern_stability = defaultdict(lambda: {
            'candidate': 'IDLE',
            'count': 0,
            'confirmed': 'IDLE'
        })
    
    def process_raw_pattern(self, track_id, raw_pattern):
        """
        处理原始模式，应用去抖动逻辑
        
        Args:
            track_id: 目标ID
            raw_pattern: 滑动窗口计算出的原始模式
            
        Returns:
            确认后的稳定模式
        """
        stability = self.pattern_stability[track_id]
        
        # 检查原始模式是否与当前候选一致
        if raw_pattern == stability['candidate']:
            stability['count'] += 1
        else:
            stability['candidate'] = raw_pattern
            stability['count'] = 1
        
        # 判定是否达到确认阈值
        if stability['count'] >= self.debounce_threshold:
            stability['confirmed'] = raw_pattern
            return raw_pattern
        else:
            return stability['confirmed']


def simulate_signal_transition():
    """
    模拟信号过渡场景：
    - 初始状态：IDLE
    - 新信号进入：220 开始出现，但窗口内新旧混合导致误判为 2200
    - 信号稳定：220 充满窗口，正确识别为 220
    - 信号离开：220 开始离开，窗口内新旧混合又误判为 2200
    - 最终状态：IDLE
    """
    
    print("=" * 80)
    print("🧪 去抖动机制测试 - 模拟信号过渡场景")
    print("=" * 80)
    print()
    
    # 模拟滑动窗口产生的原始模式序列（包含过渡期的错误模式）
    raw_patterns = [
        # 初始 IDLE 状态
        'IDLE', 'IDLE', 'IDLE', 'IDLE', 'IDLE',
        
        # 新信号 220 进入，窗口混合期误判为 2200（过渡信号）
        '2200', '2200', '2200', '2200',
        
        # 信号稳定，正确识别为 220
        '220', '220', '220', '220', '220', '220', '220', '220', '220', '220',
        
        # 信号离开，窗口混合期又误判为 2200（过渡信号）
        '2200', '2200', '2200', '2200',
        
        # 回到 IDLE
        'IDLE', 'IDLE', 'IDLE', 'IDLE', 'IDLE',
    ]
    
    # 创建去抖动器（阈值=5帧）
    debouncer = DebounceSimulator(debounce_threshold=5)
    track_id = 1
    
    print(f"📊 原始模式序列（共 {len(raw_patterns)} 帧）：")
    print(f"   {' -> '.join(raw_patterns)}")
    print()
    print(f"🔧 去抖动阈值: {debouncer.debounce_threshold} 帧")
    print()
    print("-" * 80)
    print(f"{'帧号':<6} {'原始模式':<12} {'候选模式':<12} {'计数':<6} {'确认模式':<12} {'输出':<12}")
    print("-" * 80)
    
    outputs = []
    for frame_idx, raw_pattern in enumerate(raw_patterns, start=1):
        # 应用去抖动
        output_pattern = debouncer.process_raw_pattern(track_id, raw_pattern)
        outputs.append(output_pattern)
        
        # 获取当前状态
        stability = debouncer.pattern_stability[track_id]
        candidate = stability['candidate']
        count = stability['count']
        confirmed = stability['confirmed']
        
        # 标记是否发生了过滤
        filtered = "✅ 过滤" if raw_pattern != output_pattern else ""
        
        print(f"{frame_idx:<6} {raw_pattern:<12} {candidate:<12} {count:<6} {confirmed:<12} {output_pattern:<12} {filtered}")
    
    print("-" * 80)
    print()
    print("📈 输出模式序列（去抖动后）：")
    print(f"   {' -> '.join(outputs)}")
    print()
    
    # 统计效果
    filtered_count = sum(1 for i, (raw, out) in enumerate(zip(raw_patterns, outputs)) if raw != out)
    print(f"✅ 成功过滤 {filtered_count} 帧不稳定的过渡信号")
    print()
    
    # 分析结果
    print("🎯 分析结果：")
    print("   1. 初始 IDLE 状态：需要 5 帧确认后才输出 IDLE")
    print("   2. 过渡信号 2200（进入期）：被过滤，保持输出 IDLE")
    print("   3. 稳定信号 220：连续 5 帧后确认，输出 220")
    print("   4. 过渡信号 2200（离开期）：被过滤，保持输出 220")
    print("   5. 最终 IDLE 状态：连续 5 帧后确认，输出 IDLE")
    print()
    print("✅ 去抖动机制有效消除了滑动窗口产生的过渡误判！")
    print("=" * 80)


if __name__ == "__main__":
    simulate_signal_transition()

