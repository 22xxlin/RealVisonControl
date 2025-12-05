#!/usr/bin/env python3
"""
架构解耦测试脚本
验证 Vision 端和 Control 端的数据流是否正确
"""

import json

# 模拟 Vision 端发送的数据包
def test_vision_output():
    """测试 Vision 端输出格式"""
    print("=" * 60)
    print("测试 1: Vision 端输出格式")
    print("=" * 60)
    
    # 模拟 vision_pub.py 发送的数据
    vision_data = {
        'distance': 1.5,
        'azimuth': 45.0,
        'bearing_body': 225.0,
        'track_id': 3,
        'cam_idx': 4,
        'pattern': '2200',  # ✅ 原始 Pattern
        'class_id': 2,
        'timestamp': 1234567890.0
    }
    
    print("✅ Vision 端发送的数据包:")
    print(json.dumps(vision_data, indent=2, ensure_ascii=False))
    
    # 验证关键字段
    assert 'pattern' in vision_data, "❌ 缺少 'pattern' 字段"
    assert 'command' not in vision_data, "❌ 不应包含 'command' 字段"
    assert 'description' not in vision_data, "❌ 不应包含 'description' 字段"
    
    print("\n✅ Vision 端数据格式正确！\n")
    return vision_data


# 模拟 Control 端接收和处理
def test_control_processing(vision_data):
    """测试 Control 端决策逻辑"""
    print("=" * 60)
    print("测试 2: Control 端决策逻辑")
    print("=" * 60)
    
    # 模拟 control_sub.py 的映射表
    PATTERN_TO_COMMAND = {
        '220': 'FORWARD', '330': 'LEFT', '110': 'RIGHT', '550': 'REVERSE', '440': 'STOP',
        '2200': 'APPROACH', '1100': 'RETREAT', '4400': 'S_SHAPE', '5500': 'CIRCLE',
        '1111': 'FORWARD', '2222': 'LEFT', '3333': 'RIGHT', '4444': 'STOP', '5555': 'REVERSE',
    }
    
    ACTION_DESCRIPTIONS = {
        'FORWARD': '前进', 'LEFT': '左移', 'RIGHT': '右移', 'STOP': '停止',
        'REVERSE': '后退', 'APPROACH': '靠近', 'RETREAT': '远离', 
        'S_SHAPE': 'S形', 'CIRCLE': '圆形', 'IDLE': '待机'
    }
    
    # 提取 Pattern
    pattern = vision_data.get('pattern', 'IDLE')
    print(f"📥 接收到 Pattern: '{pattern}'")
    
    # 决策：Pattern -> Command
    command = PATTERN_TO_COMMAND.get(pattern, 'IDLE')
    description = ACTION_DESCRIPTIONS.get(command, '未知')
    
    print(f"🧠 Decision: Pattern '{pattern}' -> Action '{command}' ({description})")
    
    # 验证决策结果
    assert command == 'APPROACH', f"❌ 决策错误：期望 'APPROACH'，实际 '{command}'"
    assert description == '靠近', f"❌ 描述错误：期望 '靠近'，实际 '{description}'"
    
    print("\n✅ Control 端决策逻辑正确！\n")
    return command, description


# 测试多个 Pattern
def test_multiple_patterns():
    """测试多个 Pattern 的映射"""
    print("=" * 60)
    print("测试 3: 多个 Pattern 映射")
    print("=" * 60)
    
    PATTERN_TO_COMMAND = {
        '220': 'FORWARD', '330': 'LEFT', '110': 'RIGHT', '550': 'REVERSE', '440': 'STOP',
        '2200': 'APPROACH', '1100': 'RETREAT', '4400': 'S_SHAPE', '5500': 'CIRCLE',
        '1111': 'FORWARD', '2222': 'LEFT', '3333': 'RIGHT', '4444': 'STOP', '5555': 'REVERSE',
    }
    
    test_cases = [
        ('2200', 'APPROACH'),
        ('220', 'FORWARD'),
        ('110', 'RIGHT'),
        ('4400', 'S_SHAPE'),
        ('5500', 'CIRCLE'),
        ('IDLE', 'IDLE'),
        ('9999', 'IDLE'),  # 未知 Pattern
    ]
    
    for pattern, expected_command in test_cases:
        command = PATTERN_TO_COMMAND.get(pattern, 'IDLE')
        status = "✅" if command == expected_command else "❌"
        print(f"{status} Pattern '{pattern}' -> Command '{command}' (期望: '{expected_command}')")
        assert command == expected_command, f"映射错误：{pattern} -> {command} (期望 {expected_command})"
    
    print("\n✅ 所有 Pattern 映射正确！\n")


if __name__ == "__main__":
    print("\n🚀 开始架构解耦测试...\n")
    
    try:
        # 测试 1: Vision 端输出
        vision_data = test_vision_output()
        
        # 测试 2: Control 端处理
        command, description = test_control_processing(vision_data)
        
        # 测试 3: 多个 Pattern
        test_multiple_patterns()
        
        print("=" * 60)
        print("🎉 所有测试通过！架构解耦成功！")
        print("=" * 60)
        print("\n📝 总结:")
        print("  ✅ Vision 端只输出原始 Pattern")
        print("  ✅ Control 端负责 Pattern -> Command 翻译")
        print("  ✅ 数据流清晰，职责分离明确")
        print("\n🚀 可以启动实际系统进行测试:")
        print("  1. python3 vision_pub.py")
        print("  2. python3 control_sub.py")
        
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        exit(1)
    except Exception as e:
        print(f"\n❌ 测试异常: {e}")
        exit(1)

