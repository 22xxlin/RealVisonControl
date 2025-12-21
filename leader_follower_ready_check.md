# Leader 判断 Follower 就位逻辑验证报告

## ✅ 问题已正确解决！

### 1. 协议定义 (light_driver.py)

```python
# 竞标阶段 (未就位)
"BID_LEFT":   {colors: [GREEN, BLACK], counts: FLASH}  # 绿闪
"BID_RIGHT":  {colors: [BLUE, BLACK],  counts: FLASH}  # 蓝闪

# 锁定阶段 (已就位)
"LOCK_LEFT":  {colors: [GREEN], counts: SOLID}  # 绿常亮
"LOCK_RIGHT": {colors: [BLUE],  counts: SOLID}  # 蓝常亮
```

### 2. 视觉识别 (vision_pub.py)

```python
# 第 148 行：模式判定
target_pattern = 'SOLID' if color_ratio > 0.90 else 'FLASH'

# 第 227 行：发布数据
pub_data = {
    'class_id': obj['class_id'],  # 3=GREEN, 1=BLUE
    'pattern': obj['pattern'],     # 'SOLID' 或 'FLASH'
    ...
}
```

### 3. 事件监测 (EventWatcher)

```python
# 第 293-298 行：接收数据
def ingest(self, batch):
    for msg in batch:
        cid = int(msg.get("class_id", -1))  # 颜色ID
        pat = str(msg.get("pattern", "OFF")) # 模式
        self.hist[cid].append((t, pat))

# 第 299-305 行：检查稳定性
def stable_pattern(self, class_id: int, pattern: str, need_k: int, within_s: float):
    hits = sum(1 for (t, p) in h if t >= t0 and p == pattern)
    return hits >= need_k
```

### 4. Leader 判断逻辑 (第 424-425 行)

```python
left_ready = watcher.stable_pattern(GREEN, "SOLID", 3, 0.6)
right_ready = watcher.stable_pattern(BLUE, "SOLID", 3, 0.6)
```

**含义**: 
- 在 0.6 秒内，至少看到 3 次 "绿色常亮" → 左侧就位
- 在 0.6 秒内，至少看到 3 次 "蓝色常亮" → 右侧就位

---

## 🎯 完整流程验证

### 场景 1: Follower 正在调整 (未就位)

```
Follower → light.set_cmd("BID_LEFT")
         → 灯光: 绿闪 (GREEN + FLASH)
         → 视觉: class_id=3, pattern="FLASH"
         → EventWatcher: hist[3] = [(t1, "FLASH"), (t2, "FLASH"), ...]
         → Leader 检查: watcher.stable_pattern(3, "SOLID", 3, 0.6)
         → 结果: False (因为 pattern 是 "FLASH" 不是 "SOLID")
```

### 场景 2: Follower 已锁定 (已就位)

```
Follower → light.set_cmd("LOCK_LEFT")
         → 灯光: 绿常亮 (GREEN + SOLID)
         → 视觉: class_id=3, pattern="SOLID"
         → EventWatcher: hist[3] = [(t1, "SOLID"), (t2, "SOLID"), (t3, "SOLID")]
         → Leader 检查: watcher.stable_pattern(3, "SOLID", 3, 0.6)
         → 结果: True ✅
```

---

## ✅ 结论

**逻辑完全正确！** 

`EventWatcher.stable_pattern()` 同时检查了：
1. ✅ **颜色** (class_id: GREEN=3, BLUE=1)
2. ✅ **模式** (pattern: "SOLID" vs "FLASH")
3. ✅ **稳定性** (0.6秒内至少3次)

Leader 不会误判"绿闪"为"已就位"，只有"绿常亮"才会触发。

---

## 📊 参数调优建议

当前参数: `stable_pattern(GREEN, "SOLID", 3, 0.6)`

- **need_k=3**: 需要 3 次观测
- **within_s=0.6**: 在 0.6 秒内

假设视觉融合频率 = 30Hz，0.6秒 = 18帧，要求至少 3/18 = 16.7% 的帧看到目标。

**建议**:
- 如果视觉稳定，可以提高要求: `(5, 0.6)` → 更严格
- 如果视觉抖动，可以放宽: `(2, 0.8)` → 更宽容

