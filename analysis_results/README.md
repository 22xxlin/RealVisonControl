# Analysis Results - Core Files Only

## 📊 Data Files (943 records total)
- `log_sysID_20251218_173613.csv` (322 records)
- `log_sysID_20251218_202116.csv` (286 records)
- `log_sysID_20251218_202711.csv` (335 records)

## 📈 Visualization
- `car_distance_comparison.png` - Vehicle: Vision vs Actual (by CSV & Camera)
- `ball_distance_comparison.png` - Ball: Vision vs Actual (by CSV & Camera)
- `linear_correction_analysis.png` - Linear correction analysis

## 🔧 Scripts
- `plot_distance_comparison.py` - Generate comparison plots
- `linear_correction_analysis.py` - Linear correction analysis
- `auto_calibration.py` - Auto calibration tool

## 📖 Documentation
- `ANALYSIS_SUMMARY.md` - Key findings & recommendations
- `LINEAR_CORRECTION_GUIDE.md` - Correction parameters guide

## 🚀 Usage
```bash
python3 plot_distance_comparison.py  # Generate plots
python3 linear_correction_analysis.py  # Run analysis
```

## 🎯 Key Info
- **Cameras**: 0, 2, 6
- **Targets**: CAR (fusion method), BALL (width method)
- **Correction**: Linear (slope * raw + intercept)
