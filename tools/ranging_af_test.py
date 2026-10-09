"""测距辅助对焦链路演示：模拟距离序列 -> 薄透镜算 DAC -> 推 VCM（mock）。
跑：python3 tools/ranging_af_test.py
无硬件跑通整条链路；真机时把 MockFocusDriver 换成 uvc/i2c，距离换成测距仪读数。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from drivers import focus as focus_mod
from focus.af_chain import RangingAF, displacement_um

CFG = {"autofocus": {"f_mm": 16.0, "dac_infinity": 512, "um_per_dac": 0.2,
                     "deadband_dac": 3, "max_rate_hz": 10, "driver": "mock"}}


def main():
    drv = focus_mod.create(CFG)
    drv.open()
    af = RangingAF(CFG, drv)
    # 模拟目标由远及近再远离；None 表示一次无效读数（保持不动）
    for d in [120.0, 80.0, 50.0, 30.0, 15.0, 8.0, None, 30.0, 120.0]:
        dac, moved = af.update(d, valid=d is not None)
        du = displacement_um(16.0, d) if d else 0.0
        tag = "推电机" if moved else "保持"
        print(f"距离 {str(d)+'m':>8} -> 位移 {du:6.2f}µm -> DAC {dac}  [{tag}]")
        time.sleep(0.15)
    drv.close()
    print("链路演示完成：距离->位移->DAC->电机，全通。")


if __name__ == "__main__":
    main()
