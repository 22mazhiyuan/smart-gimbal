"""自动对焦框架：清晰度评价 + 爬山搜索 + 调焦电机抽象。
跑：python3 tools/autofocus.py          # 无电机：指导模式，按提示手动转镜头
    python3 tools/autofocus.py --auto   # 有电机：闭环自动对焦（先实现 StepperFocusMotor）

三条路（代码调焦距/调焦的前提都是有执行机构）：
  1) UVC 标准控制——模组自带调焦马达才有效，下单前问卖家有没有马达版：
       v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0 --set-ctrl=focus_absolute=120
     或 cap.set(cv2.CAP_PROP_AUTOFOCUS, 0); cap.set(cv2.CAP_PROP_FOCUS, 120)
     （M12 长焦基本没有马达版，这条路大概率走不通，先问再说）
  2) 换电动变焦一体机：自带变焦+调焦马达，UVC 或串口（VISCA）控制，又贵又重。
  3) 自己加步进电机拧 M12 镜头：实现下面的 StepperFocusMotor.move() 即可闭环。
"""
import os
import sys
import time

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

from config_center import load_config
from video import source as video_source

try:
    import cv2
except ImportError:
    sys.exit("没装 opencv")

from tools.focus_assist import sharpness  # 复用清晰度评价


class FocusMotor:
    """调焦电机抽象：move(steps)，steps>0 往远调，<0 往近调。"""
    def move(self, steps: int):
        raise NotImplementedError

    @property
    def position(self) -> int:
        return 0


class ManualMotor(FocusMotor):
    """无电机：指导模式，打印提示等人手转镜头。"""
    def __init__(self):
        self._pos = 0

    def move(self, steps: int):
        direction = "往远拧一点" if steps > 0 else "往近拧一点"
        input(f"  [手动] {direction}（约 {abs(steps)} 格），转完回车继续…")
        self._pos += steps

    @property
    def position(self) -> int:
        return self._pos


class StepperFocusMotor(FocusMotor):
    """步进电机拧 M12 镜头：Jetson 侧实现（GPIO/串口驱动器二选一）。

    接线示例（DRV8825）：STEP->GPIOxx, DIR->GPIOyy, EN->GPIOzz，使能后按步数发脉冲。
    实现 move()：按 steps 符号置方向，按 abs(steps) 发脉冲，每步后留 30~50ms 消振。
    """
    def __init__(self):
        raise NotImplementedError("Jetson 侧按上面注释接好电机驱动后实现 move()")

    def move(self, steps: int):
        raise NotImplementedError


def autofocus_once(motor: FocusMotor, measure, step=8, settle=0.15, max_iter=60):
    """爬山对焦：测 -> 试探方向 -> 沿变清晰方向走 -> 过峰回退半步。返回峰值清晰度。"""
    def m():
        time.sleep(settle)
        return measure()
    s0 = m()
    motor.move(step)
    s1 = m()
    direction = 1 if s1 > s0 else -1
    if direction < 0:
        motor.move(-step)  # 试探错了，回起点
    prev, it = m(), 0
    while it < max_iter:
        motor.move(direction * step)
        s = m()
        it += 1
        if s < prev * 0.995:  # 过峰（带一点回差防抖动）
            motor.move(-direction * max(1, step // 2))
            break
        prev = s
    peak = m()
    print(f"对焦完成：峰值清晰度 {peak:.1f}，电机位置 {motor.position}")
    return peak


def main():
    auto = "--auto" in sys.argv
    cfg = load_config()
    src = video_source.create(cfg)
    src.open()
    frame_holder = {}

    def measure():
        for _ in range(30):  # 最多等 3 秒取一帧
            f = src.read()
            if f is not None:
                break
            time.sleep(0.1)
        if f is None:
            raise RuntimeError("取不到视频帧")
        gray = cv2.cvtColor(f.image, cv2.COLOR_BGR2GRAY)
        frame_holder["last"] = f.image
        return sharpness(gray)

    try:
        if auto:
            motor = StepperFocusMotor()  # 电机没实现会直接抛 NotImplementedError
            print("闭环自动对焦启动…")
        else:
            motor = ManualMotor()
            print("指导模式：按提示手动转镜头（M12 无电机，代码动不了镜头，只能测清晰度）。")
            print("对准远处目标（100m 外），开始。")
        autofocus_once(motor, measure)
        print("锁死镜头，对焦结束。")
    finally:
        src.close()


if __name__ == "__main__":
    main()
