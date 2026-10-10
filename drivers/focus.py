"""VCM 音圈调焦驱动：把链路算出的 DAC 写给电机。
VCM 调的是对焦距离（整体移动镜片改像距），焦距 f 不变。
三条硬件路（config autofocus.driver 二选一）：
  mock：只记录不发送，联调用。
  uvc ：USB 自动对焦模组（VCM 在模组固件里），走 UVC 标准控制。
        注意 smart_gimbal 吃的是 RTSP 流，不直连 /dev/video0；但 V4L2 允许
        另开 fd 设 control，Codex 推流器占用取流时一般不影响设 control：
          v4l2-ctl -d /dev/video0 --set-ctrl=focus_auto=0 --set-ctrl=focus_absolute=<dac>
        先 v4l2-ctl --list-ctrls 看 focus_absolute 量程，标定时对齐。
  i2c ：MIPI/DIY 的 VCM 驱动芯片，Jetson I2C 总线直写 DAC。
        下面是 DW9800 类芯片的常见写法，仅作参考，上机前按你的芯片手册核对
        地址和寄存器（10bit DAC 常拆两个寄存器）。
"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))


class VCMDriver:
    def open(self): ...
    def close(self): ...
    def set_dac(self, dac: int): raise NotImplementedError


class MockFocusDriver(VCMDriver):
    def open(self): print("[mock] VCM 驱动占位：DAC 只记录不发送")
    def close(self): pass
    def set_dac(self, dac: int):
        print(f"[mock] VCM DAC <- {dac}")


class UVCFocusDriver(VCMDriver):
    """USB 自动对焦模组：UVC focus_absolute 直写。"""
    def __init__(self, video_dev="/dev/video0"):
        self.dev = video_dev
        self._cap = None

    def open(self):
        import cv2
        self._cap = cv2.VideoCapture(self.dev)
        if not self._cap.isOpened():
            raise RuntimeError(f"打不开 {self.dev}")
        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)  # 关模组自对焦，链路自己算
        print(f"[VCM] UVC 对焦控制就绪：{self.dev}")

    def close(self):
        if self._cap:
            self._cap.release()

    def set_dac(self, dac: int):
        self._cap.set(cv2.CAP_PROP_FOCUS, int(dac))


class I2CFocusDriver(VCMDriver):
    """Jetson I2C 直驱 VCM。

    写 DAC 模式按两家独立仓库验证过的同一方案
    （arducam/jetson_imx708_focus_example/Focuser.py、
     ArduCAM/RaspberryPi Motorized_Focus_Camera/python/Focuser.py）：
    I2C 地址 0x0C，10 位 DAC 拆两字节写（高 2 位 -> 寄存器 0x03，低 8 位 -> 0x04）。
    地址/寄存器/位数做成参数：你的模组若不是这个方案，改配置即可。
    上电先 i2cdetect 确认地址，再小步推 DAC 看镜头是否动。"""

    def __init__(self, bus=1, addr=0x0C, reg_msb=0x03, reg_lsb=0x04, bits=10):
        self.bus, self.addr = bus, addr
        self.reg_msb, self.reg_lsb, self.bits = reg_msb, reg_lsb, bits

    def open(self):
        print(f"[VCM] I2C：bus={self.bus} addr={hex(self.addr)} "
              f"reg={hex(self.reg_msb)}/{hex(self.reg_lsb)} {self.bits}bit")

    def close(self): pass

    def set_dac(self, dac: int):
        import subprocess
        dac = max(0, min((1 << self.bits) - 1, int(dac)))
        msb, lsb = (dac >> 8) & 0x03, dac & 0xFF
        a = f"0x{self.addr:02X}"
        subprocess.run(["i2cset", "-y", str(self.bus), a,
                        f"0x{self.reg_msb:02X}", str(msb)], check=True)
        subprocess.run(["i2cset", "-y", str(self.bus), a,
                        f"0x{self.reg_lsb:02X}", str(lsb)], check=True)


def create(cfg):
    c = cfg.get("autofocus", {})
    kind = c.get("driver", "mock")
    if kind == "uvc":
        return UVCFocusDriver(c.get("video_dev", "/dev/video0"))
    if kind == "i2c":
        return I2CFocusDriver(c.get("i2c_bus", 1), c.get("i2c_addr", 0x0C),
                             c.get("i2c_reg_msb", 0x03), c.get("i2c_reg_lsb", 0x04),
                             c.get("i2c_bits", 10))
    return MockFocusDriver()
