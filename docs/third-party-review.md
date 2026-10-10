# 第三方代码评审（2026-10-10）

8 个仓库已克隆到 Muse 本地 `~/workspace/vendor/` 逐个读过。结论先行：

## 1. 测距仪：我们的 SDBM-100 驱动被独立第三方代码逐字节验证

`wuihee/serial-sensors` → `src/.../laser_bb2x_jrt.py`（JRT BB2X，MIT）：
- 13 字节帧、帧头 0xAA、校验 `sum(bytes[1:-1]) % 256 == last`
- 距离 = bytes[6:10]（毫米）、信号强度 = bytes[10:12]

与我们的 `drivers/rangefinder.py` → `SDBM100.parse_frame` **逐字节一致**。
结论：SDBM-100 大概率同协议，我方驱动可信度大幅提高；到货后
`python3 tools/rangefinder_test.py --port /dev/ttyUSBx` 看原始 hex 确认即可。

`aerospacejam/tfluna`（MIT）：TF-Luna 驱动。协议不同（用不上），但它的串口
健壮性写法（timeout、读前清 buffer）值得对照我们的驱动补一下。

## 2. VCM 对焦：I2C 写 DAC 模式被两家独立仓库验证，已填入驱动

- `arducam/jetson_imx708_focus_example/Focuser.py`
- `ArduCAM/RaspberryPi` → `Motorized_Focus_Camera/python/Focuser.py`

两家独立实现完全一致：I2C 地址 0x0C，10 位 DAC 拆两字节写
（高 2 位 → 寄存器 0x03，低 8 位 → 寄存器 0x04）。
已按此填实 `drivers/focus.py` → `I2CFocusDriver`，地址/寄存器/位数全部可配
（`i2c_addr`/`i2c_reg_msb`/`i2c_reg_lsb`/`i2c_bits`），实物不对改配置即可。
注意：这是 ArduCAM 自家模组方案；用户摄像头硬件未知，上电先 `i2cdetect`
确认地址，再小步推 DAC 看镜头是否动。

`ArduCAM/.../Autofocus.py`：Laplacian 清晰度 + ROI + 步进扫描，
与我们的 `tools/autofocus.py` 同思路，可互相印证。
`minindupasan/imx708-jetson`：内核/ISP 层移植，太深，用不上。

许可证注意：ArduCAM 仓库没找到 LICENSE 文件——只借鉴思路，不直接拷贝代码。

## 3. 云台：硬件不同，只做架构参考（Muse 已评审，不发 Codex）

- `ysterbal/drone_turret_v2`（MIT）：YOLOv8+ByteTrack + TFMini 测距 + 双轴 PID，
  全网最接近的完整系统。但它是 Arduino/PWM 舵机，我们是 Modbus 伺服，
  驱动层不能移植；PID 整定思想可参考。
- `maxboels/TrackingPanTiltCam`：Jetson 上实测过的 YOLO 云台，同上，硬件层不同。
- `wcy-creator/pigimbal`（MIT）：pip 库，SG90 小舵机，不适用 TS6004。

结论：我们的 TS6004 Modbus 驱动已按手册写完，这三家不移植。

## 4. 开源缺口（实话）

测距 → 对焦位置的闭环代码，开源圈没有现成的（只有测距驱动和对焦驱动，
没有连起来的）。我们的 `focus/af_chain.py`（距离→薄透镜位移→DAC→死区/限频）
正是补这个缺口的。

## 给 Codex 的评审范围

只看第 1、2 节（测距串口健壮性、对焦调参流程），第 3 节云台指向链路
按其安全边界跳过。视频/检测相关它自己定。
