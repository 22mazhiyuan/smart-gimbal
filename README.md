# 机载智能跟踪云台 · 软件框架（第 1 周基线）

按需求文档 §1–§8 实现的整体框架：**一条命令启动、同一套配置、同一个 frame_id 全链路**。
mock 与 real 共用同一套上层接口，只改配置，不另写逻辑。

## 跑起来

```bash
pip install pyyaml opencv-python pyserial   # Jetson 上按需装
python3 bringup.py    # 联调自检：依赖→配置→视频→串口→云台只读
python3 main.py       # 一条命令启动全部
# 手机/电脑浏览器打开 http://<jetson-ip>:8080/video 看实时画面
# Ctrl+C 退出：先停转，再关串口/视频/网页
```

真实云台：`GIMBAL_REAL_MOTION=1 python3 main.py`（先过守门顺序，软限位空着拒绝启动）。

## 开机自启

```bash
sudo bash deploy/install_autostart.sh   # 装好后通电自动跑 main.py，崩溃自动重启
```

默认全 mock，真实云台运动永不自启。依赖 Codex 的 `rtsp_stream.service` /
`ros_track.service` 先起（已写 After 顺序）。停用：`sudo systemctl disable --now smart_gimbal`。

## 目录

| 文件 | 对应文档 |
|---|---|
| `config.yaml` | §8 全套参数，`schema_version` 校验 |
| `config_center.py` | §3 配置中心：校验+打印生效值 |
| `main.py` | §1 一条命令启动：配置→视频→模型→串口→日志 |
| `state_machine.py` | §3 五态机：IDLE/TRACK/LOST/ERROR/STOPPED，变迁记时间+原因 |
| `ibvs.py` | §4 IBVS：Kp/死区/限速/**变化率限制**/方向符号 |
| `drivers/ts6004_driver.py` | §5 云台驱动：位置模式+位置增量，无速度寄存器 |
| `drivers/rangefinder.py` | §6 测距：mock 占位显示"距离无效"；SDBM-100 接口已固定待对接 |
| `video/source.py` | §3 视频源：RTSP（只读流，不碰 /dev/video0）/mock；断流归零重连 |
| `video/mjpeg_server.py` | §3/§7 HTTP MJPEG：框/类别/du/dv/状态/FPS/距离/mock-real |
| `detection/interface.py` | §2 检测接口：`DetectionResult(frame_id, bbox, du_px, dv_px, visible)` |
| `runlog/run_logger.py` | §3/§7 运行日志：独立目录+配置快照+延迟统计（中位数/P95/最大） |
| `bringup.py` | §5 联调自检 |
| `tools/focus_assist.py` | 对焦辅助：实时清晰度分数，手动转镜头调到峰值锁死 |
| `tools/autofocus.py` | 自动对焦框架：清晰度评价+爬山搜索+电机抽象 |
| `drivers/focus.py` | VCM 音圈调焦驱动：mock/uvc/i2c |
| `focus/af_chain.py` | 测距辅助对焦链路：距离->薄透镜位移->DAC->电机（含死区/限频） |
| `tools/ranging_af_test.py` | 链路演示：模拟距离序列跑通全链路 |

## 已对接 / 待对接

已按 Codex 实测填好：
- 检测：`model.backend=ros_track` 订阅 `/counter_uav/tracking/primary_target`
 （`anti_drone_interfaces/msg/Track`），转 `DetectionResult` 进统一 frame_id 链路；
  `backend=mock` 切回虚拟目标
- 串口：VID:PID=`10c4:ea60` + 序列号已进配置，`bringup.py` 自动识别；
  稳定路径见 `99-ts6004-rs485.rules`（先插拔确认序列号是转接器再安装）
- 6022/6024/6026：手册 10-08 已提供并审计——6022=最大速度、6024=加速度、
  6026=减速度（曲线参数）；驱动无速度寄存器，速度经位置增量下发

待 Codex / 到货：
1. **SDBM-100 协议**：实现 `drivers/rangefinder.py` 的 `SDBM100`，按实物资料填串口解析
2. **插拔确认串口**：确认序列号是 RS485 转接器（不是旧云台），再装 udev 规则
3. **3mm 相机到货**：`video.source` 切真实采集；SDBM-100 到货切 `rangefinder.backend=sdbm100`

## 安全（文档 §5，已 baked in）

- 6022 是最大速度曲线参数，无速度寄存器——速度经位置增量下发
- 速度 0 = 目标冻结停转；急停写 6006=0；限位触发/串口异常走急停
- 丢目标超时速度归零；断流/模型异常/CRC 错统一清零进 ERROR
- 软限位空着 real 模式拒绝启动；端口边界 ttyUSB0/ttyACM0 勿碰
