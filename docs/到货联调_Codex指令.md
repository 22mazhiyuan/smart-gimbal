# 到货联调 Codex 指令（硬件到货当天用，用户转发）

> 前置：Muse 已把 SDBM-100 驱动的读阻塞 bug 修好并 push（3723450）。
> 先 `git pull` 确认最新提交是 3723450 再开始。

```
Codex，硬件到了，smart_gimbal 到货联调，一次做完。仓库 /home/jetson/visual_gimbal_lab/smart_gimbal，先 git pull 到最新（应看到 3723450）。


1）接线：模块供电 3.3V（不是 5V），TTL/UART 经 USB 转 TTL 接 Jetson。先别上电，把接线拍照发我确认。
2）上电后跑：python3 tools/rangefinder_test.py --port /dev/ttyUSBx --frames 10
（port 用 auto 或实际设备名；先试波特率 115200，不通再试 19200）
3）看输出：必须出现 raw= 开头的 13 字节 hex 帧；工具会同时算大端和小端两种距离，
拿一个已知距离的目标（如 5 米外的墙）对照，哪个准用哪个。
4）如果上电就自动往外吐帧（没发命令也有输出），记下来：config 里 poll_each_read 改 false。
5）定下来后改 config.yaml：rangefinder.backend=sdbm100，baud/byteorder/poll_each_read 按实测填，
然后 python3 bringup.py 确认测距项通过。
6）如果 30 秒没收到有效帧：检查接线、波特率、port 是否写对，把完整输出贴回来停下，不要猜。


1）插上后跑：udevadm info -a -n /dev/ttyUSBx | grep -i serial，记下序列号。
2）把序列号填进 config.yaml 和 99-ts6004-rs485.rules（走 /dev/ts6004_rs485 稳定路径）。
注意：serial 为空时禁止按 VID:PID 裸识别，旧云台转接器序列号 9cf94d227a89ef11a6c5ad95ef8776e9 已从配置移除，不要加回来。
3）/dev/ttyUSB0 是旧云台，/dev/ttyACM0 是飞控，两个都不许碰。


1）确认是定焦还是带 VCM 自动对焦：跑 v4l2-ctl -d /dev/video0 --list-ctrls，看有没有 focus_absolute；
有就是 VCM，没有就是定焦。
2）确认支持 MJPEG：v4l2-ctl -d /dev/video0 --list-formats-ext | grep MJPEG。
3）定焦的话：autofocus 保持关闭（3mm 景深极大，锁无穷远即可，VCM 链路本来也派不上用场）。
VCM 的话：把结果告诉我，驱动由 Muse 侧填实，不要自己写。


按仓库外文档 TS6004-10-Jetson调试代码.md 的守门顺序，一步一步来：
只读三帧全对（带地址/功能码/CRC 校验）→ 2~5° 空载点动并记录低头方向（=IBVS 方向符号）
→ 机械组装 → 断电配平（任意角度松手不倒）→ 带载小角度复测。
哪步不过停哪步，贴完整输出。GIMBAL_REAL_MOTION 保持 0，全程不开真实云台运动。


全部做完后：python3 bringup.py 全绿；SMART_GIMBAL_DETECTOR=ros_track timeout 60 python3 main.py
（镜头前放多旋翼）确认检测框和 du/dv 正常；最后报告五项逐项过/不过。
约束：不改仓库代码，只改 config.yaml 和 udev 规则；真实云台运动永不自启。
```
