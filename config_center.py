"""配置中心：集中管理、启动校验、打印生效值（文档 §3）。"""
import sys
import yaml

SCHEMA_VERSION = "1.0"
REQUIRED_TOP = ["video", "model", "target_select", "ibvs", "gimbal",
                "rangefinder", "state_machine", "logging"]


def load_config(path="config.yaml"):
    with open(path, encoding="utf-8") as f:
        cfg = yaml.safe_load(f)
    ver = str(cfg.get("schema_version", ""))
    if ver != SCHEMA_VERSION:
        sys.exit(f"配置版本不符：文件 {ver}，程序要 {SCHEMA_VERSION}，先升级配置")
    missing = [k for k in REQUIRED_TOP if k not in cfg]
    if missing:
        sys.exit(f"配置缺顶级段：{missing}")
    g = cfg["gimbal"]
    if g.get("mode") == "real" and not g.get("soft_limit_deg"):
        sys.exit("real 模式必须先实测机械范围并填 gimbal.soft_limit_deg，空着不许启动")
    print("== 生效配置 ==")
    print(f"  视频: {cfg['video']['source']} {cfg['video'].get('rtsp_url','')}")
    print(f"  模型: {cfg['model']['backend']}  云台: {cfg['gimbal']['mode']}"
          f"  测距: {cfg['rangefinder']['backend']}")
    print(f"  IBVS: kp={cfg['ibvs']['kp']} 死区={cfg['ibvs']['deadzone_px']}px"
          f" 限速={cfg['ibvs']['max_vel_dps']}°/s 俯仰符号={cfg['ibvs']['dir_sign_pitch']}")
    print(f"  画面: http://<jetson-ip>:{cfg['video']['mjpeg_port']}/video")
    return cfg
