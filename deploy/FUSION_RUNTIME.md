# Detection-service fusion wiring

Active directories have no date suffix:

- `/home/jetson/multirotor_tracker` — existing FusionCore and official FEAR runtime/assets.
- `/home/jetson/visual_gimbal_lab/multirotor_accuracy_repair` — current model, camera/RTSP/ROS runtime.

`sudo bash deploy/install_detection.sh` copies the three reviewed detector-side
files to the existing runtime and restarts the shared RTSP/ROS service plus its
consumer. Model/checkpoint bytes are not overwritten or downloaded. The canonical
`ros_track.service` has alias `rtsp_stream.service`; never run duplicate publishers.

`fusion_bridge.py` imports the existing `FusionCore` in `YOLO_FEAR_KALMAN` mode,
uses the current single MULTIROTOR YOLO weight, and requires the pinned official
FEAR checkpoint/blocks SHA256 and CUDA. There is no silent YOLO-only fallback.
The camera and RTSP component still have a single owner in this same process.

`config.yaml:target_select` is actually read by the bridge:

- `kalman_process_noise` is variance multiplying `G(dt)G(dt)^T` (Q).
- `kalman_measure_noise` is measurement variance multiplying `I4` (R).
- `predict_horizon_s` caps the existing tracker's coast limit.
- `recapture_iou_thresh` feeds the existing spatial-association gate.

Existing detector, tracker confidence and verification policy are retained; this
commit is wiring, not threshold optimization or a precision claim.

`/counter_uav/tracking/fusion_status` (std_msgs/String JSON) reports the exact
Track sequence, source, config, metrics, and hidden-prediction status. Green
`[YOLO]` is a current detector observation, amber `[FEAR]` is a measured tracker
observation with recent YOLO class verification. Pure Kalman predictions are
hidden and are never `is_primary` or Detection2D observations. Unknown provenance
is fail-closed after fusion metadata is present.

`GIMBAL_REAL_MOTION=0` remains explicit. No actuator drivers, IBVS, rangefinder,
or autofocus code is changed. Software fixtures do not replace live target
continuity or labelled accuracy validation. Historical evidence retains its
original paths; those historical strings are not runnable references.
