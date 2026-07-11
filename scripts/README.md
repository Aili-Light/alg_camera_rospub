## Usage

```bash
# After building and sourcing:
source install/setup.bash

# Record for 10 seconds with default topics:
python3 install/alg_camera_rospub/lib/alg_camera_rospub/validate_topics.py --duration 10

# Custom topics and output directory:
python3 install/alg_camera_rospub/lib/alg_camera_rospub/validate_topics.py \
  --camera-topic /image \
  --imu-topic /imu/data \
  --output-dir ./my_recordings \
  --duration 30

# Run until Ctrl+C:
python3 install/alg_camera_rospub/lib/alg_camera_rospub/validate_topics.py
```
