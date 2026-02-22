# apple-silicon-accelerometer

more information: [read the article on Medium](https://medium.com/@oli.bourbonnais/your-macbook-has-an-accelerometer-and-you-can-read-it-in-real-time-in-python-28d9395fb180)

modern MacBook Pros have an undocumented MEMS accelerometer + gyroscope managed by the Sensor Processing Unit (SPU).
this project reads both via IOKit HID, along with lid angle and ambient light sensors from the same interface.

![demo](assets/demo.gif)

## what is this

Apple Silicon chips (M1/M2/M3/M4/M5) include a MEMS IMU (accelerometer + gyroscope) managed by the SPU.
it's not exposed through any public API or framework.
this project reads raw 3-axis acceleration and angular velocity data at ~800 Hz via IOKit HID callbacks.

only tested on MacBook Pro M5 — might work on other Apple Silicon Macs but no guarantees.

## how it works

the sensor lives under `AppleSPUHIDDevice` in the IOKit registry, on vendor usage page `0xFF00`.
usage 3 is the accelerometer, usage 9 is the gyroscope (same physical IMU, believed to be Bosch BMI286 based on teardowns).

we open it with `IOHIDDeviceCreate` and register an async callback via `IOHIDDeviceRegisterInputReportCallback`.
data comes as 22-byte HID reports with x/y/z as int32 little-endian at byte offsets 6, 10, 14.
divide by 65536 to get g (accel) or deg/s (gyro). callback rate is ~100 Hz (decimated from ~800 Hz native).

orientation is computed by fusing accel + gyro with a Mahony AHRS quaternion filter, displayed as roll/pitch/yaw gauges.

verify the device exists on your machine:

    ioreg -l -w0 | grep -A5 AppleSPUHIDDevice

## quick start

    git clone https://github.com/Ruchi12377/apple-silicon-accelerometer
    cd apple-silicon-accelerometer
    pip install -e .
    python3 motion_live.py

### with uv

    uvx git+https://github.com/Ruchi12377/apple-silicon-accelerometer.git

## code structure

| file             | description                                                                                    |
| ---------------- | ---------------------------------------------------------------------------------------------- |
| `spu_sensor.py`  | IOKit bindings, device discovery, accel/gyro/lid/ALS HID callbacks, shared memory ring buffers |
| `motion_live.py` | vibration detection pipeline, heartbeat BCG, terminal UI, main loop                            |
| `tilt_maze.py`   | Panda3D ball-in-maze game controlled by tilt                                                   |
| `models/`        | 3D assets for the maze demo (`ball.egg.pz`, `maze.egg.pz`)                                     |

the sensor logic is isolated in `spu_sensor.py` so you can import and reuse it independently.

## heartbeat demo

place your wrists on the laptop near the trackpad and wait 10–20 seconds for the signal to stabilize.
this uses ballistocardiography — mechanical vibrations from your heartbeat transmitted through your arms into the chassis.
experimental, not reliable, just a fun use-case to show what the sensor can pick up.
BCG bandpass is 0.8–3 Hz; BPM is estimated via autocorrelation on the filtered signal.

## Panda3D tilt maze

guide a ball through a labyrinth by tilting your MacBook — falls into holes if you're not careful.

![maze demo](assets/demo_ball.gif)

    pip install -e .
    tilt-maze

arrow keys also work if sensor access fails.

## notes

- experimental / undocumented AppleSPU HID path
- may break on future macOS updates
- use at your own risk
- not for medical use

## tested on

- MacBook Pro M5, macOS 15.6.1
- Python 3.14

## license

BSD-3-Clause

---

not affiliated with Apple or any employer
