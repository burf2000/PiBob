# PiBob

A Raspberry Pi-powered robot with dual arms controlled via a web interface. PiBob uses an Adafruit PCA9685 servo hat to drive 16 PWM channels, with a Flask web app for real-time control.

## Hardware

- Raspberry Pi (accessible at `pibob.local`)
- Adafruit PCA9685 16-channel servo hat
- 10 servos (2 head + 8 arm)

### Arm Channel Mapping

| Channel | Joint | Notes |
|---------|-------|-------|
| CH2 | Head Rotation | Left/right, full range (0°–180°) |
| CH3 | Head Tilt | Up/down (50°–140°) |
| CH4 | Right Bicep | 90° = straight, 180° = fully bent |
| CH5 | Right Arm Rotation | Reversed, side-to-side |
| CH6 | Right Shoulder Lift | Reversed, chicken wing style |
| CH7 | Right Shoulder Rotation | Reversed, arm up/down front |
| CH8 | Left Bicep | Reversed, mirrors right |
| CH9 | Left Arm Rotation | Side-to-side |
| CH10 | Left Shoulder Lift | Chicken wing style |
| CH11 | Left Shoulder Rotation | Arm up/down front |

## Features

- **16-channel control** — servo, continuous rotation, or raw PWM modes per channel
- **Web UI** — real-time sliders, enable/disable, reverse direction, pulse width tuning
- **Safe limits** — configurable min/max per channel, persisted to `config.json`
- **Presets** — save and load servo positions
- **Gestures** — pre-programmed movements with smooth interpolation:
  Demo, Wave, Point, Run, Scared, Angry, Celebrate, Shrug, Dab, Fight, Dance
- **Sweep test** — automatically sweep all servos through their range
- **PWM frequency** — adjustable from 24–1526 Hz

## Setup

### On the Raspberry Pi

1. Install dependencies:
   ```bash
   pip install flask adafruit-circuitpython-servokit
   ```

2. Copy `app.py` to the Pi:
   ```bash
   scp app.py burf2000@pibob.local:/home/burf2000/servo-control/app.py
   ```

3. Run the app (requires root for port 80):
   ```bash
   sudo python3 app.py
   ```

### Systemd Service

The app runs as `servo-control.service` on the Pi. After deploying a new version:

```bash
sudo systemctl restart servo-control.service
```

## API

All endpoints accept JSON via POST.

| Endpoint | Description |
|----------|-------------|
| `GET /` | Web UI |
| `POST /set` | Set channel value `{channel, value, mode}` |
| `POST /mode` | Change channel mode `{channel, mode}` |
| `POST /enable` | Enable/disable channel `{channel, enabled}` |
| `POST /reverse` | Toggle reverse `{channel, reversed}` |
| `POST /pulse` | Set pulse width range `{channel, min_pulse, max_pulse}` |
| `POST /config` | Update channel name/limits `{channel, name?, safe_min?, safe_max?}` |
| `POST /frequency` | Set PWM frequency `{frequency}` |
| `POST /gesture` | Run a gesture `{name}` |
| `POST /sweep` | Start/stop sweep `{active}` |
| `GET /state` | Get current state of all channels |
| `POST /presets/save` | Save preset `{name, state}` |
| `POST /presets/load` | Load preset `{name}` |
| `POST /presets/delete` | Delete preset `{name}` |
