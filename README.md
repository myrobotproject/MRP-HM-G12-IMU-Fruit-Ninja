# IMU Fruit Ninja

A dependency-free browser Fruit Ninja game controlled by HM-G11/HM-G12 IMU
attitude. The game uses the same Web Serial protocol and ESKF implementation
as the IMU dashboard.

## Run

Open [`fruit-ninja/index.html`](fruit-ninja/index.html) in desktop Chrome or
Edge. No build step, server, package manager, or installation is required.

The page loads the shared parser and ESKF files from `dashboard/`. Keep the
repository layout unchanged when copying the files.

## Controls

- Select the IMU model and matching baud/ranges.
- Click **Connect IMU**, choose the serial port, and hold the IMU still.
- Click **Set neutral pose** before starting.
- Yaw controls left/right (reversed for natural hand movement); roll controls
  up/down.
- The round allows up to 20 missed fruits. Slicing a bomb ends the round.

The default parser ranges are ±4000 dps and ±16 g, and the default attitude
source is ESKF. The game does not connect to a serial device automatically.
