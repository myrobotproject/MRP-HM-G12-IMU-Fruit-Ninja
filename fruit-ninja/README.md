# IMU Fruit Ninja

This folder contains the playable page. It is dependency-free and uses the
HM-G11 / HM-G12 attitude stream to control a canvas blade.

## Run

Open `index.html` in desktop Chrome or Edge. For Web Serial, use `localhost`
or HTTPS if the browser blocks `file://` serial access.

1. Choose the IMU model and matching baud/ranges.
2. Click **Connect IMU** and select the port.
3. Hold the device still while ESKF initializes.
4. Click **Set neutral pose**.
5. Click **Start game**.

The page never connects to a serial device automatically. A mouse can move the
blade before an IMU is connected, which is useful for checking the game loop.

## Controls

- Yaw controls screen left/right. The yaw sign is reversed for natural motion.
- Roll controls screen up/down.
- The blade angle follows the actual movement vector.
- The round lasts 120 seconds and allows 20 missed fruits.
- Slicing a bomb ends the round.

## Attitude sources

- **ESKF**: default; estimates attitude from raw gyro and accelerometer data.
- **Built-in**: HM-G12 attitude frames when available.
- **Gyro integration**: lightweight fallback based on the device timestamp.

The parser defaults to ±4000 dps and ±16 g. Update the selectors when the
device is configured differently. Shared browser code is loaded from
`../dashboard/protocol.js` and `../dashboard/eskf.js`.
