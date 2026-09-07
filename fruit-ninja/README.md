# IMU Fruit Ninja

`index.html` is a dependency-free browser game for HM-G11 and HM-G12. The
IMU orientation controls a blade on the canvas: move the sensor to aim and
make a quick wrist movement to slice fruit. A successful hit splits the fruit
into two falling halves with a short separation animation. Bombs end the round
and 20 missed fruits also end the round.

## Run

Open [`index.html`](index.html) in desktop Chrome or Edge. No server, package
manager, or installation is required. Click **Connect IMU**, choose the serial
port, hold the sensor still for about five seconds, and click **Set neutral
pose**. The default attitude source is **ESKF**; HM-G12 also offers built-in
attitude and both devices offer gyro integration.

The game shares the SDK's browser protocol and ESKF implementation from
`../dashboard/protocol.js` and `../dashboard/eskf.js`. It uses the same Web
Serial framing, CRC checks, HM-G11 request/response flow, and HM-G12 stream
command as the dashboard. A mouse can move the blade before an IMU is
connected, which makes it possible to verify the game without hardware.

The parser defaults to the full sensor ranges (±4000 dps and ±16 g). Change
the selectors if the device was configured differently. With the SDK body
axes (X=right, Y=forward, Z=up), the control mapping is yaw for screen X and
roll for screen Y. Yaw is reversed so rotating left moves the blade left on
screen. Both axes use the same angle and pixel sensitivity; the common pixel
span is based on the longer screen dimension so the blade can reach the full
play area. The blade direction follows the actual movement vector, as it would
for a hand-held knife. Set a neutral pose after holding the device in the
desired starting orientation.

When ESKF is selected, the game shows gyro-integrated angles immediately while
the filter collects its stationary initialization window, then switches to the
ESKF quaternion automatically. A parser or ESKF error is kept from stopping
the serial reader; the live integrated angle remains available as a fallback.

## Controls

- **Connect IMU**: opens the browser serial-port picker and starts streaming.
- **Set neutral pose**: records the current orientation as the blade centre.
- **Start/Pause game**: starts or pauses a 120-second round. The round ends
  after 20 missed fruits.
- Device, baud rate, and sensor ranges must match the device configuration.

If the browser reports that a port is busy or cannot be opened, close or
disconnect the dashboard/fruit-game page that currently owns it, then refresh
this page. Only one browser page can open a serial port at a time. Chrome or
Edge on `localhost` is recommended for Web Serial; opening the file directly
may be blocked by browser policy.
