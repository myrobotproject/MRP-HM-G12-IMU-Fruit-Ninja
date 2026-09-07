<div align="center">

# IMU Fruit Ninja

**Use HM-G11 / HM-G12 motion data as a real-time fruit-slicing blade.**

<p>
  <a href="https://github.com/myrobotproject/MRP-HM-G12-IMU-Fruit-Ninja"><img src="https://img.shields.io/badge/dependencies-zero-2ea44f" alt="Zero dependencies"></a>
  <img src="https://img.shields.io/badge/Web%20Serial-Chrome%20%7C%20Edge-4285f4" alt="Web Serial support">
  <img src="https://img.shields.io/badge/ESKF-included-7c3aed" alt="ESKF included">
</p>

<p>
  <a href="fruit-ninja/index.html">Launch the game</a>
  &nbsp;·&nbsp;
  <a href="fruit-ninja/README.md">Game documentation</a>
</p>

</div>

## What it is

IMU Fruit Ninja is a self-contained browser game for HM-G11 and HM-G12
high-precision IMUs. The browser reads the sensor through Web Serial, computes
attitude with the bundled ESKF, and renders a responsive blade, fruit, bombs,
cut animations, particles, score, misses, and round timer on a canvas.

There is no framework, build step, package manager, server, or native helper.
The repository contains the HTML, CSS, JavaScript, protocol parser, and ESKF
implementation needed to run the game.

## Quick start

1. Use a desktop version of Chrome or Edge. Web Serial is not available in
   most mobile browsers.
2. Open [`index.html`](index.html), or open
   [`fruit-ninja/index.html`](fruit-ninja/index.html) directly.
3. Select the IMU model, baud rate, and sensor ranges to match the device.
4. Click **Connect IMU** and choose the serial port in the browser dialog.
5. Hold the device still during ESKF initialization, then click
   **Set neutral pose**.
6. Click **Start game** and move the IMU to aim. A quick wrist movement cuts
   fruit; slicing a bomb ends the round.

For Web Serial, use a secure context such as `https://` or `http://localhost`.
If opening a local file is blocked by the browser, serve this folder with any
static file server or open the hosted page instead.

## Motion controls

| IMU motion | Blade movement |
| --- | --- |
| Yaw | Left / right |
| Roll | Up / down |
| Fast attitude change | Blade trail and cut direction |

Yaw is reversed in the game so the physical movement feels natural. The blade
can reach the full play area and follows the actual pointer movement vector.
The neutral-pose button makes the current orientation the centre position.

## Attitude and protocol

- **ESKF** is the default attitude source and uses raw gyroscope and
  accelerometer samples.
- **Built-in attitude** is available for HM-G12 devices that provide it.
- **Gyro integration** is available as a lightweight fallback.
- Default parser ranges are **±4000 dps** and **±16 g**; change them when the
  device configuration differs.
- The game uses the same HM-G11 request/response framing, HM-G12 raw stream
  framing, and CRC checks as the dashboard.
- The serial port is never opened automatically. The user must select it in
  the browser's port picker.

## Repository layout

```text
.
├── index.html              # Root entry point
├── fruit-ninja/
│   ├── index.html          # Game page
│   ├── game.js             # Game loop, rendering, controls, Web Serial glue
│   ├── style.css           # Responsive full-screen layout
│   └── README.md           # Detailed game notes
└── dashboard/
    ├── protocol.js         # HM-G11 / HM-G12 parser and CRC handling
    └── eskf.js             # Browser ESKF attitude estimator
```

## Troubleshooting

**The serial picker does not appear**

Use desktop Chrome or Edge from `localhost` or HTTPS. Do not open another page
that already owns the same serial port.

**The blade moves in the wrong direction**

Hold the IMU in the desired starting pose and click **Set neutral pose** again.
Confirm that the selected model, baud rate, and sensor ranges match the device.

**The status shows CRC or ESKF errors**

Check the selected device profile and baud rate first. The frame counter and
error counters are shown in the bottom status bar without stopping the game.

## License

The project is distributed under the [Apache License 2.0](LICENSE).
