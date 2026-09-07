(function () {
  "use strict";

  var Protocol = window.ImuProtocol;
  var Eskf = window.ImuEskf;
  var canvas = document.getElementById("gameCanvas");
  var context = canvas.getContext("2d");
  var elements = {
    device: document.getElementById("deviceSelect"),
    attitude: document.getElementById("attitudeSelect"),
    baud: document.getElementById("baudSelect"),
    gyroRange: document.getElementById("gyroRangeSelect"),
    accelRange: document.getElementById("accelRangeSelect"),
    connect: document.getElementById("connectButton"),
    calibrate: document.getElementById("calibrateButton"),
    start: document.getElementById("startButton"),
    status: document.getElementById("connectionStatus"),
    score: document.getElementById("scoreValue"),
    misses: document.getElementById("missValue"),
    time: document.getElementById("timeValue"),
    frames: document.getElementById("frameReadout"),
    input: document.getElementById("inputReadout"),
    overlay: document.getElementById("overlay"),
    overlayTitle: document.getElementById("overlayTitle"),
    overlayMessage: document.getElementById("overlayMessage"),
    overlayButton: document.getElementById("overlayButton")
  };
  var ROUND_SECONDS = 120;
  var MAX_MISSES = 20;
  var CONTROL_ANGLE_SCALE = 60;
  var CONTROL_SCREEN_SPAN = .44;
  // How quickly the blade catches up with the IMU-derived target.  A higher
  // value makes fast wrist movements feel more immediate while retaining a
  // small amount of smoothing for noisy attitude samples.
  var POINTER_RESPONSE = 30;

  var state = {
    width: 0, height: 0, dpr: 1, lastFrameAt: 0, running: false, gameOver: false,
    score: 0, misses: 0, timeLeft: ROUND_SECONDS, spawnAt: 0, fruits: [], particles: [], slashes: [], trail: [],
    pointer: { x: 0, y: 0, targetX: 0, targetY: 0, angle: 0 },
    port: null, reader: null, writer: null, active: false, connecting: false, readTask: null, serialBuffer: new Uint8Array(0),
    // The game is intended for the full-range sensor configuration.  Keep these
    // values in sync with the parser until a device configuration is read.
    parser: null, device: "hm-g12", attitudeSource: "eskf", gyroRange: 4000, accelRange: 16,
    integrationQuaternion: [1, 0, 0, 0], eskf: new Eskf.AttitudeEskf(), latestQuaternion: [1, 0, 0, 0],
    neutralQuaternion: [1, 0, 0, 0], lastTimerUs: null, frameCount: 0, crcErrors: 0, processingErrors: 0,
    rng: 0x41c6ce57
  };

  var FRUIT_COLORS = ["#ff5a64", "#ffad43", "#ffdf55", "#71e36f", "#50c9ff", "#c17bff"];
  var FRUIT_NAMES = ["apple", "orange", "lemon", "lime", "blue", "plum"];

  // Body-frame convention used by the SDK and the IMU mouse example:
  // X points right, Y points forward, Z points up. Yaw drives screen X and
  // roll drives screen Y. Both axes use the same scale and screen span.
  // The blade angle itself is always taken from the actual pointer movement
  // (see update()).
  var CONTROL_AXES = Object.freeze({
    // Reverse yaw so rotating the IMU left moves the blade left on screen.
    horizontal: { axis: "yaw", scale: CONTROL_ANGLE_SCALE, sign: -1 },
    vertical: { axis: "roll", scale: CONTROL_ANGLE_SCALE, sign: 1 }
  });

  function random() {
    state.rng = (Math.imul(1664525, state.rng) + 1013904223) >>> 0;
    return state.rng / 4294967296;
  }
  function clamp(value, low, high) { return Math.max(low, Math.min(high, value)); }
  function normalize(q) { var n = Math.hypot(q[0], q[1], q[2], q[3]) || 1; return q.map(function (v) { return v / n; }); }
  function multiply(a, b) {
    return [a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
      a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
      a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
      a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]];
  }
  function conjugate(q) { return [q[0], -q[1], -q[2], -q[3]]; }
  function fromEuler(roll, pitch, yaw) {
    var r = roll * Math.PI / 360, p = pitch * Math.PI / 360, y = yaw * Math.PI / 360;
    var cr = Math.cos(r), sr = Math.sin(r), cp = Math.cos(p), sp = Math.sin(p), cy = Math.cos(y), sy = Math.sin(y);
    return normalize([cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy, cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]);
  }
  function toEuler(q) {
    q = normalize(q);
    return {
      roll: Math.atan2(2 * (q[0] * q[1] + q[2] * q[3]), 1 - 2 * (q[1] * q[1] + q[2] * q[2])) * 180 / Math.PI,
      pitch: Math.asin(clamp(2 * (q[0] * q[2] - q[3] * q[1]), -1, 1)) * 180 / Math.PI,
      yaw: Math.atan2(2 * (q[0] * q[3] + q[1] * q[2]), 1 - 2 * (q[2] * q[2] + q[3] * q[3])) * 180 / Math.PI
    };
  }
  function integrate(q, gyroDps, dt) {
    var wx = gyroDps[0] * Math.PI / 180 * dt, wy = gyroDps[1] * Math.PI / 180 * dt, wz = gyroDps[2] * Math.PI / 180 * dt;
    var angle = Math.hypot(wx, wy, wz), half = angle * .5, scale = angle < 1e-8 ? .5 : Math.sin(half) / angle;
    return normalize(multiply(q, [Math.cos(half), wx * scale, wy * scale, wz * scale]));
  }
  function timerDelta(current, previous) { return (((current >>> 0) - (previous >>> 0)) >>> 0) * 1e-6; }

  function setStatus(text, type) { elements.status.textContent = text; elements.status.className = "status " + (type || "idle"); }
  function serialErrorMessage(error) {
    if (!error) return "Could not open serial port";
    if (error.name === "NotFoundError") return "No serial port selected";
    if (error.name === "NetworkError" || /already open|busy|in use/i.test(error.message || "")) return "Serial port is busy or already open; close other IMU pages and retry";
    if (error.name === "SecurityError") return "Serial access blocked; use Chrome or Edge on localhost";
    return error.message || "Could not open serial port";
  }
  function fillRangeSelect(select, values, suffix) {
    select.innerHTML = "";
    values.forEach(function (value, index) { var option = document.createElement("option"); option.value = index; option.textContent = "±" + value + suffix; select.appendChild(option); });
  }
  function updateRanges() {
    state.gyroRange = Protocol.GYRO_RANGES[Number(elements.gyroRange.value)] || 4000;
    state.accelRange = Protocol.ACCEL_RANGES[Number(elements.accelRange.value)] || 16;
    elements.attitude.querySelector('option[value="builtin"]').disabled = elements.device.value !== "hm-g12";
    if (elements.device.value !== "hm-g12" && elements.attitude.value === "builtin") { elements.attitude.value = "integration"; }
  }

  function resize() {
    var rect = canvas.getBoundingClientRect(); state.dpr = Math.min(window.devicePixelRatio || 1, 2); state.width = rect.width; state.height = rect.height;
    canvas.width = Math.max(1, Math.floor(rect.width * state.dpr)); canvas.height = Math.max(1, Math.floor(rect.height * state.dpr)); context.setTransform(state.dpr, 0, 0, state.dpr, 0, 0);
    if (!state.pointer.x) { state.pointer.x = state.pointer.targetX = state.width * .5; state.pointer.y = state.pointer.targetY = state.height * .55; }
  }

  function resetGame() {
    state.running = false; state.gameOver = false; state.score = 0; state.misses = 0; state.timeLeft = ROUND_SECONDS; state.spawnAt = 0; state.fruits = []; state.particles = []; state.slashes = []; state.trail = [];
    updateHud(); elements.overlay.classList.remove("hidden"); elements.overlayTitle.textContent = "Ready to slice?"; elements.overlayMessage.textContent = "Connect an IMU, hold it flat, set the neutral pose, then swing your wrist to move the blade."; elements.overlayButton.textContent = "Start game"; elements.start.textContent = "Start game";
  }
  function updateHud() { elements.score.textContent = String(state.score); elements.misses.textContent = state.misses + " / " + MAX_MISSES; elements.time.textContent = String(Math.max(0, Math.ceil(state.timeLeft))); elements.frames.textContent = "Frames: " + state.frameCount + (state.crcErrors ? " · CRC: " + state.crcErrors : "") + (state.processingErrors ? " · ESKF: " + state.processingErrors : ""); }
  function startGame() { if (state.running) { state.running = false; elements.start.textContent = "Resume game"; return; } if (state.gameOver) { resetGame(); } state.running = true; state.gameOver = false; elements.overlay.classList.add("hidden"); elements.start.textContent = "Pause game"; state.lastFrameAt = performance.now(); }
  function endGame(message) { state.running = false; state.gameOver = true; elements.start.textContent = "Play again"; elements.overlayTitle.textContent = "Round over"; elements.overlayMessage.textContent = message + " Final score: " + state.score + "."; elements.overlayButton.textContent = "Play again"; elements.overlay.classList.remove("hidden"); }

  function spawnFruit() {
    var radius = 27 + random() * 8, x = state.width * (.12 + random() * .76);
    state.fruits.push({ x: x, y: state.height + radius + 8, vx: (random() - .5) * state.width * .42, vy: -(state.height * (.84 + random() * .16)), gravity: state.height * (.72 + random() * .12), radius: radius, color: FRUIT_COLORS[Math.floor(random() * FRUIT_COLORS.length)], name: FRUIT_NAMES[Math.floor(random() * FRUIT_NAMES.length)], bomb: random() < .12, sliced: false, spin: random() * 6.28 });
  }
  function distanceToSegment(px, py, x1, y1, x2, y2) {
    var dx = x2 - x1, dy = y2 - y1, length = dx * dx + dy * dy, t = length ? ((px - x1) * dx + (py - y1) * dy) / length : 0; t = clamp(t, 0, 1); var x = x1 + t * dx, y = y1 + t * dy; return Math.hypot(px - x, py - y);
  }
  function sliceBetween(x1, y1, x2, y2) {
    if (Math.hypot(x2 - x1, y2 - y1) < 5) return;
    state.fruits.forEach(function (fruit) {
      if (fruit.sliced || distanceToSegment(fruit.x, fruit.y, x1, y1, x2, y2) > fruit.radius + 16) return;
      state.slashes.push({ x1: x1, y1: y1, x2: x2, y2: y2, life: .22, color: fruit.bomb ? "#ff405b" : "#ffffff" });
      if (fruit.bomb) { fruit.sliced = true; burst(fruit.x, fruit.y, "#ff405b", 30); endGame("Boom! A bomb was sliced."); return; }
      fruit.sliced = true;
      fruit.cut = {
        x: fruit.x,
        y: fruit.y,
        vx: fruit.vx,
        vy: fruit.vy,
        gravity: fruit.gravity,
        angle: Math.atan2(y2 - y1, x2 - x1),
        separation: 0,
        rotation: 0,
        life: .8
      };
      state.score += 10; burst(fruit.x, fruit.y, fruit.color, 15); updateHud();
    });
  }
  function burst(x, y, color, count) { for (var i = 0; i < count; i += 1) { var angle = random() * Math.PI * 2, speed = 50 + random() * 230; state.particles.push({ x: x, y: y, vx: Math.cos(angle) * speed, vy: Math.sin(angle) * speed, life: .35 + random() * .45, maxLife: .8, size: 2 + random() * 4, color: color }); } }
  function update(dt) {
    state.timeLeft -= dt; if (state.timeLeft <= 0) { endGame("Time is up!"); return; }
    if (performance.now() >= state.spawnAt) { spawnFruit(); if (random() > .74) spawnFruit(); state.spawnAt = performance.now() + Math.max(350, 820 - (ROUND_SECONDS - state.timeLeft) * 2.5); }
    var previousX = state.pointer.x, previousY = state.pointer.y;
    var pointerAlpha = Math.min(1, dt * POINTER_RESPONSE);
    state.pointer.x += (state.pointer.targetX - state.pointer.x) * pointerAlpha; state.pointer.y += (state.pointer.targetY - state.pointer.y) * pointerAlpha; var movementX = state.pointer.x - previousX, movementY = state.pointer.y - previousY, movement = Math.hypot(movementX, movementY);
    // A knife follows the direction in which it is moving. Keep its last
    // direction while stationary instead of forcing it to an IMU Euler angle.
    if (movement > 1.5) state.pointer.angle = Math.atan2(movementY, movementX);
    sliceBetween(previousX, previousY, state.pointer.x, state.pointer.y); state.trail.push({ x: state.pointer.x, y: state.pointer.y, life: .22 });
    while (state.trail.length > 18) state.trail.shift(); state.trail.forEach(function (point) { point.life -= dt; }); while (state.trail.length && state.trail[0].life <= 0) state.trail.shift();
    state.fruits.forEach(function (fruit) {
      if (fruit.sliced && fruit.cut) {
        fruit.cut.life -= dt;
        fruit.cut.separation += dt * 125;
        fruit.cut.rotation += dt * 3.5;
        fruit.cut.vy += fruit.cut.gravity * dt;
        fruit.cut.x += fruit.cut.vx * dt;
        fruit.cut.y += fruit.cut.vy * dt;
      } else {
        fruit.vy += fruit.gravity * dt;
        fruit.x += fruit.vx * dt;
        fruit.y += fruit.vy * dt;
        fruit.spin += dt * 2;
      }
    });
    state.fruits = state.fruits.filter(function (fruit) {
      if (fruit.sliced && fruit.cut) { return fruit.cut.life > 0 && fruit.cut.y < state.height + 160; }
      if (fruit.y > state.height + 100) { if (!fruit.sliced && !fruit.bomb) { state.misses += 1; updateHud(); if (state.misses >= MAX_MISSES) endGame(MAX_MISSES + " fruits got away."); } return false; }
      return true;
    });
    state.particles.forEach(function (particle) { particle.life -= dt; particle.vy += 260 * dt; particle.x += particle.vx * dt; particle.y += particle.vy * dt; }); state.particles = state.particles.filter(function (particle) { return particle.life > 0; });
    state.slashes.forEach(function (slash) { slash.life -= dt; }); state.slashes = state.slashes.filter(function (slash) { return slash.life > 0; });
    updateHud();
  }

  function drawBackground() {
    var gradient = context.createLinearGradient(0, 0, 0, state.height); gradient.addColorStop(0, "#10152b"); gradient.addColorStop(.55, "#1b1c42"); gradient.addColorStop(1, "#321d42"); context.fillStyle = gradient; context.fillRect(0, 0, state.width, state.height);
    var glow = context.createRadialGradient(state.width * .5, state.height * .42, 5, state.width * .5, state.height * .42, state.width * .7); glow.addColorStop(0, "rgba(93, 122, 255, .19)"); glow.addColorStop(1, "rgba(93, 122, 255, 0)"); context.fillStyle = glow; context.fillRect(0, 0, state.width, state.height);
    context.strokeStyle = "rgba(160, 176, 255, .08)"; context.lineWidth = 1; for (var x = -state.height; x < state.width + state.height; x += 52) { context.beginPath(); context.moveTo(x, state.height); context.lineTo(x + state.height, 0); context.stroke(); }
  }
  function drawFruit(fruit) {
    if (fruit.sliced && fruit.cut && !fruit.bomb) {
      drawSlicedFruit(fruit);
      return;
    }
    context.save(); context.translate(fruit.x, fruit.y); context.rotate(Math.sin(fruit.spin) * .14); context.shadowColor = "rgba(0,0,0,.35)"; context.shadowBlur = 15; context.shadowOffsetY = 8;
    if (fruit.bomb) { context.fillStyle = "#10131d"; context.beginPath(); context.arc(0, 0, fruit.radius, 0, Math.PI * 2); context.fill(); context.shadowColor = "transparent"; context.strokeStyle = "#ff4e6d"; context.lineWidth = 3; context.stroke(); context.fillStyle = "#ff6b5c"; context.fillRect(-3, -fruit.radius - 10, 6, 12); context.strokeStyle = "#ffc34a"; context.beginPath(); context.arc(7, -fruit.radius - 13, 7, Math.PI, Math.PI * 1.65); context.stroke(); context.fillStyle = "rgba(255,255,255,.75)"; context.beginPath(); context.arc(-fruit.radius * .32, -fruit.radius * .35, 5, 0, Math.PI * 2); context.fill();
    } else { var fruitGradient = context.createRadialGradient(-fruit.radius * .3, -fruit.radius * .4, 2, 0, 0, fruit.radius * 1.2); fruitGradient.addColorStop(0, "#fff5c7"); fruitGradient.addColorStop(.12, fruit.color); fruitGradient.addColorStop(1, "#7c315f"); context.fillStyle = fruitGradient; context.beginPath(); context.arc(0, 0, fruit.radius, 0, Math.PI * 2); context.fill(); context.shadowColor = "transparent"; context.fillStyle = "rgba(255,255,255,.62)"; context.beginPath(); context.arc(-fruit.radius * .32, -fruit.radius * .36, fruit.radius * .13, 0, Math.PI * 2); context.fill(); context.fillStyle = "#64d583"; context.rotate(-.55); context.beginPath(); context.ellipse(7, -fruit.radius - 2, 10, 5, 0, 0, Math.PI * 2); context.fill(); }
    context.restore();
  }
  function drawSlicedFruit(fruit) {
    var cut = fruit.cut, radius = fruit.radius;
    context.save();
    context.translate(cut.x, cut.y);
    context.rotate(cut.angle);
    [-1, 1].forEach(function (side) {
      context.save();
      context.translate(side * (cut.separation + radius * .38), side * cut.separation * .18);
      context.rotate(side * cut.rotation);
      context.shadowColor = "rgba(0,0,0,.34)";
      context.shadowBlur = 13;
      context.shadowOffsetY = 8;
      context.beginPath();
      if (side < 0) {
        context.rect(-radius - 2, -radius - 2, radius + 2, radius * 2 + 4);
      } else {
        context.rect(0, -radius - 2, radius + 2, radius * 2 + 4);
      }
      context.clip();
      var gradient = context.createRadialGradient(-radius * .28, -radius * .36, 2, 0, 0, radius * 1.25);
      gradient.addColorStop(0, "#fff5c7");
      gradient.addColorStop(.12, fruit.color);
      gradient.addColorStop(1, "#7c315f");
      context.fillStyle = gradient;
      context.beginPath();
      context.arc(0, 0, radius, 0, Math.PI * 2);
      context.fill();
      context.fillStyle = "rgba(255,239,190,.9)";
      context.beginPath();
      context.arc(0, 0, radius * .72, 0, Math.PI * 2);
      context.fill();
      context.fillStyle = fruit.color;
      context.globalAlpha = .78;
      context.beginPath();
      context.arc(0, 0, radius * .59, 0, Math.PI * 2);
      context.fill();
      context.globalAlpha = 1;
      context.restore();
    });
    context.restore();
  }
  function draw() {
    drawBackground(); state.fruits.forEach(drawFruit); state.slashes.forEach(function (slash) { context.save(); context.globalAlpha = Math.max(0, slash.life / .22); context.lineCap = "round"; context.shadowColor = slash.color; context.shadowBlur = 20; context.strokeStyle = slash.color; context.lineWidth = 7; context.beginPath(); context.moveTo(slash.x1, slash.y1); context.lineTo(slash.x2, slash.y2); context.stroke(); context.strokeStyle = "#54e1f4"; context.lineWidth = 2; context.stroke(); context.restore(); }); state.particles.forEach(function (particle) { context.globalAlpha = Math.max(0, particle.life / particle.maxLife); context.fillStyle = particle.color; context.beginPath(); context.arc(particle.x, particle.y, particle.size, 0, Math.PI * 2); context.fill(); }); context.globalAlpha = 1;
    if (state.trail.length > 1) { context.lineCap = "round"; context.lineJoin = "round"; context.beginPath(); state.trail.forEach(function (point, index) { if (!index) context.moveTo(point.x, point.y); else context.lineTo(point.x, point.y); }); var trailGradient = context.createLinearGradient(state.pointer.x, state.pointer.y, state.pointer.x - 130, state.pointer.y - 130); trailGradient.addColorStop(0, "rgba(255,255,255,.95)"); trailGradient.addColorStop(1, "rgba(84,225,244,0)"); context.strokeStyle = trailGradient; context.lineWidth = 10; context.globalAlpha = .75; context.stroke(); context.globalAlpha = 1; }
    context.save(); context.translate(state.pointer.x, state.pointer.y); context.rotate(Number.isFinite(state.pointer.angle) ? state.pointer.angle : -.5); context.shadowColor = "rgba(84,225,244,.8)"; context.shadowBlur = 18; context.strokeStyle = "#fff"; context.lineWidth = 3; context.beginPath(); context.moveTo(-72, 0); context.lineTo(45, 0); context.stroke(); context.strokeStyle = "#54e1f4"; context.lineWidth = 1; context.beginPath(); context.moveTo(-70, -3); context.lineTo(48, -3); context.stroke(); context.fillStyle = "#fff"; context.beginPath(); context.arc(48, -2, 5, 0, Math.PI * 2); context.fill(); context.restore();
  }
  function frame(now) { var dt = Math.min(.05, (now - (state.lastFrameAt || now)) / 1000); state.lastFrameAt = now; if (state.running && !state.gameOver) update(dt); draw(); window.requestAnimationFrame(frame); }

  function resetAttitude() { state.integrationQuaternion = [1, 0, 0, 0]; state.eskf.reset(); state.lastTimerUs = null; state.latestQuaternion = [1, 0, 0, 0]; state.neutralQuaternion = [1, 0, 0, 0]; }
  function calibrate() { state.neutralQuaternion = state.latestQuaternion.slice(); elements.input.textContent = "Neutral pose saved"; }
  function applyAttitude(sample) {
    var current = state.latestQuaternion, dt = state.lastTimerUs === null ? 0 : timerDelta(sample.timerUs, state.lastTimerUs);
    if (dt > 0 && dt < .1) state.integrationQuaternion = integrate(state.integrationQuaternion, sample.gyro, dt); state.lastTimerUs = sample.timerUs;
    if (state.attitudeSource === "builtin" && sample.builtinAttitude) current = fromEuler(sample.builtinAttitude.roll, sample.builtinAttitude.pitch, sample.builtinAttitude.yaw);
    else if (state.attitudeSource === "eskf" && sample.gyroRaw && sample.accelRaw) {
      try {
        var eskfOutput = state.eskf.process({ timerUs: sample.timerUs, gyroRaw: sample.gyroRaw, accelRaw: sample.accelRaw, gyroRangeDps: state.gyroRange, accelRangeG: state.accelRange });
        // ESKF needs a short stationary window before it can publish its first
        // quaternion. Show the continuously integrated pose during that window,
        // then switch to the ESKF result as soon as it enters RUNNING.
        current = eskfOutput.state === "RUNNING" ? eskfOutput.quaternion : state.integrationQuaternion;
      } catch (error) {
        // A bad range/configuration must not stop the serial reader. Keep the
        // live gyro-integrated angle visible and report the error in the HUD.
        state.processingErrors += 1;
        current = state.integrationQuaternion;
      }
    }
    else current = state.integrationQuaternion;
    state.latestQuaternion = normalize(current); var relative = normalize(multiply(conjugate(state.neutralQuaternion), state.latestQuaternion)); var euler = toEuler(relative);
    var horizontal = CONTROL_AXES.horizontal;
    var vertical = CONTROL_AXES.vertical;
    var horizontalAngle = euler[horizontal.axis] * horizontal.sign;
    var verticalAngle = euler[vertical.axis] * vertical.sign;
    // Use one physical pixel span for both axes, but size it from the long
    // screen dimension.  The old short-edge span left the blade trapped near
    // the center on a landscape display while fruits could travel farther.
    // The final clamps below still keep the blade safely inside the HUD area.
    var controlPixelSpan = Math.max(state.width, state.height) * CONTROL_SCREEN_SPAN;
    state.pointer.targetX = state.width * .5 + clamp(horizontalAngle / horizontal.scale, -1, 1) * controlPixelSpan;
    state.pointer.targetY = state.height * .57 - clamp(verticalAngle / vertical.scale, -1, 1) * controlPixelSpan;
    state.pointer.targetX = clamp(state.pointer.targetX, 22, state.width - 22); state.pointer.targetY = clamp(state.pointer.targetY, 90, state.height - 90);
    elements.input.textContent = "R " + euler.roll.toFixed(0) + "° · P " + euler.pitch.toFixed(0) + "° · Y " + euler.yaw.toFixed(0) + "°";
  }
  function appendSerial(chunk) { var incoming = new Uint8Array(chunk); var combined = new Uint8Array(state.serialBuffer.length + incoming.length); combined.set(state.serialBuffer); combined.set(incoming, state.serialBuffer.length); state.serialBuffer = combined; }
  async function readExact(size) { while (state.active && state.serialBuffer.length < size) { var result = await state.reader.read(); if (result.done) throw new Error("The serial stream ended"); if (result.value) appendSerial(result.value); } var frame = state.serialBuffer.slice(0, size); state.serialBuffer = state.serialBuffer.slice(size); return frame; }
  async function runG11() { var first = true; while (state.active) { await state.writer.write(new Uint8Array(first ? [0x80, 0x00] : [0x00, 0x00])); first = false; try { var sample = Protocol.parseHM_G11Burst(await readExact(34), state.gyroRange, state.accelRange); applyAttitude(sample); state.frameCount += 1; } catch (error) { state.crcErrors += 1; } } }
  async function runG12() { state.parser = new Protocol.HM_G12StreamParser(state.gyroRange, state.accelRange, Protocol.HM_G12_RAW_STREAM_ID); await state.writer.write(Protocol.makeHM_G12Command(Protocol.HM_G12_RAW_STREAM_ID, true)); while (state.active) { var result = await state.reader.read(); if (result.done) throw new Error("The serial stream ended"); if (!result.value) continue; var previous = state.parser.counters.crcErrors; state.parser.feed(result.value).forEach(function (sample) { applyAttitude(sample); state.frameCount += 1; }); state.crcErrors += state.parser.counters.crcErrors - previous; } }
  async function disconnect() { state.active = false; state.connecting = false; var reader = state.reader, writer = state.writer, port = state.port; state.reader = null; state.writer = null; state.port = null; if (writer && state.device === "hm-g12") { try { await writer.write(Protocol.makeHM_G12Command(Protocol.HM_G12_RAW_STREAM_ID, false)); } catch (error) {} } if (reader) { try { await reader.cancel(); } catch (error) {} try { reader.releaseLock(); } catch (error) {} } if (writer) { try { writer.releaseLock(); } catch (error) {} } if (port) { try { await port.close(); } catch (error) {} } elements.connect.disabled = false; elements.connect.textContent = "Connect IMU"; elements.calibrate.disabled = true; setStatus("Not connected", "idle"); }
  async function connect() { if (!navigator.serial) { setStatus("Web Serial unavailable", "error"); elements.input.textContent = "Open this page in desktop Chrome or Edge"; return; } if (state.connecting) return; if (state.active) { await disconnect(); return; } state.connecting = true; state.device = elements.device.value; state.attitudeSource = elements.attitude.value; updateRanges(); resetAttitude(); state.serialBuffer = new Uint8Array(0); state.frameCount = 0; state.crcErrors = 0; state.processingErrors = 0; elements.connect.disabled = true; elements.input.textContent = "Waiting for IMU data…"; setStatus("Choose serial port", "idle"); try { var port = await navigator.serial.requestPort();
    // Save the handle before opening it so every failure path can close it.
    state.port = port;
    await port.open({ baudRate: Number(elements.baud.value), bufferSize: 4096, dataBits: 8, stopBits: 1, parity: "none", flowControl: "none" });
    state.reader = port.readable.getReader(); state.writer = port.writable.getWriter(); state.active = true; state.connecting = false; elements.connect.disabled = false; elements.connect.textContent = "Disconnect IMU"; elements.calibrate.disabled = false; setStatus(state.device.toUpperCase() + " connected", "connected"); state.readTask = state.device === "hm-g12" ? runG12() : runG11(); state.readTask.catch(function (error) { if (state.active) { disconnect().then(function () { setStatus(serialErrorMessage(error), "error"); }); } });
  } catch (error) { await disconnect(); elements.connect.disabled = false; setStatus(serialErrorMessage(error), "error"); } }

  elements.gyroRange.addEventListener("change", updateRanges); elements.accelRange.addEventListener("change", updateRanges); elements.device.addEventListener("change", updateRanges); elements.attitude.addEventListener("change", function () { state.attitudeSource = elements.attitude.value; resetAttitude(); }); elements.connect.addEventListener("click", connect); elements.calibrate.addEventListener("click", calibrate); elements.start.addEventListener("click", startGame); elements.overlayButton.addEventListener("click", function () { if (state.gameOver) resetGame(); startGame(); }); canvas.addEventListener("pointermove", function (event) { if (state.active) return; var rect = canvas.getBoundingClientRect(); state.pointer.targetX = event.clientX - rect.left; state.pointer.targetY = event.clientY - rect.top; }); window.addEventListener("resize", resize); window.addEventListener("beforeunload", disconnect);
  fillRangeSelect(elements.gyroRange, Protocol.GYRO_RANGES, " dps"); fillRangeSelect(elements.accelRange, Protocol.ACCEL_RANGES, " g");
  // Default parser scale: ±4000 dps and ±16 g (range indices 5 and 3).
  elements.gyroRange.value = "5"; elements.accelRange.value = "3"; elements.attitude.value = "eskf"; state.attitudeSource = "eskf"; updateRanges(); resize(); resetGame(); window.requestAnimationFrame(frame);
}());
