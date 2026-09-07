(function (root) {
  "use strict";

  var HM_G12_RAW_STREAM_ID = 0x0101;
  var HM_G12_CONFIG_ID = 0x0102;
  var HM_G12_DEVICE_INFO_ID = 0x0000;
  var HM_G12_STATUS_ID = 0xffff;
  var HM_G12_STREAM_LENGTH = 46;
  var DEVICE_CONFIG_LENGTH = 46;
  var HM_G12_MAX_PAYLOAD = 512;
  var GYRO_RANGES = Object.freeze([125, 250, 500, 1000, 2000, 4000]);
  var ACCEL_RANGES = Object.freeze([2, 4, 8, 16]);
  var BAUD_RATES = Object.freeze([115200, 921600, 1500000, 3000000]);
  var GYRO_LOW_PASS = Object.freeze([
    ["4.3 Hz", "8.3 Hz", "16.7 Hz", "33 Hz", "67 Hz", "133 Hz", "222 Hz", "274 Hz"],
    ["4.3 Hz", "8.3 Hz", "16.7 Hz", "33 Hz", "67 Hz", "128 Hz", "186 Hz", "212 Hz"],
    ["4.3 Hz", "8.3 Hz", "16.7 Hz", "33 Hz", "67 Hz", "112 Hz", "140 Hz", "150 Hz"],
    ["4.3 Hz", "8.3 Hz", "16.7 Hz", "33 Hz", "67 Hz", "134 Hz", "260 Hz", "390 Hz"],
    ["4.3 Hz", "8.3 Hz", "16.7 Hz", "34 Hz", "62 Hz", "86 Hz", "96 Hz", "99 Hz"],
    ["4.3 Hz", "8.3 Hz", "16.9 Hz", "31 Hz", "43 Hz", "48 Hz", "49 Hz", "50 Hz"],
    ["4.3 Hz", "8.3 Hz", "13.4 Hz", "19 Hz", "23 Hz", "24.6 Hz", "25 Hz", "25 Hz"],
    ["4.3 Hz", "8.3 Hz", "9.8 Hz", "11.6 Hz", "12.2 Hz", "12.4 Hz", "12.6 Hz", "12.6 Hz"]
  ]);
  var ACCEL_FILTER = Object.freeze([
    ["3.125 Hz", "6.500 Hz", "13.000 Hz", "26.000 Hz", "52.000 Hz", "104.000 Hz", "208.250 Hz", "416.750 Hz"],
    ["1.250 Hz", "2.600 Hz", "5.200 Hz", "10.400 Hz", "20.800 Hz", "41.600 Hz", "83.300 Hz", "166.700 Hz"],
    ["0.625 Hz", "1.300 Hz", "2.600 Hz", "5.200 Hz", "10.400 Hz", "20.800 Hz", "41.650 Hz", "83.350 Hz"],
    ["0.278 Hz", "0.578 Hz", "1.156 Hz", "2.311 Hz", "4.6222 Hz", "9.2444 Hz", "18.511 Hz", "37.044 Hz"],
    ["0.125 Hz", "0.260 Hz", "0.520 Hz", "1.040 Hz", "2.080 Hz", "4.160 Hz", "8.330 Hz", "16.670 Hz"],
    ["0.063 Hz", "0.130 Hz", "0.260 Hz", "0.520 Hz", "1.040 Hz", "2.080 Hz", "4.165 Hz", "8.335 Hz"],
    ["0.031 Hz", "0.065 Hz", "0.130 Hz", "0.260 Hz", "0.520 Hz", "1.040 Hz", "2.083 Hz", "4.168 Hz"],
    ["0.016 Hz", "0.033 Hz", "0.065 Hz", "0.130 Hz", "0.260 Hz", "0.520 Hz", "1.041 Hz", "2.084 Hz"]
  ]);
  var GYRO_HIGH_PASS = Object.freeze(["16 mHz", "65 mHz", "260 mHz", "1.04 Hz"]);
  var AXIS_DEFINITIONS = Object.freeze([
    "+Ux, +Uy, +Uz", "-Ux, -Uy, +Uz", "-Uy, +Ux, +Uz", "+Uy, -Ux, +Uz",
    "-Ux, +Uy, -Uz", "+Ux, -Uy, -Uz", "+Ux, +Uy, -Uz", "-Uy, -Ux, -Uz",
    "-Uz, +Uy, +Ux", "+Uz, -Uy, +Ux", "+Uy, +Uz, +Ux", "-Uy, -Uz, +Ux",
    "+Uz, +Uy, -Ux", "-Uz, -Uy, -Ux", "-Uy, +Uz, -Ux", "+Uy, -Uz, -Ux",
    "-Ux, +Uz, +Uy", "+Ux, -Uz, +Uy", "+Uz, +Ux, +Uy", "-Uz, -Ux, +Uy",
    "+Ux, +Uz, -Uy", "-Ux, -Uz, -Uy", "-Uz, +Ux, -Uy", "+Uz, -Ux, -Uy"
  ]);

  function crc16Ccitt(bytes, init) {
    var crc = Number.isFinite(init) ? init & 0xffff : 0;
    for (var index = 0; index < bytes.length; index += 1) {
      crc ^= bytes[index] << 8;
      for (var bit = 0; bit < 8; bit += 1) {
        crc = crc & 0x8000 ? ((crc << 1) ^ 0x1021) & 0xffff : (crc << 1) & 0xffff;
      }
    }
    return crc;
  }

  function combineSigned32(lowWord, highWord) {
    return ((highWord << 16) | lowWord) | 0;
  }

  function scaleGyro(raw, rangeDps) {
    return raw / 65536 * 35 / 1000000 * rangeDps;
  }

  function scaleAccel(raw, rangeG) {
    return raw / 65536 / 32768 * rangeG;
  }

  function createDefaultDeviceConfig() {
    return {
      odr: 1000,
      gyroRange: 2,
      accelRange: 3,
      gyroLowPassEnabled: false,
      gyroLowPassBandwidth: 0,
      gyroHighPassEnabled: false,
      gyroHighPassBandwidth: 0,
      accelFilterMode: 0,
      accelFilterBandwidth: 0,
      selfTest: 0,
      gyroOffset: [0, 0, 0],
      accelOffset: [0, 0, 0],
      axisDefinition: 101,
      baudRate: 1
    };
  }

  function odrBandIndex(odr) {
    if (odr >= 1 && odr <= 12) { return 0; }
    if (odr <= 26) { return 1; }
    if (odr <= 52) { return 2; }
    if (odr <= 104) { return 3; }
    if (odr <= 208) { return 4; }
    if (odr <= 416) { return 5; }
    if (odr <= 833) { return 6; }
    return 7;
  }

  function gyroLowPassLabel(odr, selector) {
    return GYRO_LOW_PASS[selector][odrBandIndex(odr)];
  }

  function accelFilterLabel(odr, selector) {
    return ACCEL_FILTER[selector][odrBandIndex(odr)];
  }

  function validateHM_G12Envelope(frame) {
    if (!(frame instanceof Uint8Array) || frame.byteLength < 8) {
      throw new Error("HM-G12 frame is incomplete");
    }
    var view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
    if (view.getUint8(0) !== 0xaa || view.getUint8(1) !== 0x55) {
      throw new Error("HM-G12 frame header is invalid");
    }
    var payloadLength = view.getUint16(4, true);
    if (frame.byteLength !== payloadLength + 8) {
      throw new Error("HM-G12 frame length is invalid");
    }
    var receivedCrc = view.getUint16(frame.byteLength - 2, true);
    var calculatedCrc = crc16Ccitt(frame.subarray(0, frame.byteLength - 2));
    if (receivedCrc !== calculatedCrc) {
      throw new Error("HM-G12 CRC validation failed");
    }
    return {
      packetId: view.getUint16(2, false),
      payloadLength: payloadLength,
      payload: new DataView(frame.buffer, frame.byteOffset + 6, payloadLength)
    };
  }

  function parseHM_G11Burst(frame, gyroRangeDps, accelRangeG) {
    if (!(frame instanceof Uint8Array) || frame.byteLength !== 34) {
      throw new Error("HM-G11 frame must contain 34 bytes");
    }

    var view = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
    var receivedCrc = view.getUint16(32, true);
    var calculatedCrc = crc16Ccitt(frame.subarray(0, 32));
    if (receivedCrc !== calculatedCrc) {
      throw new Error("HM-G11 CRC validation failed");
    }

    var words = [];
    for (var offset = 0; offset < 32; offset += 2) {
      words.push(view.getUint16(offset, false));
    }

    var timerUs = ((words[1] << 16) | words[0]) >>> 0;
    var temperatureRaw = combineSigned32(words[2], words[3]);
    var gyroRaw = [
      combineSigned32(words[4], words[5]),
      combineSigned32(words[6], words[7]),
      combineSigned32(words[8], words[9])
    ];
    var accelRaw = [
      combineSigned32(words[10], words[11]),
      combineSigned32(words[12], words[13]),
      combineSigned32(words[14], words[15])
    ];

    return {
      device: "hm-g11",
      timerUs: timerUs,
      gyroRaw: gyroRaw,
      accelRaw: accelRaw,
      gyro: gyroRaw.map(function (value) { return scaleGyro(value, gyroRangeDps); }),
      accel: accelRaw.map(function (value) { return scaleAccel(value, accelRangeG); }),
      temperatureC: temperatureRaw / 256,
      builtinAttitude: null
    };
  }

  function decodeHM_G12BuiltinAttitude(payloadView) {
    var pitch = payloadView.getInt32(32, true) / 32768 * 180;
    var roll = payloadView.getInt32(36, true) / 32768 * 180;
    var yaw = payloadView.getInt32(40, true) / 32768 * 180;
    var values = [roll, pitch, yaw];
    var valid = values.every(function (value) {
      return Number.isFinite(value) && Math.abs(value) <= 720;
    });
    if (!valid) {
      return null;
    }
    return { roll: roll, pitch: pitch, yaw: yaw };
  }

  function parseHM_G12Frame(frame, gyroRangeDps, accelRangeG) {
    var envelope = validateHM_G12Envelope(frame);
    if (
      envelope.packetId !== HM_G12_RAW_STREAM_ID ||
      envelope.payloadLength !== HM_G12_STREAM_LENGTH
    ) {
      return null;
    }

    var payload = envelope.payload;
    var gyroRaw = [
      payload.getInt32(8, true),
      payload.getInt32(12, true),
      payload.getInt32(16, true)
    ];
    var accelRaw = [
      payload.getInt32(20, true),
      payload.getInt32(24, true),
      payload.getInt32(28, true)
    ];

    return {
      device: "hm-g12",
      streamId: envelope.packetId,
      timerUs: payload.getUint32(0, true),
      status: payload.getUint16(44, true),
      gyroRaw: gyroRaw,
      accelRaw: accelRaw,
      gyro: gyroRaw.map(function (value) { return scaleGyro(value, gyroRangeDps); }),
      accel: accelRaw.map(function (value) { return scaleAccel(value, accelRangeG); }),
      temperatureC: payload.getInt32(4, true) / 256,
      builtinAttitude: decodeHM_G12BuiltinAttitude(payload)
    };
  }

  function parseHM_G12DeviceConfigFrame(frame) {
    var envelope = validateHM_G12Envelope(frame);
    if (envelope.packetId !== HM_G12_CONFIG_ID || envelope.payloadLength !== DEVICE_CONFIG_LENGTH) {
      throw new Error("Frame is not an HM-G12 device configuration frame");
    }
    var payload = envelope.payload;
    return {
      odr: payload.getUint16(0, true),
      gyroRange: payload.getUint16(2, true),
      accelRange: payload.getUint16(4, true),
      gyroLowPassEnabled: payload.getUint8(6) === 1,
      gyroLowPassBandwidth: payload.getUint16(7, true),
      gyroHighPassEnabled: payload.getUint8(9) === 1,
      gyroHighPassBandwidth: payload.getUint16(10, true),
      accelFilterMode: payload.getUint8(12),
      accelFilterBandwidth: payload.getUint16(13, true),
      selfTest: payload.getUint8(15),
      gyroOffset: [
        payload.getInt32(16, true) / 524288,
        payload.getInt32(20, true) / 524288,
        payload.getInt32(24, true) / 524288
      ],
      accelOffset: [
        payload.getInt32(28, true) / 134217728,
        payload.getInt32(32, true) / 134217728,
        payload.getInt32(36, true) / 134217728
      ],
      axisDefinition: payload.getUint16(40, true),
      baudRate: payload.getUint32(42, true)
    };
  }

  function boundedOffset(value, minimum, maximum, scale) {
    if (!Number.isFinite(value) || value < minimum || value > maximum) {
      throw new Error("Bias value is outside the allowed range");
    }
    return Math.floor(value * scale);
  }

  function encodeHM_G12DeviceConfigPayload(config) {
    var payload = new Uint8Array(DEVICE_CONFIG_LENGTH);
    var view = new DataView(payload.buffer);
    view.setUint16(0, config.odr, true);
    view.setUint16(2, config.gyroRange, true);
    view.setUint16(4, config.accelRange, true);
    view.setUint8(6, config.gyroLowPassEnabled ? 1 : 0);
    view.setUint16(7, config.gyroLowPassBandwidth, true);
    view.setUint8(9, config.gyroHighPassEnabled ? 1 : 0);
    view.setUint16(10, config.gyroHighPassBandwidth, true);
    view.setUint8(12, config.accelFilterMode);
    view.setUint16(13, config.accelFilterBandwidth, true);
    view.setUint8(15, config.selfTest);
    config.gyroOffset.forEach(function (value, index) {
      view.setInt32(16 + index * 4, boundedOffset(value, -30, 30, 524288), true);
    });
    config.accelOffset.forEach(function (value, index) {
      view.setInt32(28 + index * 4, boundedOffset(value, -0.5, 0.5, 134217728), true);
    });
    view.setUint16(40, config.axisDefinition, true);
    view.setUint32(42, config.baudRate, true);
    return payload;
  }

  function makeHM_G12Frame(packetId, payloadBytes) {
    var payload = payloadBytes instanceof Uint8Array ? payloadBytes : new Uint8Array(payloadBytes || 0);
    var body = new Uint8Array(6 + payload.byteLength);
    body.set([0x55, 0xaa], 0);
    var view = new DataView(body.buffer);
    view.setUint16(2, packetId, false);
    view.setUint16(4, payload.byteLength, true);
    body.set(payload, 6);
    var frame = new Uint8Array(body.byteLength + 2);
    frame.set(body, 0);
    new DataView(frame.buffer).setUint16(body.byteLength, crc16Ccitt(body), true);
    return frame;
  }

  function makeHM_G12ConfigReadCommand() {
    return makeHM_G12Frame(HM_G12_CONFIG_ID, new Uint8Array([0]));
  }

  function makeHM_G12ConfigWriteCommand(config) {
    return makeHM_G12Frame(HM_G12_CONFIG_ID, encodeHM_G12DeviceConfigPayload(config));
  }

  function makeHM_G12DeviceInfoCommand() {
    return makeHM_G12Frame(HM_G12_DEVICE_INFO_ID, new Uint8Array([0]));
  }

  function parseHM_G12StatusFrame(frame) {
    var envelope = validateHM_G12Envelope(frame);
    if (envelope.packetId !== HM_G12_STATUS_ID || envelope.payloadLength !== 1) {
      throw new Error("Frame is not an HM-G12 status frame");
    }
    return envelope.payload.getUint8(0);
  }

  function parseHM_G12DeviceInfoFrame(frame) {
    var envelope = validateHM_G12Envelope(frame);
    if (envelope.packetId !== HM_G12_DEVICE_INFO_ID || envelope.payloadLength !== 28) {
      throw new Error("Frame is not an HM-G12 device information frame");
    }
    var bytes = new Uint8Array(frame.buffer, frame.byteOffset + 6, envelope.payloadLength);
    var text = Array.from(bytes).map(function (value) {
      return value >= 32 && value <= 126 ? String.fromCharCode(value) : "\u0000";
    }).join("");
    var parts = text.split(/\u0000+/).map(function (part) { return part.trim(); }).filter(Boolean);
    return { model: parts[0] || "--", version: parts[1] || "--" };
  }

  function encodeHM_G11GyroFilter(config) {
    var value = 0;
    if (config.gyroLowPassEnabled || config.gyroHighPassEnabled) { value |= 0x80; }
    if (config.gyroLowPassEnabled) { value |= 0x40; }
    if (config.gyroHighPassEnabled) { value |= 0x20; }
    value |= (config.gyroLowPassBandwidth & 0x07) << 2;
    value |= config.gyroHighPassBandwidth & 0x03;
    return value;
  }

  function decodeHM_G11GyroFilter(value) {
    var enabled = (value & 0x80) !== 0;
    return {
      gyroLowPassEnabled: enabled && (value & 0x40) !== 0,
      gyroLowPassBandwidth: (value >> 2) & 0x07,
      gyroHighPassEnabled: enabled && (value & 0x20) !== 0,
      gyroHighPassBandwidth: value & 0x03
    };
  }

  function encodeHM_G11AccelFilter(mode, bandwidth) {
    var value = bandwidth & 0x07;
    if (mode !== 0) { value |= 0x80; }
    if (mode === 1) { value |= 0x40; }
    if (mode === 2) { value |= 0x20; }
    return value;
  }

  function decodeHM_G11AccelFilter(value) {
    var mode = 0;
    if (value & 0x80) {
      mode = value & 0x40 ? 1 : value & 0x20 ? 2 : 0;
    }
    return { accelFilterMode: mode, accelFilterBandwidth: value & 0x07 };
  }

  function encodeHM_G11Offset(value, kind) {
    var isGyro = kind === "gyro";
    var raw = boundedOffset(value, isGyro ? -30 : -0.5, isGyro ? 30 : 0.5, isGyro ? 524288 : 134217728);
    return new Uint8Array([(raw >>> 8) & 0xff, raw & 0xff, (raw >>> 24) & 0xff, (raw >>> 16) & 0xff]);
  }

  function decodeHM_G11Offset(bytes, kind) {
    var raw = ((bytes[2] << 24) | (bytes[3] << 16) | (bytes[0] << 8) | bytes[1]) | 0;
    return raw / (kind === "gyro" ? 524288 : 134217728);
  }

  function concatBytes(left, right) {
    var result = new Uint8Array(left.byteLength + right.byteLength);
    result.set(left, 0);
    result.set(right, left.byteLength);
    return result;
  }

  function HM_G12StreamParser(gyroRangeDps, accelRangeG, expectedStreamId, maxBuffer) {
    this.gyroRangeDps = gyroRangeDps;
    this.accelRangeG = accelRangeG;
    this.expectedStreamId = expectedStreamId || HM_G12_RAW_STREAM_ID;
    this.maxBuffer = maxBuffer || 4096;
    this.buffer = new Uint8Array(0);
    this.counters = {
      frames: 0,
      crcErrors: 0,
      lengthErrors: 0,
      controlFrames: 0,
      unsupportedFrames: 0,
      discardedBytes: 0
    };
    this.controlStatuses = [];
  }

  HM_G12StreamParser.prototype.reset = function () {
    this.buffer = new Uint8Array(0);
    Object.keys(this.counters).forEach(function (key) {
      this.counters[key] = 0;
    }, this);
    this.controlStatuses = [];
  };

  HM_G12StreamParser.prototype.takeControlStatuses = function () {
    return this.controlStatuses.splice(0, this.controlStatuses.length);
  };

  HM_G12StreamParser.prototype.feed = function (chunk) {
    var incoming = chunk instanceof Uint8Array ? chunk : new Uint8Array(chunk || 0);
    this.buffer = concatBytes(this.buffer, incoming);
    if (this.buffer.byteLength > this.maxBuffer) {
      var overflow = this.buffer.byteLength - this.maxBuffer;
      this.buffer = this.buffer.slice(overflow);
      this.counters.discardedBytes += overflow;
    }

    var samples = [];
    while (true) {
      var start = -1;
      for (var index = 0; index < this.buffer.byteLength - 1; index += 1) {
        if (this.buffer[index] === 0xaa && this.buffer[index + 1] === 0x55) {
          start = index;
          break;
        }
      }

      if (start < 0) {
        var keep = this.buffer.byteLength && this.buffer[this.buffer.byteLength - 1] === 0xaa ? 1 : 0;
        this.counters.discardedBytes += this.buffer.byteLength - keep;
        this.buffer = keep ? this.buffer.slice(-1) : new Uint8Array(0);
        break;
      }
      if (start > 0) {
        this.counters.discardedBytes += start;
        this.buffer = this.buffer.slice(start);
      }
      if (this.buffer.byteLength < 6) {
        break;
      }

      var header = new DataView(this.buffer.buffer, this.buffer.byteOffset, this.buffer.byteLength);
      var payloadLength = header.getUint16(4, true);
      if (payloadLength > HM_G12_MAX_PAYLOAD || payloadLength + 8 > this.maxBuffer) {
        this.counters.lengthErrors += 1;
        this.counters.discardedBytes += 1;
        this.buffer = this.buffer.slice(1);
        continue;
      }

      var totalLength = payloadLength + 8;
      if (this.buffer.byteLength < totalLength) {
        break;
      }

      var frame = this.buffer.slice(0, totalLength);
      var frameView = new DataView(frame.buffer, frame.byteOffset, frame.byteLength);
      var expected = frameView.getUint16(totalLength - 2, true);
      var actual = crc16Ccitt(frame.subarray(0, totalLength - 2));
      if (expected !== actual) {
        this.counters.crcErrors += 1;
        this.counters.discardedBytes += 1;
        this.buffer = this.buffer.slice(1);
        continue;
      }

      this.buffer = this.buffer.slice(totalLength);
      var packetId = frameView.getUint16(2, false);
      if (packetId === HM_G12_STATUS_ID && payloadLength === 1) {
        this.controlStatuses.push(frame[6]);
        this.counters.controlFrames += 1;
        continue;
      }
      if (packetId !== this.expectedStreamId || payloadLength !== HM_G12_STREAM_LENGTH) {
        this.counters.unsupportedFrames += 1;
        continue;
      }
      var sample = parseHM_G12Frame(frame, this.gyroRangeDps, this.accelRangeG);
      if (sample) {
        samples.push(sample);
        this.counters.frames += 1;
      }
    }
    return samples;
  };

  function makeHM_G12Command(streamId, enabled) {
    if (streamId !== HM_G12_RAW_STREAM_ID) {
      throw new Error("Unsupported HM-G12 stream ID");
    }
    return makeHM_G12Frame(streamId, new Uint8Array([enabled ? 1 : 0]));
  }

  root.ImuProtocol = Object.freeze({
    HM_G12_RAW_STREAM_ID: HM_G12_RAW_STREAM_ID,
    HM_G12_CONFIG_ID: HM_G12_CONFIG_ID,
    HM_G12_DEVICE_INFO_ID: HM_G12_DEVICE_INFO_ID,
    HM_G12_STATUS_ID: HM_G12_STATUS_ID,
    HM_G12_STREAM_LENGTH: HM_G12_STREAM_LENGTH,
    DEVICE_CONFIG_LENGTH: DEVICE_CONFIG_LENGTH,
    GYRO_RANGES: GYRO_RANGES,
    ACCEL_RANGES: ACCEL_RANGES,
    BAUD_RATES: BAUD_RATES,
    GYRO_HIGH_PASS: GYRO_HIGH_PASS,
    AXIS_DEFINITIONS: AXIS_DEFINITIONS,
    crc16Ccitt: crc16Ccitt,
    scaleGyro: scaleGyro,
    scaleAccel: scaleAccel,
    parseHM_G11Burst: parseHM_G11Burst,
    parseHM_G12Frame: parseHM_G12Frame,
    validateHM_G12Envelope: validateHM_G12Envelope,
    parseHM_G12DeviceConfigFrame: parseHM_G12DeviceConfigFrame,
    parseHM_G12DeviceInfoFrame: parseHM_G12DeviceInfoFrame,
    parseHM_G12StatusFrame: parseHM_G12StatusFrame,
    makeHM_G12ConfigReadCommand: makeHM_G12ConfigReadCommand,
    makeHM_G12ConfigWriteCommand: makeHM_G12ConfigWriteCommand,
    makeHM_G12DeviceInfoCommand: makeHM_G12DeviceInfoCommand,
    createDefaultDeviceConfig: createDefaultDeviceConfig,
    gyroLowPassLabel: gyroLowPassLabel,
    accelFilterLabel: accelFilterLabel,
    encodeHM_G11GyroFilter: encodeHM_G11GyroFilter,
    decodeHM_G11GyroFilter: decodeHM_G11GyroFilter,
    encodeHM_G11AccelFilter: encodeHM_G11AccelFilter,
    decodeHM_G11AccelFilter: decodeHM_G11AccelFilter,
    encodeHM_G11Offset: encodeHM_G11Offset,
    decodeHM_G11Offset: decodeHM_G11Offset,
    HM_G12StreamParser: HM_G12StreamParser,
    makeHM_G12Command: makeHM_G12Command
  });
}(typeof window !== "undefined" ? window : globalThis));
