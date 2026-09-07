(function (root) {
  "use strict";

  var RAD_PER_DEG = Math.PI / 180;
  var DEG_PER_RAD = 180 / Math.PI;
  var U32_RANGE = 0x100000000;
  var GRAVITY_DIRECTION = [0, 0, 1];

  function createDefaultConfig(overrides) {
    var config = {
      gyroTiltStd1sDeg: 0.3,
      sigmaGyro: 0,
      sigmaGyroBias: 1e-5,
      gyroScaleFactorError: 0.01,
      accelVarianceBase: 5e-2,
      accelG3Sigma: 0.1,
      accelReleaseTau: 0.3,
      nisReject: 50,
      nisInflateGamma: 6,
      zaruVariance: 1e-6,
      zaruStaticFrames: 300,
      motionWindowLength: 200,
      staticGyroThreshold: 3 * RAD_PER_DEG,
      staticAccelRelativeStdThreshold: 0.026,
      motionGyroFull: 20 * RAD_PER_DEG,
      motionAccelFull: 0.30,
      initializationMode: "static",
      initializationStationarySeconds: 5,
      initializationTiltSeconds: 0.5,
      initialAttitudeVariance: Math.pow(2 * RAD_PER_DEG, 2),
      initialBiasVariance: Math.pow(5e-3, 2),
      maximumDeltaSeconds: 0.05
    };
    Object.keys(overrides || {}).forEach(function (key) {
      if (!(key in config)) {
        throw new Error("Unknown ESKF configuration option: " + key);
      }
      config[key] = overrides[key];
    });
    if (config.motionWindowLength < 2) {
      throw new Error("The ESKF stationary-detection window requires at least 2 samples");
    }
    if (config.initializationMode !== "static" && config.initializationMode !== "tilt") {
      throw new Error("The ESKF initialization mode must be static or tilt");
    }
    if (!(config.sigmaGyro > 0)) {
      config.sigmaGyro = config.gyroTiltStd1sDeg * RAD_PER_DEG;
    }
    return config;
  }

  function vectorNorm(vector) {
    return Math.hypot(vector[0], vector[1], vector[2]);
  }

  function scaleGyroscopeRaw(rawValues, rangeDps) {
    if (!(Number.isFinite(rangeDps) && rangeDps > 0)) {
      throw new Error("The ESKF gyroscope range is invalid");
    }
    return Array.from(rawValues, function (value) {
      return value / 65536 * 35 / 1000000 * rangeDps * RAD_PER_DEG;
    });
  }

  function scaleAccelerationRaw(rawValues, rangeG) {
    if (!(Number.isFinite(rangeG) && rangeG > 0)) {
      throw new Error("The ESKF accelerometer range is invalid");
    }
    return Array.from(rawValues, function (value) {
      return value / 65536 / 32768 * rangeG;
    });
  }

  function quaternionNormalize(quaternion) {
    var norm = Math.hypot(quaternion[0], quaternion[1], quaternion[2], quaternion[3]);
    if (norm < 1e-12) {
      return [1, 0, 0, 0];
    }
    var result = quaternion.map(function (value) { return value / norm; });
    if (result[0] < 0) {
      result = result.map(function (value) { return -value; });
    }
    return result;
  }

  function quaternionMultiply(left, right) {
    var lw = left[0];
    var lx = left[1];
    var ly = left[2];
    var lz = left[3];
    var rw = right[0];
    var rx = right[1];
    var ry = right[2];
    var rz = right[3];
    return [
      lw * rw - lx * rx - ly * ry - lz * rz,
      lw * rx + lx * rw + ly * rz - lz * ry,
      lw * ry - lx * rz + ly * rw + lz * rx,
      lw * rz + lx * ry - ly * rx + lz * rw
    ];
  }

  function quaternionFromRotationVector(vector) {
    var angle = vectorNorm(vector);
    if (angle < 1e-8) {
      return quaternionNormalize([1, vector[0] * 0.5, vector[1] * 0.5, vector[2] * 0.5]);
    }
    var halfAngle = angle * 0.5;
    var scale = Math.sin(halfAngle) / angle;
    return [Math.cos(halfAngle), vector[0] * scale, vector[1] * scale, vector[2] * scale];
  }

  function quaternionFromEuler(roll, pitch, yaw) {
    var cr = Math.cos(roll * 0.5);
    var sr = Math.sin(roll * 0.5);
    var cp = Math.cos(pitch * 0.5);
    var sp = Math.sin(pitch * 0.5);
    var cy = Math.cos(yaw * 0.5);
    var sy = Math.sin(yaw * 0.5);
    return quaternionNormalize([
      cr * cp * cy + sr * sp * sy,
      sr * cp * cy - cr * sp * sy,
      cr * sp * cy + sr * cp * sy,
      cr * cp * sy - sr * sp * cy
    ]);
  }

  function quaternionToRotationMatrix(quaternion) {
    var q = quaternionNormalize(quaternion);
    var w = q[0];
    var x = q[1];
    var y = q[2];
    var z = q[3];
    return new Float64Array([
      1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y),
      2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x),
      2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)
    ]);
  }

  function quaternionToEulerDegrees(quaternion) {
    var q = quaternionNormalize(quaternion);
    var w = q[0];
    var x = q[1];
    var y = q[2];
    var z = q[3];
    var roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y));
    var sinPitch = Math.max(-1, Math.min(1, 2 * (w * y - z * x)));
    var pitch = Math.asin(sinPitch);
    var yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z));
    return [roll * DEG_PER_RAD, pitch * DEG_PER_RAD, yaw * DEG_PER_RAD];
  }

  function rollPitchFromAcceleration(acceleration) {
    return [
      Math.atan2(acceleration[1], acceleration[2]),
      Math.atan2(-acceleration[0], Math.hypot(acceleration[1], acceleration[2]))
    ];
  }

  function identityMatrix(size) {
    var result = new Float64Array(size * size);
    for (var index = 0; index < size; index += 1) {
      result[index * size + index] = 1;
    }
    return result;
  }

  function multiplyMatrices(left, leftRows, sharedSize, right, rightColumns) {
    var result = new Float64Array(leftRows * rightColumns);
    for (var row = 0; row < leftRows; row += 1) {
      for (var column = 0; column < rightColumns; column += 1) {
        var value = 0;
        for (var shared = 0; shared < sharedSize; shared += 1) {
          value += left[row * sharedSize + shared] * right[shared * rightColumns + column];
        }
        result[row * rightColumns + column] = value;
      }
    }
    return result;
  }

  function transposeMatrix(matrix, rows, columns) {
    var result = new Float64Array(rows * columns);
    for (var row = 0; row < rows; row += 1) {
      for (var column = 0; column < columns; column += 1) {
        result[column * rows + row] = matrix[row * columns + column];
      }
    }
    return result;
  }

  function inverseThreeByThree(matrix) {
    var a = matrix[0];
    var b = matrix[1];
    var c = matrix[2];
    var d = matrix[3];
    var e = matrix[4];
    var f = matrix[5];
    var g = matrix[6];
    var h = matrix[7];
    var i = matrix[8];
    var A = e * i - f * h;
    var B = c * h - b * i;
    var C = b * f - c * e;
    var D = f * g - d * i;
    var E = a * i - c * g;
    var F = c * d - a * f;
    var G = d * h - e * g;
    var H = b * g - a * h;
    var I = a * e - b * d;
    var determinant = a * A + b * D + c * G;
    if (!Number.isFinite(determinant) || Math.abs(determinant) < 1e-18) {
      throw new Error("The ESKF innovation covariance is singular");
    }
    var inverseDeterminant = 1 / determinant;
    return new Float64Array([
      A * inverseDeterminant, B * inverseDeterminant, C * inverseDeterminant,
      D * inverseDeterminant, E * inverseDeterminant, F * inverseDeterminant,
      G * inverseDeterminant, H * inverseDeterminant, I * inverseDeterminant
    ]);
  }

  function multiplyMatrixVector(matrix, rows, columns, vector) {
    var result = new Float64Array(rows);
    for (var row = 0; row < rows; row += 1) {
      var value = 0;
      for (var column = 0; column < columns; column += 1) {
        value += matrix[row * columns + column] * vector[column];
      }
      result[row] = value;
    }
    return result;
  }

  function MotionDetector(config) {
    this.config = config;
    this.reset();
  }

  MotionDetector.prototype.reset = function () {
    this.gyroscopeNorms = [];
    this.accelerationNorms = [];
  };

  MotionDetector.prototype.update = function (gyroscope, acceleration) {
    var config = this.config;
    this.gyroscopeNorms.push(vectorNorm(gyroscope));
    this.accelerationNorms.push(vectorNorm(acceleration));
    if (this.gyroscopeNorms.length > config.motionWindowLength) {
      this.gyroscopeNorms.shift();
      this.accelerationNorms.shift();
    }

    var count = this.gyroscopeNorms.length;
    var gyroscopeMean = 0;
    var accelerationMean = 0;
    for (var index = 0; index < count; index += 1) {
      gyroscopeMean += this.gyroscopeNorms[index];
      accelerationMean += this.accelerationNorms[index];
    }
    gyroscopeMean /= count;
    accelerationMean /= count;
    var accelerationVariance = 0;
    for (var sampleIndex = 0; sampleIndex < count; sampleIndex += 1) {
      var difference = this.accelerationNorms[sampleIndex] - accelerationMean;
      accelerationVariance += difference * difference;
    }
    var gravityReference = accelerationMean > 1e-9 ? accelerationMean : 1;
    var accelerationRelativeStd = Math.sqrt(accelerationVariance / count) / gravityReference;
    var accelerationRelativeDeviation = Math.abs(this.accelerationNorms[count - 1] - accelerationMean) / gravityReference;
    var isStatic =
      count >= config.motionWindowLength &&
      gyroscopeMean < config.staticGyroThreshold &&
      accelerationRelativeStd < config.staticAccelRelativeStdThreshold;
    var gyroscopeMotion = gyroscopeMean / Math.max(config.motionGyroFull, 1e-9);
    var accelerationMotion = accelerationRelativeDeviation / Math.max(config.motionAccelFull, 1e-9);

    return {
      isStatic: isStatic,
      motionMetric: Math.max(0, Math.min(1, Math.max(gyroscopeMotion, accelerationMotion))),
      gyroscopeMeanNorm: gyroscopeMean,
      accelerationRelativeStd: accelerationRelativeStd,
      accelerationRelativeDeviation: accelerationRelativeDeviation,
      gravityReference: gravityReference
    };
  };

  function AttitudeEskf(config) {
    this.config = createDefaultConfig(config);
    this.detector = new MotionDetector(this.config);
    this.reset();
  }

  AttitudeEskf.prototype.reset = function () {
    this.state = "INIT";
    this.quaternion = [1, 0, 0, 0];
    this.gyroscopeBias = [0, 0, 0];
    this.covariance = identityMatrix(6);
    this.detector.reset();
    this.previousTimeUs = null;
    this.initializationStartedUs = null;
    this.initializationGyroscopeSum = [0, 0, 0];
    this.initializationAccelerationSum = [0, 0, 0];
    this.initializationCount = 0;
    this.gravityReference = 1;
    this.mode = "INIT";
    this.accelerationDeviationHold = 0;
    this.staticRun = 0;
    this.lastAccelerationDeviation = 0;
    this.lastAccelerationCovariance = null;
    this.lastAccelerationGain = null;
    this.inputMode = null;
    this.gyroscopeRangeDps = null;
    this.accelerationRangeG = null;
    this.lastOutput = null;
  };

  AttitudeEskf.prototype.deltaSeconds = function (previousTimeUs, currentTimeUs) {
    var difference = (currentTimeUs >>> 0) - (previousTimeUs >>> 0);
    if (difference < 0) {
      difference += U32_RANGE;
    }
    return difference * 1e-6;
  };

  AttitudeEskf.prototype.process = function (sample) {
    var hasRawSample = sample.gyroRaw && sample.accelRaw;
    var gyroscope;
    var acceleration;
    if (hasRawSample) {
      gyroscope = scaleGyroscopeRaw(sample.gyroRaw, sample.gyroRangeDps);
      acceleration = scaleAccelerationRaw(sample.accelRaw, sample.accelRangeG);
      this.inputMode = "raw";
      this.gyroscopeRangeDps = sample.gyroRangeDps;
      this.accelerationRangeG = sample.accelRangeG;
    } else {
      gyroscope = Array.from(sample.gyroDps, function (value) { return value * RAD_PER_DEG; });
      acceleration = Array.from(sample.accelG);
      this.inputMode = "scaled";
      this.gyroscopeRangeDps = null;
      this.accelerationRangeG = null;
    }
    var motion = this.detector.update(gyroscope, acceleration);
    var currentTimeUs = sample.timerUs >>> 0;
    var output;
    if (this.state === "INIT") {
      output = this.processInitialization(gyroscope, acceleration, motion, currentTimeUs);
    } else {
      output = this.processRunning(gyroscope, acceleration, motion, currentTimeUs);
    }
    this.previousTimeUs = currentTimeUs;
    this.lastOutput = output;
    return output;
  };

  AttitudeEskf.prototype.clearInitializationAccumulation = function () {
    this.initializationStartedUs = null;
    this.initializationGyroscopeSum = [0, 0, 0];
    this.initializationAccelerationSum = [0, 0, 0];
    this.initializationCount = 0;
  };

  AttitudeEskf.prototype.processInitialization = function (gyroscope, acceleration, motion, currentTimeUs) {
    if (this.config.initializationMode === "tilt") {
      return this.processTiltInitialization(gyroscope, acceleration, motion, currentTimeUs);
    }
    if (!motion.isStatic) {
      this.clearInitializationAccumulation();
    } else {
      if (this.initializationStartedUs === null) {
        this.initializationStartedUs = currentTimeUs;
      }
      for (var axis = 0; axis < 3; axis += 1) {
        this.initializationGyroscopeSum[axis] += gyroscope[axis];
        this.initializationAccelerationSum[axis] += acceleration[axis];
      }
      this.initializationCount += 1;
      var elapsed = this.deltaSeconds(this.initializationStartedUs, currentTimeUs);
      if (elapsed >= this.config.initializationStationarySeconds && this.initializationCount > 1) {
        this.finalizeInitialization();
      }
    }
    return this.makeOutput(motion, false, false, gyroscope, acceleration);
  };

  AttitudeEskf.prototype.processTiltInitialization = function (gyroscope, acceleration, motion, currentTimeUs) {
    if (this.initializationStartedUs === null) {
      this.initializationStartedUs = currentTimeUs;
    }
    for (var axis = 0; axis < 3; axis += 1) {
      this.initializationAccelerationSum[axis] += acceleration[axis];
    }
    this.initializationCount += 1;
    var elapsed = this.deltaSeconds(this.initializationStartedUs, currentTimeUs);
    if (elapsed >= this.config.initializationTiltSeconds && this.initializationCount > 1) {
      var meanAcceleration = this.initializationAccelerationSum.map(function (value) {
        return value / this.initializationCount;
      }, this);
      this.gravityReference = vectorNorm(meanAcceleration);
      var tilt = rollPitchFromAcceleration(meanAcceleration);
      this.quaternion = quaternionFromEuler(tilt[0], tilt[1], 0);
      this.gyroscopeBias = [0, 0, 0];
      this.initializeCovariance();
      this.state = "RUNNING";
    }
    return this.makeOutput(motion, false, false, gyroscope, acceleration);
  };

  AttitudeEskf.prototype.initializeCovariance = function () {
    this.covariance = new Float64Array(36);
    for (var axis = 0; axis < 3; axis += 1) {
      this.covariance[axis * 6 + axis] = this.config.initialAttitudeVariance;
      this.covariance[(axis + 3) * 6 + axis + 3] = this.config.initialBiasVariance;
    }
  };

  AttitudeEskf.prototype.finalizeInitialization = function () {
    var count = Math.max(this.initializationCount, 1);
    this.gyroscopeBias = this.initializationGyroscopeSum.map(function (value) { return value / count; });
    var meanAcceleration = this.initializationAccelerationSum.map(function (value) { return value / count; });
    this.gravityReference = vectorNorm(meanAcceleration);
    var tilt = rollPitchFromAcceleration(meanAcceleration);
    this.quaternion = quaternionFromEuler(tilt[0], tilt[1], 0);
    this.initializeCovariance();
    this.state = "RUNNING";
  };

  AttitudeEskf.prototype.processRunning = function (gyroscope, acceleration, motion, currentTimeUs) {
    var deltaSeconds = this.previousTimeUs === null ? 0 : this.deltaSeconds(this.previousTimeUs, currentTimeUs);
    if (deltaSeconds > 0 && deltaSeconds <= this.config.maximumDeltaSeconds) {
      this.predict(gyroscope, deltaSeconds);
    }
    var accelerationUsed = this.updateAcceleration(acceleration, deltaSeconds);
    if (motion.isStatic) {
      this.staticRun += 1;
    } else {
      this.staticRun = 0;
    }
    var zaruUsed = false;
    if (this.staticRun >= this.config.zaruStaticFrames) {
      this.updateZaru(gyroscope);
      zaruUsed = true;
    }
    this.quaternion = quaternionNormalize(this.quaternion);
    return this.makeOutput(motion, accelerationUsed, zaruUsed, gyroscope, acceleration);
  };

  AttitudeEskf.prototype.predict = function (gyroscope, deltaSeconds) {
    var angularRate = gyroscope.map(function (value, axis) {
      return value - this.gyroscopeBias[axis];
    }, this);
    var deltaQuaternion = quaternionFromRotationVector(angularRate.map(function (value) {
      return value * deltaSeconds;
    }));
    this.quaternion = quaternionNormalize(quaternionMultiply(this.quaternion, deltaQuaternion));

    var transition = identityMatrix(6);
    var errorRotation = quaternionToRotationMatrix(quaternionFromRotationVector(angularRate.map(function (value) {
      return -value * deltaSeconds;
    })));
    for (var row = 0; row < 3; row += 1) {
      for (var column = 0; column < 3; column += 1) {
        transition[row * 6 + column] = errorRotation[row * 3 + column];
      }
      transition[row * 6 + row + 3] = -deltaSeconds;
    }
    var propagated = multiplyMatrices(transition, 6, 6, this.covariance, 6);
    propagated = multiplyMatrices(propagated, 6, 6, transposeMatrix(transition, 6, 6), 6);
    var angularRateNorm = vectorNorm(angularRate);
    var attitudeNoise = (
      this.config.sigmaGyro * this.config.sigmaGyro +
      Math.pow(this.config.gyroScaleFactorError * angularRateNorm, 2)
    ) * deltaSeconds;
    var biasNoise = this.config.sigmaGyroBias * this.config.sigmaGyroBias * deltaSeconds;
    for (var axis = 0; axis < 3; axis += 1) {
      propagated[axis * 6 + axis] += attitudeNoise;
      propagated[(axis + 3) * 6 + axis + 3] += biasNoise;
    }
    this.covariance = propagated;
  };

  AttitudeEskf.prototype.updateAcceleration = function (acceleration, deltaSeconds) {
    this.lastAccelerationCovariance = null;
    this.lastAccelerationGain = null;
    var norm = vectorNorm(acceleration);
    if (norm < 1e-9) {
      this.mode = "NO_ACC";
      this.lastAccelerationDeviation = 0;
      return false;
    }
    var deviation = Math.abs(norm / Math.max(this.gravityReference, 1e-9) - 1);
    this.lastAccelerationDeviation = deviation;
    var decay;
    if (this.config.accelReleaseTau > 0 && deltaSeconds > 0) {
      decay = Math.exp(-deltaSeconds / this.config.accelReleaseTau);
    } else if (this.config.accelReleaseTau > 0) {
      decay = 1;
    } else {
      decay = 0;
    }
    this.accelerationDeviationHold = Math.max(deviation, this.accelerationDeviationHold * decay);
    var measurement = acceleration.map(function (value) { return value / norm; });
    var rotation = quaternionToRotationMatrix(this.quaternion);
    var predictedGravity = [rotation[6], rotation[7], rotation[8]];
    var innovation = measurement.map(function (value, axis) { return value - predictedGravity[axis]; });
    var measurementMatrix = new Float64Array(18);
    measurementMatrix[1] = -predictedGravity[2];
    measurementMatrix[2] = predictedGravity[1];
    measurementMatrix[6] = predictedGravity[2];
    measurementMatrix[8] = -predictedGravity[0];
    measurementMatrix[12] = -predictedGravity[1];
    measurementMatrix[13] = predictedGravity[0];

    var sigmaDeviation = this.config.accelG3Sigma / 3;
    var exponent = Math.min(0.5 * Math.pow(this.accelerationDeviationHold / sigmaDeviation, 2), 50);
    var baseVariance = this.config.accelVarianceBase * Math.exp(exponent);
    var transpose = transposeMatrix(measurementMatrix, 3, 6);
    var covarianceTimesTranspose = multiplyMatrices(this.covariance, 6, 6, transpose, 3);
    var innovationCovariance = multiplyMatrices(measurementMatrix, 3, 6, covarianceTimesTranspose, 3);
    for (var diagonal = 0; diagonal < 3; diagonal += 1) {
      innovationCovariance[diagonal * 3 + diagonal] += baseVariance;
    }
    var solvedInnovation = multiplyMatrixVector(inverseThreeByThree(innovationCovariance), 3, 3, innovation);
    var nis = innovation[0] * solvedInnovation[0] + innovation[1] * solvedInnovation[1] + innovation[2] * solvedInnovation[2];
    if (nis > this.config.nisReject) {
      this.mode = "REJECT";
      return false;
    }
    var variance = baseVariance * Math.max(1, nis / this.config.nisInflateGamma);
    this.mode = "RUN";
    this.lastAccelerationGain = this.kalmanUpdate(innovation, measurementMatrix, variance);
    this.lastAccelerationCovariance = new Float64Array([
      variance, 0, 0,
      0, variance, 0,
      0, 0, variance
    ]);
    return true;
  };

  AttitudeEskf.prototype.updateZaru = function (gyroscope) {
    var innovation = gyroscope.map(function (value, axis) {
      return value - this.gyroscopeBias[axis];
    }, this);
    var measurementMatrix = new Float64Array(18);
    measurementMatrix[3] = 1;
    measurementMatrix[10] = 1;
    measurementMatrix[17] = 1;
    this.kalmanUpdate(innovation, measurementMatrix, this.config.zaruVariance);
  };

  AttitudeEskf.prototype.kalmanUpdate = function (innovation, measurementMatrix, variance) {
    var measurementTranspose = transposeMatrix(measurementMatrix, 3, 6);
    var covarianceTimesTranspose = multiplyMatrices(this.covariance, 6, 6, measurementTranspose, 3);
    var innovationCovariance = multiplyMatrices(measurementMatrix, 3, 6, covarianceTimesTranspose, 3);
    for (var diagonal = 0; diagonal < 3; diagonal += 1) {
      innovationCovariance[diagonal * 3 + diagonal] += variance;
    }
    var gain = multiplyMatrices(covarianceTimesTranspose, 6, 3, inverseThreeByThree(innovationCovariance), 3);
    var correction = multiplyMatrixVector(gain, 6, 3, innovation);
    this.quaternion = quaternionNormalize(quaternionMultiply(
      this.quaternion,
      quaternionFromRotationVector([correction[0], correction[1], correction[2]])
    ));
    for (var axis = 0; axis < 3; axis += 1) {
      this.gyroscopeBias[axis] += correction[axis + 3];
    }

    var gainTimesMeasurement = multiplyMatrices(gain, 6, 3, measurementMatrix, 6);
    var josephLeft = identityMatrix(6);
    for (var entry = 0; entry < 36; entry += 1) {
      josephLeft[entry] -= gainTimesMeasurement[entry];
    }
    var josephCovariance = multiplyMatrices(josephLeft, 6, 6, this.covariance, 6);
    josephCovariance = multiplyMatrices(josephCovariance, 6, 6, transposeMatrix(josephLeft, 6, 6), 6);
    var gainTranspose = transposeMatrix(gain, 6, 3);
    var gainNoise = multiplyMatrices(gain, 6, 3, gainTranspose, 6);
    for (var index = 0; index < 36; index += 1) {
      josephCovariance[index] += gainNoise[index] * variance;
    }
    for (var row = 0; row < 6; row += 1) {
      for (var column = row + 1; column < 6; column += 1) {
        var symmetric = 0.5 * (josephCovariance[row * 6 + column] + josephCovariance[column * 6 + row]);
        josephCovariance[row * 6 + column] = symmetric;
        josephCovariance[column * 6 + row] = symmetric;
      }
    }
    this.covariance = josephCovariance;
    return gain;
  };

  AttitudeEskf.prototype.makeOutput = function (motion, accelerationUsed, zaruUsed, gyroscope, acceleration) {
    return {
      state: this.state,
      eulerDegrees: quaternionToEulerDegrees(this.quaternion),
      quaternion: this.quaternion.slice(),
      gyroscopeBiasRad: this.gyroscopeBias.slice(),
      gyroscopeBiasDps: this.gyroscopeBias.map(function (value) { return value * DEG_PER_RAD; }),
      isStatic: motion.isStatic,
      motionMetric: motion.motionMetric,
      accelerationUsed: accelerationUsed,
      zaruUsed: zaruUsed,
      mode: this.mode,
      gyroscopeRad: gyroscope.slice(),
      accelerationG: acceleration.slice(),
      inputMode: this.inputMode,
      gyroscopeRangeDps: this.gyroscopeRangeDps,
      accelerationRangeG: this.accelerationRangeG,
      accelerationDeviation: this.lastAccelerationDeviation,
      covariance: Array.from(this.covariance),
      accelerationCovariance: this.lastAccelerationCovariance ? Array.from(this.lastAccelerationCovariance) : null,
      accelerationGain: this.lastAccelerationGain ? Array.from(this.lastAccelerationGain) : null
    };
  };

  root.ImuEskf = Object.freeze({
    AttitudeEskf: AttitudeEskf,
    createDefaultConfig: createDefaultConfig,
    scaleGyroscopeRaw: scaleGyroscopeRaw,
    scaleAccelerationRaw: scaleAccelerationRaw,
    quaternionToEulerDegrees: quaternionToEulerDegrees
  });
}(typeof window !== "undefined" ? window : globalThis));
