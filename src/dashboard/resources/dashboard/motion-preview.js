// SPDX-License-Identifier: Apache-2.0
"use strict";

// Pure display interpolation of a backend-validated, non-executable motion plan.
(() => {
  const finite = value => typeof value === "number" && Number.isFinite(value);
  const clamp = (value, low, high) => Math.max(low, Math.min(high, value));
  const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
  const close = (a, b, tolerance = 1e-5) => Math.abs(a - b) <= tolerance * Math.max(1, Math.abs(a), Math.abs(b));
  const point = value => value && finite(value.x_m) && finite(value.y_m) && Math.abs(value.x_m) <= 10000 && Math.abs(value.y_m) <= 10000;
  function evaluateBezier(curve, u) {
    u = clamp(u, 0, 1); const v = 1 - u;
    return {x_m: v ** 3 * curve.p0.x_m + 3 * v * v * u * curve.p1.x_m + 3 * v * u * u * curve.p2.x_m + u ** 3 * curve.p3.x_m,
      y_m: v ** 3 * curve.p0.y_m + 3 * v * v * u * curve.p1.y_m + 3 * v * u * u * curve.p2.y_m + u ** 3 * curve.p3.y_m};
  }
  function derivative(curve, u) {
    const v = 1 - u;
    return {x: 3 * (v * v * (curve.p1.x_m - curve.p0.x_m) + 2 * v * u * (curve.p2.x_m - curve.p1.x_m) + u * u * (curve.p3.x_m - curve.p2.x_m)),
      y: 3 * (v * v * (curve.p1.y_m - curve.p0.y_m) + 2 * v * u * (curve.p2.y_m - curve.p1.y_m) + u * u * (curve.p3.y_m - curve.p2.y_m))};
  }
  function curvature(curve, u) {
    const d = derivative(curve, u);
    const x = 6 * ((1 - u) * (curve.p2.x_m - 2 * curve.p1.x_m + curve.p0.x_m) + u * (curve.p3.x_m - 2 * curve.p2.x_m + curve.p1.x_m));
    const y = 6 * ((1 - u) * (curve.p2.y_m - 2 * curve.p1.y_m + curve.p0.y_m) + u * (curve.p3.y_m - 2 * curve.p2.y_m + curve.p1.y_m));
    const speed = Math.hypot(d.x, d.y);
    return speed > 1e-12 ? (d.x * y - d.y * x) / speed ** 3 : 0;
  }
  // Five-point Gaussian quadrature in 32 fixed bins. Inversion is bounded and
  // evaluates the original cubic; it never interpolates between XY samples.
  const nodes = [0, -.5384693101056831, .5384693101056831, -.906179845938664, .906179845938664];
  const weights = [.5688888888888889, .4786286704993665, .4786286704993665, .2369268850561891, .2369268850561891];
  function integrate(curve, low, high) {
    const half = (high - low) / 2, middle = (low + high) / 2;
    let sum = 0;
    for (let i = 0; i < nodes.length; i++) { const d = derivative(curve, middle + half * nodes[i]); sum += weights[i] * Math.hypot(d.x, d.y); }
    return sum * half;
  }
  function arcTable(curve) {
    const lengths = [0];
    for (let i = 1; i <= 32; i++) lengths.push(lengths[i - 1] + integrate(curve, (i - 1) / 32, i / 32));
    return {curve, lengths, length: lengths[32]};
  }
  function arcLength(curve, u = 1) {
    const table = arcTable(curve); u = clamp(u, 0, 1);
    const bin = Math.min(31, Math.floor(u * 32));
    return table.lengths[bin] + integrate(curve, bin / 32, u);
  }
  function inverse(table, distance) {
    if (distance <= 0 || table.length <= 1e-12) return 0;
    if (distance >= table.length) return 1;
    let bin = 0;
    while (bin < 31 && table.lengths[bin + 1] < distance) bin++;
    let low = bin / 32, high = (bin + 1) / 32;
    const remainder = distance - table.lengths[bin];
    for (let i = 0; i < 28; i++) {
      const middle = (low + high) / 2;
      if (integrate(table.curve, bin / 32, middle) < remainder) low = middle; else high = middle;
    }
    return (low + high) / 2;
  }
  function inverseArcLength(curve, distance) { return inverse(arcTable(curve), distance); }
  function create(motion) {
    if (!motion || motion.status !== "SUCCESS" || motion.executable !== false || motion.geometry_kind !== "cubic_bezier") throw new Error("No validated, non-executable Bézier motion profile is available.");
    if (![motion.duration_s, motion.length_m, motion.peak_speed_mps].every(value => finite(value) && value >= 0 && value <= 10000)) throw new Error("Motion summary must be finite and bounded.");
    if (!Array.isArray(motion.curves) || motion.curves.length > 4096 || !Array.isArray(motion.samples) || !motion.samples.length || motion.samples.length > 20000) throw new Error("Motion arrays exceed display limits.");
    const curves = motion.curves.map(curve => {
      if (![curve.p0, curve.p1, curve.p2, curve.p3].every(point) || !finite(curve.start_distance_m) || !finite(curve.length_m) || curve.length_m < 0) throw new Error("Invalid Bézier control points or curve distances.");
      return {...curve, table: arcTable(curve)};
    });
    let endDistance = 0;
    for (let i = 0; i < curves.length; i++) {
      const curve = curves[i];
      if (!close(curve.start_distance_m, endDistance) || !close(curve.table.length, curve.length_m, 2e-3)) throw new Error("Curve arclength metadata is inconsistent.");
      if (i && Math.hypot(curve.p0.x_m - curves[i - 1].p3.x_m, curve.p0.y_m - curves[i - 1].p3.y_m) > 1e-6) throw new Error("Bézier segments are disconnected.");
      endDistance += curve.length_m;
    }
    if (!close(endDistance, motion.length_m)) throw new Error("Motion length does not match its curves.");
    const fields = ["time_s", "distance_m", "u", "x_m", "y_m", "heading_rad", "tangent_rad", "speed_mps", "acceleration_mps2", "vx_mps", "vy_mps", "omega_radps"];
    const samples = motion.samples.map(sample => ({...sample}));
    for (let i = 0; i < samples.length; i++) {
      const sample = samples[i];
      if (!fields.every(key => finite(sample[key]) && Math.abs(sample[key]) <= 1e6) || sample.speed_mps < 0 || sample.distance_m < 0 || sample.u < 0 || sample.u > 1 || !Number.isInteger(sample.segment)) throw new Error("Invalid motion sample.");
      if (curves.length) {
        const curve = curves[sample.segment]; if (!curve) throw new Error("Motion sample references an absent curve.");
        const p = evaluateBezier(curve, sample.u);
        if (Math.hypot(p.x_m - sample.x_m, p.y_m - sample.y_m) > 1e-5) throw new Error("Motion sample does not lie on its cubic.");
        const bin = Math.min(31, Math.floor(sample.u * 32));
        const measured = curve.table.lengths[bin] + integrate(curve, bin / 32, sample.u);
        const expectedDistance = curve.start_distance_m + (curve.table.length ? measured / curve.table.length * curve.length_m : 0);
        if (!close(sample.distance_m, expectedDistance, 5e-4)) throw new Error("Motion sample arclength is inconsistent with its cubic parameter.");
      } else if (motion.length_m !== 0 || sample.distance_m !== 0 || Math.hypot(sample.x_m - samples[0].x_m, sample.y_m - samples[0].y_m) > 1e-8) throw new Error("A stationary profile cannot translate.");
      if (i) {
        const previous = samples[i - 1], dt = sample.time_s - previous.time_s;
        if (dt <= 0 || sample.distance_m < previous.distance_m || !close(sample.distance_m - previous.distance_m, (previous.speed_mps + sample.speed_mps) * dt / 2)
            || !close(wrap(sample.heading_rad - previous.heading_rad), (previous.omega_radps + sample.omega_radps) * dt / 2)) throw new Error("Motion samples do not describe analytic linear speed/angular-speed intervals.");
      }
    }
    if (!close(samples[0].time_s, 0) || !close(samples[0].distance_m, 0) || !close(samples.at(-1).time_s, motion.duration_s) || !close(samples.at(-1).distance_m, motion.length_m)) throw new Error("Motion endpoints do not match its duration and length.");
    if (!close(Math.max(...samples.map(sample => sample.speed_mps)), motion.peak_speed_mps)) throw new Error("Motion peak speed does not match its samples.");
    if (motion.duration_s === 0 && samples.length !== 1) throw new Error("Zero-duration motion needs exactly one stationary sample.");
    return {motion, curves, samples, duration: motion.duration_s, length: motion.length_m};
  }
  function location(model, distance, fallback) {
    if (!model.curves.length) return {x_m: fallback.x_m, y_m: fallback.y_m, segment: fallback.segment, u: fallback.u, tangent_rad: fallback.tangent_rad};
    let segment = model.curves.findIndex(curve => distance <= curve.start_distance_m + curve.length_m + 1e-10);
    if (segment < 0) segment = model.curves.length - 1;
    const curve = model.curves[segment];
    const local = curve.length_m ? clamp((distance - curve.start_distance_m) / curve.length_m, 0, 1) : 0;
    const u = inverse(curve.table, local * curve.table.length), p = evaluateBezier(curve, u), d = derivative(curve, u);
    return {...p, segment, u, tangent_rad: Math.hypot(d.x, d.y) > 1e-12 ? Math.atan2(d.y, d.x) : fallback.tangent_rad};
  }
  function sampleAt(model, time) {
    if (!finite(time)) throw new Error("Preview time must be finite.");
    time = clamp(time, 0, model.duration);
    const samples = model.samples;
    if (!model.duration || time === model.duration) return {...samples.at(-1), time_s: time};
    let low = 0, high = samples.length - 1;
    while (low + 1 < high) { const middle = Math.floor((low + high) / 2); if (samples[middle].time_s <= time) low = middle; else high = middle; }
    const a = samples[low], b = samples[high], dt = b.time_s - a.time_s, elapsed = time - a.time_s;
    const acceleration = (b.speed_mps - a.speed_mps) / dt, angularAcceleration = (b.omega_radps - a.omega_radps) / dt;
    const distance = clamp(a.distance_m + a.speed_mps * elapsed + acceleration * elapsed * elapsed / 2, a.distance_m, b.distance_m);
    const p = location(model, distance, a), speed = a.speed_mps + acceleration * elapsed;
    const centripetal = model.curves.length ? Math.abs(curvature(model.curves[p.segment], p.u)) * speed * speed : 0;
    return {...p, time_s: time, distance_m: distance, heading_rad: wrap(a.heading_rad + a.omega_radps * elapsed + angularAcceleration * elapsed * elapsed / 2),
      speed_mps: speed, acceleration_mps2: acceleration, total_acceleration_mps2: Math.hypot(acceleration, centripetal),
      vx_mps: speed * Math.cos(p.tangent_rad), vy_mps: speed * Math.sin(p.tangent_rad), omega_radps: a.omega_radps + angularAcceleration * elapsed};
  }
  function createPlayback(model, isCurrent = () => true) {
    const playback = {time: 0, playing: false, finished: false, cancelled: false, sample: sampleAt(model, 0)};
    playback.play = () => { if (!playback.cancelled && isCurrent()) playback.playing = true; return playback.playing; };
    playback.pause = () => { playback.playing = false; };
    playback.cancel = () => { playback.cancelled = true; playback.playing = false; };
    playback.advance = seconds => {
      if (!finite(seconds) || seconds < 0) throw new Error("Playback advance must be finite and nonnegative.");
      if (!isCurrent()) playback.cancel();
      if (!playback.playing || playback.cancelled) return playback.sample;
      playback.time = Math.min(model.duration, playback.time + seconds);
      playback.sample = sampleAt(model, playback.time);
      playback.finished = playback.time >= model.duration;
      if (playback.finished) playback.pause();
      return playback.sample;
    };
    return playback;
  }
  globalThis.MotionPreview = Object.freeze({create, sampleAt, evaluateBezier, arcLength, inverseArcLength, createPlayback});
})();
