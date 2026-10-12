// SPDX-License-Identifier: Apache-2.0
// Tiny authored interpolation records, not solver or hardware qualifications.
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';

const context = vm.createContext({});
vm.runInContext(readFileSync(new URL('../../src/dashboard/resources/dashboard/motion-preview.js', import.meta.url), 'utf8'), context);
const api = context.MotionPreview;
const wrap = angle => Math.atan2(Math.sin(angle), Math.cos(angle));
const near = (a, b, tolerance = 1e-7) => assert.ok(Math.abs(a - b) <= tolerance, `${a} differs from ${b}`);
const point = (x_m, y_m) => ({x_m, y_m});
const line = (start, end, distance = 0) => ({p0: point(start, 0), p1: point(start + (end - start) / 3, 0),
  p2: point(start + 2 * (end - start) / 3, 0), p3: point(end, 0), start_distance_m: distance, length_m: Math.abs(end - start)});
function sample(curve, time, distance, u, speed, heading = 0, omega = 0, segment = 0) {
  const p = api.evaluateBezier(curve, u);
  return {...p, time_s: time, distance_m: distance, segment, u, speed_mps: speed,
    acceleration_mps2: time < 1 ? 2 : -2, heading_rad: wrap(heading), tangent_rad: 0,
    vx_mps: speed, vy_mps: 0, omega_radps: omega};
}
function triangle(curves = [line(0, 2)]) {
  const first = curves[0], last = curves.at(-1);
  return {status: 'SUCCESS', executable: false, geometry_kind: 'cubic_bezier', duration_s: 2,
    length_m: 2, peak_speed_mps: 2, candidate_count: 1, detail: 'Authored interpolation test', curves,
    samples: [sample(first, 0, 0, 0, 0), sample(first, 1, 1, curves.length === 1 ? .5 : 1, 2),
      sample(last, 2, 2, 1, 0, 0, 0, curves.length - 1)]};
}

let passed = 0;
function check(name, test) {
  try { test(); passed++; }
  catch (error) { throw new Error(`${name}: ${error.message}`, {cause: error}); }
}

check('accelerating and braking intervals integrate speed analytically', () => {
  const model = api.create(triangle()), accelerating = api.sampleAt(model, .5), braking = api.sampleAt(model, 1.5);
  near(accelerating.distance_m, .25); near(accelerating.x_m, .25); near(accelerating.speed_mps, 1); near(accelerating.acceleration_mps2, 2);
  near(braking.distance_m, 1.75); near(braking.x_m, 1.75); near(braking.speed_mps, 1); near(braking.acceleration_mps2, -2);
});
check('inverse arclength evaluates the cubic rather than a sample chord', () => {
  const curve = {p0: point(0, 0), p1: point(0, 1), p2: point(1, 1), p3: point(1, 0), start_distance_m: 0, length_m: 2};
  const profile = triangle([curve]);
  const model = api.create(profile), middle = api.sampleAt(model, 1), early = api.sampleAt(model, .5);
  near(middle.x_m, .5); near(middle.y_m, .75); near(api.arcLength(curve), 2);
  near(api.arcLength(curve, early.u), .25); assert.ok(early.y_m > .2);
  assert.ok(early.total_acceleration_mps2 > Math.abs(early.acceleration_mps2));
});
check('elapsed lookup crosses segment joins without interpolating reset u', () => {
  const model = api.create(triangle([line(0, 1), line(1, 2, 1)]));
  const value = api.sampleAt(model, 1.5);
  assert.equal(value.segment, 1); near(value.u, .75); near(value.x_m, 1.75);
  const player = api.createPlayback(model); player.play(); player.advance(1.7);
  near(player.time, 1.7); assert.equal(player.sample.segment, 1);
});
check('holonomic heading wraps independently of travel tangent', () => {
  const profile = triangle(), startHeading = Math.PI - .2;
  profile.samples[0].heading_rad = startHeading;
  profile.samples[1].heading_rad = wrap(startHeading + .5); profile.samples[1].omega_radps = 1;
  profile.samples[2].heading_rad = wrap(startHeading + 1);
  const value = api.sampleAt(api.create(profile), .5);
  near(value.heading_rad, wrap(startHeading + .125)); near(value.omega_radps, .5); near(value.tangent_rad, 0);
  assert.ok(Math.abs(value.heading_rad - value.tangent_rad) > 2);
});
check('stationary rotation preserves position while angular speed varies', () => {
  const profile = triangle(); profile.curves = []; profile.length_m = 0; profile.peak_speed_mps = 0;
  profile.samples.forEach((value, index) => Object.assign(value, {x_m: 2, y_m: 3, distance_m: 0, speed_mps: 0, vx_mps: 0, acceleration_mps2: 0,
    heading_rad: [0, .5, 1][index], omega_radps: [0, 1, 0][index]}));
  const value = api.sampleAt(api.create(profile), 1.5);
  near(value.x_m, 2); near(value.y_m, 3); near(value.heading_rad, .875); near(value.speed_mps, 0);
});
check('zero duration and endpoint clamping are explicit', () => {
  const profile = triangle(); profile.curves = []; profile.samples = [profile.samples[0]];
  profile.length_m = 0; profile.duration_s = 0; profile.peak_speed_mps = 0;
  const player = api.createPlayback(api.create(profile)); player.play(); player.advance(0);
  assert.equal(player.finished, true); near(player.sample.x_m, 0);
  const last = api.sampleAt(api.create(triangle()), 999); near(last.x_m, 2); near(last.speed_mps, 0); near(last.time_s, 2);
});
check('pause freezes profile time and resume preserves accumulated time', () => {
  const player = api.createPlayback(api.create(triangle())); player.play(); player.advance(.4); player.pause(); player.advance(1);
  near(player.time, .4); player.play(); player.advance(.3); near(player.time, .7);
});
check('cancellation and an obsolete-context guard cannot resume playback', () => {
  let current = true;
  const player = api.createPlayback(api.create(triangle()), () => current); player.play(); player.advance(.4);
  current = false; player.advance(.3); near(player.time, .4); assert.equal(player.cancelled, true);
  current = true; assert.equal(player.play(), false); player.advance(1); near(player.time, .4);
  const cancelled = api.createPlayback(api.create(triangle())); cancelled.play(); cancelled.cancel(); assert.equal(cancelled.play(), false);
});
check('nonzero boundary speeds are preserved rather than zeroed', () => {
  const curve = line(0, 2), profile = triangle(); profile.duration_s = 1;
  profile.samples = [sample(curve, 0, 0, 0, 2), sample(curve, 1, 2, 1, 2)];
  const model = api.create(profile); near(api.sampleAt(model, 0).speed_mps, 2); near(api.sampleAt(model, 1).speed_mps, 2);
});
check('typed failure or executable motion cannot become a preview', () => {
  const failure = triangle(); failure.status = 'NO_PATH'; assert.throws(() => api.create(failure), /validated/);
  const executable = triangle(); executable.executable = true; assert.throws(() => api.create(executable), /non-executable/);
});
check('invalid geometry, timing or sample references fail closed', () => {
  for (const mutate of [p => { p.curves[0].p1.x_m = Infinity; }, p => { p.samples[1].segment = 5; }, p => { p.samples[1].u = 2; },
    p => { p.samples[1].time_s = 0; }, p => { p.samples[1].x_m = 99; }, p => { p.samples[1].distance_m = 1.2; },
    p => { p.samples[1].heading_rad = .2; }]) { const profile = triangle(); mutate(profile); assert.throws(() => api.create(profile)); }
});
check('oversized arrays and nonfinite preview time are rejected', () => {
  const profile = triangle(); profile.samples = Array(20001).fill(profile.samples[0]); assert.throws(() => api.create(profile), /limits/);
  assert.throws(() => api.sampleAt(api.create(triangle()), Infinity), /finite/);
  assert.throws(() => api.createPlayback(api.create(triangle())).advance(-1), /nonnegative/);
});

check('actual dashboard currentness expires between HTTP polls with exact long clocks', () => {
  const app = readFileSync(new URL('../../src/dashboard/resources/dashboard/app.js', import.meta.url), 'utf8');
  const robotClock = app.match(/  function robotNowUs\(\) \{[^\n]+\}/)?.[0];
  const guard = app.match(/  function geometryCurrent\(\) \{[^\n]+\}/)?.[0];
  assert.ok(robotClock && guard, 'Cannot locate actual clock/currentness functions');
  const base = 9007199254740993n;
  const clockContext = vm.createContext({age: 10, arrival: 0, draftEditing: false, busy: false, pendingMutations: 0,
    state: {robot_us: String(base), connection: {connected: true, stale: false}},
    plan: {status: 'SUCCESS', valid_until_us: String(base + 15000n)},
    big: value => BigInt(value ?? 0), freshHttp: () => true, matchingContext: () => true});
  clockContext.performance = {now: () => clockContext.age};
  vm.runInContext(robotClock + guard, clockContext);
  assert.equal(clockContext.geometryCurrent(), true);
  assert.equal(clockContext.robotNowUs(), base + 10000n);
  clockContext.age = 15; // Same owner frame, no replacement/poll occurred.
  assert.equal(clockContext.geometryCurrent(), false);
  clockContext.age = 20;
  assert.equal(clockContext.geometryCurrent(), false);
});

console.log(`PASS ${passed} motion preview interpolation and retirement checks`);
