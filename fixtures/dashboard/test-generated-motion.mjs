// SPDX-License-Identifier: Apache-2.0
// Cross-producer/evaluator smoke test. Supply a state captured from the local Java Sandbox.
// Runtime packets are generated evidence, not checked-in fixtures or hardware qualification.
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';

if (process.argv.length !== 3) throw new Error('Usage: node fixtures/dashboard/test-generated-motion.mjs PATH_TO_SANDBOX_STATE_JSON');
const state = JSON.parse(readFileSync(process.argv[2], 'utf8'));
assert.equal(state.mode, 'sandbox');
assert.equal(state.source_kind, 'synthetic_mock');
assert.equal(state.plan.status, 'SUCCESS');
assert.equal(state.plan.motion.executable, false);
assert.equal(typeof state.plan.motion.planned_start_us, 'string');
assert.ok(BigInt(state.plan.motion.planned_start_us) < BigInt(state.plan.valid_until_us));
const context = vm.createContext({});
vm.runInContext(readFileSync(new URL('../../src/dashboard/resources/dashboard/motion-preview.js', import.meta.url), 'utf8'), context);
const api = context.MotionPreview, motion = api.create(state.plan.motion);
const constraints = state.scenario.constraints;
const near = (a, b) => assert.ok(Math.abs(a - b) <= 1e-5, `${a} differs from ${b}`);
let peak = 0, totalAcceleration = 0;
for (let i = 0; i <= 240; i++) {
  const sample = api.sampleAt(motion, motion.duration * i / 240);
  assert.ok(sample.speed_mps >= -1e-8 && sample.speed_mps <= constraints.max_speed_mps + 1e-6);
  assert.ok(sample.total_acceleration_mps2 <= constraints.max_acceleration_mps2 + 1e-6);
  assert.ok(Math.abs(sample.omega_radps) <= constraints.max_angular_speed_radps + 1e-6);
  near(Math.hypot(sample.vx_mps, sample.vy_mps), sample.speed_mps);
  if (motion.curves.length) {
    const point = api.evaluateBezier(motion.curves[sample.segment], sample.u);
    near(point.x_m, sample.x_m); near(point.y_m, sample.y_m);
  }
  peak = Math.max(peak, sample.speed_mps);
  totalAcceleration = Math.max(totalAcceleration, sample.total_acceleration_mps2);
}
const first = api.sampleAt(motion, 0), last = api.sampleAt(motion, motion.duration);
near(first.x_m, state.scenario.start.x_m); near(first.y_m, state.scenario.start.y_m);
near(first.speed_mps, Math.hypot(state.scenario.start.vx_mps, state.scenario.start.vy_mps));
near(first.omega_radps, state.scenario.start.omega_radps);
near(last.x_m, state.scenario.goal.x_m); near(last.y_m, state.scenario.goal.y_m);
near(last.speed_mps, state.scenario.motion_limits.goal_end_speed_mps);
near(last.omega_radps, 0);
near(Math.atan2(Math.sin(last.heading_rad - state.scenario.goal.heading_rad), Math.cos(last.heading_rad - state.scenario.goal.heading_rad)), 0);
assert.ok(state.plan.motion.validated_prefix_s >= motion.duration);
console.log(JSON.stringify({result: 'PASS', samples: 241, curves: motion.curves.length,
  duration_s: motion.duration, peak_speed_mps: peak, total_acceleration_mps2: totalAcceleration,
  scope: 'actual Java Sandbox output + actual JS evaluator; no hardware qualification'}));
