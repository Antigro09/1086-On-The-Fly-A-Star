// SPDX-License-Identifier: Apache-2.0
// Verify the actual optional renderer uses returned points without altering them.
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';

const app = readFileSync(new URL('../../src/dashboard/resources/dashboard/app.js', import.meta.url), 'utf8');
const start = app.indexOf('  function drawCurveControls('), end = app.indexOf('\n\n  function draw()', start);
assert.ok(start >= 0 && end > start, 'Actual curve-controls renderer was not found');
const calls = [];
const ctx = {};
for (const method of ['save', 'restore', 'beginPath', 'moveTo', 'lineTo', 'setLineDash', 'stroke', 'fill', 'fillRect', 'strokeRect', 'arc']) {
  ctx[method] = (...args) => calls.push({method, args, stroke: ctx.strokeStyle, fill: ctx.fillStyle});
}
const screen = p => ({x: p.x_m * 40, y: 400 - p.y_m * 40});
const context = vm.createContext({ctx, screen});
vm.runInContext(app.slice(start, end), context);
const curve = Object.freeze({p0: Object.freeze({x_m: 1, y_m: 2}), p1: Object.freeze({x_m: 2, y_m: 4}),
  p2: Object.freeze({x_m: 6, y_m: 4}), p3: Object.freeze({x_m: 7, y_m: 2}), start_distance_m: 0, length_m: 7});
const before = JSON.stringify(curve);
context.drawCurveControls([curve], false);
const anchors = [screen(curve.p0), screen(curve.p3)], handles = [screen(curve.p1), screen(curve.p2)];
assert.deepEqual(calls.find(call => call.method === 'moveTo').args, [anchors[0].x, anchors[0].y]);
assert.deepEqual(calls.filter(call => call.method === 'lineTo').map(call => call.args),
  [handles[0], handles[1], anchors[1]].map(p => [p.x, p.y]));
assert.deepEqual(calls.filter(call => call.method === 'fillRect').map(call => call.args), anchors.map(p => [p.x - 3, p.y - 3, 6, 6]));
assert.deepEqual(calls.filter(call => call.method === 'arc').map(call => call.args.slice(0, 3)), handles.map(p => [p.x, p.y, 3.5]));
assert.equal(JSON.stringify(curve), before);
assert.ok(calls.filter(call => call.method === 'stroke').every(call => call.stroke === '#426a9a'));
calls.length = 0;
context.drawCurveControls([curve], true);
assert.ok(calls.filter(call => call.method === 'stroke').every(call => call.stroke === '#8b9691'));
assert.ok(calls.filter(call => call.method === 'fillRect').every(call => call.fill === '#8b9691'));
calls.length = 0;
context.drawCurveControls([], false);
assert.equal(calls.filter(call => ['moveTo', 'lineTo', 'arc', 'fillRect'].includes(call.method)).length, 0);
console.log('PASS exact returned control polygon, immutable display, stale styling, and empty geometry checks');
