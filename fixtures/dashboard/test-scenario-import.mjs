// SPDX-License-Identifier: Apache-2.0
// Opt-in, dependency-free regression for the actual browser scenario importer.
import {readFileSync} from 'node:fs';
import vm from 'node:vm';
import assert from 'node:assert/strict';

const source = readFileSync(new URL('../../src/dashboard/resources/dashboard/app.js', import.meta.url), 'utf8');
function sourceSlice(startMarker, endMarker) {
  const start = source.indexOf(startMarker);
  const end = source.indexOf(endMarker, start + startMarker.length);
  assert.ok(start >= 0 && end > start, `Cannot locate actual importer source: ${startMarker}`);
  return source.slice(start, end);
}
const declarations = sourceSlice('  const DEFAULT =', '  let mode =');
const importer = sourceSlice('  async function readJson(', '\n\n  function pauseReplay()');

function setup() {
  const input = {};
  const context = vm.createContext({
    mode: 'sandbox', modeRevision: 0, editRevision: 0, scenarioImportRevision: 0,
    scenario: {sentinel: 'original'}, image: 'original-image', imageConfig: 'original-config', zoom: 2,
    localError: '', commits: 0, renders: 0,
    finite: value => typeof value === 'number' && Number.isFinite(value),
    clone: value => JSON.parse(JSON.stringify(value)),
    $: () => input
  });
  // Only the UI/server boundaries are stubbed. The handler, file decoder,
  // default scenario and validation functions execute directly from app.js.
  context.commitScenario = () => { context.commits++; context.editRevision++; };
  context.renderUI = () => { context.renders++; };
  vm.runInContext(declarations + importer, context, {filename: 'app.js scenario importer'});
  const sample = vm.runInContext('JSON.parse(JSON.stringify(DEFAULT))', context);
  const file = value => ({size: 1000, text: async () => JSON.stringify(value)});
  const delayed = () => {
    let resolve;
    const promise = new Promise(done => { resolve = done; });
    return {file: {size: 1000, text: () => promise}, resolve};
  };
  const event = selected => ({target: {files: [selected], value: 'selected.json'}});
  return {context, sample, file, delayed, event, handler: input.onchange};
}

let passed = 0;
async function check(name, body) {
  try { await body(); passed++; }
  catch (error) { throw new Error(`${name}: ${error.message}`, {cause: error}); }
}

await check('current typed scenario imports normally', async () => {
  const test = setup(); test.sample.goal.x_m = 6;
  const selected = test.event(test.file(test.sample));
  await test.handler(selected);
  assert.equal(test.context.commits, 1);
  assert.equal(test.context.scenario.goal.x_m, 6);
  assert.equal(test.context.scenario.field.map_id, 'dashboard-lab-8x4');
  assert.equal(selected.target.value, '');
});

for (const mode of ['live', 'replay']) {
  await check(`pending import cannot overwrite ${mode}`, async () => {
    const test = setup(), pending = test.delayed();
    const result = test.handler(test.event(pending.file));
    test.context.mode = mode; test.context.modeRevision++;
    test.context.scenario = {sentinel: mode};
    pending.resolve(JSON.stringify(test.sample)); await result;
    assert.equal(test.context.commits, 0);
    assert.equal(test.context.scenario.sentinel, mode);
    assert.equal(test.context.image, 'original-image');
    assert.equal(test.context.imageConfig, 'original-config');
    assert.equal(test.context.zoom, 2);
  });
}

await check('leaving and returning to sandbox retires old import', async () => {
  const test = setup(), pending = test.delayed();
  const result = test.handler(test.event(pending.file));
  test.context.modeRevision += 2;
  pending.resolve(JSON.stringify(test.sample)); await result;
  assert.equal(test.context.commits, 0);
  assert.equal(test.context.scenario.sentinel, 'original');
});

await check('newer edit retires old import', async () => {
  const test = setup(), pending = test.delayed();
  const result = test.handler(test.event(pending.file));
  test.context.editRevision++; test.context.scenario = {sentinel: 'newer-edit'};
  pending.resolve(JSON.stringify(test.sample)); await result;
  assert.equal(test.context.commits, 0);
  assert.equal(test.context.scenario.sentinel, 'newer-edit');
});

await check('newer import wins even when the older file finishes last', async () => {
  const test = setup(), pending = test.delayed();
  const oldSelection = test.event(pending.file);
  const result = test.handler(oldSelection);
  const newer = JSON.parse(JSON.stringify(test.sample)); newer.goal.x_m = 5;
  await test.handler(test.event(test.file(newer)));
  pending.resolve(JSON.stringify(test.sample)); await result;
  assert.equal(test.context.commits, 1);
  assert.equal(test.context.scenario.goal.x_m, 5);
  assert.equal(oldSelection.target.value, 'selected.json');
});

await check('obsolete invalid JSON cannot leak an error into another mode', async () => {
  const test = setup(), pending = test.delayed();
  const result = test.handler(test.event(pending.file));
  test.context.mode = 'live'; test.context.modeRevision++;
  pending.resolve('{invalid-json'); await result;
  assert.equal(test.context.localError, '');
  assert.equal(test.context.renders, 0);
});

await check('current invalid JSON remains visibly rejected', async () => {
  const test = setup();
  await test.handler(test.event({size: 20, text: async () => '{invalid-json'}));
  assert.equal(test.context.commits, 0);
  assert.match(test.context.localError, /^Import rejected:/);
  assert.equal(test.context.renders, 1);
});

console.log(`PASS ${passed} scenario import checks using the actual handler and validators`);
