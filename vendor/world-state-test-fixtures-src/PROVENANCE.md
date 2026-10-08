# Test-only World-State integration exports

World-State supplied these implementations for a direct cross-repository CPU adapter test on 2026-10-08. Files are exact, unmodified copies from its own source at `FRC-World-State/src/main/java/org/frcworldstate/core`, now committed as **`1f8b9aace3fd320e3545744e3125aa348089bc99`**. The hashes below pin the exact test inputs and match that owner commit. They are not a replacement schema. PlannerBackend `frc-planner/1` remains held at immutable API source pin `09553f637d9b687ebfc2fcdca7671c1af317b605`, unchanged in the newer owner implementation.

| Exact owner source | SHA256 |
| --- | --- |
| `ObstacleEnvelopeBuilder.java` | `8a07be95384fa2f519f01d9b4b678fc4cb3910d839b0b084d53a2443bf8188c4` |
| `PlannerValidation.java` | `5ab942113a6b0eac6674de11bb56b224f553f5516e46757706c642bc302b189c` |

Gradle and the JDK25 verification script check these hashes. Files compile only in test source sets / test fixture jars and are **excluded from all production jars**. The test builds envelopes using configured physical size, age, uncertainty, bounded motion, timing and latency, passes them into the A* adapter, then calls the owner validator on the resulting geometry. Empty observations with stale coverage, stale measurements, missing geometry and changed epoch/map are rejected. Future owner updates require coordination and new hashes; this checkout never edits the owner's files.

## Attribution and distribution license

These are new owner-authored `Antigro09/FRC-World-State` exports supplied for this component, not third-party library binaries. They are distributed here under this repository's existing [Apache-2.0 license](LICENSE). The full license is retained beside the copies. No separate license or NOTICE accompanied the recorded source export. Original source bytes and the hashes above are unchanged. No published Maven coordinates are assumed.
