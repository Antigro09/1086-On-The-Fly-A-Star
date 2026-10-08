# Test-only World-State integration exports

Exact, unmodified implementations from [World-State public revision `0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab`](https://github.com/Antigro09/FRC-World-State/tree/0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab/src/main/java/org/frcworldstate/core) supply the direct CPU integration test. The hashes below match the public commit. `PlannerBackend.java` remains byte-identical to the held `frc-planner/1` export; [API provenance](../world-state-api-src/PROVENANCE.md) distinguishes earlier support copies from newer owner validation.

| Exact owner source | SHA256 |
| --- | --- |
| `ObstacleEnvelopeBuilder.java` | `8a07be95384fa2f519f01d9b4b678fc4cb3910d839b0b084d53a2443bf8188c4` |
| `PlannerValidation.java` | `5ab942113a6b0eac6674de11bb56b224f553f5516e46757706c642bc302b189c` |

Gradle and the JDK25 verification script check these hashes. Files compile only in test source sets / test fixture jars and are **excluded from all production jars**. The test builds envelopes using configured physical size, age, uncertainty, bounded motion, timing and latency, passes them into the A* adapter, then calls the owner validator on the resulting geometry. Empty observations with stale coverage, stale measurements, missing geometry and changed epoch/map are rejected. Future owner updates require coordination and new hashes; this checkout never edits the owner's files.

## Attribution and distribution license

These are new owner-authored `Antigro09/FRC-World-State` exports supplied for this component, not third-party library binaries. They are distributed here under this repository's existing [Apache-2.0 license](LICENSE). The full license is retained beside the copies. No separate license or NOTICE accompanied the recorded source export. Original source bytes and the hashes above are unchanged. No published Maven coordinates are assumed.
