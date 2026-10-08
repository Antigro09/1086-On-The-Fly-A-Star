# World-State API source pin

Exact, unmodified files exported from the owner repository `FRC-World-State`, commit `09553f637d9b687ebfc2fcdca7671c1af317b605` (Define controller-independent world and planner contracts v1). Contract `frc-planner/1`; Java 17; no dependencies. World-State owns this schema. These sources support reproducible local builds and must never be edited as a competing contract.

| File | SHA256 |
| --- | --- |
| `org/frcworldstate/core/PlannerBackend.java` | `5f6c6e709516c522c2bfc765e8d7a241c274cb4ac1d2e1d03e5d0599980665aa` |
| `org/frcworldstate/core/Geometry.java` | `e94a4b1ce2d245dbdddda03fb25c6d7be77aeeedf62b831804d5311984f1bdc2` |
| `org/frcworldstate/core/World.java` | `e1dbc25da7c754959067b628016663cf0799c4d32e29974a78db4e94129d494a` |

Initial checkout was `<World-State checkout>`; this build does not read or write that checkout. The adapter jar omits these classes. Integrations load the owner API separately. Gradle verifies these source hashes before compiling. When using an external API jar override, pass its recorded SHA256 using `-PworldStateApiSha256` and record its source pin explicitly; an API upgrade requires owner coordination and re-running contract tests.

## Attribution and distribution license

These are new owner-authored `Antigro09/FRC-World-State` exports supplied for this component, not third-party library binaries. They are distributed here under this repository's existing [Apache-2.0 license](LICENSE). The full license is retained beside the copies. No separate license or NOTICE accompanied the recorded source export. Original source bytes and the hashes above are unchanged. No published Maven coordinates are assumed.
