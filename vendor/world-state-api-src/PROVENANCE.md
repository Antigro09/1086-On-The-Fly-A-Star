# World-State API source pin

Exact, unmodified owner exports are preserved in the [public A* source snapshot](https://github.com/Antigro09/1086-On-The-Fly-A-Star/tree/59ad897d895315a751df67c5751e30370850a784/vendor/world-state-api-src), revision `59ad897d895315a751df67c5751e30370850a784`. Contract `frc-planner/1`; Java17; no dependencies. [World-State](https://github.com/Antigro09/FRC-World-State/tree/0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab) owns the schema; its public `PlannerBackend.java` is byte-identical to this held export. These copies support reproducible standalone builds and must never become a competing contract.

| File | SHA256 |
| --- | --- |
| `org/frcworldstate/core/PlannerBackend.java` | `5f6c6e709516c522c2bfc765e8d7a241c274cb4ac1d2e1d03e5d0599980665aa` |
| `org/frcworldstate/core/Geometry.java` | `e94a4b1ce2d245dbdddda03fb25c6d7be77aeeedf62b831804d5311984f1bdc2` |
| `org/frcworldstate/core/World.java` | `e1dbc25da7c754959067b628016663cf0799c4d32e29974a78db4e94129d494a` |

The default build reads only these vendored files and requires no sibling checkout. The adapter jar omits these classes. Integrations load the owner API separately. Gradle verifies these source hashes before compiling. When using an external API jar override, pass its recorded SHA256 using `-PworldStateApiSha256` and record its source pin explicitly; an API upgrade requires owner coordination and re-running contract tests.

## Attribution and distribution license

These are new owner-authored `Antigro09/FRC-World-State` exports supplied for this component, not third-party library binaries. They are distributed here under this repository's existing [Apache-2.0 license](LICENSE). The full license is retained beside the copies. No separate license or NOTICE accompanied the recorded source export. Original source bytes and the hashes above are unchanged. No published Maven coordinates are assumed.

## Public owner compatibility pin

Public owner revision: [`0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab`](https://github.com/Antigro09/FRC-World-State/tree/0e5b6c3f85d47205cde8b0c80e13fa79d35e95ab). `PlannerBackend.java` has the exact SHA256 above. The public owner's `Geometry.java` (`4dc7c42e1c196681fcd8c176dc9cc9aa132def99da805d7838b26344375c8b28`) improves covariance PSD validation; `World.java` (`7157b4ded74429d8e372b35f3e0aa13f49187e1b75019f000555ec2117378bf3`) permits signed finite target heights and caps snapshots at256 tracks. This checkout deliberately retains the earlier support copies identified by the table. It does not claim those two files match the newer public owner. Production jars omit all owner classes and use the separately supplied owner API at runtime. World-State reported20 direct checks from clean public clones of this owner revision and A* `59ad897d895315a751df67c5751e30370850a784`.
