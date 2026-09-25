## Structural Requirement Analysis

### Conductor: Eli Sells | 9/12/2026

Some requirements cannot be verified through automated methods. These have been manually verified below:

| Requirement tag | Requirement text | Remarks | Test status |
| --- | --- | --- | --- |
| `RQ.MOD.00001` | The library shall be designed to interface with the Sun Devil Rocketry "driver" library. | Many of the modules in this library are dependent on interfaces provided by driver. | <span style="color: #15803d"><strong>PASS</strong></span> |
| `RQ.MOD.00002` | The library shall be organized into self-contained modules with header files organized next to source files. | All modules in the library satisfy this requirement. | <span style="color: #15803d"><strong>PASS</strong></span> |
| `RQ.MOD.00003` | The library shall be designed to be compiled with GNU GCC for the desired target architecture. | The library is compiled into two projects that use different variants of GCC. The test environment requires GCC. | <span style="color: #15803d"><strong>PASS</strong></span> |