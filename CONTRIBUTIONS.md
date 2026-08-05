# Contributions

This repository is a fork of the wineslab
[`ns-o-ran-ns3-mmwave`](https://github.com/wineslab/ns-o-ran-ns3-mmwave) original. This file records
what was added on top of that upstream, for the BME / Nokia Bell Labs project **"Hybrid SON and
Multi-Agent AI for Autonomous Optimization of Future Mobile Networks"**.

All figures below are derived directly from git, comparing this branch (`master`) against the tracked
upstream (`upstream/master`). They were produced with:

```
git diff --name-only --diff-filter=A upstream/master..master   # files created
git diff --name-only --diff-filter=M upstream/master..master   # files modified
git diff --numstat upstream/master..master                     # per-file +/- line counts
git ls-files | wc -l                                           # total tracked files
```

## Table 1 — Files created from scratch

| File | Lines | What it does |
|---|---:|---|
| `contrib/oran-interface/model/zmq-database-client.h` | 74 | C++ ZeroMQ client for synchronous KPI and action handshakes with the Python Gymnasium environment. |
| `scratch/scenario-marl-zmq.cc` | 1148 | Dedicated simulation scenario branched from `scenario-three.cc`. Contains the MARL control loop wiring, dynamic ZMQ client lifecycle management, and isolates AI experiments from the baseline scenarios. |

## Table 2 — Files modified

| File | +added / −removed | What was changed |
|---|---:|---|
| `src/lte/model/lte-enb-rrc.cc` | +82 / −10 | Implements the Cell Individual Offset (CIO) handover-ranking feature. Adds `SetCellIndividualOffset()` (clamps the offset to [−6, +6] dB and stores it as a linear multiplier); default-initializes CIO to 1.0 (0 dB) in `DoRecvUeSinrUpdate`; and in the three cell-selection loops (`TriggerUeAssociationUpdate`, `UpdateUeHandoverAssociation`, `EvictUsersFromSecondaryCell`) ranks candidate cells by CIO-biased SINR while re-reading the raw, unbiased SINR of the chosen cell for the outage check, so a positive CIO cannot mask a real outage. |
| `src/lte/model/lte-enb-net-device.cc` | +68 / −0 | Adds a new branch to `ReadControlFile()` that parses `cio_actions_for_ns3.csv` and applies each row via `LteEnbRrc::SetCellIndividualOffset`. Also implements `ApplyControlPayload` to parse JSON multi-agent actions from the ZeroMQ bridge and route them to the CIO logic. |
| `src/lte/model/lte-enb-rrc.h` | +17 / −1 | Declares the new `SetCellIndividualOffset()` method and the `m_cellIndividualOffset` map member (cellId → linear multiplier). The single removed line is a whitespace change to an existing doc comment. |
| `.gitignore` | +14 / −0 | Added `build_log*.txt` and `*.bak` ignore patterns, and some intermediate `.txt` files. |
| `contrib/.gitignore` | +3 / -0  | Added the exclusion of `contrib/oran-interface/model/zmq-database-client.h`. |
| `src/wifi/model/sta-wifi-mac.h` | +1 / −0 | Added `#include <cstdint>` (makes the fixed-width integer types this header uses explicitly available). |
| `src/lte/model/lte-enb-net-device.h` | +7 / −0 | Included `nlohmann/json.hpp` and declared `ApplyControlPayload(const nlohmann::json&)` as a public method to accept MARL actions. |
| `contrib/oran-interface/CMakeLists.txt` | +6 / −0 | Added `PkgConfig` resolution for `libzmq`, linked ZMQ libraries to the module, and registered the new ZMQ client header. |

## Summary

- **Total lines added:** 1,478 (11 lines removed).
- **Files created:** 2
- **Files modified:** 8
- **Files untouched:** 3,936 (of 3,947 tracked files total)

## Line-level authorship

`git blame <file>` shows line-level authorship for anything in the repository, including which
lines in the modified files above came from upstream versus this fork.
