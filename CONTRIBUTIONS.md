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
| `scratch/scenario-marl-zmq.cc` | 1195 | Dedicated simulation scenario branched from `scenario-three.cc`. Contains the MARL control loop wiring, dynamic ZMQ client lifecycle management, and isolates AI experiments from the baseline scenarios. The control loop iterates every real mmWave cell (not a single hardcoded one) to push live per-cell KPIs (DL PRB usage, RLC buffer occupancy, MAC DL volume, active UE count, per-UE L3 serving SINR) to the Python Gym environment, and applies the per-cell CIO actions received back. |

## Table 2 — Files modified

| File | +added / −removed | What was changed |
|---|---:|---|
| `src/lte/model/lte-enb-rrc.cc` | +82 / −10 | Implements the Cell Individual Offset (CIO) handover-ranking feature. Adds `SetCellIndividualOffset()` (clamps the offset to [−6, +6] dB and stores it as a linear multiplier); default-initializes CIO to 1.0 (0 dB) in `DoRecvUeSinrUpdate`; and in the three cell-selection loops (`TriggerUeAssociationUpdate`, `UpdateUeHandoverAssociation`, `EvictUsersFromSecondaryCell`) ranks candidate cells by CIO-biased SINR while re-reading the raw, unbiased SINR of the chosen cell for the outage check, so a positive CIO cannot mask a real outage. |
| `src/lte/model/lte-enb-net-device.cc` | +77 / −0 | Adds a new branch to `ReadControlFile()` that parses `cio_actions_for_ns3.csv` and applies each row via `LteEnbRrc::SetCellIndividualOffset`. Also implements `ApplyControlPayload` to parse JSON multi-agent actions from the ZeroMQ bridge: it reads the same per-cell `"cells": {"<cellId>": {...}}` shape used for the KPI payload, applying a `cio_offset` for every real cell present in one call. |
| `src/lte/model/lte-enb-rrc.h` | +17 / −1 | Declares the new `SetCellIndividualOffset()` method and the `m_cellIndividualOffset` map member (cellId → linear multiplier). The single removed line is a whitespace change to an existing doc comment. |
| `src/mmwave/model/mmwave-enb-net-device.cc` | +15 / −0 | Adds `GetRlcBufferOccupancyCellSpecific()` and `GetDlPrbUsage()`, caching the DL PRB usage and cell-wide RLC tx-buffer occupancy at the exact point `BuildRicIndicationMessageDu` already computes them (mirroring the pre-existing `m_macVolumeCellSpecific`/`GetMacVolumeCellSpecific()` pattern), so the ZMQ KPI loop can read real per-cell values instead of hardcoded ones. |
| `src/mmwave/model/mmwave-enb-net-device.h` | +25 / −0 | Declares the two new getters and their backing members (`m_rlcBufferOccupCellSpecific`, `m_dlPrbUsage`). |
| `.gitignore` | +14 / −0 | Added `build_log*.txt` and `*.bak` ignore patterns, and some intermediate `.txt` files. |
| `contrib/.gitignore` | +3 / -0  | Added the exclusion of `contrib/oran-interface/model/zmq-database-client.h`. |
| `src/wifi/model/sta-wifi-mac.h` | +1 / −0 | Added `#include <cstdint>` (makes the fixed-width integer types this header uses explicitly available). |
| `src/lte/model/lte-enb-net-device.h` | +9 / −0 | Included `nlohmann/json.hpp` and declared `ApplyControlPayload(const nlohmann::json&)` as a public method to accept MARL actions. |
| `README.md` | +21 / −0 | Adds an "About this fork" header above the original upstream README, summarizing the CIO handover lever and linking to `CONTRIBUTIONS.md` and the companion [ns-o-ran-gym](https://github.com/Raadelasmar/ns-o-ran-gym) repository. |

## Summary

- **Total lines added:** 1,581 (11 lines removed).
- **Files created:** 2
- **Files modified:** 10
- **Files untouched:** 3,934 (of 3,947 tracked files total)

## Line-level authorship

`git blame <file>` shows line-level authorship for anything in the repository, including which
lines in the modified files above came from upstream versus this fork.
