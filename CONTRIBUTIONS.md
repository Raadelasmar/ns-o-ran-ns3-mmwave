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

None. No files were created from scratch in this fork; all changes are modifications to existing
upstream ns-3 source files.

## Table 2 — Files modified

| File | +added / −removed | What was changed |
|---|---:|---|
| `src/lte/model/lte-enb-rrc.cc` | +82 / −10 | Implements the Cell Individual Offset (CIO) handover-ranking feature. Adds `SetCellIndividualOffset()` (clamps the offset to [−6, +6] dB and stores it as a linear multiplier); default-initializes CIO to 1.0 (0 dB) in `DoRecvUeSinrUpdate`; and in the three cell-selection loops (`TriggerUeAssociationUpdate`, `UpdateUeHandoverAssociation`, `EvictUsersFromSecondaryCell`) ranks candidate cells by CIO-biased SINR while re-reading the raw, unbiased SINR of the chosen cell for the outage check, so a positive CIO cannot mask a real outage. |
| `src/lte/model/lte-enb-net-device.cc` | +47 / −0 | Adds a new branch to `ReadControlFile()` that parses `cio_actions_for_ns3.csv` (rows of `timestamp,cellId,cioDb`) and applies each row via `LteEnbRrc::SetCellIndividualOffset`, either immediately or pre-scheduled with `Simulator::Schedule`. |
| `src/lte/model/lte-enb-rrc.h` | +17 / −1 | Declares the new `SetCellIndividualOffset()` method and the `m_cellIndividualOffset` map member (cellId → linear multiplier). The single removed line is a whitespace change to an existing doc comment. |
| `.gitignore` | +2 / −0 | Added `build_log*.txt` and `*.bak` ignore patterns. |
| `src/wifi/model/sta-wifi-mac.h` | +1 / −0 | Added `#include <cstdint>` (makes the fixed-width integer types this header uses explicitly available). |

## Summary

- **Total lines added:** 149 (11 lines removed).
- **Files created:** 0
- **Files modified:** 5
- **Files untouched:** 3,939 (of 3,944 tracked files total)

## Line-level authorship

`git blame <file>` shows line-level authorship for anything in the repository, including which
lines in the modified files above came from upstream versus this fork.
