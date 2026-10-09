# Simulator baseline (sim-baseline-v1)

This commit is the exact ns-3 source that the v8 MARL/MLB training run uses.
The tag `sim-baseline-v1` and the branch `sim-baseline` both point at it.

## Build

Same configuration as the v8 build (`cmake-cache/`, Ninja, release, ccache):

```bash
./ns3 configure -d optimized -- -DNS3_EMU=OFF -DNS3_TAP=OFF && ./ns3 build
```

which is equivalent to:

```bash
cmake -S . -B cmake-cache -G Ninja \
  -DCMAKE_BUILD_TYPE=release -DNS3_NATIVE_OPTIMIZATIONS=ON \
  -DNS3_EMU=OFF -DNS3_TAP=OFF -DNS3_EXAMPLES=ON -DNS3_TESTS=OFF
cmake --build cmake-cache -j"$(nproc)"
```

The scenario binary is `build/scratch/ns3.38.rc1-scenario-marl-zmq-optimized`.
The v8 build used g++ 15.2.0 and CMake 4.2.3. `NS3_NATIVE_OPTIMIZATIONS=ON`
adds `-march=native -mtune=native`, so a build on a different CPU can give
slightly different floating-point results for the same seed.

## The 6 switches v8 turns on

| Flag | v8 value | Default | Effect |
|---|---|---|---|
| `channelConditionUpdatePeriodMs` | `0` | `100` | Each UE-cell link draws LOS/NLOS once and keeps it, instead of redrawing every 100 ms. |
| `handoverSinrFilterTauMs` | `400` | `0` (off) | L3-style SINR filter (time constant in ms) on the SINR the handover decision uses. |
| `handoverHysteresisDb` | `3` | `0` (off) | Handover margin (dB) on the CIO-biased SINR for TTT-based handover. |
| `tttFromBiasedSinr` | `1` | `0` (off) | DynamicTtt uses the CIO-biased SINR difference instead of the raw one. |
| `controlPhaseOffsetS` | `0.001` | `0` (off) | Runs the MARL control step 1 ms after the k*controlInterval grid, so it reads the E2 report built 0.8 ms into the step. Needs `e2du=true` (the default). |
| `hotspotClipToUeDisc` | `1` | `0` (off) | Redraws hotspot UE positions until they lie within isd of the network centre. |

**Every switch defaults to OFF**, meaning the previous behaviour. For
`channelConditionUpdatePeriodMs`, OFF is the historical value `100`. With no
new flags the simulator gives the same results as before these changes. Most
switches only set their attribute when given a non-default value.

## Hotspot and traffic settings used by v8

```
--trafficModel=3 --ues=5 --simTime=30 --controlInterval=0.1 --indicationPeriodicity=0.1
--udpFullBufferIntervalUs=2000
--burstyDlRate1=8Mbps --burstyDlRate2=5Mbps --burstyDlRate3=2.5Mbps
--burstyOnMeanS=10 --burstyOffMeanS=10
--hotspotFraction=0.4 --hotspotRadius=500 --hotspotFbShare=0.78 --hotspotClipToUeDisc=1
--hotspotCellId=<per episode, 2..8>
```

Other v8 arguments: `--configuration=0 --handoverMode=DynamicTtt --outageThreshold=-5.0
--numberOfRaPreambles=40 --e2nrEnabled=1 --rlcAmEnabled=1 --reducedPmValues=0
--heuristicType=-1 --bsOn=5 --bsIdle=0 --bsSleep=0 --bsOff=2`, plus per-worker
`--RngRun` and `--zmqPort`. The trace outputs are sent to `/dev/null`.

New knobs and their defaults: `burstyOnMeanS` and `burstyOffMeanS` default to
1.0 (previous behaviour). `hotspotFraction` defaults to 0 (off).
`hotspotCellId` defaults to 2. `hotspotRadius` defaults to 500.
`hotspotFbShare` defaults to -1 (proportional). `logChannelConditions` defaults
to false. It is a diagnostic flag and changes the RNG.

## Interaction with origin/feature/mro-agent (MRO)

`origin/feature/mro-agent` (40b46ba3, "Wire MRO handover-margin action into
LteEnbRrc / ApplyControlPayload") also changes `src/lte/model/lte-enb-rrc.cc`,
`lte-enb-rrc.h` and `lte-enb-net-device.{cc,h}`. It merges onto this commit
with no textual conflicts. **The merged tree has not been compiled or run
yet.**

Handover hysteresis is normally MRO's knob. v8 fixes `handoverHysteresisDb=3`
as part of the environment. If the MRO agent controls the handover margin,
decide which one sets it: leave `handoverHysteresisDb` at 0, or make sure the
MRO action and this attribute do not both apply to the same decision.
