# NTS-1 Stereo Compressor for Field Streaming

[日本語版はこちら / Japanese version](README_ja.md)

A set of KORG NTS-1 (first generation) MOD plugins (MODFX modules) for
higher-quality live streaming from field recorders such as the DJI Mic Mini
or a preamped stereo ECM.

## Modules

### stcomp — Stereo Compressor/Limiter

A stereo compressor/limiter optimized for field streaming. Combined with a
DJI Mic Mini and an AMS-22, it enables high-quality streaming outdoors.

**Controls**

- **TIME (A):** Compression amount. Clockwise, threshold ranges from
  -6 dBFS to -36 dBFS.
- **DEPTH (B):** Post-compressor makeup gain, 0 to +40 dB.

**Fixed values**

- Ratio: fixed 4:1
- Attack: fixed 5 ms
- Release: fixed 150 ms
- Stereo detector: `max(abs(L), abs(R))`; the same gain is applied to both channels
- Output limiter: zero-lookahead, -1 dBFS ceiling, 50 ms release

### agccomp — Auto Gain + Compressor/Limiter

A hands-off variant that automatically rides the input level toward a fixed
target loudness of about **-14 LUFS**, then compresses and limits. The makeup
gain of `stcomp` is removed: the AGC stage sets the optimal level instead.

Signal chain: **AGC → Compressor → Limiter**

**Controls**

- **TIME (A):** AGC reaction time. Clockwise, 1 s to 30 s (logarithmic).
- **DEPTH (B):** Unused.

**Fixed values**

- AGC target: approx. -14 LUFS (BS.1770 approximation, no K-weighting)
- AGC gain range: -24 dB to +40 dB, frozen below -45 dBFS RMS (silence gate)
- AGC loudness window: 400 ms (momentary)
- Compressor: fixed -18 dBFS threshold, 4:1 ratio, 5 ms attack, 150 ms release
- Output limiter: zero-lookahead, -1 dBFS ceiling, 50 ms release
- Stereo detector: `max(abs(L), abs(R))`; the same gain is applied to both channels

### fieldcomp — Level + Auto-Makeup Compressor/Limiter

Designed for low-level sources such as the DJI Mic Mini. A manual **LEVEL**
knob roughly raises the input, then the compressor's **auto makeup gain**
precisely lands the output at about **-14 LUFS**. Together they provide up to
+50 dB of gain (LEVEL +30 dB + makeup +20 dB). Unlike `agccomp`, you actively
set how hard the compressor is driven with the LEVEL knob.

Signal chain: **LEVEL → Compressor → Auto Makeup → Limiter**

**Controls**

- **TIME (A):** Auto-makeup reaction time. Clockwise, 1 s to 10 s (logarithmic).
- **LEVEL (B):** Manual input gain, 0 dB to +30 dB.

**Fixed values**

- Auto-makeup target: approx. -14 LUFS, gain range 0 to +20 dB (never attenuates)
- Auto-makeup measures the compressor output; frozen below -45 dBFS RMS (silence gate)
- Compressor: fixed -18 dBFS threshold, 4:1 ratio, 5 ms attack, 150 ms release
- Output limiter: 2 ms lookahead with peak-hold, -1 dBFS ceiling, 50 ms release
  (the lookahead ducks transients smoothly to avoid clip-like distortion on
  speech; it adds about 2 ms of latency)
- Stereo detector: `max(abs(L), abs(R))`; the same gain is applied to both channels

## Build

Docker, git and make are required. The `logue-sdk` and its Docker image are
fetched and built automatically on the first build.

```sh
# Build all MODFX modules into dist/
$ make

# Build a specific module
$ make stcomp

# Remove build artifacts (keeps logue-sdk)
$ make clean

# Remove everything including logue-sdk
$ make distclean
```

Built `*.ntkdigunit` files are placed in `dist/`.

## Adding a new MODFX

1. Create a directory named after the module (e.g. `mymodfx/`).
2. Place `mymodfx.cpp`, `project.mk` and `manifest.json` inside it.
3. Add the module name to `MODFX_MODULES` in the `Makefile`.
