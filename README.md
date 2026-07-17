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
