# NTS-1 野外配信向けステレオコンプレッサ

こちらは、野外配信に最適な、ステレオコンプリミッターです。DJI Mic MiniとAMS-22を組み合わせることで、野外で高音質な配信が可能です。
NTS-1(初代)向け、MODFXモジュールとしてインストールされます。

# ビルド

ビルドには、Docker, git, make などが必要です。

```
$ make
```

## stcomp

ステレオコンプリミッターです。

操作

- TIME(A): コンプレッション度、 時計回りに -6dbFS から -36dbFS
- DEPTH(B): コンプレッサー後段のメークアップゲイン、0 から +40 dB.

固定値

- Ratio: fixed 4:1.
- Attack: fixed 5 ms.
- Release: fixed 150 ms.
- Stereo detector: max(abs(L), abs(R)); the same gain is applied to both channels.
- Output limiter: zero-lookahead, -1 dBFS ceiling, 50 ms release.
