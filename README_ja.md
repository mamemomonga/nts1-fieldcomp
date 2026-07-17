# NTS-1 野外配信向けステレオコンプレッサ

[English version / 英語版はこちら](README.md)

DJI Mic MiniやプリアンプをつけたステレオECMなどを、より高音質にライブ配信するための
KORG NTS-1(初代)用MODプラグイン(MODFXモジュール)のセットです。

## モジュール一覧

### stcomp — ステレオコンプリミッター

野外配信に最適なステレオコンプリミッターです。DJI Mic MiniとAMS-22を組み合わせることで、
野外で高音質な配信が可能です。

**操作**

- **TIME (A):** コンプレッション度。時計回りにスレッショルド -6 dBFS から -36 dBFS。
- **DEPTH (B):** コンプレッサー後段のメークアップゲイン。0 から +40 dB。

**固定値**

- Ratio: 固定 4:1
- Attack: 固定 5 ms
- Release: 固定 150 ms
- ステレオ検出: `max(abs(L), abs(R))`。左右チャンネルへ同じゲインを適用
- 出力リミッター: ルックアヘッド無し、-1 dBFS シーリング、リリース 50 ms

## ビルド

ビルドには Docker, git, make などが必要です。`logue-sdk` とそのDockerイメージは
初回ビルド時に自動的に取得・ビルドされます。

```sh
# 全MODFXモジュールを dist/ へビルド
$ make

# 特定のモジュールのみビルド
$ make stcomp

# ビルド成果物を削除 (logue-sdk は残す)
$ make clean

# logue-sdk も含めて完全に削除
$ make distclean
```

ビルドされた `*.ntkdigunit` は `dist/` に出力されます。

## MODFXの追加方法

1. モジュール名と同名のディレクトリ (例: `mymodfx/`) を作成します。
2. その中に `mymodfx.cpp` / `project.mk` / `manifest.json` を配置します。
3. `Makefile` の `MODFX_MODULES` にモジュール名を追記します。
