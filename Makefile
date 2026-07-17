# #############################################################################
# NTS-1 野外配信向けMODFX ビルド用Makefile
#
# 複数のMODFXモジュールをまとめてビルド・クリーンアップできる。
# 新しいMODFXを追加する場合は、モジュール名と同名のディレクトリを作成し、
# <name>.cpp / project.mk / manifest.json を配置して MODFX_MODULES に追記する。
# #############################################################################

# ビルド対象のMODFXモジュール一覧
MODFX_MODULES := stcomp agccomp fieldcomp

# パス・イメージ設定
SDK_DIR      := logue-sdk
PLATFORM_DIR := $(SDK_DIR)/platform/nutekt-digital
DIST_DIR     := dist
DOCKER_IMAGE := logue-sdk-dev-env:latest

.PHONY: all build clean distclean $(MODFX_MODULES)

# デフォルトターゲット: 全MODFXモジュールをビルド
all: build

# 全MODFXモジュールをビルド
build: $(MODFX_MODULES)

# 各MODFXモジュールを個別にビルド可能にする (例: make stcomp)
# logue-sdk 内にビルド用ディレクトリを用意し、docker経由でビルドして dist/ へ収集する
$(MODFX_MODULES): %: logue-sdk
	@echo "==> MODFX '$*' をビルドします"
	rm -rf $(PLATFORM_DIR)/$*
	cp -a $(PLATFORM_DIR)/dummy-modfx $(PLATFORM_DIR)/$*
	cp $*/$*.cpp $*/project.mk $*/manifest.json $(PLATFORM_DIR)/$*/
	cd $(SDK_DIR) && docker/run_cmd.sh --image=$(DOCKER_IMAGE) build nutekt-digital/$*
	mkdir -p $(DIST_DIR)
	cp $(PLATFORM_DIR)/$*/$*.ntkdigunit $(DIST_DIR)/$*.ntkdigunit

# logue-sdk をGitHubから取得し、dockerイメージを独自にビルドする
logue-sdk:
	git clone https://github.com/korginc/logue-sdk.git
	cd $(SDK_DIR) && git submodule update --init --recursive
	cd $(SDK_DIR)/docker && ./build_image.sh

# ビルド成果物と各MODFXの一時ビルドディレクトリを削除
clean:
	rm -rf $(DIST_DIR)
	$(foreach m,$(MODFX_MODULES),rm -rf $(PLATFORM_DIR)/$(m);)

# logue-sdk を含めて完全に削除する
distclean: clean
	rm -rf $(SDK_DIR)
