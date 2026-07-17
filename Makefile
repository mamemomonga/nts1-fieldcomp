build: logue-sdk/platform/nutekt-digital/stcomp

logue-sdk/platform/nutekt-digital/stcomp:
	MODFX=stcomp $(MAKE) build-modfx

build-modfx: logue-sdk
	cp -a logue-sdk/platform/nutekt-digital/dummy-modfx logue-sdk/platform/nutekt-digital/$(MODFX)
	cp $(MODFX)/$(MODFX).cpp logue-sdk/platform/nutekt-digital/$(MODFX)/
	cp $(MODFX)/project.mk logue-sdk/platform/nutekt-digital/$(MODFX)/
	cp $(MODFX)/manifest.json logue-sdk/platform/nutekt-digital/$(MODFX)/
	cd logue-sdk && docker/run_cmd.sh --image=logue-sdk-dev-env:latest build nutekt-digital/$(MODFX)
	mkdir -p dist
	cp logue-sdk/platform/nutekt-digital/$(MODFX)/$(MODFX).ntkdigunit dist/$(MODFX).ntkdigunit

logue-sdk:
	git clone https://github.com/korginc/logue-sdk.git
	cd logue-sdk && git submodule update --init --recursive
	cd logue-sdk/docker && ./build_image.sh

clean:
	rm -rf logue-sdk/platform/nutekt-digital/stcomp
	rm -rf dist
