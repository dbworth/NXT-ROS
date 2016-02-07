all: nxt-python

TARBALL     = build/nxt-python-2.2.2.zip
TARBALL_URL = http://github.com/Eelviny/nxt-python/archive/v2.2.2.zip
UNPACK_CMD  = unzip
SOURCE_DIR  = build/nxt-python-2.2.2
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = $(shell rospack find nxt_python)


nxt-python: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	rm -rf src/nxt
	cd $(SOURCE_DIR) && python setup.py install --prefix=$(INSTALL_DIR) --install-purelib=$(INSTALL_DIR)/src
	patch -p0 bin/nxt_filer  patches/nxt_filer.patch
	patch -p0 bin/nxt_push   patches/nxt_push.patch
	patch -p0 bin/nxt_server patches/nxt_server.patch
	patch -p0 bin/nxt_test   patches/nxt_test.patch
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) bin lib src/nxt installed

wipe: clean
	rm -rf build
