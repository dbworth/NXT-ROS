all: nxt-python

TARBALL     = build/nxt-python-2.2.2.zip
TARBALL_URL = http://github.com/Eelviny/nxt-python/archive/v2.2.2.zip
UNPACK_CMD  = unzip
SOURCE_DIR  = build/nxt-python-2.2.2
include $(shell rospack find mk)/download_unpack_build.mk

INSTALL_DIR = $(shell rospack find nxt_python)


nxt-python: $(INSTALL_DIR)/installed

$(INSTALL_DIR)/installed: $(SOURCE_DIR)/unpacked
	rm -rf src
	cd $(SOURCE_DIR) && python setup.py install --prefix=$(INSTALL_DIR) --install-purelib=$(INSTALL_DIR)/src
	touch $(INSTALL_DIR)/installed

clean:
	rm -rf $(SOURCE_DIR) bin lib src installed

wipe: clean
	rm -rf build
