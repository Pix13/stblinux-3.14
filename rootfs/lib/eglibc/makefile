include $(ROOTDIR)/config.arch

unexport MAKEFILE_LIST

BOGUS_TARGETS		:= FORCE makefile $(ROOTDIR)/config.arch

# This guard is necessary because the generated Makefile in the build folder
# does make -C on this directory causing an infinite loop.
ifneq ($(EGLIBC_MAKEFILE_GUARD),1)
EGLIBC_MAKEFILE_GUARD = 1

# Only build locally if necessary
ifeq ($(CONFIG_LIB_LIBC_FROM_ROOTFS),y)
libc-source := rootfs
else
libc-source := toolchain
endif

.PHONY: all
all:: all-$(libc-source)

.PHONY: all-rootfs
all-rootfs:
	if [ ! -e build/Makefile ]; then \
		mkdir -p build; \
		cd build; \
		../configure --host=$(shell $(CC) -dumpmachine) \
			--prefix=/usr \
			--with-sysroot=$(shell $(CC) -print-sysroot) \
			--enable-obsolete-rpc \
			--disable-nscd; \
	fi
	$(MAKE) -C build

.PHONY: all-toolchain
all-toolchain:

.PHONY: distclean
distclean::
	rm -rf build temp

.PHONY: romfs
romfs:: romfs-$(libc-source)

# We install to a temporary folder first and generate the list of
# files we actually want from that folder, then copy those to the romfs.
# This is based off of Fedora's glibc RPM spec.
.PHONY: romfs-rootfs
romfs-rootfs::
	mkdir -p temp
	make -C build install DESTDIR=$(CURDIR)/temp
	mkdir -p $(ROMFSDIR)/lib
	cp -df temp/lib/*.so* $(ROMFSDIR)/lib
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDD temp/bin/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDCONFIG temp/sbin/ldconfig /sbin/ldconfig

.PHONY: romfs-toolchain
romfs-toolchain::
	mkdir -p $(ROMFSDIR)/lib
	cp -df $(GCCSYSROOT)/lib/*.so* $(ROMFSDIR)/lib
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDD $(GCCSYSROOT)/usr/bin/ldd /bin/ldd
	$(ROMFSINST) -e CONFIG_LIB_LIBC_LDCONFIG $(GCCSYSROOT)/sbin/ldconfig /sbin/ldconfig

endif # EGLIBC_MAKEFILE_GUARD

# These targets must not be passed through to the original Makefile
.PHONY: $(BOGUS_TARGETS)
$(BOGUS_TARGETS)::

# Everything else (maybe including clean, distclean) does get passed through
.NOTPARALLEL: %
%:: FORCE
	$(MAKE) -f Makefile "$@"
