##
## MRLaserTag
##
## apriltag targets for Windows (MSYS2).
##

apriltag:	bin/libapriltag.dll

bin/libapriltag.dll:
	rm -rf build/apriltag
	mkdir -p build/apriltag
	cd build/apriltag; \
	cmake ../../thirdparty/apriltag -DCMAKE_BUILD_TYPE=Release -G "MinGW Makefiles"; \
	mingw32-make -j32; \
	cp lib/libapriltag.dll ../../bin;
