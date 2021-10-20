##
## MRLaserTag
##
## apriltag targets for Linux.
##

apriltag:	bin/libapriltag.so

bin/libapriltag.so:
	rm -rf build/apriltag
	mkdir -p build/apriltag
	cd build/apriltag; \
	cmake ../../thirdparty/apriltag -DCMAKE_BUILD_TYPE=Release; \
	make -j32; \
	cp lib/libapriltag.so ../../bin;
