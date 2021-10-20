##
## MRLaserTag	Makefile for Windows (MSYS2)
##
## Builds all C/C++ code and dependencies.
##

all: apriltag

.PHONY: clean
clean:
	rm -rf build
	rm -rf bin/*.exe
	rm -rf bin/*.dll

include thirdparty/apriltag/Makefile.Win32.inc

