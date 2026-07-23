# On-board build on Radxa Zero 3W (Debian aarch64)
# Dependencies: g++ libgstreamer1.0-dev libgstreamer-plugins-base1.0-dev
#               libopencv-dev librknnrt libdrm-dev

CXX       := g++
STRIP     := strip

CXXFLAGS  := -std=c++11 -O2 -Wall
CXXFLAGS  += $(shell pkg-config --cflags gstreamer-app-1.0 2>/dev/null)
CXXFLAGS  += $(shell pkg-config --cflags opencv4 2>/dev/null)
CXXFLAGS  += $(shell pkg-config --cflags libdrm 2>/dev/null)
CXXFLAGS  += -I/usr/include

LDFLAGS   := $(shell pkg-config --libs gstreamer-app-1.0 2>/dev/null)
LDFLAGS   += $(shell pkg-config --libs opencv4 2>/dev/null)
LDFLAGS   += $(shell pkg-config --libs libdrm 2>/dev/null)
LDFLAGS   += -lrknnrt -lpthread -ldl

TARGET    := zero3w_camera_ai
TARGET_DRM := zero3w_camera_drm

.PHONY: all clean drm

all: $(TARGET)

$(TARGET): main.cc
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	$(STRIP) $@
	@ls -la $@
	@file $@

drm: $(TARGET_DRM)

$(TARGET_DRM): main_drm.cc drm_display.cc
	$(CXX) $(CXXFLAGS) -o $@ $^ $(LDFLAGS)
	$(STRIP) $@
	@ls -la $@
	@file $@

clean:
	rm -f $(TARGET) $(TARGET_DRM)
