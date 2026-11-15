SDK ?= /Users/bikash/rplidar_sdk
OUT := bin/lidar_bridge
CXXFLAGS := -std=c++11 -O2 -I$(SDK)/sdk/include -I$(SDK)/sdk/src
LDFLAGS := $(SDK)/output/Darwin/Release/libsl_lidar_sdk.a -lpthread

all: $(OUT)

$(OUT): src/lidar_bridge.cpp
	@mkdir -p bin
	$(CXX) $(CXXFLAGS) $< -o $@ $(LDFLAGS)

clean:
	rm -f $(OUT)
