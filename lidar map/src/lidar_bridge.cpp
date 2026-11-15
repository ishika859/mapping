#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <string>

#include "sl_lidar.h"
#include "sl_lidar_driver.h"

using namespace sl;
static volatile std::sig_atomic_t stopping = 0;
static void on_signal(int) { stopping = 1; }

int main(int argc, char **argv) {
    const char *port = argc > 1 ? argv[1] : "/dev/cu.usbserial-10";
    const int baud = argc > 2 ? std::atoi(argv[2]) : 460800;
    std::signal(SIGINT, on_signal);
    std::signal(SIGTERM, on_signal);

    ILidarDriver *drv = *createLidarDriver();
    if (!drv) { std::cerr << "bridge: cannot create lidar driver\n"; return 2; }
    IChannel *channel = *createSerialPortChannel(port, baud);
    if (!channel || !SL_IS_OK(drv->connect(channel))) {
        std::cerr << "bridge: cannot connect to " << port << " at " << baud << " baud\n";
        delete drv;
        return 3;
    }
    sl_lidar_response_device_info_t info{};
    if (!SL_IS_OK(drv->getDeviceInfo(info))) {
        std::cerr << "bridge: connected but device info failed\n";
        delete drv;
        return 4;
    }
    std::cout << "{\"type\":\"connected\",\"model\":" << info.model
              << ",\"firmware\":\"" << (info.firmware_version >> 8) << "."
              << (info.firmware_version & 0xff) << "\"}\n" << std::flush;

    if (!SL_IS_OK(drv->setMotorSpeed()) || !SL_IS_OK(drv->startScan(false, true))) {
        std::cerr << "bridge: failed to start scan\n";
        delete drv;
        return 5;
    }
    std::cout << "{\"type\":\"scanning\"}\n" << std::flush;

    sl_lidar_response_measurement_node_hq_t nodes[8192];
    while (!stopping) {
        size_t count = sizeof(nodes) / sizeof(nodes[0]);
        sl_result result = drv->grabScanDataHq(nodes, count, 1000);
        if (!SL_IS_OK(result)) continue;
        drv->ascendScanData(nodes, count);
        std::cout << "{\"type\":\"scan\",\"points\":[";
        bool first = true;
        for (size_t i = 0; i < count; ++i) {
            const float distance = nodes[i].dist_mm_q2 / 4000.0f;
            const float angle = (nodes[i].angle_z_q14 * 90.0f) / 16384.0f;
            const int quality = nodes[i].quality >> SL_LIDAR_RESP_MEASUREMENT_QUALITY_SHIFT;
            if (!(distance > 0.02f) || !std::isfinite(distance)) continue;
            if (!first) std::cout << ',';
            first = false;
            std::cout << '[' << angle << ',' << distance << ',' << quality << ']';
        }
        std::cout << "]}\n" << std::flush;
    }
    drv->stop();
    drv->setMotorSpeed(0);
    std::cout << "{\"type\":\"stopped\"}\n" << std::flush;
    delete drv;
    return 0;
}
