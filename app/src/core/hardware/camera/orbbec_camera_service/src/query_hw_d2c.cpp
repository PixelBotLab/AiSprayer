#include <iostream>
#include <iomanip>
#include <memory>
#include <map>
#include <set>
#include <vector>
#include <libobsensor/ObSensor.hpp>

int main() {
    try {
        ob::Context ctx;
        auto dev_list = ctx.queryDeviceList();
        if (dev_list->deviceCount() == 0) {
            std::cout << "No Orbbec devices found!" << std::endl;
            return 1;
        }

        auto dev = dev_list->getDevice(0);
        auto dev_info = dev->getDeviceInfo();
        std::cout << "Device: " << dev_info->name() << " (SN: " << dev_info->serialNumber() << ", FW: " << dev_info->firmwareVersion() << ")\n" << std::endl;

        ob::Pipeline pipe(dev);
        auto color_sensor = dev->getSensor(OB_SENSOR_COLOR);
        if (!color_sensor) {
            std::cout << "No Color sensor found!" << std::endl;
            return 1;
        }

        auto color_profiles = color_sensor->getStreamProfileList();
        int color_count = color_profiles->count();

        struct ResFps {
            int w, h, fps, fmt;
        };

        std::map<std::pair<int, int>, std::vector<std::pair<int, std::string>>> supported_map;
        std::map<std::pair<int, int>, std::set<std::pair<int, int>>> matching_depth_res;

        for (int i = 0; i < color_count; ++i) {
            auto profile = color_profiles->getProfile(i)->as<ob::VideoStreamProfile>();
            int w = profile->width();
            int h = profile->height();
            int fps = profile->fps();
            int fmt = profile->format();

            std::string fmt_str = (fmt == OB_FORMAT_BGR) ? "BGR" :
                                  (fmt == OB_FORMAT_RGB) ? "RGB" :
                                  (fmt == OB_FORMAT_MJPG) ? "MJPG" :
                                  (fmt == OB_FORMAT_YUYV) ? "YUYV" : ("Fmt_" + std::to_string(fmt));

            try {
                auto depth_profiles = pipe.getD2CDepthProfileList(profile, ALIGN_D2C_HW_MODE);
                if (depth_profiles && depth_profiles->count() > 0) {
                    supported_map[{w, h}].push_back({fps, fmt_str});
                    for (uint32_t j = 0; j < depth_profiles->count(); ++j) {
                        auto d_p = depth_profiles->getProfile(j)->as<ob::VideoStreamProfile>();
                        matching_depth_res[{w, h}].insert({d_p->width(), d_p->height()});
                    }
                }
            } catch (...) {}
        }

        std::cout << "==========================================================================" << std::endl;
        std::cout << "           SUPPORTED HARDWARE D2C ALIGNMENT RESOLUTIONS (Gemini 336L)      " << std::endl;
        std::cout << "==========================================================================" << std::endl;

        for (const auto& kv : supported_map) {
            int w = kv.first.first;
            int h = kv.first.second;
            std::cout << "\n[Color Resolution]: \033[32m" << w << "x" << h << "\033[0m" << std::endl;

            // Supported Depth Resolutions
            std::cout << "  -> Matching Depth Resolutions: ";
            for (const auto& d_res : matching_depth_res[{w, h}]) {
                std::cout << d_res.first << "x" << d_res.second << "  ";
            }
            std::cout << "\n  -> Supported FPS & Formats: ";
            std::set<std::string> fps_fmt_set;
            for (const auto& item : kv.second) {
                fps_fmt_set.insert(std::to_string(item.first) + "fps (" + item.second + ")");
            }
            for (const auto& s : fps_fmt_set) {
                std::cout << s << " | ";
            }
            std::cout << std::endl;
        }

        std::cout << "\n==========================================================================" << std::endl;
        std::cout << "Note: 1280x800 Color resolution only supports Software D2C in current firmware (1.4.60)." << std::endl;
        std::cout << "==========================================================================" << std::endl;

    } catch (const ob::Error& e) {
        std::cerr << "ObError: " << e.getMessage() << std::endl;
        return 1;
    }
    return 0;
}
