#include <MQTTOverlay/MQTTOverlay.hpp>
#include <MQTTOverlay/MQTTOverlay_dialog.hpp>
#include <iort_lib/iort.hpp>

// #define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

namespace {

double computeWeightedDelta(std::vector<double> previous_vals, double weight_param) {
    double ret = 0;
    double divisor = (pow(weight_param, previous_vals.size()) - 1);
    // Example n = 5, weight_param = 2:
    // Weights:        1/31 + 2/31 + 4/31 + 8/31 + 16/31 = 31/31 = 1
    // Values:         1st    2nd    3rd    4th    5th                  (1st is oldest)
    for (int i = 0; i < previous_vals.size(); i++) {
        ret += (weight_param - 1) * previous_vals.at(i) / divisor;
        divisor /= weight_param;
    }
    return ret;
}

std::map<std::string, std::vector<double>> deltas;
std::map<std::string, int> last_values;
int counter = 0;
bool estimation = false;
bool color = false;
uint64_t current_time;
uint64_t sim_latency_log = 0;

void drawtorect(cv::Mat &mat, cv::Rect target, const std::string &str,
                int face = cv::FONT_HERSHEY_COMPLEX_SMALL, int thickness = 1,
                cv::Scalar color = cv::Scalar(0, 0, 0, 255)) {
    cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
    double scalex = (double)target.width / (double)rect.width;
    double scaley = (double)target.height / (double)rect.height;
    double scale = std::min(scalex, scaley);
    int marginx =
        scale == scalex ? 0 : (int)((double)target.width * (scalex - scale) / scalex * 0.5);
    int marginy =
        scale == scaley ? 0 : (int)((double)target.height * (scaley - scale) / scaley * 0.5);
    cv::putText(
        mat, str,
        cv::Point(target.x + marginx, target.y + target.height - marginy), face,
        scale, color, thickness, 8, false);
}

}  // namespace

namespace iort_filters {
/*
Filter Implementation
*/
MQTTOverlay::MQTTOverlay(void) {
    iortSub = nullptr;
}

void MQTTOverlay::onCore(Json::Value data) {
    if (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - sim_latency_log > settings.get("sim_lat", 0).asInt64() * 1000) {
        settings["data"] = data;
        color = false;
        for (std::string q : ((MQTTOverlayDialog *)settingsDialog)->getQueries()) {
            if (data.get(q, "").isInt()) {
                if (deltas[q].size() == 0) {
                    deltas[q].push_back(0);
                } else if (deltas[q].size() < settings.get("sample_size", 10).asInt64()) {
                    deltas[q].push_back((data.get(q, 0).asInt64() - last_values[q]));
                } else {
                    while (deltas[q].size() >= settings.get("sample_size", 10).asInt64()) {
                        deltas[q].erase(deltas[q].begin());
                    }
                    deltas[q].push_back((data.get(q, 0).asInt64() - last_values[q]));
                    if (settings["estimate"].asBool()) {
                        estimation = true;
                    }
                }
                last_values[q] = data.get(q, 0).asInt64();
            }
        }
        current_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
        sim_latency_log = current_time;
    }
}

void MQTTOverlay::filterInit(void) {
    settingsDialog = new MQTTOverlayDialog(this);
    setSize(QSize(160, 240));
}

void MQTTOverlay::onDelete(void) {
    delete iortSub;
}

const cv::Scalar white(255, 255, 255, 255);
const cv::Scalar blue(0, 0, 255, 255);
const cv::Scalar red(255, 0, 0, 255);
const cv::Scalar green(0, 255, 0, 255);

const cv::Mat MQTTOverlay::apply(void) {
    int h = height();
    int w = width();

    cv::Mat ret = cv::Mat(
        h,
        w,
        CV_8UC4,
        cv::Scalar(0, 0, 0, 60));

    // Get uuid from settings, if changed change it, subscribe to iortSub
    if (settings.get("uuid_changed", false).asBool()) {
        uuid = settings.get("uuid", "").asString();
        delete iortSub;
        iortSub = core.subscribe(uuid, &MQTTOverlay::onCore, this);
        settings["uuid_changed"] = false;
        settings["update_list"] = true;  // flag to update dialog list once we receive a data object
    }
    if (uuid == "") {
        drawtorect(ret, cv::Rect(0, 0, w, h), "no uuid, edit filter", 1, 1, white);
    } else if (settings["data"] != Json::nullValue) {
        if (settings.get("update_list", false).asBool()) {
            ((MQTTOverlayDialog *)settingsDialog)->updateList();
            settings["update_list"] = false;
        }

        int n = ((MQTTOverlayDialog *)settingsDialog)->getQueries().size() + (settings["generate_bars"].asBool() ? settings["bars"].asInt() : 0);
        int i = 0;
        bool updateTime = false;
        for (std::string q : ((MQTTOverlayDialog *)settingsDialog)->getQueries()) {
            bool isInt = settings["data"].get(q, 0).isInt();
            if (settings["estimate"].asBool()) {
                if (estimation && isInt) {
                    int dt = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count() - current_time;
                    if (dt > settings.get("threshold", 250).asInt64() * 1000) {
                        double average = computeWeightedDelta(deltas[q], settings.get("weight_param", 2).asDouble());
                        settings["data"][q] = (int)(settings["data"].get(q, 0).asInt64() + average);
                        deltas[q].erase(deltas[q].begin());
                        deltas[q].push_back(average / 2);
                        color = true;
                        updateTime = true;
                    }
                }
            }
            std::string data;
            if (q == "valve 1" || q == "valve 2") {  // for demo
                data = q + ": " + std::to_string((int)(settings["data"].get(q, 0).asInt64() / 4095.0 * 500.0)) + " psi";
            } else {
                data = q + ": " + settings["data"].get(q, 0).asString();
            }
            drawtorect(ret, cv::Rect(0, h / n * (i++), w, h / n), data, 1, 1, (color && isInt) ? blue : white);
            if (settings["generate_bars"].asBool() && settings["data"].get(q, 0).isInt()) {
                int64_t dataAsInt = settings["data"].get(q, 0).asInt64();
                int64_t actual = last_values[q];
                cv::Scalar ryg_color(2 * (int)(actual / 4095.0 * 255.0), 2 * (255 - (int)(actual / 4095.0 * 255.0)), 0, 255);
                cv::rectangle(ret, cv::Rect(0, h / n * i + (h / n / 3), (actual * w) / 4095, h / n / 3), ryg_color, -1);
                if (color) {
                    cv::rectangle(ret, cv::Rect(0, h / n * i + (h / n / 3), (dataAsInt * w) / 4095, h / n / 3), blue, 1);
                }
                i++;
            }
        }
        if (updateTime)
            current_time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::system_clock::now().time_since_epoch()).count();
    } else {
        /* waiting for mqtt messages on topic, display detected uuid */
        drawtorect(ret, cv::Rect(0, 0, w, h / 2), uuid, 1, 1, white);
        drawtorect(ret, cv::Rect(0, h / 2, w, h / 2), "waiting for messages", 1, 1, white);
    }

    // TODO edit your overlay from user settings
    // e.g. settings.get("key", defaultValue).asType()

    return ret;
}

}  // end namespace iort_filters

PLUGINLIB_EXPORT_CLASS(iort_filters::MQTTOverlay, insitu::Filter);
