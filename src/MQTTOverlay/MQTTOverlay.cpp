#include <MQTTOverlay/MQTTOverlay.hpp>
#include <MQTTOverlay/MQTTOverlay_dialog.hpp>

#include <iort_lib/iort.hpp>

// #define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

namespace
{

  std::vector<std::string> queries;

  void drawtorect(cv::Mat &mat, cv::Rect target, const std::string &str,
                  int face = cv::FONT_HERSHEY_COMPLEX_SMALL, int thickness = 1,
                  cv::Scalar color = cv::Scalar(0, 0, 0, 255))
  {
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

}

namespace iort_filters
{
  /*
Filter Implementation
*/
  MQTTOverlay::MQTTOverlay(void)
  {
    iortSub = nullptr;
  }

  void MQTTOverlay::onCore(Json::Value data)
  {
    settings["data"] = data;
  }

  void MQTTOverlay::filterInit(void)
  {
    settingsDialog = new MQTTOverlayDialog(this);
    setSize(QSize(160, 240));
  }

  void MQTTOverlay::onDelete(void)
  {
    delete iortSub;
  }

  const cv::Scalar white(255, 255, 255, 255);

  const cv::Mat MQTTOverlay::apply(void)
  {
    int h = height();
    int w = width();

    cv::Mat ret = cv::Mat(
        h,
        w,
        CV_8UC4,
        cv::Scalar(0, 0, 0, 60));

    // Get uuid from settings, if changed change it, subscribe to iortSub
    if (settings.get("uuid_changed", false).asBool())
    {
      uuid = settings.get("uuid", "").asString();
      delete iortSub;
      iortSub = core.subscribe(uuid, &MQTTOverlay::onCore, this);
      settings["uuid_changed"] = false;
      settings["update_list"] = true; // flag to update dialog list once we receive a data object
    }
    if (uuid == "")
    {
      drawtorect(ret, cv::Rect(0, 0, w, h), "no uuid, edit filter", 1, 1, white);
    }
    else if (settings["data"] != Json::nullValue)
    {
      if (settings.get("update_list", false).asBool())
      {
        ((MQTTOverlayDialog *)settingsDialog)->updateList();
        settings["update_list"] = false;
      }

      int n = ((MQTTOverlayDialog *)settingsDialog)->getQueries().size() + (settings["generate_bars"].asBool() ? settings["bars"].asInt() : 0);
      int i = 0;
      for (std::string q : ((MQTTOverlayDialog *)settingsDialog)->getQueries())
      {
        std::string data = q + ": " + settings["data"].get(q, 0).asString();
        drawtorect(ret, cv::Rect(0, h / n * (i++), w, h / n), data, 1, 1, white);
        if (settings["generate_bars"].asBool() && settings["data"].get(q, 0).isInt())
        {
          int64_t dataAsInt = settings["data"].get(q, 0).asInt64();
          cv::rectangle(ret, cv::Rect(0, h / n * (i++) + (h / n / 3), (dataAsInt * w) / 4095, h / n / 3), cv::Scalar(255, 0, 0, 170), -1);
        }
      }
    }
    else
    {
      /* waiting for mqtt messages on topic, display detected uuid */
      drawtorect(ret, cv::Rect(0, 0, w, h / 2), uuid, 1, 1, white);
      drawtorect(ret, cv::Rect(0, h / 2, w, h / 2), "waiting for messages", 1, 1, white);
    }

    // TODO edit your overlay from user settings
    // e.g. settings.get("key", defaultValue).asType()

    return ret;
  }

} // end namespace iort_filters

PLUGINLIB_EXPORT_CLASS(iort_filters::MQTTOverlay, insitu::Filter);
