#ifndef iort_filters_MQTTOverlay_HPP
    #define iort_filters_MQTTOverlay_HPP

#include <pluginlib/class_list_macros.h>
#include <insitu/filter.hpp>
#include <iort_lib/iort.hpp>

// C++ Standard includes
#include <vector>
#include <mutex>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

// OpenCV includes
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>

namespace iort_filters {

  namespace {
  void drawtorect(cv::Mat& mat, cv::Rect target, const std::string& str,
                    int face = cv::FONT_HERSHEY_PLAIN, int thickness = 1,
                  cv::Scalar color = cv::Scalar(0, 0, 0, 255))
  {
      cv::Size rect = cv::getTextSize(str, face, 1.0, thickness, 0);
      double scalex = (double)target.width / (double)rect.width;
      double scaley = (double)target.height / (double)rect.height;
      double scale = std::min(scalex, scaley);
      int marginx =
          scale == scalex ?
              0 :
              (int)((double)target.width * (scalex - scale) / scalex * 0.5);
      int marginy =
          scale == scaley ?
              0 :
              (int)((double)target.height * (scaley - scale) / scaley * 0.5);
      cv::putText(
          mat, str,
          cv::Point(target.x + marginx, target.y + target.height - marginy), face,
          scale, color, thickness, 8, false);
  }
  }

  class MQTTOverlay : public insitu::Filter
  {

  public:
      MQTTOverlay(void);

      const cv::Mat apply (void);

      bool hasSettingEditor(void) const
      {
          return true;
      }

  private:
      void filterInit(void);

      void onDelete(void);
      
      void onCore(Json::Value);

      iort::Core core;

      iort::Subscriber* iortSub;

      std::string uuid;

  }; // end class MQTTOverlay

  } // end namespace iort_filters

  #endif // end iort_filters_MQTTOverlay_HPP
  
