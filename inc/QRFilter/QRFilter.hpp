#ifndef iort_filters_QRFilter_HPP
#define iort_filters_QRFilter_HPP

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

// ZBar
#include <zbar.h>

namespace iort_filters
{
class QRFilter : public insitu::Filter
{
public:
    QRFilter(void);

    const cv::Mat apply(void);

    bool hasSettingEditor(void)
    {
        return true;
    }

private:
    void onInit(void);

    void onDelete(void);

    void onCore(Json::Value);

    void imageCB(const sensor_msgs::Image::ConstPtr& msg);

    iort::Core core;

    iort::Subscriber* iortSub;

    image_transport::Subscriber imgSub;

    zbar::ImageScanner scanner;

    std::string uuid;

    cv_bridge::CvImageConstPtr cv_ptr;

    std::mutex qrMutex;

    std::vector<cv::Point2f> qrBBox;

};    // end class QRFilter

}    // end namespace iort_filters

#endif    // end iort_filters_QRFilter_HPP
