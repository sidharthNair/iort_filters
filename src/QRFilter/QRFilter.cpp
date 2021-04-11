#include <QRFilter/QRFilter.hpp>
#include <QRFilter/QRFilter_dialog.hpp>

#include <opencv2/calib3d.hpp>

#include <iostream>

namespace iort_filters
{
/*
    Filter Implementation
*/
QRFilter::QRFilter(void)
{
    iortSub = nullptr;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

void QRFilter::onCore(Json::Value data)
{
    settings["data"] = data;
}

void QRFilter::onInit(void)
{
    settingsDialog = new QRFilterDialog(this);

    /* initialize image subscriber (hardcoded for now) */
    image_transport::ImageTransport it(getNodeHandle());
    imgSub = it.subscribe("/usb_cam/image_raw", 1, &QRFilter::imageCB, this);
}

void QRFilter::onDelete(void)
{
    // TODO cleanup code
    delete iortSub;
}

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

const cv::Mat QRFilter::apply(void)
{
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret = cv::Mat(settings.get("height", 720).asInt(),
                          settings.get("width", 1280).asInt(), CV_8UC4,
                          cv::Scalar(255, 255, 255, 0));

    if (uuid.length() > 0)
    {
        /* there is a sensor currently detected in the frame */
        cv::Mat datMat = cv::Mat(600, 600, CV_8UC4, cv::Scalar(255, 255, 255, 255));
        std::vector<cv::Point2f> datRect = {
            cv::Point2f(0,0),
            cv::Point2f(0,600),
            cv::Point2f(600,600),
            cv::Point2f(600,0)
        };

        if (settings["data"] != Json::nullValue)
        {
            /* lambda response recieved, overlay data */
            std::string data;
            data = "color is " + settings["data"].get("light", "null").asString();
            drawtorect(datMat, cv::Rect(0, 0, 600, 150), data, 1, 8);

            if (settings["data"].get("tilt", "null").asString() == "true") {
                data = "tilted";
            } else {
                data = "not tilted";
            }
            drawtorect(datMat, cv::Rect(0, 150, 600, 150), data, 1, 8);

            data = "door is " + settings["data"].get("hall", "null").asString();
            drawtorect(datMat, cv::Rect(0, 300, 600, 150), data, 1, 8);

            int64_t pot = settings["data"].get("pot", 0).asInt64();
            cv::rectangle(datMat, cv::Rect(0, 450, (pot * 600) / 4095, 150), cv::Scalar(255, 0, 0, 255), -1);
        }
        else
        {
            /* waiting for curl to connect, display detected uuid */
            drawtorect(datMat, cv::Rect(0, 0, 600, 600), uuid, 1, 8);
        }

        /* draw bounding box */
        qrMutex.lock();
        cv::Mat trans = cv::findHomography(datRect, qrBBox, 0);
        qrMutex.unlock();

        cv::warpPerspective(datMat, ret, trans, ret.size());
    }

    return ret;
}

void QRFilter::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        /* convert ROS image to OpenCV image */
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

        /* convert OpenCV image to ZBar image */
        zbar::Image zImg(cv_ptr->image.cols, cv_ptr->image.rows, "Y800",
                         (uchar*)cv_ptr->image.data,
                         cv_ptr->image.cols * cv_ptr->image.rows);

        std::string detected_uuid;
        if (scanner.scan(zImg) > 0)
        {
            /* we only care about one QR code for now */
            auto symbol = zImg.symbol_begin();
            detected_uuid = symbol->get_data();

            qrMutex.lock();
            qrBBox.clear();
            for (int i = 0; i < symbol->get_location_size(); ++i)
            {
                qrBBox.push_back(cv::Point2f(float(symbol->get_location_x(i)),
                                             float(symbol->get_location_y(i))));
            }
            qrMutex.unlock();
        }

        if (detected_uuid.length() > 0)
        {
            if (detected_uuid != uuid)
            {
                uuid = detected_uuid;
                if (iortSub != nullptr) delete iortSub;
                iortSub = core.subscribe(uuid, &QRFilter::onCore, this);
            }
        }
        else
        {
            uuid.clear();
        }
    }
    catch (std::exception e)
    {
        qWarning("Failed to convert image: %s", e.what());
        return;
    }
}

}    // end namespace iort_filters

PLUGINLIB_EXPORT_CLASS(iort_filters::QRFilter, insitu::Filter);
