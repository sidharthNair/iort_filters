#include <QRFilter/QRFilter.hpp>
#include <QRFilter/QRFilter_dialog.hpp>

#include <opencv2/calib3d.hpp>

// #define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

namespace
{

    void drawtorect(cv::Mat &mat, cv::Rect target, const std::string &str,
                    int face = cv::FONT_HERSHEY_PLAIN, int thickness = 1,
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
    QRFilter::QRFilter(void)
    {
        iortSub = nullptr;
        scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
    }

    void QRFilter::onCore(Json::Value data)
    {
        settings["data"] = data;
    }

    void QRFilter::filterInit(void)
    {
        settingsDialog = new QRFilterDialog(this);
        image_transport::ImageTransport it(getNodeHandle());
        std::string image_topic = imageTopic();
        if (image_topic.find("compressed", 0) == image_topic.length()-10) {
            image_topic = image_topic.substr(0, image_topic.length()-11);
        }
        imgSub = it.subscribe(image_topic, 1, &QRFilter::imageCB, this);
    }

    void QRFilter::onDelete(void)
    {
        // TODO cleanup code
        imgSub.shutdown();
        delete iortSub;
    }

    const cv::Scalar black(0, 0, 0, 255);

    const cv::Mat QRFilter::apply(void)
    {
        /*
        Create a transparent image to construct your overlay on
    */
        cv::Mat ret = cv::Mat(settings.get("height", 480).asInt(),
                              settings.get("width", 640).asInt(), CV_8UC4,
                              cv::Scalar(255, 255, 255, 0));

        if (uuid.length() > 0)
        {
            int h = 600;
            int w = 600;
            /* there is a sensor currently detected in the frame */
            cv::Mat datMat = cv::Mat(600, 600, CV_8UC4, cv::Scalar(255, 255, 255, 255));
            std::vector<cv::Point2f> datRect = {
                cv::Point2f(0, 0),
                cv::Point2f(0, 600),
                cv::Point2f(600, 600),
                cv::Point2f(600, 0)};

            if (settings["data"] != Json::nullValue)
            {
                if (settings.get("update_list", false).asBool())
                {
                    ((QRFilterDialog *)settingsDialog)->updateList();
                    settings["update_list"] = false;
                }

                int n = ((QRFilterDialog *)settingsDialog)->getQueries().size() + (settings["generate_bars"].asBool() ? settings["bars"].asInt() : 0);
                int i = 0;
                for (std::string q : ((QRFilterDialog *)settingsDialog)->getQueries())
                {
                    std::string data = q + ": " + settings["data"].get(q, 0).asString();
                    drawtorect(datMat, cv::Rect(0, h / n * (i++), w, h / n), data, 1, 8, black);
                    if (settings["generate_bars"].asBool() && settings["data"].get(q, 0).isInt())
                    {
                        int64_t dataAsInt = settings["data"].get(q, 0).asInt64();
                        cv::rectangle(datMat, cv::Rect(0, h / n * (i++) + (h / n / 3), (dataAsInt * w) / 4095, h / n / 3), cv::Scalar(255, 0, 0, 255), -1);
                    }
                }
            }
            else
            {
                /* waiting for mqtt messages on topic, display detected uuid */
                drawtorect(datMat, cv::Rect(0, 0, w, h / 2), uuid, 1, 8, black);
                drawtorect(datMat, cv::Rect(0, h / 2, w, h / 2), "waiting for messages", 1, 8, black);
            }

            /* draw bounding box */
            qrMutex.lock();
            cv::Mat trans = cv::findHomography(datRect, qrBBox, 0);
            qrMutex.unlock();

            cv::warpPerspective(datMat, ret, trans, ret.size());
        }

        return ret;
    }

    void QRFilter::imageCB(const sensor_msgs::Image::ConstPtr &msg)
    {
        try
        {
            /* convert ROS image to OpenCV image */
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

#ifdef DEBUG
            std::cout << "dims: " << cv_ptr->image.cols << "x" << cv_ptr->image.rows << "\n";
#endif

            /* convert OpenCV image to ZBar image */
            zbar::Image zImg(cv_ptr->image.cols, cv_ptr->image.rows, "Y800",
                             (uchar *)cv_ptr->image.data,
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
                    delete iortSub;
                    settings["update_list"] = true;  // flag to update dialog list once we receive a data object
                    iortSub = core.subscribe(uuid, &QRFilter::onCore, this);
                }
            }
#ifdef DEBUG
            std::cout << "detected_uuid: " << detected_uuid << "\n";
#endif
        }
        catch (std::exception e)
        {
            qWarning("Failed to convert image: %s", e.what());
            return;
        }
    }

} // end namespace iort_filters

PLUGINLIB_EXPORT_CLASS(iort_filters::QRFilter, insitu::Filter);
