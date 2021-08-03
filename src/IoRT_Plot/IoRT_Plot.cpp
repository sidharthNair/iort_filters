#include <IoRT_Plot/IoRT_Plot.hpp>
#include <IoRT_Plot/IoRT_Plot_dialog.hpp>

#include <opencv2/calib3d.hpp>

// #define DEBUG
#ifdef DEBUG
#include <iostream>
#endif

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

namespace iort_filters {

/*
    Filter Implementation
*/
IoRT_Plot::IoRT_Plot(void)
{
    iortSub = nullptr;
    scanner.set_config(zbar::ZBAR_QRCODE, zbar::ZBAR_CFG_ENABLE, 1);
}

void IoRT_Plot::filterInit(void)
{
    settingsDialog = new IoRT_PlotDialog(this);

    
    image_transport::ImageTransport it(getNodeHandle());
    std::string image_topic = imageTopic();
    if (image_topic.find("compressed", 0) == image_topic.length()-10) {
        image_topic = image_topic.substr(0, image_topic.length()-11);
    }
    imgSub = it.subscribe(image_topic, 1, &IoRT_Plot::imageCB, this);
}

void IoRT_Plot::onDelete(void)
{
    // TODO cleanup code
    imgSub.shutdown();
    delete iortSub;
}

void IoRT_Plot::onCore(Json::Value data)
{
    qrMutex.lock();
    iortQueue.push_front(data.get("pot", 0).asInt64());
    if (iortQueue.size() > 64) iortQueue.pop_back();
    qrMutex.unlock();
}

const cv::Scalar blue(255, 0, 0, 255);
const cv::Scalar green(0, 255, 0, 255);
const cv::Scalar red(0, 0, 255, 255);

const cv::Mat IoRT_Plot::apply (void)
{
    /*
        Create a transparent image to construct your overlay on
    */
    cv::Mat ret = cv::Mat(settings.get("height", 480).asInt(),
                          settings.get("width", 640).asInt(), CV_8UC4,
                          cv::Scalar(255, 255, 255, 0));

    if (uuid.length() > 0)
    {
        if (iortQueue.size() > 1)
        {
            // draw plot heading
            cv::putText(ret, uuid + " potentiometer:", cv::Point(10,20), cv::FONT_HERSHEY_PLAIN, 1.0, green);
            
            // draw plot axis
            int x = 32;
            cv::line(ret, cv::Point(x, 460), cv::Point(x, 40), green);
            cv::line(ret, cv::Point(x, 461), cv::Point(608, 461), green);

            /* plot data */
            qrMutex.lock();
            for (auto it = iortQueue.begin(); it < iortQueue.end()-1; ++it) {
                int y1 = 460 - ((*it * 420) / 4096);
                int y2 = 460 - ((*(it+1) * 420) / 4096);
                cv::line(ret, cv::Point(x, y1), cv::Point(x+9, y2), blue);
                x += 9;
            }
            qrMutex.unlock();
        }
        else
        {
            /* waiting for curl to connect, display detected uuid */
            cv::putText(ret, "waiting for data from: " + uuid, cv::Point(10,10), cv::FONT_HERSHEY_PLAIN, 1.0, green);
        }
    }

    return ret;
}

void IoRT_Plot::imageCB(const sensor_msgs::Image::ConstPtr& msg)
{
    try
    {
        if (uuid.length() == 0) {
            /* convert ROS image to OpenCV image */
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::MONO8);

#ifdef DEBUG
            std::cout << "dims: " << cv_ptr->image.cols << "x" << cv_ptr->image.rows << "\n";
#endif

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
                uuid = detected_uuid;
                // if (iortSub != nullptr) delete iortSub;
                iortSub = core.subscribe(uuid, &IoRT_Plot::onCore, this);
            }
#ifdef DEBUG
            std::cout << "detected_uuid: " << detected_uuid << "\n";
#endif
        }
    }
    catch (std::exception e)
    {
        qWarning("Failed to convert image: %s", e.what());
        return;
    }
}

} // end namespace iort_filters

PLUGINLIB_EXPORT_CLASS(iort_filters::IoRT_Plot, insitu::Filter);
    
