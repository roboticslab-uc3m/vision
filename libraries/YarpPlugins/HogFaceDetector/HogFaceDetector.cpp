#include "HogFaceDetector.hpp"

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/cv/Cv.h>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(HFD, "rl.HogFaceDetector")
}

bool HogFaceDetector::open(yarp::os::Searchable& parameters)
{
    yarp::os::ResourceFinder rf;
    rf.setDefaultContext("HogFaceDetector");

    hogFaceDetector = dlib::get_frontal_face_detector();

    dlib::deserialize("/home/carlos/shape_predictor_68_face_landmarks.dat") >> predictor;

    return true;
}

bool HogFaceDetector::detect(const yarp::sig::Image & inYarpImg, yarp::os::Bottle & detectedObjects)
{
    yarp::sig::ImageOf<yarp::sig::PixelBgr> inYarpImgBgr;
    inYarpImgBgr.copy(inYarpImg);
    cv::Mat inCvMat = yarp::cv::toCvMat(inYarpImgBgr);
    cv::Mat inCvMatGray;

    cv::cvtColor(inCvMat, inCvMatGray, cv::COLOR_BGR2GRAY);

    dlib::cv_image<unsigned char> dlibImage(inCvMatGray);

    std::vector<dlib::rectangle> faceRects = hogFaceDetector(dlibImage);

    for (const auto & faceRect : faceRects)
    {
        cv::Rect object;

        object = cv::Rect(cv::Point2i(faceRect.left(), faceRect.top()), cv::Point2i(faceRect.right() + 1, faceRect.bottom() + 1));
        shape = predictor(dlibImage, faceRect);

        std::vector<yarp::os::Value> lms;
        yarp::os::Bottle detectedlms;

        for (unsigned long j = 0; j < shape.num_parts(); j++)
        {
            detectedlms.addInt32(shape.part(j).x());
            detectedlms.addInt32(shape.part(j).y());
        }

        detectedObjects.addDict() = {
            {"tlx", yarp::os::Value(object.x)},
            {"tly", yarp::os::Value(object.y)},
            {"brx", yarp::os::Value(object.x + object.width)},
            {"bry", yarp::os::Value(object.y + object.height)}
        };
        
        detectedObjects.addList() = detectedlms;

    }

    return true;
}
