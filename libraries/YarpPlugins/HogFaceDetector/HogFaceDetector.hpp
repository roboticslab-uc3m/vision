#ifndef __HOG_FACE_DETECTOR_HPP__
#define __HOG_FACE_DETECTOR_HPP__

#include <yarp/dev/DeviceDriver.h>

#include <dlib/image_processing/frontal_face_detector.h>

#include <dlib/image_processing.h>

#include <dlib/opencv.h>

#include "IDetector.hpp"

namespace roboticslab
{

/**
 * @ingroup YarpPlugins
 * @defgroup HogFaceDetector
 * @brief Contains roboticslab::HogFaceDetector.
 */
class HogFaceDetector : public yarp::dev::DeviceDriver,
                        public IDetector
{
public:
    bool open(yarp::os::Searchable& config) override;
    bool detect(const yarp::sig::Image& inYarpImg, yarp::os::Bottle& detectedObjects) override;

private:
    dlib::frontal_face_detector hogFaceDetector;
    dlib::full_object_detection shape;
    dlib::shape_predictor predictor;
};

} // namespace roboticslab

#endif // __HOG_FACE_DETECTOR_HPP__
