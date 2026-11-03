// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __YARP_CROP_CALLBACK_HPP__
#define __YARP_CROP_CALLBACK_HPP__

#include <mutex>
#include <utility> // std::pair
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/os/TypedReaderCallback.h>

namespace roboticslab
{

/**
 * @ingroup vision_libraries
 * @defgroup YarpCropCallback
 * @brief A typed callback reader for rectangular crop areas.
 */
class YarpCropCallback : public yarp::os::TypedReaderCallback<yarp::os::Bottle>
{
public:
    using VertexType = std::pair<int, int>;

    void onRead(yarp::os::Bottle & bot, const yarp::os::TypedReader<yarp::os::Bottle> & reader) override;
    std::vector<VertexType> getVertices() const;

private:
    std::vector<VertexType> cropVertices;
    mutable std::mutex cropMutex;
    bool isCropping {false};
};

} // namespace roboticslab

#endif // __YARP_CROP_CALLBACK_HPP__
