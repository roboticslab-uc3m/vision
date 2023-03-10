// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "YarpCropCallback.hpp"

#include <algorithm> // std::min, std::max

#include <yarp/os/LogComponent.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/TypedReader.h>

using namespace roboticslab;

namespace
{
    YARP_LOG_COMPONENT(YCCB, "rl.YarpCropCallback")
}

void YarpCropCallback::onRead(yarp::os::Bottle & bot, const yarp::os::TypedReader<yarp::os::Bottle> & reader)
{
    if (bot.size() == 4)
    {
        auto x1 = bot.get(0).asInt32();
        auto y1 = bot.get(1).asInt32();
        auto x2 = bot.get(2).asInt32();
        auto y2 = bot.get(3).asInt32();

        cropMutex.lock();
        cropVertices = {
            {std::min(x1, x2), std::min(y1, y2)}, // left-top corner
            {std::max(x1, x2), std::max(y1, y2)}  // right-bottom corner
        };
        cropMutex.unlock();

        yCInfo(YCCB, "Port %s cropping input frames: (x1: %d, y1: %d) (x2: %d, y2: %d)",
               reader.getName().c_str(),
               cropVertices[0].first, cropVertices[0].second,
               cropVertices[1].first, cropVertices[1].second);

        isCropping = true;
    }
    else if (isCropping)
    {
        yCInfo(YCCB) << "Crop disabled at port" << reader.getName();

        cropMutex.lock();
        cropVertices.clear();
        cropMutex.unlock();

        isCropping = false;
    }
}

yarp::sig::VectorOf<YarpCropCallback::VertexType> YarpCropCallback::getVertices() const
{
    std::lock_guard lock(cropMutex);
    return cropVertices;
}
