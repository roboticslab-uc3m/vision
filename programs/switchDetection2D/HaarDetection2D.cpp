// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "HaarDetection2D.hpp"

namespace roboticslab
{

HaarDetectionTransformation::HaarDetectionTransformation(yarp::os::Searchable* parameters)
{
    if(!parameters->check("switchMode"))
    {
        CD_ERROR("**** \"context\" parameter for HaarDetectionTransformation NOT found\n");
        return;
    }
    std::string context = parameters->find("swicthMode").asString();

    if (parameters->check("xmlCascade"))
    {
        xmlCascade = parameters->find("xmlCascade").asString();
    }

    if (cascade.empty() || !object_cascade.load(cascade))
    {
        CD_ERROR("No cascade!\n");
        std::exit(1);
    }

    if(!parameters->check("xmlCascade"))
    {
        CD_ERROR("**** \"xmlCascade\" parameter for HaarDetectionTransformation NOT found\n");
        return;
    }

    //std::string xmlCascade = DEFAULT_XMLCASCADE;
    std::string xmlCascade = parameters->find("xmlCascade").asString();
    CD_DEBUG("**** \"xmlCascade\" parameter for HaarDetectionTransformation found: \"%s\"\n", xmlCascade.c_str());
    CD_DEBUG("\t--xmlCascade [file.xml] (default: \"%s\")\n", xmlCascade.c_str());

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext(context);
    std::string xmlCascadeFullName = rf.findFileByName(xmlCascade);
    if(xmlCascadeFullName.empty())
    {
        CD_ERROR("**** full path for file NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for file found: \"%s\"\n", xmlCascadeFullName.c_str());

    std::string cascade = rf.findFileByName(xmlCascade);

    valid = true;
}

// /*****************************************************************/

double HaarDetectionTransformation::transform(const double value)
{
    return value * m + b;
}

/*****************************************************************/


/*****************************************************************/
yarp::sig::ImageOf<yarp::sig::PixelRgb> HaarDetection2D::run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg, cv::CascadeClassifier object_cascade) {


    cv::Mat inCvMat = cv::cvarrToMat((IplImage*)inYarpImg.getIplImage());
    cv::cvtColor(inCvMat, inCvMat, CV_RGB2GRAY);

    std::vector<cv::Rect> objects;

    object_cascade.detectMultiScale(inCvMat, objects, 1.1, 2, 0 | CV_HAAR_SCALE_IMAGE, cv::Size(30, 30));

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg = inYarpImg;
    yarp::sig::PixelRgb red(255, 0, 0);
    yarp::sig::PixelRgb green(0, 255, 0);

    yarp::os::Bottle output;

    int closestObject = 999999;
    int minimumDistance = 999999;

    for (int i = 0; i < objects.size(); i++)
    {
        const int pxX = objects[i].x + objects[i].width / 2;
        const int pxY = objects[i].y + objects[i].height / 2;

        int centerX = inCvMat.cols / 2;
        int centerY = inCvMat.rows / 2;

        int distance = std::sqrt(std::pow(pxX - centerX, 2) + std::pow(pxY - centerY, 2));

        if (distance < minimumDistance)
        {
            minimumDistance = distance;
            closestObject = i;
        }
    }

    for (int i = 0; i < objects.size(); i++)
    {
        const int pxX = objects[i].x + objects[i].width / 2;
        const int pxY = objects[i].y + objects[i].height / 2;

        if (i == closestObject)
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg, green, pxX, pxY,
            objects[i].width / 2, objects[i].height / 2);

            // scale centroids and fit into [-1, 1] range
            double cX = 2.0 * pxX / inCvMat.cols - 1.0;
            double cY = 2.0 * pxY / inCvMat.rows - 1.0;

            output.addFloat64(cX); // Points right
            output.addFloat64(cY); // Points down
        }
        else
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg, red, pxX, pxY,
            objects[i].width / 2, objects[i].height / 2);
        }
    }

    return outYarpImg;


}



/************************************************************************/

}  // namespace roboticslab
