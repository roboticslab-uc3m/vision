// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "Transformation.hpp"

#include <fstream>

namespace roboticslab
{

// -----------------------------------------------------------------------------


HaarDetectionTransformation::HaarDetectionTransformation(yarp::os::Searchable* parameters)
{
    if(!parameters->check("switchMode"))
    {
      CD_ERROR("**** \"context\" parameter for HaarDetectionTransformation NOT found\n");
      return;
    }
    std::string context = parameters->find("swicthMode").asString();

    if(!parameters->check("xmlCascade"))
    {
        CD_ERROR("**** \"xmlCascade\" parameter for HaarDetectionTransformation NOT found\n");
        return;
    }
    std::string xmlCascade = parameters->find("xmlCascade").asString();
    CD_DEBUG("**** \"xmlCascade\" parameter for HaarDetectionTransformation found: \"%s\"\n", xmlCascade.c_str());

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

    valid = true;
}

// -----------------------------------------------------------------------------

double HaarDetectionTransformation::transform(const double value)
{
    return value * m + b;
}

// -----------------------------------------------------------------------------


ColorRegionDetectionTransformation::ColorRegionDetectionTransformation(yarp::os::Searchable* parameters)
{
    if(!parameters->check("context"))
    {
     CD_ERROR("**** \"context\" parameter for HaarDetectionTransformation NOT found\n");
     return;
    }
    std::string context = parameters->find("context").asString();

    if(!parameters->check("algorithm"))
    {
        CD_ERROR("**** \"algorithm\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    std::string algorithm = parameters->find("algorithm").asString();

    if(!parameters->check("locate"))
    {
        CD_ERROR("**** \"locate\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    std::string locate = parameters->find("locate").asString();

    if(!parameters->check("maxNumBlobs"))
    {
        CD_ERROR("**** \"maxNumBlobs\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    int maxNumBlobs = parameters->find("maxNumBlobs").asInt32();

    if(!parameters->check("morphClosing"))
    {
        CD_ERROR("**** \"morphClosing\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    double morphClosing = parameters->find("morphClosing").asFloat64();

    if(!parameters->check("outFeaturesFormat"))
    {
        CD_ERROR("**** \"outFeaturesFormat\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    int outFeaturesFormat = parameters->find("outFeaturesFormat").asInt32();

    if(!parameters->check("outImage"))
    {
        CD_ERROR("**** \"outImage\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    int outImage = parameters->find("outImage").asInt32();

    if(!parameters->check("threshold"))
    {
        CD_ERROR("**** \"threshold\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    int threshold = parameters->find("threshold").asInt32();

    if(!parameters->check("seeBounding"))
    {
        CD_ERROR("**** \"seeBounding\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
    int seeBounding = parameters->find("seeBounding").asInt32();

    if(!parameters->check("outFeatures"))
    {
        CD_ERROR("**** \"outFeatures\" parameter for ColorRegionDetectionTransformation NOT found\n");
        return;
    }
  //  yarp::os::Bottle outFeatures = parameters->find("outFeatures").asList();


    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext(context);

    valid = true;
}

// -----------------------------------------------------------------------------

double ColorRegionDetectionTransformation::transform(const double value)
{
    return value * m + b;
}

// -----------------------------------------------------------------------------


TensorflowDetectionTransformation::TensorflowDetectionTransformation(yarp::os::Searchable* parameters)
{

    if(!parameters->check("context"))
    {
      CD_ERROR("**** \"context\" parameter for HaarDetectionTransformation NOT found\n");
      return;
    }
    std::string context = parameters->find("context").asString();

    if(!parameters->check("trainedModel"))
    {
        CD_ERROR("**** \"trainedModel\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }
    std::string trainedModel = parameters->find("trainedModel").asString();
    CD_DEBUG("**** \"trainedModel\" parameter for TensorflowDetectionTransformation found: \"%s\"\n", trainedModel.c_str());

    if(!parameters->check("trainedModelLabels"))
    {
        CD_ERROR("**** \"trainedModelLabels\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }
    std::string trainedModelLabels = parameters->find("trainedModelLabels").asString();
    CD_DEBUG("**** \"trainedModelLabels\" parameter for TensorflowDetectionTransformation found: \"%s\"\n", trainedModelLabels.c_str());


    if(!parameters->check("context"))
    {
        CD_ERROR("**** \"context\" parameter for TensorflowDetectionTransformation NOT found\n");
        return;
    }

    yarp::os::ResourceFinder rf;
    rf.setVerbose(false);
    rf.setDefaultContext(context);

    std::string trainedModelFullName = rf.findFileByName(trainedModel);

    if(trainedModelFullName.empty())
    {
        CD_ERROR("**** full path for file NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for file found: \"%s\"\n", trainedModelFullName.c_str());

    std::string trainedModelLabelsFullName = rf.findFileByName(trainedModelLabels);
    if(trainedModelLabelsFullName.empty())
    {
        CD_ERROR("**** full path for file NOT found\n");
        return;
    }
    CD_DEBUG("**** full path for file found: \"%s\"\n", trainedModelLabelsFullName.c_str());



    valid = true;
}

// -----------------------------------------------------------------------------

double TensorflowDetectionTransformation::transform(const double value)
{
    return value * m + b;
}

// -----------------------------------------------------------------------------

} // namespace roboticslab
