// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

#include "ColorRegionDetector.hpp"

namespace roboticslab
{

/*****************************************************************/

ColorRegionDetector::ColorRegionDetector(yarp::os::Searchable* parameters)
{
    algorithm = DEFAULT_ALGORITHM;
    CD_DEBUG("*** \"algorithm\" (default: \"%s\")\n", algorithm.c_str());
    if(parameters->check("algorithm"))
    {
        CD_INFO("**** \"algorithm\" parameter for ColorRegionDetectionTransformation found\n");
        algorithm = parameters->find("algorithm").asString();
    }

    locate = DEFAULT_LOCATE;
    if(parameters->check("locate"))
    {
        CD_INFO("**** \"locate\" parameter for ColorRegionDetectionTransformation found\n");
        locate = parameters->find("locate").asString();
    }

    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    if(parameters->check("maxNumBlobs"))
    {
        CD_INFO("**** \"maxNumBlobs\" parameter for ColorRegionDetectionTransformation found\n");
        maxNumBlobs = parameters->find("maxNumBlobs").asInt32();
    }

    morphClosing = DEFAULT_MORPH_CLOSING;
    if(parameters->check("morphClosing"))
    {
        CD_INFO("**** \"morphClosing\" parameter for ColorRegionDetectionTransformation found\n");
        morphClosing = parameters->find("morphClosing").asFloat64();
    }

    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    if(parameters->check("outFeaturesFormat"))
    {
        CD_INFO("**** \"outFeaturesFormat\" parameter for ColorRegionDetectionTransformation found\n");
        outFeaturesFormat = parameters->find("outFeaturesFormat").asInt32();
    }

    outImage = DEFAULT_OUT_IMAGE;
    if(parameters->check("outImage"))
    {
        CD_INFO("**** \"outImage\" parameter for ColorRegionDetectionTransformation found\n");
        outImage = parameters->find("outImage").asInt32();
    }

    threshold = DEFAULT_THRESHOLD;
    if(parameters->check("threshold"))
    {
        CD_INFO("**** \"threshold\" parameter for ColorRegionDetectionTransformation found\n");
        threshold = parameters->find("threshold").asInt32();
    }

    seeBounding = DEFAULT_SEE_BOUNDING;
    if(parameters->check("seeBounding"))
    {
        CD_INFO("**** \"seeBounding\" parameter for ColorRegionDetectionTransformation found\n");
        seeBounding = parameters->find("seeBounding").asInt32();
    }

    outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
    if(parameters->check("outFeatures"))
    {
        CD_INFO("**** \"outFeatures\" parameter for ColorRegionDetectionTransformation found\n");
        outFeatures = *(parameters->find("outFeatures").asList());  // simple overrride
    }

    printf("DetectorThread using outImage: %d, seeBounding: %d, threshold: %d.\n", outImage, seeBounding, threshold);
    printf("DetectorThread using outFeatures: (%s).\n", outFeatures.toString().c_str());
    printf("DetectorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
           algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);

    valid = true;
}

/*****************************************************************/

bool ColorRegionDetector::detect(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg,
                                 std::vector<DetectedObject*>& detectedObjects,
                                 yarp::sig::ImageOf<yarp::sig::PixelRgb>& ret)
{

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg.width(), inYarpImg.height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)inYarpImg.getIplImage(), inIplImage, CV_RGB2BGR);
    cv::Mat inCvMat = cv::cvarrToMat(inIplImage);

    // Because Travis stuff goes with [openCv Mat Bgr] for now
    Travis travis(false,true);    // ::Travis(quiet=true, overwrite=true);
    travis.setCvMat(inCvMat);
    if (algorithm=="hue")
        travis.binarize("hue", threshold-5,threshold+5);
    else if(algorithm=="canny")
        travis.binarize("canny");
    else
        travis.binarize(algorithm.c_str(), threshold);
    travis.morphClosing(inYarpImg.width() * morphClosing / 100.0 );
    int numBlobs = travis.blobize(maxNumBlobs);
    if( 0 == numBlobs )
    {
        travis.release();
        return false;
    }
    std::vector<cv::Point2d> blobsXY;
    if( ! travis.getBlobsXY(blobsXY) )
    {
        travis.release();
        return false;
    }
    std::vector<double> blobsAngle,blobsArea,blobsAspectRatio,blobsAxisFirst,blobsAxisSecond,blobsPerimeter;
    std::vector<double> blobsRectangularity,blobsSolidity;
    std::vector<double> blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev;
    travis.getBlobsArea(blobsArea);
    travis.getBlobsPerimeter(blobsPerimeter);
    travis.getBlobsSolidity(blobsSolidity);
    travis.getBlobsHSV(blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev);
    if( ! travis.getBlobsAngle(0,blobsAngle) )  // method: 0=box, 1=ellipse; note check for return as 1 can break
    {
        travis.release();
        return false;
    }
    travis.getBlobsAspectRatio(blobsAspectRatio,blobsAxisFirst,blobsAxisSecond);  // must be called after getBlobsAngle!!!!
    travis.getBlobsRectangularity(blobsRectangularity);  // must be called after getBlobsAngle!!!!
    cv::Mat outCvMat = travis.getCvMat(outImage,seeBounding);
    travis.release();

    // { openCv Mat Bgr -> yarp ImageOf Rgb}
    IplImage outIplImage = outCvMat;
    cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);
    char sequence[] = "RGB";
    strcpy (outIplImage.channelSeq,sequence);
    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    outYarpImg.wrapIplImage(&outIplImage);
    yarp::sig::PixelRgb blue(0,0,255);
    for( int i = 0; i < blobsXY.size(); i++)
        yarp::sig::draw::addCircle(outYarpImg,blue,blobsXY[i].x,blobsXY[i].y,3);
    outImageProcessed=outYarpImg;

    //return outYarpImg; crea conflicto con return; de funciones travis

    // Take advantage we have the travis object and get features for text output
    yarp::os::Bottle output;
    for (int elem = 0; elem < outFeatures.size() ; elem++)
    {
        if ( outFeatures.get(elem).asString() == "locX" )
        {
            if ( outFeaturesFormat == 1 )
            {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].x);
            }
            else
            {
                yarp::os::Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(blobsXY[i].x);
                output.addList() = locXs;
            }
        }
        else if ( outFeatures.get(elem).asString() == "locY" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].y);
            }
            else
            {
                yarp::os::Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(blobsXY[i].y);
                output.addList() = locYs;
            }
        }
        else if ( outFeatures.get(elem).asString() == "angle" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAngle[0]);
            }
            else
            {
                yarp::os::Bottle angles;
                for (int i = 0; i < blobsAngle.size(); i++)
                    angles.addFloat64(blobsAngle[i]);
                output.addList() = angles;
            }
        }
        else if ( outFeatures.get(elem).asString() == "area" )
        {
            if ( outFeaturesFormat == 1 )
            {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsArea[0]);
            }
            else
            {
                yarp::os::Bottle areas;
                for (int i = 0; i < blobsArea.size(); i++)
                    areas.addFloat64(blobsArea[i]);
                output.addList() = areas;
            }
        }
        else if ( outFeatures.get(elem).asString() == "perimeter" )
        {
            if ( outFeaturesFormat == 1 )
            {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsPerimeter[0]);
            }
            else
            {
                yarp::os::Bottle areas;
                for (int i = 0; i < blobsPerimeter.size(); i++)
                    areas.addFloat64(blobsPerimeter[i]);
                output.addList() = areas;
            }
       }
        else if ( outFeatures.get(elem).asString() == "aspectRatio" )
       {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAspectRatio[0]);
            }
            else
            {
                yarp::os::Bottle aspectRatios;
                for (int i = 0; i < blobsAspectRatio.size(); i++)
                    aspectRatios.addFloat64(blobsAspectRatio[i]);
                    output.addList() = aspectRatios;
            }
        }
        else if ( outFeatures.get(elem).asString() == "rectangularity" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsRectangularity[0]);
            }
            else
            {
                yarp::os::Bottle rectangularities;
                for (int i = 0; i < blobsRectangularity.size(); i++)
                    rectangularities.addFloat64(blobsRectangularity[i]);
                output.addList() = rectangularities;
            }
        }
        else if ( outFeatures.get(elem).asString() == "axisFirst" )
        {
            if ( outFeaturesFormat == 1 )
            {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisFirst[0]);
            }
            else
            {
                yarp::os::Bottle axisFirsts;
                for (int i = 0; i < blobsAxisFirst.size(); i++)
                    axisFirsts.addFloat64(blobsAxisFirst[i]);
                    output.addList() = axisFirsts;
            }
        }
        else if ( outFeatures.get(elem).asString() == "axisSecond" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisSecond[0]);
            }
            else
            {
                yarp::os::Bottle axisSeconds;
                for (int i = 0; i < blobsAxisSecond.size(); i++)
                    axisSeconds.addFloat64(blobsAxisSecond[i]);
                output.addList() = axisSeconds;
            }
        }
        else if ( outFeatures.get(elem).asString() == "solidity" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSolidity[0]);
            }
            else
            {
                yarp::os::Bottle solidities;
                for (int i = 0; i < blobsSolidity.size(); i++)
                    solidities.addFloat64(blobsSolidity[i]);
                output.addList() = solidities;
            }
      }
        else if ( outFeatures.get(elem).asString() == "hue" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHue[0]);
            }
            else
            {
                yarp::os::Bottle hues;
                for (int i = 0; i < blobsHue.size(); i++)
                    hues.addFloat64(blobsHue[i]);
                output.addList() = hues;
            }
        }
        else if ( outFeatures.get(elem).asString() == "sat" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSat[0]);
            }
            else
            {
                yarp::os::Bottle sats;
                for (int i = 0; i < blobsSat.size(); i++)
                    sats.addFloat64(blobsSat[i]);
                output.addList() = sats;
            }
        }
        else if ( outFeatures.get(elem).asString() == "val" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsVal[0]);
            } else
            {
                yarp::os::Bottle vals;
                for (int i = 0; i < blobsVal.size(); i++)
                    vals.addFloat64(blobsVal[i]);
                    output.addList() = vals;
            }
      }
        else if ( outFeatures.get(elem).asString() == "hueStdDev" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHueStdDev[0]);
            }
            else
            {
                yarp::os::Bottle hueStdDevs;
                for (int i = 0; i < blobsHueStdDev.size(); i++)
                    hueStdDevs.addFloat64(blobsHueStdDev[i]);
                output.addList() = hueStdDevs;
            }
        }
        else if ( outFeatures.get(elem).asString() == "satStdDev" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSatStdDev[0]);
            }
            else
            {
                yarp::os::Bottle satStdDevs;
                for (int i = 0; i < blobsSatStdDev.size(); i++)
                    satStdDevs.addFloat64(blobsSatStdDev[i]);
                output.addList() = satStdDevs;
            }
       }
        else if ( outFeatures.get(elem).asString() == "valStdDev" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(blobsValStdDev[0]);
            }
            else
            {
                yarp::os::Bottle valStdDevs;
                for (int i = 0; i < blobsValStdDev.size(); i++)
                    valStdDevs.addFloat64(blobsValStdDev[i]);
                output.addList() = valStdDevs;
            }
        }
        else if ( outFeatures.get(elem).asString() == "time" )
        {
            if ( outFeaturesFormat == 1 )
            {
                // 0: Bottled, 1: Minimal
                output.addFloat64(yarp::os::Time::now());
            }
            else
            {
                yarp::os::Bottle times;
                for (int i = 0; i < blobsArea.size(); i++)
                    times.addFloat64(yarp::os::Time::now());
                output.addList() = times;
            }
        }
        else fprintf(stderr,"[DetectorThread] warning: bogus outFeatures.\n");
    }

    outputProcessed=output;
    //pOutPort->write(output);

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();  // cvReleaseImage( &outIplImage );  // release the memory for the image

    ret = outYarpImg;
    return true;
}

/************************************************************************/

}  // namespace roboticslab
