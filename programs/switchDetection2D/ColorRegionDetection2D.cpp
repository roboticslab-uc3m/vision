// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "ColorRegionDetection2D.hpp"
#include "SegmentorThread.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <string>
#include <vector>

#include <yarp/os/Bottle.h>
#include <yarp/sig/ImageDraw.h>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <ColorDebug.h>

using namespace std;

namespace roboticslab
{

/*****************************************************************/
void ColorRegionDetection2D::run(yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg, std::string algorithm, std::string locate, double morphClosing, int maxNumBlobs, int threshold) {



/***************************/

  outImage = DEFAULT_OUT_IMAGE;
  outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
  outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
  int rateMs = DEFAULT_RATE_MS;
  seeBounding = DEFAULT_SEE_BOUNDING;


//********************


    // {yarp ImageOf Rgb -> openCv Mat Bgr}
   IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg.width(), inYarpImg.height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)inYarpImg.getIplImage(), inIplImage, CV_RGB2BGR);
    Mat inCvMat = cvarrToMat(inIplImage);

    // Because Travis stuff goes with [openCv Mat Bgr] for now
        Travis travis(false,true);    // ::Travis(quiet=true, overwrite=true);
        travis.setCvMat(inCvMat);
        if(algorithm=="hue") travis.binarize("hue", threshold-5,threshold+5);
        else if(algorithm=="canny") travis.binarize("canny");
        else travis.binarize(algorithm.c_str(), threshold);
        travis.morphClosing(inYarpImg.width() * morphClosing / 100.0 );
        int numBlobs = travis.blobize(maxNumBlobs);
        if( 0 == numBlobs )
        {
            travis.release();
            return;
        }
        vector<cv::Point2d> blobsXY;
        if( ! travis.getBlobsXY(blobsXY) )
        {
            travis.release();
            return;
        }
        vector<double> blobsAngle,blobsArea,blobsAspectRatio,blobsAxisFirst,blobsAxisSecond,blobsPerimeter;
        vector<double> blobsRectangularity,blobsSolidity;
        vector<double> blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev;
        travis.getBlobsArea(blobsArea);
        travis.getBlobsPerimeter(blobsPerimeter);
        travis.getBlobsSolidity(blobsSolidity);
        travis.getBlobsHSV(blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev);
        if( ! travis.getBlobsAngle(0,blobsAngle) )  // method: 0=box, 1=ellipse; note check for return as 1 can break
        {
            travis.release();
            return;
        }
        travis.getBlobsAspectRatio(blobsAspectRatio,blobsAxisFirst,blobsAxisSecond);  // must be called after getBlobsAngle!!!!
        travis.getBlobsRectangularity(blobsRectangularity);  // must be called after getBlobsAngle!!!!
        Mat outCvMat = travis.getCvMat(outImage,seeBounding);
        travis.release();

        // { openCv Mat Bgr -> yarp ImageOf Rgb}
        IplImage outIplImage = outCvMat;
        cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);
        char sequence[] = "RGB";
        strcpy (outIplImage.channelSeq,sequence);
        ImageOf<PixelRgb> outYarpImg;
        outYarpImg.wrapIplImage(&outIplImage);
        PixelRgb blue(0,0,255);
        for( int i = 0; i < blobsXY.size(); i++)
           addCircle(outYarpImg,blue,blobsXY[i].x,blobsXY[i].y,3);
        outImageProcessed=outYarpImg;

        //return outYarpImg; crea conflicto con return; de funciones travis


    // Take advantage we have the travis object and get features for text output
    yarp::os::Bottle output;
    for (int elem = 0; elem < outFeatures.size() ; elem++) {
        if ( outFeatures.get(elem).asString() == "locX" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].x);
            } else {
                yarp::os::Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(blobsXY[i].x);
                output.addList() = locXs;
            }
        } else if ( outFeatures.get(elem).asString() == "locY" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].y);
            } else {
                yarp::os::Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(blobsXY[i].y);
                output.addList() = locYs;
            }
        } else if ( outFeatures.get(elem).asString() == "angle" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAngle[0]);
            } else {
                yarp::os::Bottle angles;
                for (int i = 0; i < blobsAngle.size(); i++)
                    angles.addFloat64(blobsAngle[i]);
                output.addList() = angles;
            }
        } else if ( outFeatures.get(elem).asString() == "area" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsArea[0]);
            } else {
                yarp::os::Bottle areas;
                for (int i = 0; i < blobsArea.size(); i++)
                    areas.addFloat64(blobsArea[i]);
                output.addList() = areas;
            }
        } else if ( outFeatures.get(elem).asString() == "perimeter" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsPerimeter[0]);
            } else {
                yarp::os::Bottle areas;
                for (int i = 0; i < blobsPerimeter.size(); i++)
                    areas.addFloat64(blobsPerimeter[i]);
                output.addList() = areas;
            }
        } else if ( outFeatures.get(elem).asString() == "aspectRatio" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAspectRatio[0]);
            } else {
                yarp::os::Bottle aspectRatios;
                for (int i = 0; i < blobsAspectRatio.size(); i++)
                    aspectRatios.addFloat64(blobsAspectRatio[i]);
                output.addList() = aspectRatios;
            }
        } else if ( outFeatures.get(elem).asString() == "rectangularity" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsRectangularity[0]);
            } else {
                yarp::os::Bottle rectangularities;
                for (int i = 0; i < blobsRectangularity.size(); i++)
                    rectangularities.addFloat64(blobsRectangularity[i]);
                output.addList() = rectangularities;
            }
        } else if ( outFeatures.get(elem).asString() == "axisFirst" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisFirst[0]);
            } else {
                yarp::os::Bottle axisFirsts;
                for (int i = 0; i < blobsAxisFirst.size(); i++)
                    axisFirsts.addFloat64(blobsAxisFirst[i]);
                output.addList() = axisFirsts;
            }
        } else if ( outFeatures.get(elem).asString() == "axisSecond" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisSecond[0]);
            } else {
                yarp::os::Bottle axisSeconds;
                for (int i = 0; i < blobsAxisSecond.size(); i++)
                    axisSeconds.addFloat64(blobsAxisSecond[i]);
                output.addList() = axisSeconds;
            }
        } else if ( outFeatures.get(elem).asString() == "solidity" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSolidity[0]);
            } else {
                yarp::os::Bottle solidities;
                for (int i = 0; i < blobsSolidity.size(); i++)
                    solidities.addFloat64(blobsSolidity[i]);
                output.addList() = solidities;
            }
        } else if ( outFeatures.get(elem).asString() == "hue" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHue[0]);
            } else {
                yarp::os::Bottle hues;
                for (int i = 0; i < blobsHue.size(); i++)
                    hues.addFloat64(blobsHue[i]);
                output.addList() = hues;
            }
        } else if ( outFeatures.get(elem).asString() == "sat" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSat[0]);
            } else {
                yarp::os::Bottle sats;
                for (int i = 0; i < blobsSat.size(); i++)
                    sats.addFloat64(blobsSat[i]);
                output.addList() = sats;
            }
        } else if ( outFeatures.get(elem).asString() == "val" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsVal[0]);
            } else {
                yarp::os::Bottle vals;
                for (int i = 0; i < blobsVal.size(); i++)
                    vals.addFloat64(blobsVal[i]);
                output.addList() = vals;
            }
        } else if ( outFeatures.get(elem).asString() == "hueStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHueStdDev[0]);
            } else {
                yarp::os::Bottle hueStdDevs;
                for (int i = 0; i < blobsHueStdDev.size(); i++)
                    hueStdDevs.addFloat64(blobsHueStdDev[i]);
                output.addList() = hueStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "satStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSatStdDev[0]);
            } else {
                yarp::os::Bottle satStdDevs;
                for (int i = 0; i < blobsSatStdDev.size(); i++)
                    satStdDevs.addFloat64(blobsSatStdDev[i]);
                output.addList() = satStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "valStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsValStdDev[0]);
            } else {
                yarp::os::Bottle valStdDevs;
                for (int i = 0; i < blobsValStdDev.size(); i++)
                    valStdDevs.addFloat64(blobsValStdDev[i]);
                output.addList() = valStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "time" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(yarp::os::Time::now());
            } else {
                yarp::os::Bottle times;
                for (int i = 0; i < blobsArea.size(); i++)
                    times.addFloat64(yarp::os::Time::now());
                output.addList() = times;
            }
        } else fprintf(stderr,"[SegmentorThread] warning: bogus outFeatures.\n");
    }

    outputProcessed=output;
    //pOutPort->write(output);

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();  // cvReleaseImage( &outIplImage );  // release the memory for the image

    //return outYarpImg;
}

/************************************************************************/

}  // namespace roboticslab
