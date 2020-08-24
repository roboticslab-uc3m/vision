// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

#include <opencv2/core/core_c.h>
#include <opencv2/imgproc/imgproc_c.h>

using namespace std;

namespace roboticslab
{

/************************************************************************/

void SegmentorThread::setInImg(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pInImg)
{
    pInImg = _pInImg;
}

/************************************************************************/

void SegmentorThread::setOutImg(BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/
void SegmentorThread::setOutPort(Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/
void SegmentorThread::init(ResourceFinder &rf) {

    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    outImage = DEFAULT_OUT_IMAGE;
    outFeatures.fromString(DEFAULT_OUT_FEATURES);  // it's a bottle!!
    outFeaturesFormat = DEFAULT_OUT_FEATURES_FORMAT;
    int rateMs = DEFAULT_RATE_MS;
    seeBounding = DEFAULT_SEE_BOUNDING;
    threshold = DEFAULT_THRESHOLD;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("SegmentorThread options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
        printf("\t--algorithm (redMinusBlue,greenMinusRed...; default: \"%s\")\n",algorithm.c_str());
        printf("\t--locate (centroid,bottom; default: \"%s\")\n",locate.c_str());
        printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        printf("\t--outFeatures (default: \"(%s)\")\n",outFeatures.toString().c_str());
        printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        printf("\t--outImage (0=rgb,1=bw; default: \"%d\")\n",outImage);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--seeBounding (0=none,1=contour,2=box,3=both; default: \"%d\")\n",seeBounding);
        printf("\t--threshold (default: \"%d\")\n",threshold);
    }

    if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
    if (rf.check("locate")) locate = rf.find("locate").asString();
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt32();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asFloat64();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt32();
    printf("SegmentorThread using algorithm: %s, locate: %s, maxNumBlobs: %d, morphClosing: %f, outFeaturesFormat: %d.\n",
        algorithm.c_str(),locate.c_str(),maxNumBlobs,morphClosing,outFeaturesFormat);

    if (rf.check("outFeatures")) {
        outFeatures = *(rf.find("outFeatures").asList());  // simple overrride
    }
    printf("SegmentorThread using outFeatures: (%s).\n", outFeatures.toString().c_str());

    if (rf.check("outImage")) outImage = rf.find("outImage").asInt32();
    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt32();
    if (rf.check("threshold")) threshold = rf.find("threshold").asInt32();
    if (rf.check("seeBounding")) seeBounding = rf.find("seeBounding").asInt32();
    printf("SegmentorThread using outImage: %d, rateMs: %d, seeBounding: %d, threshold: %d.\n",
        outImage, rateMs, seeBounding, threshold);

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    this->setPeriod(rateMs * 0.001);
    this->start();

}

/************************************************************************/
void SegmentorThread::run() {
    //printf("[SegmentorThread] run()\n");

    ImageOf<PixelRgb> *inYarpImg = pInImg->read(false);
    if (inYarpImg==NULL) {
        //printf("No img yet...\n");
        return;
    };

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    IplImage *inIplImage = cvCreateImage(cvSize(inYarpImg->width(), inYarpImg->height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)inYarpImg->getIplImage(), inIplImage, CV_RGB2BGR);
    Mat inCvMat = cvarrToMat(inIplImage);

    // Because Travis stuff goes with [openCv Mat Bgr] for now
    Travis travis(false,true);    // ::Travis(quiet=true, overwrite=true);
    travis.setCvMat(inCvMat);
    if(algorithm=="hue") travis.binarize("hue", threshold-5,threshold+5);
    else if(algorithm=="canny") travis.binarize("canny");
    else travis.binarize(algorithm.c_str(), threshold);
    travis.morphClosing( inYarpImg->width() * morphClosing / 100.0 );
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
    IplImage outIplImage = cvIplImage(outCvMat);
    cvCvtColor(&outIplImage,&outIplImage, CV_BGR2RGB);
    char sequence[] = "RGB";
    strcpy (outIplImage.channelSeq,sequence);
    ImageOf<PixelRgb> outYarpImg;
    outYarpImg.wrapIplImage(&outIplImage);
    PixelRgb blue(0,0,255);
    for( int i = 0; i < blobsXY.size(); i++)
       addCircle(outYarpImg,blue,blobsXY[i].x,blobsXY[i].y,3);
    pOutImg->prepare() = outYarpImg;
    pOutImg->write();

    // Take advantage we have the travis object and get features for text output
    Bottle output;
    for (int elem = 0; elem < outFeatures.size() ; elem++) {
        if ( outFeatures.get(elem).asString() == "locX" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].x);
            } else {
                Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(blobsXY[i].x);
                output.addList() = locXs;
            }
        } else if ( outFeatures.get(elem).asString() == "locY" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].y);
            } else {
                Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(blobsXY[i].y);
                output.addList() = locYs;
            }
        } else if ( outFeatures.get(elem).asString() == "angle" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAngle[0]);
            } else {
                Bottle angles;
                for (int i = 0; i < blobsAngle.size(); i++)
                    angles.addFloat64(blobsAngle[i]);
                output.addList() = angles;
            }
        } else if ( outFeatures.get(elem).asString() == "area" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsArea[0]);
            } else {
                Bottle areas;
                for (int i = 0; i < blobsArea.size(); i++)
                    areas.addFloat64(blobsArea[i]);
                output.addList() = areas;
            }
        } else if ( outFeatures.get(elem).asString() == "perimeter" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsPerimeter[0]);
            } else {
                Bottle areas;
                for (int i = 0; i < blobsPerimeter.size(); i++)
                    areas.addFloat64(blobsPerimeter[i]);
                output.addList() = areas;
            }
        } else if ( outFeatures.get(elem).asString() == "aspectRatio" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAspectRatio[0]);
            } else {
                Bottle aspectRatios;
                for (int i = 0; i < blobsAspectRatio.size(); i++)
                    aspectRatios.addFloat64(blobsAspectRatio[i]);
                output.addList() = aspectRatios;
            }
        } else if ( outFeatures.get(elem).asString() == "rectangularity" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsRectangularity[0]);
            } else {
                Bottle rectangularities;
                for (int i = 0; i < blobsRectangularity.size(); i++)
                    rectangularities.addFloat64(blobsRectangularity[i]);
                output.addList() = rectangularities;
            }
        } else if ( outFeatures.get(elem).asString() == "axisFirst" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisFirst[0]);
            } else {
                Bottle axisFirsts;
                for (int i = 0; i < blobsAxisFirst.size(); i++)
                    axisFirsts.addFloat64(blobsAxisFirst[i]);
                output.addList() = axisFirsts;
            }
        } else if ( outFeatures.get(elem).asString() == "axisSecond" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsAxisSecond[0]);
            } else {
                Bottle axisSeconds;
                for (int i = 0; i < blobsAxisSecond.size(); i++)
                    axisSeconds.addFloat64(blobsAxisSecond[i]);
                output.addList() = axisSeconds;
            }
        } else if ( outFeatures.get(elem).asString() == "solidity" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSolidity[0]);
            } else {
                Bottle solidities;
                for (int i = 0; i < blobsSolidity.size(); i++)
                    solidities.addFloat64(blobsSolidity[i]);
                output.addList() = solidities;
            }
        } else if ( outFeatures.get(elem).asString() == "hue" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHue[0]);
            } else {
                Bottle hues;
                for (int i = 0; i < blobsHue.size(); i++)
                    hues.addFloat64(blobsHue[i]);
                output.addList() = hues;
            }
        } else if ( outFeatures.get(elem).asString() == "sat" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSat[0]);
            } else {
                Bottle sats;
                for (int i = 0; i < blobsSat.size(); i++)
                    sats.addFloat64(blobsSat[i]);
                output.addList() = sats;
            }
        } else if ( outFeatures.get(elem).asString() == "val" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsVal[0]);
            } else {
                Bottle vals;
                for (int i = 0; i < blobsVal.size(); i++)
                    vals.addFloat64(blobsVal[i]);
                output.addList() = vals;
            }
        } else if ( outFeatures.get(elem).asString() == "hueStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsHueStdDev[0]);
            } else {
                Bottle hueStdDevs;
                for (int i = 0; i < blobsHueStdDev.size(); i++)
                    hueStdDevs.addFloat64(blobsHueStdDev[i]);
                output.addList() = hueStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "satStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsSatStdDev[0]);
            } else {
                Bottle satStdDevs;
                for (int i = 0; i < blobsSatStdDev.size(); i++)
                    satStdDevs.addFloat64(blobsSatStdDev[i]);
                output.addList() = satStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "valStdDev" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsValStdDev[0]);
            } else {
                Bottle valStdDevs;
                for (int i = 0; i < blobsValStdDev.size(); i++)
                    valStdDevs.addFloat64(blobsValStdDev[i]);
                output.addList() = valStdDevs;
            }
        } else if ( outFeatures.get(elem).asString() == "time" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(Time::now());
            } else {
                Bottle times;
                for (int i = 0; i < blobsArea.size(); i++)
                    times.addFloat64(Time::now());
                output.addList() = times;
            }
        } else fprintf(stderr,"[SegmentorThread] warning: bogus outFeatures.\n");
    }
    pOutPort->write(output);

    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();  // cvReleaseImage( &outIplImage );  // release the memory for the image

/*  Mat mask= Mat::zeros(imageFile.rows, imageFile.cols, CV_8UC1);

    //get biggest contour
    vector <Point> biggestCont = getBiggestContour(imageFile);

    //calculate mask of image with biggest contour
    calcMask(mask,biggestCont);

    //area of mask (with convexhull)
    calcArea(area,biggestCont);

    //location xy of the centre of biggest contour
    calcLocationXY(locX,locY, biggestCont);

    //object area over rect area
    calcRectangularity(rectangularity, biggestCont);

    //mass center
    calcMassCenter(massCenterlocX,massCenterlocY,biggestCont);

    //aspect ratio width/height
    calcAspectRatio(aspectRatio, axisFirst, axisSecond, biggestCont);

    //solidity. area object / area convexhull
    calcSolidity(solidity,biggestCont);

    //hsv peaks
    calcHSVPeakColor(imageFile, mask, hue_mode, hue_peak, value_mode, value_peak);

    //hsv mean and stddev
    calcHSVMeanStdDev(imageFile, mask,
                      hue_mean, hue_stddev,
                      saturation_mean, saturation_stddev,
                      value_mean, value_stddev);

    //arc
    calcArcLength(arc,biggestCont);

    //radius
    calcCircle(radius,biggestCont);*/

    // --- ALGORITHMS FINISH SOMEWHERE HERE ---

//    cvReleaseImage( &rgb ); //release the memory for the image

/*    printf("***** FEATURES *****\n");
    printf("Area: %f\n",area);
    printf("Mass Center Location (x,y): (%f, %f)\n",massCenterlocX,massCenterlocY);
    printf("Rectangularity (areaObject/areaRectangle): %f\n",rectangularity);
    printf("Aspect Ratio (width/height): %f\n",aspectRatio);
    printf("Ellipse Axis 1: %f\n",axisFirst);
    printf("Ellipse Axis 2: %f\n",axisSecond);
    printf("Solidity (object/convexHull): %f\n",solidity);
    printf("Arc: %f\n",arc);
    printf("Radius: %f\n",radius);
    printf("Hue Peak: %f\n",hue_peak);
    printf("Hue Mean: %f\n",hue_mean);
    printf("Hue StdDev: %f\n",hue_stddev);
    printf("Saturation Peak: %f\n",saturation_peak);
    printf("Saturation Mean: %f\n",saturation_mean);
    printf("Saturation StdDev: %f\n",hue_stddev);
    printf("Value Peak: %f\n",value_peak);
    printf("Value Mean: %f\n",value_mean);
    printf("Value StdDev: %f\n",value_stddev);
    printf("Contour Location (x,y): (%f, %f)\n",locX,locY);*/

/*    Bottle b;
    b.addFloat64(massCenterlocX);  // 1
    b.addFloat64(massCenterlocY);  // 2
    b.addFloat64(aspectRatio);  // 3
    b.addFloat64(area);  // 4
    b.addFloat64(rectangularity);  // 5
    b.addFloat64(axisFirst);  // 6
    b.addFloat64(axisSecond);  // 7
    b.addFloat64(solidity);  // 8
    b.addFloat64(arc);  // 9
    b.addFloat64(radius);  // 10
    b.addFloat64(hue_peak);  // 11
    b.addFloat64(value_peak); */ // 12
    /*b.addFloat64(hue_mean);  // 13
    b.addFloat64(hue_stddev);  // 14
    b.addFloat64(saturation_peak);  // 15
    b.addFloat64(saturation_mean);  // 16
    b.addFloat64(saturation_stddev);  // 17
    b.addFloat64(value_mean);  // 18
    b.addFloat64(value_stddev);  // 19
    b.addFloat64(locX);  // 20
    b.addFloat64(locY);  // 21
    b.addFloat64(value_mode);  // 22
    b.addFloat64(hue_mode);  // 23
*/

    //pOutPort->write(b);

}

/************************************************************************/

}  // namespace roboticslab
