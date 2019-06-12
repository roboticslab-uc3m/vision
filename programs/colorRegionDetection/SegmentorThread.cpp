// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

#include <yarp/os/Time.h>

namespace
{
    inline void scaleXY(const yarp::sig::Image & frame1, const yarp::sig::Image & frame2, double px1, double py1, double * px2, double * py2)
    {
        if (frame1.width() != frame2.width() || frame1.height() != frame2.height())
        {
            *px2 = px1 * ((double)frame2.width() / (double)frame1.width());
            *py2 = py1 * ((double)frame2.height() / (double)frame1.height());
        }
        else
        {
            *px2 = px1;
            *py2 = py1;
        }
    }
}

namespace roboticslab
{

/************************************************************************/
void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor) {
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/
void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg) {
    pOutImg = _pOutImg;
}

/************************************************************************/
void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort) {
    pOutPort = _pOutPort;
}

/************************************************************************/
void SegmentorThread::init(yarp::os::ResourceFinder &rf) {
    yarp::os::Property rgbIntrinsicParams;
    yarp::os::Property depthIntrinsicParams;

    iRGBDSensor->getRgbIntrinsicParam(rgbIntrinsicParams);
    iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams);

    fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();

    fx_rgb = rgbIntrinsicParams.find("focalLengthX").asFloat64();
    fy_rgb = rgbIntrinsicParams.find("focalLengthY").asFloat64();
    cx_rgb = rgbIntrinsicParams.find("principalPointX").asFloat64();
    cy_rgb = rgbIntrinsicParams.find("principalPointY").asFloat64();

    algorithm = DEFAULT_ALGORITHM;
    locate = DEFAULT_LOCATE;
    maxNumBlobs = DEFAULT_MAX_NUM_BLOBS;
    morphClosing = DEFAULT_MORPH_CLOSING;
    morphOpening = DEFAULT_MORPH_OPENING;
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
        printf("\t--algorithm (default: \"%s\")\n",algorithm.c_str());
        printf("\t--locate (centroid or bottom; default: \"%s\")\n",locate.c_str());
        printf("\t--maxNumBlobs (default: \"%d\")\n",maxNumBlobs);
        printf("\t--morphClosing (percentage, 2 or 4 okay; default: \"%f\")\n",morphClosing);
        printf("\t--morphOpening (percentage, 2 or 4 okay; default: \"%f\")\n",morphOpening);
        printf("\t--outFeatures (mmX,mmY,mmZ,pxXpos,pxYpos,pxX,pxY,angle,area,aspectRatio,rectangularity,axisFirst,axisSecond \
solidity,hue,sat,val,hueStdDev,satStdDev,valStdDev,time; \
default: \"(%s)\")\n",outFeatures.toString().c_str());
        printf("\t--outFeaturesFormat (0=bottled,1=minimal; default: \"%d\")\n",outFeaturesFormat);
        printf("\t--outImage (0=rgb,1=bin; default: \"%d\")\n",outImage);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--seeBounding (0=none,1=box,2=contour,3=both; default: \"%d\")\n",seeBounding);
        printf("\t--threshold (default: \"%d\")\n",threshold);
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("algorithm")) algorithm = rf.find("algorithm").asString();
    if (rf.check("locate")) locate = rf.find("locate").asString();
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt32();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asFloat64();
    if (rf.check("morphOpening")) morphOpening = rf.find("morphOpening").asFloat64();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt32();

    printf("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n",
        fx_d,fy_d,cx_d,cy_d);
    printf("SegmentorThread using fx_rgb: %f, fy_rgb: %f, cx_rgb: %f, cy_rgb: %f.\n",
        fx_rgb,fy_rgb,cx_rgb,cy_rgb);
    printf("SegmentorThread using algorithm: %s, locate: %s.\n",
        algorithm.c_str(),locate.c_str());
    printf("SegmentorThread using maxNumBlobs: %d, morphClosing: %.2f, outFeaturesFormat: %d.\n",
        maxNumBlobs,morphClosing,outFeaturesFormat);

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

    if(cropSelector != 0) {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    yarp::os::Time::delay(1);

    this->setPeriod(rateMs * 0.001);
    this->start();
}

/************************************************************************/
void SegmentorThread::run() {
    // printf("[SegmentorThread] run()\n");

    /*ImageOf<PixelRgb> *inYarpImg = pInImg->read(false);
    ImageOf<PixelFloat> *depth = pInDepth->read(false);
    if (inYarpImg==NULL) {
        //printf("No img yet...\n");
        return;
    };
    if (depth==NULL) {
        //printf("No depth yet...\n");
        return;
    };*/

    yarp::sig::FlexImage colorFrame;
    yarp::sig::ImageOf<yarp::sig::PixelFloat> depthFrame;
    if (!iRGBDSensor->getImages(colorFrame, depthFrame)) {
        return;
    }

    // {yarp ImageOf Rgb -> openCv Mat Bgr}
    IplImage *inIplImage = cvCreateImage(cvSize(colorFrame.width(), colorFrame.height()),
                                         IPL_DEPTH_8U, 3 );
    cvCvtColor((IplImage*)colorFrame.getIplImage(), inIplImage, CV_RGB2BGR);
    cv::Mat inCvMat( cv::cvarrToMat(inIplImage) );

    // publish the original yarp img if crop selector invoked.
    if(cropSelector != 0) {
        //printf("1 x: %d, y: %d, w: %d, h: %d.\n",processor.x,processor.y,processor.w,processor.h);
        yarp::sig::ImageOf<yarp::sig::PixelRgb> inYarpImg;
        inYarpImg.copy(colorFrame);
        if( (processor.w!=0)&&(processor.h!=0)) {
            travisCrop(processor.x,processor.y,processor.w,processor.h,inCvMat);
            yarp::sig::PixelRgb green(0,255,0);
            yarp::sig::draw::addRectangleOutline(inYarpImg,green,processor.x+processor.w/2.0,processor.y+processor.h/2.0,processor.w/2.0,processor.h/2.0);
        }
        outCropSelectorImg->prepare() = inYarpImg;
        outCropSelectorImg->write();
    }

    // Because Travis stuff goes with [openCv Mat Bgr] for now
    //Travis travis;  // ::Travis(quiet=true, overwrite=true);
    Travis travis(false,true);  // ::Travis(quiet=true, overwrite=true);
    travis.setCvMat(inCvMat);
    if(algorithm=="hue") travis.binarize("hue", threshold-5,threshold+5);
    else if(algorithm=="canny") travis.binarize("canny");
    else travis.binarize(algorithm.c_str(), threshold);
    travis.morphOpening( colorFrame.width() * morphOpening / 100.0 );  // percent
    travis.morphClosing( colorFrame.width() * morphClosing / 100.0 );  // percent
    //travis.morphOpening( morphOpening );
    //travis.morphClosing( morphClosing );
    int numBlobs = travis.blobize(maxNumBlobs);
    if( 0 == numBlobs )
    {
        travis.release();
        return;
    }
    std::vector<cv::Point2d> blobsXY;
    if( ! travis.getBlobsXY(blobsXY) )
    {
        travis.release();
        return;
    }
    std::vector<double> blobsAngle,blobsArea,blobsAspectRatio,blobsAxisFirst,blobsAxisSecond;
    std::vector<double> blobsRectangularity,blobsSolidity;
    std::vector<double> blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev;
    travis.getBlobsArea(blobsArea);
    travis.getBlobsSolidity(blobsSolidity);
    travis.getBlobsHSV(blobsHue,blobsSat,blobsVal,blobsHueStdDev,blobsSatStdDev,blobsValStdDev);
    if( ! travis.getBlobsAngle(0,blobsAngle) )  // method: 0=box, 1=ellipse; note check for return as 1 can break
    {
        travis.release();
        return;
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
    std::vector<double> mmX, mmY, mmZ;
    if(blobsXY.size() < 1) {
        fprintf(stderr,"[warning] SegmentorThread run(): blobsXY.size() < 1.\n");
        //return;
    }
    for( int i = 0; i < blobsXY.size(); i++) {
        yarp::sig::draw::addCircle(outYarpImg,blue,blobsXY[i].x,blobsXY[i].y,3);
        if (blobsXY[i].x<0) {
            fprintf(stderr,"[warning] SegmentorThread run(): blobsXY[%d].x < 0.\n",i);
            //return;
            blobsXY[i].x = 0;
        }
        if (blobsXY[i].y<0) {
            fprintf(stderr,"[warning] SegmentorThread run(): blobsXY[%d].y < 0.\n",i);
            //return;
            blobsXY[i].y = 0;
        }
        // double mmZ_tmp = depth->pixel(int(blobsXY[i].x +cx_d-cx_rgb),int(blobsXY[i].y +cy_d-cy_rgb));
        double depthX, depthY;
        scaleXY(colorFrame, depthFrame, blobsXY[i].x, blobsXY[i].y, &depthX, &depthY);
        double mmZ_tmp = depthFrame.pixel(int(depthX), int(depthY));

        if (mmZ_tmp < 0.001) {
            fprintf(stderr,"[warning] SegmentorThread run(): mmZ_tmp[%d] < 0.001.\n",i);
            cvReleaseImage( &inIplImage );  // release the memory for the image
            outCvMat.release();
            return;
        }

        double mmX_tmp = 1000.0 * ( (blobsXY[i].x - cx_d) * mmZ_tmp/1000.0 ) / fx_d;
        double mmY_tmp = 1000.0 * ( (blobsXY[i].y - cy_d) * mmZ_tmp/1000.0 ) / fy_d;

        mmX.push_back( - mmX_tmp );  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
        mmY.push_back( mmY_tmp );    // Points down.
        mmZ.push_back( mmZ_tmp );    // oints forward.

    }

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();
    cvReleaseImage( &inIplImage );  // release the memory for the image
    outCvMat.release();  // cvReleaseImage( &outIplImage );  // release the memory for the image

    if ( ( blobsXY.size() < 1) && ( outFeaturesFormat == 1 ) ) return;

    // Take advantage we have the travis object and get features for text output
    yarp::os::Bottle output;
    for (int elem = 0; elem < outFeatures.size() ; elem++) {
        if ( outFeatures.get(elem).asString() == "mmX" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(mmX[0]);
            } else {
                yarp::os::Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(mmX[i]);
                output.addList() = locXs;
            }
        } else if ( outFeatures.get(elem).asString() == "mmY" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(mmY[0]);
            } else {
                yarp::os::Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(mmY[i]);
                output.addList() = locYs;
            }
        } else if ( outFeatures.get(elem).asString() == "mmZ" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(mmZ[0]);
            } else {
                yarp::os::Bottle locZs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locZs.addFloat64(mmZ[i]);
                output.addList() = locZs;
            }
        } else if ( outFeatures.get(elem).asString() == "pxXpos" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].x);
            } else {
                yarp::os::Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(blobsXY[i].x);
                output.addList() = locXs;
            }
        } else if ( outFeatures.get(elem).asString() == "pxYpos" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].y);
            } else {
                yarp::os::Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(blobsXY[i].y);
                output.addList() = locYs;
            }
        } else if ( outFeatures.get(elem).asString() == "pxX" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].x - cx_d);
            } else {
                yarp::os::Bottle locXs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locXs.addFloat64(blobsXY[i].x - cx_d);
                output.addList() = locXs;
            }
        } else if ( outFeatures.get(elem).asString() == "pxY" ) {
            if ( outFeaturesFormat == 1 ) {  // 0: Bottled, 1: Minimal
                output.addFloat64(blobsXY[0].y - cy_d);
            } else {
                yarp::os::Bottle locYs;
                for (int i = 0; i < blobsXY.size(); i++)
                    locYs.addFloat64(blobsXY[i].y - cy_d);
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
        } else {
            fprintf(stderr,"[SegmentorThread] [error] bogus outFeatures: %s\n",
                    outFeatures.get(elem).asString().c_str());
            ::exit(0);
        }
    }
    pOutPort->write(output);

}

}  // namespace roboticslab
