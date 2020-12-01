// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <yarp/conf/version.h>
#include <yarp/os/Time.h>

#include <ColorDebug.h>

#include "SegmentorThread.hpp"

#define DEFAULT_DETECTOR "HaarDetector"

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

void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor)
{
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/

void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelRgb> > * _pOutImg)
{
    pOutImg = _pOutImg;
}

/************************************************************************/

void SegmentorThread::setOutPort(yarp::os::Port * _pOutPort)
{
    pOutPort = _pOutPort;
}

/************************************************************************/

bool SegmentorThread::init(yarp::os::ResourceFinder &rf)
{
    int rateMs = DEFAULT_RATE_MS;

    yarp::os::Property depthIntrinsicParams;

    iRGBDSensor->getDepthIntrinsicParam(depthIntrinsicParams);

    fx_d = depthIntrinsicParams.find("focalLengthX").asFloat64();
    fy_d = depthIntrinsicParams.find("focalLengthY").asFloat64();
    cx_d = depthIntrinsicParams.find("principalPointX").asFloat64();
    cy_d = depthIntrinsicParams.find("principalPointY").asFloat64();

    printf("SegmentorThread options:\n");
    printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");
    printf("\t--rateMs (default: \"%d\")\n",rateMs);
    std::printf("\t--detector (default: \"%s\")\n", DEFAULT_DETECTOR);

    printf("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n",
        fx_d,fy_d,cx_d,cy_d);

    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt32();

    if(cropSelector != 0) {
        processor.reset();
        inCropSelectorPort->setReader(processor);
    }

#if YARP_VERSION_MINOR < 5
    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    // https://github.com/roboticslab-uc3m/vision/issues/88
    yarp::os::Time::delay(1);
#endif

    if(!setPeriod(rateMs * 0.001))
    {
        CD_ERROR("\n");
        return false;
    }

    if(!start())
    {
        CD_ERROR("\n");
        return false;
    }
}

/************************************************************************/
void SegmentorThread::run()
{
    //printf("[SegmentorThread] run()\n");

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

    if (!iRGBDSensor->getImages(colorFrame, depthFrame))
    {
        return;
    }

    std::vector<yarp::os::Property> detectedObjects;

    /*if (!iDetector->detect(colorFrame, detectedObjects))
    {
        CD_WARNING("Detector failed!\n");
        return;
    }*/

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    outYarpImg.copy(colorFrame);
    yarp::sig::PixelRgb red(255,0,0);
    yarp::sig::PixelRgb green(0,255,0);
    yarp::os::Bottle output;

    /*double minZ = 999999;
    int closestFace = 999999;
    for( int i = 0; i < faces.size(); i++ )
    {
        int pxX = faces[i].x+faces[i].width/2;
        int pxY = faces[i].y+faces[i].height/2;
        double depthX, depthY;
        scaleXY(colorFrame, depthFrame, pxX, pxY, &depthX, &depthY);
        double mmZ_tmp = depthFrame.pixel(int(depthX), int(depthY));

        if (mmZ_tmp < 0.001)
        {
            fprintf(stderr,"[warning] SegmentorThread run(): mmZ_tmp[%d] < 0.001.\n",i);
            cvReleaseImage( &inIplImage );  // release the memory for the image
            return;
        }

        if (mmZ_tmp < minZ) {
            minZ = mmZ_tmp;
            closestFace = i;
        }
    }

    for( int i = 0; i < faces.size(); i++ )
    {

        int pxX = faces[i].x+faces[i].width/2;
        int pxY = faces[i].y+faces[i].height/2;
        double mmZ_tmp = depthFrame.pixel(pxX,pxY);

        if (mmZ_tmp < 0.001)
        {
            fprintf(stderr,"[warning] SegmentorThread run(): mmZ_tmp[%d] < 0.001.\n",i);
            cvReleaseImage( &inIplImage );  // release the memory for the image
            return;
        }

        double mmX_tmp = 1000.0 * ( (pxX - cx_d) * mmZ_tmp/1000.0 ) / fx_d;
        double mmY_tmp = 1000.0 * ( (pxY - cy_d) * mmZ_tmp/1000.0 ) / fy_d;

        if( i == closestFace )
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg,green,faces[i].x+faces[i].width/2,faces[i].y+faces[i].height/2,
                                faces[i].width/2,faces[i].height/2);

            output.addFloat64( - mmX_tmp );  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
            output.addFloat64( mmY_tmp );    // Points down.
            output.addFloat64( mmZ_tmp );    // Points forward.
        }
        else
        {
            yarp::sig::draw::addRectangleOutline(outYarpImg,red,faces[i].x+faces[i].width/2,faces[i].y+faces[i].height/2,
                                faces[i].width/2,faces[i].height/2);
        }
    }

    pOutImg->prepare() = outYarpImg;
    pOutImg->write();
    cvReleaseImage( &inIplImage );  // release the memory for the image

    if (output.size() > 0)
        pOutPort->write(output);*/

}

}  // namespace roboticslab
