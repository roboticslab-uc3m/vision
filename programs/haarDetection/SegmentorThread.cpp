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

    fx_d = DEFAULT_FX_D;
    fy_d = DEFAULT_FY_D;
    cx_d = DEFAULT_CX_D;
    cy_d = DEFAULT_CY_D;

    int rateMs = DEFAULT_RATE_MS;

    std::string xmlCascade = DEFAULT_XMLCASCADE;

    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("SegmentorThread options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");

        printf("\t--fx_d (default: \"%f\")\n",fx_d);
        printf("\t--fy_d (default: \"%f\")\n",fy_d);
        printf("\t--cx_d (default: \"%f\")\n",cx_d);
        printf("\t--cy_d (default: \"%f\")\n",cy_d);
        printf("\t--rateMs (default: \"%d\")\n",rateMs);
        printf("\t--xmlCascade [file.xml] (default: \"%s\")\n", xmlCascade.c_str());
        // Do not exit: let last layer exit so we get help from the complete chain.
    }

    if (rf.check("fx_d")) fx_d = rf.find("fx_d").asDouble();
    if (rf.check("fy_d")) fy_d = rf.find("fy_d").asDouble();
    if (rf.check("cx_d")) cx_d = rf.find("cx_d").asDouble();
    if (rf.check("cy_d")) cy_d = rf.find("cy_d").asDouble();

    printf("SegmentorThread using fx_d: %f, fy_d: %f, cx_d: %f, cy_d: %f.\n",
        fx_d,fy_d,cx_d,cy_d);



    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt();
    if (rf.check("xmlCascade")) xmlCascade = rf.find("xmlCascade").asString();

    printf("--------------------------------------------------------------\n");
    if(rf.check("help")) {
        ::exit(1);
    }

    std::string cascade = rf.findFileByName(xmlCascade);
    if( ! face_cascade.load( cascade ) ) {
        printf("[error] no cascade!\n");
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
                                         IPL_DEPTH_8U, 1 );
    cvCvtColor((IplImage*)colorFrame.getIplImage(), inIplImage, CV_RGB2GRAY);
    cv::Mat inCvMat( cv::cvarrToMat(inIplImage) );

    std::vector<cv::Rect> faces;
    //face_cascade.detectMultiScale( inCvMat, faces, 1.1, 2, 0, Size(70, 70));
    face_cascade.detectMultiScale( inCvMat, faces, 1.1, 2, 0|CV_HAAR_SCALE_IMAGE, cv::Size(30, 30) );

    yarp::sig::ImageOf<yarp::sig::PixelRgb> outYarpImg;
    outYarpImg.copy(colorFrame);
    yarp::sig::PixelRgb red(255,0,0);
    yarp::sig::PixelRgb green(0,255,0);
    yarp::os::Bottle output;

    double minZ = 999999;
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

            output.addDouble( - mmX_tmp );  // Points right thanks to change sign so (x ^ y = z). Expects --noMirror.
            output.addDouble( mmY_tmp );    // Points down.
            output.addDouble( mmZ_tmp );    // Points forward.
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
        pOutPort->write(output);

}

}  // namespace roboticslab
