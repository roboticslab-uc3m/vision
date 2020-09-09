// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

#include <iostream>

#include <yarp/os/Time.h>

namespace roboticslab
{

/************************************************************************/
void SegmentorThread::setIRGBDSensor(yarp::dev::IRGBDSensor *_iRGBDSensor) {
    iRGBDSensor = _iRGBDSensor;
}

/************************************************************************/
void SegmentorThread::setOutImg(yarp::os::BufferedPort<yarp::sig::ImageOf<yarp::sig::PixelMono16> > * _pOutImg) {
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
    //VoxelOccupancy
    searchAreaDilatation=DEFAULT_SEARCH_AREA_DILATATION;
    depthLowThreshold=DEFAULT_DEPTH_LOW_THRESHOLD;
    depthHighThreshold=DEFAULT_DEPTH_HIGH_THRESHOLD;
    occupancyThreshold=DEFAULT_OCCUPANCY_THRESHOLD;
    voxelResolution=DEFAULT_VOXEL_RESOLUTION;
    utilityDepthLowThreshold=DEFAULT_UTILITY_DEPTH_LOW_THRESHOLD;
    utilityDepthHighThreshold=DEFAULT_UTILITY_DEPTH_HIGH_THRESHOLD;
    numberUtilityVoxels=DEFAULT_NUMBER_UTILITY_VOXELS;
    lowWThreshold= DEFAULT_LOW_W_THRESHOLD;
    highWThreshold= DEFAULT_HIGH_W_THRESHOLD;
    lowHThreshold= DEFAULT_LOW_H_THRESHOLD;
    highHThreshold= DEFAULT_HIGH_H_THRESHOLD;


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

    /***********************Filter List Initialization*****************************************/
//    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth = kinect->getDepthFrame();
//    if (depth.height()<10) {
//        //printf("No depth yet...\n");
//        return;
//    };

//    int H=depth.height(); //Height resolution
//    int W=depth.width();

//    std::vector<int> col(W,0);

//    for(int i=0; i<H;i++)
//    {
//        filter_list.push_back(col);
//    }

//    printf("Calibrating..................");
    /***********************************************************************************/

    // Wait for the first few frames to arrive. We kept receiving invalid pixel codes
    // from the depthCamera device if started straight away.
    yarp::os::Time::delay(1);

    this->setPeriod(rateMs * 0.001);
    this->start();

}

/************************************************************************/
void SegmentorThread::run() {
    // printf("[SegmentorThread] run()\n");

    yarp::sig::ImageOf<yarp::sig::PixelFloat> depth;
    if (!iRGBDSensor->getDepthImage(depth)) {
        printf("No depth yet...\n");
        return;
    };

    //Camera Resolutions:
    int H=depth.height(); //Height resolution
    int W=depth.width();
    //printf("H is %d: ", H);
    //printf("W is %d: ", W);
    

    std::vector<int> occupancy_indices;

    //Yarp Bottle
    yarp::os::Bottle output;
    output.clear(); //Clear bottle

    //Define bounds
    int lowH= floor(lowHThreshold*H);
    int highH= ceil(highHThreshold*H);
    int lowW= floor(lowWThreshold*W);
    int highW= ceil(highWThreshold*W);

    //"Explore" loop
    for(int i=lowH; i<highH; i++){ //Camera H umbral (lowHThreshold,highHThreshold)H. The region of search is something like a horizontal line.
        for(int j=lowW; j<highW;j++){

            //double x = (j-W/2);

            //We have 4 voxel.
            int ix=(highW-lowW)/voxelResolution;
            float depthRegion=depthHighThreshold-depthLowThreshold;
            //yarp::os::Time::delay();

            //check depth
            if(depthLowThreshold<depth.pixel(j,i) && depth.pixel(j,i)<depthHighThreshold){
                //Calculate the number of occupancy pixels around that pixel.
                //std::cout<<"Entre con una depth de "<<depth.pixel(j,i)<<" en "<<depthLowThreshold<<" y "<<depthHighThreshold<<std::endl;
                int numberOccupancyIndices=0;

                //Define a search area to see the occupancy around that pixel.
                int lowWOcCheck=j-searchAreaDilatation;
                int highWOcCheck=j+searchAreaDilatation;
                int lowHOcCheck=i-searchAreaDilatation;
                int highHOcCheck=i+searchAreaDilatation;
                if(lowWOcCheck<0){
                    lowWOcCheck=0;
                }
                if(highWOcCheck>W){
                    highWOcCheck=W;
                }
                if(lowHOcCheck<0){
                    lowHOcCheck=0;
                }
                if(highHOcCheck>H){
                    highHOcCheck=H;
                }
                //std::cout<<"Los bordes son "<<depthHighThreshold<<"y por abajo "<<depthLowThreshold<<std::endl;
                //Check area around detected pixel for pixel occupancy.
                for(int k=lowHOcCheck; k<highHOcCheck; k++){
                    for(int l=lowWOcCheck; l<highWOcCheck;l++){
                        if(depth.pixel(l,k)<depthHighThreshold && depth.pixel(l,k)>depthLowThreshold){
                            numberOccupancyIndices++;
                        }
                    }
                }

                //If we have more occupancy pixels than the threshold, that pixel is considered occupied.
                if(numberOccupancyIndices>occupancyThreshold){
                    //????pOutPort->write(output);
                    //Find the voxel it belongs
                    for(int c=0;c<voxelResolution;c++){
                        if(j<(lowW+(c+1)*ix)){ //Find Voxel_column
                            for(int r=0; r<voxelResolution; r++){
                                if(depth.pixel(j,i)<((float(r+1)/voxelResolution)*depthRegion+depthLowThreshold)){ //Find voxel row
                                    //std::cout<<"Con una depth de "<<depth.pixel(j,i)<<" entre en "<<r<<std::endl;
                                    output.clear(); //Clear bottle
                                    // Send it out
                                    output.addInt32(c);
                                    output.addInt32(r);
                                    pOutPort->write(output);
                                    yarp::os::Time::delay(0.1);
                                    std::cout<<"!!!!!!!!!!!!!!!!!!!!!ENTRE EN EL VOXEL"<<c<<" "<<r<<std::endl;
                                    return;
                                }
                            }
                        }
                    }
                }

                //If not enough occupancy pixels. The region is saved for later removal of search area.
                else{
                    //occupancy_indices.push_back(i);
                    occupancy_indices.push_back(j);
                }

            }//END_IF SEARCH_AREA


            //Is inside the utility area
            else if(utilityDepthLowThreshold<depth.pixel(j,i) && depth.pixel(j,i)<utilityDepthHighThreshold){
                //Calculate the number of occupancy pixels around that pixel.
                int numberOccupancyIndices=0;

                //Define a search area to see the occupancy around that pixel.
                int lowWOcCheck=j-searchAreaDilatation;
                int highWOcCheck=j+searchAreaDilatation;
                int lowHOcCheck=i-searchAreaDilatation;
                int highHOcCheck=i+searchAreaDilatation;
                if(lowWOcCheck<0){
                    lowWOcCheck=0;
                }
                if(highWOcCheck>W){
                    highWOcCheck=W;
                }
                if(lowHOcCheck<0){
                    lowHOcCheck=0;
                }
                if(highHOcCheck>H){
                    highHOcCheck=H;
                }
                //std::cout<<"Los bordes son "<<depthHighThreshold<<"y por abajo "<<depthLowThreshold<<std::endl;
                //Check area around detected pixel for pixel occupancy.
                for(int k=lowHOcCheck; k<highHOcCheck; k++){
                    for(int l=lowWOcCheck; l<highWOcCheck;l++){
                        if(depth.pixel(l,k)<depthHighThreshold && depth.pixel(l,k)>depthLowThreshold){
                            numberOccupancyIndices++;
                        }
                    }
                }

                //If we have more occupancy pixels than the threshold, that pixel is considered occupied.
                if(numberOccupancyIndices>occupancyThreshold){

                    int uix=(highW-lowW)/numberUtilityVoxels;
                    //pOutPort->write(output);

                    for(int c=0;c<numberUtilityVoxels;c++){
                        if(j<(lowW+(c+1)*uix)){ //Voxel_column
                            std::cout<<"UTILITY VOXEL "<<std::endl;
                            output.clear(); //Clear bottle
                            output.addInt32(c);
                            output.addInt32(voxelResolution);
                            pOutPort->write(output);
                            yarp::os::Time::delay(0.1);
                            std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!ENTRE EN EL VOXEL"<<" "<<c<<voxelResolution<<std::endl;
                            return;
                        }
                    }
                }
            }
        }

        //*****Delete the area pixels that have already been searched without success*****
        if(!occupancy_indices.empty()){
            for(int m=0; m<(occupancy_indices.size()-1); m++){
                for(int k=lowH; k<highH; k++){
                    for(int l=occupancy_indices[m]-searchAreaDilatation; l<occupancy_indices[m]+searchAreaDilatation;l++){
                        //Reset all the pixels in the areas already searched
                        depth.pixel(l,k)=0;
                    }
                }
            }
        }
        /********************************************************************************/
    }

      //The area pixels are (60cm from kinect) H:107 (20cm 2/5 Height) H:130 (25cm 1/2Height) Weidth:all (320 pix, 68cm).
    }

}  // namespace roboticslab
