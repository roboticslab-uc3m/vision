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
    areaLowThreshold=DEFAULT_AREA_LOW_THRESHOLD;
    areaHighThreshold=DEFAULT_AREA_HIGH_THRESHOLD;
    occupancyThreshold=DEFAULT_OCCUPANCY_THRESHOLD;
    RGBDCalibrationValue=DEFAULT_CALIBRATION_VALUE_KINECT;
    lowXBox=DEFAULT_LOW_X_BOX_VALUE;
    highXBox=DEFAULT_HIGH_X_BOX_VALUE;
    lowYBox=DEFAULT_LOW_Y_BOX_VALUE;
    highYBox=DEFAULT_HIGH_Y_BOX_VALUE;
    voxelResolution=DEFAULT_VOXEL_RESOLUTION;
    utilityAreaLowThreshold=DEFAULT_UTILITY_AREA_LOW_THRESHOLD;
    utilityAreaHighThreshold=DEFAULT_UTILITY_AREA_HIGH_THRESHOLD;
    numberUtilityVoxels=DEFAULT_NUMBER_UTILITY_VOXELS;




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
    if (rf.check("maxNumBlobs")) maxNumBlobs = rf.find("maxNumBlobs").asInt();
    if (rf.check("morphClosing")) morphClosing = rf.find("morphClosing").asDouble();
    if (rf.check("morphOpening")) morphOpening = rf.find("morphOpening").asDouble();
    if (rf.check("outFeaturesFormat")) outFeaturesFormat = rf.find("outFeaturesFormat").asInt();

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

    if (rf.check("outImage")) outImage = rf.find("outImage").asInt();
    if (rf.check("rateMs")) rateMs = rf.find("rateMs").asInt();
    if (rf.check("threshold")) threshold = rf.find("threshold").asInt();
    if (rf.check("seeBounding")) seeBounding = rf.find("seeBounding").asInt();
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
        //printf("No depth yet...\n");
        return;
    };

    //Camera Resolutions:
    int H=depth.height(); //Height resolution
    int W=depth.width();

    std::vector<int> occupancy_indices;

    //Yarp Bottle
    yarp::os::Bottle output;
    output.clear(); //Clear bottle

    //Find pixels with a depth inside the interest area (occupancy pixels)

    //"Explore" loop
    for(int i=floor(0.45*H); i<ceil(0.54*H); i++){ //Camera H umbral (0.45,0.54)H. The region of search is something like a horizontal line.
        for(int j=0; j<W;j++){
            //First convert to REAL WORLD coordinates to use the real area
            double x = (j-W/2)*depth.pixel(j,i)*RGBDCalibrationValue;
            double y = (i-H/2)*depth.pixel(j,i)*RGBDCalibrationValue;

            //We have 4 voxel. This should be parametric.
            int ix=(highXBox-lowXBox)/voxelResolution;
            int areaRegion=areaHighThreshold-areaLowThreshold;

            //Is inside the search area?
            if(lowXBox<x && x<highXBox && lowYBox<y && y<highYBox && areaLowThreshold<depth.pixel(j,i) && depth.pixel(j,i)<areaHighThreshold){
                //Calculate the number of occupancy pixels around that pixel.
                int numberOccupancyIndices=0;

                //Define a search area to see the occupancy around that pixel.
                int lowX=j-searchAreaDilatation;
                int highX=j+searchAreaDilatation;
                if(lowX<0){
                    lowX=0;
                }
                if(highX>W){
                    highX=W;
                }

                //Check area around detected pixel for pixel occupancy.
                for(int k=floor(0.45*H); k<ceil(0.54*H); k++){
                    for(int l=lowX; l<highX;l++){
                        if(depth.pixel(l,k)<areaHighThreshold && depth.pixel(l,k)>areaLowThreshold){
                            numberOccupancyIndices++;
                        }
                    }
                }

                //If we have more occupancy pixels than the threshold, that pixel is considered occupied.
                if(numberOccupancyIndices>occupancyThreshold){

                    std::cout<<" X "<<x<<std::endl;
                    std::cout<<" Y "<<y<<std::endl;
                    std::cout<<" Z "<<depth.pixel(j,i)<<std::endl;
                    //std::cout<<" Incremento "<<ix<<std::endl;
                    //output.addDouble(x);
                    //output.addDouble(y);
                    //std::cout<<" PIXEL "<<x<<" "<<y<<" is considered occupied"<<std::endl;
                    pOutPort->write(output);

                    std::cout<<"The x limits are: "<<lowXBox<<" "<<highXBox<<std::endl;
                    std::cout<<"The y limits are: "<<lowYBox<<" "<<highYBox<<std::endl;

                    for(int c=0;c<voxelResolution;c++){
                        if(lowXBox<x && x<(lowXBox+(c+1)*ix) && lowYBox<y && y<highYBox){ //Voxel_column
                            std::cout<<"I AM IN A ROW"<<std::endl;
                            for(int r=0; r<voxelResolution; r++){
                                std::cout<<c<<std::endl;
                                //double bonud=(float(c+1)/(voxelResolution*2))*areaRegion+areaLowThreshold;
                                //std::cout<<"Has to be lower than "<<bonud<<std::endl;
                                if(depth.pixel(j,i)<((float(r+1)/voxelResolution)*areaRegion+areaLowThreshold)){
                                    std::cout<<"I AM IN A VOXEL "<<std::endl;
                                    output.clear(); //Clear bottle
                                    output.addInt(c);
                                    output.addInt(r);
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
            else if(lowXBox<x && x<highXBox && lowYBox<y && y<highYBox && utilityAreaLowThreshold<depth.pixel(j,i) && depth.pixel(j,i)<utilityAreaHighThreshold){
                //Calculate the number of occupancy pixels around that pixel.
                int numberOccupancyIndices=0;
                //std::cout<<" VAMOS CON LOS UTILITY PIXELS "<<std::endl;


                //Define a search area to see the occupancy around that pixel.
                int lowX=j-searchAreaDilatation;
                int highX=j+searchAreaDilatation;
                if(lowX<0){
                    lowX=0;
                }
                if(highX>W){
                    highX=W;
                }

                //Check area around detected pixel for pixel occupancy.
                for(int k=floor(0.45*H); k<ceil(0.54*H); k++){
                    for(int l=lowX; l<highX;l++){
                        if(depth.pixel(l,k)<utilityAreaHighThreshold && depth.pixel(l,k)>utilityAreaLowThreshold){
                            numberOccupancyIndices++;
                        }
                    }
                }

                //If we have more occupancy pixels than the threshold, that pixel is considered occupied.
                if(numberOccupancyIndices>occupancyThreshold){

                    std::cout<<" X "<<x<<std::endl;
                    std::cout<<" Y "<<y<<std::endl;
                    std::cout<<" Z "<<depth.pixel(j,i)<<std::endl;
                    //std::cout<<" Incremento "<<ix<<std::endl;
                    //output.addInt(x);
                    //output.addInt(y);
                    int uix=(highXBox-lowXBox)/numberUtilityVoxels;
                    //std::cout<<" PIXEL "<<x<<" "<<y<<" is considered occupied"<<std::endl;
                    pOutPort->write(output);

                    for(int c=0;c<numberUtilityVoxels;c++){
                        if(lowXBox<x && x<(lowXBox+(c+1)*uix) && lowYBox<y && y<highYBox){ //Voxel_column
                            std::cout<<"UTILITY VOXEL "<<std::endl;
                            output.clear(); //Clear bottle
                            output.addInt(c);
                            output.addInt(voxelResolution);
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
                for(int k=floor(0.45*H); k<ceil(0.54*H); k++){
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
