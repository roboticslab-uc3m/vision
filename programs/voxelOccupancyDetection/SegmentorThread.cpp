// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "SegmentorThread.hpp"

namespace roboticslab
{

/************************************************************************/
void SegmentorThread::setIKinectDeviceDriver(yarp::dev::IOpenNI2DeviceDriver *_kinect) {
    kinect = _kinect;
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

    fx_d = DEFAULT_FX_D;
    fy_d = DEFAULT_FY_D;
    cx_d = DEFAULT_CX_D;
    cy_d = DEFAULT_CY_D;
    fx_rgb = DEFAULT_FX_RGB;
    fy_rgb = DEFAULT_FY_RGB;
    cx_rgb = DEFAULT_CX_RGB;
    cy_rgb = DEFAULT_CY_RGB;

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
    kinectCalibrationValue=DEFAULT_CALIBRATION_VALUE_KINECT;
    lowXBox=DEFAULT_LOW_X_BOX_VALUE;
    highXBox=DEFAULT_HIGH_X_BOX_VALUE;
    lowYBox=DEFAULT_LOW_Y_BOX_VALUE;
    highYBox=DEFAULT_HIGH_Y_BOX_VALUE;




    printf("--------------------------------------------------------------\n");
    if (rf.check("help")) {
        printf("SegmentorThread options:\n");
        printf("\t--help (this help)\t--from [file.ini]\t--context [path]\n");

        printf("\t--fx_d (default: \"%f\")\n",fx_d);
        printf("\t--fy_d (default: \"%f\")\n",fy_d);
        printf("\t--cx_d (default: \"%f\")\n",cx_d);
        printf("\t--cy_d (default: \"%f\")\n",cy_d);
        printf("\t--fx_rgb (default: \"%f\")\n",fx_rgb);
        printf("\t--fy_rgb (default: \"%f\")\n",fy_rgb);
        printf("\t--cx_rgb (default: \"%f\")\n",cx_rgb);
        printf("\t--cy_rgb (default: \"%f\")\n",cy_rgb);

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

    if (rf.check("fx_d")) fx_d = rf.find("fx_d").asDouble();
    if (rf.check("fy_d")) fy_d = rf.find("fy_d").asDouble();
    if (rf.check("cx_d")) cx_d = rf.find("cx_d").asDouble();
    if (rf.check("cy_d")) cy_d = rf.find("cy_d").asDouble();
    if (rf.check("fx_rgb")) fx_rgb = rf.find("fx_rgb").asDouble();
    if (rf.check("fy_rgb")) fy_rgb = rf.find("fy_rgb").asDouble();
    if (rf.check("cx_rgb")) cx_rgb = rf.find("cx_rgb").asDouble();
    if (rf.check("cy_rgb")) cy_rgb = rf.find("cy_rgb").asDouble();
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

    this->setRate(rateMs);
    this->start();

}

/************************************************************************/
void SegmentorThread::run() {
    // printf("[SegmentorThread] run()\n");

//    std::cout<<"CHECKING"<<std::endl;

    yarp::sig::ImageOf<yarp::sig::PixelMono16> depth = kinect->getDepthFrame();
    if (depth.height()<10) {
        //printf("No depth yet...\n");
        return;
    };
    int H=depth.height(); //Height resolution
    int W=depth.width();

//    double x = (320-W/2)*600/520;
//    double y = (130-H/2)*600/520;
//    std::cout<<" REAL X ES "<<x<<std::endl;
//    std::cout<<" REAL Y ES "<<y<<std::endl;

    //printf(" The depth of the low pixel is %d\n", depth.pixel(0, floor(0.45*H))); //107 for 240
    //printf(" The depth of the low pixel is %d\n", depth.pixel(0, ceil(0.54*H))); //130 for 240

    //                std::cout<<" EL umbral bajo es::::::::::::::: "<<areaLowThreshold<<std::endl;
    //                std::cout<<" EL umbral alto es::::::::::::::: "<<areaHighThreshold<<std::endl;

    std::vector<int> occupancy_indices;

    //"Explore" loop
    for(int i=floor(0.45*H); i<ceil(0.54*H); i++){
        for(int j=0; j<W;j++){
//            std::cout<<"LA PROFUNDIDAD DEL PIXEL "<<i<<"   "<<j<< "ES:: "<<depth.pixel(j,i)<<std::endl;
            //Find pixels with a depth inside the interest area (occupancy pixels)
            //First convert to REAL WORLD coordinates to use the real area
            double x = (j-W/2)*depth.pixel(j,i)*kinectCalibrationValue;
            double y = (i-H/2)*depth.pixel(j,i)*kinectCalibrationValue;
            int ix=(highXBox-lowXBox)/4;
            int areaRegion=areaHighThreshold-areaLowThreshold;
            //if(depth.pixel(j,i)<areaHighThreshold && depth.pixel(j,i)>areaLowThreshold)
            if(lowXBox<x && x<highXBox && lowYBox<y && y<highYBox && areaLowThreshold<depth.pixel(j,i) && depth.pixel(j,i)<areaHighThreshold){
                //Calculate the number of occupancy pixels around that pixel
                //std::cout<<"Detecté Pixel dentro del área de interés"<<std::endl;
                int numberOccupancyIndices=0;
                int lowX=j-searchAreaDilatation;
                int highX=j+searchAreaDilatation;
                if(lowX<0){
                    lowX=0;
                }
                if(highX>W){
                    highX=W;
                }
//                std::cout<<" EL umbral bajo es::::::::::::::: "<<areaLowThreshold<<std::endl;
//                std::cout<<" EL umbral alto es::::::::::::::: "<<areaHighThreshold<<std::endl;
                //Check area around detected pixel for pixel occupancy.
//                std::cout<<"Umbral bajo es:: "<<areaLowThreshold<<std::endl;
//                std::cout<<"Umbral alto es:: "<<areaHighThreshold<<std::endl;
                for(int k=floor(0.45*H); k<ceil(0.54*H); k++){
                    for(int l=lowX; l<highX;l++){
//                        std::cout<<"HASTA AQUI LLEGUE"<<std::endl;
                        if(depth.pixel(l,k)<areaHighThreshold && depth.pixel(l,k)>areaLowThreshold){
                            numberOccupancyIndices++;
                        }
                    }
                }
                //If we have more occupancy pixels than the threshold, that voxel is considered occupied.
//                std::cout<<"THE NUMBER OF OCCUPANCY INDICES IS"<< numberOccupancyIndices<<std::endl;
                if(numberOccupancyIndices>occupancyThreshold){
                    //Yarp Bottle
                    yarp::os::Bottle output;
//                    std::cout<<" REAL X ES "<<areaLowThreshold<<std::endl;
//                    std::cout<<" REAL Y ES "<<areaHighThreshold<<std::endl;
                    std::cout<<" X "<<x<<std::endl;
                    std::cout<<" Y "<<y<<std::endl;
                    std::cout<<" Z "<<depth.pixel(j,i)<<std::endl;
                    std::cout<<" Incremento "<<ix<<std::endl;
                    if(lowXBox<x && x<(lowXBox+ix) && lowYBox<y && y<highYBox){ //Voxel_row_1
                        if(depth.pixel(j,i)<(areaRegion/4+areaLowThreshold)){ //Voxel_col_1
                            output.addInt(0);
                            output.addInt(0);
                            pOutPort->write(output);
                            std::cout<<" ENTRE EN EL VOXEL"<<0<<" "<<0<<std::endl;
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion/2+areaLowThreshold)){ //Voxel_col_2
                            output.addInt(0);
                            output.addInt(1);
                            pOutPort->write(output);
                            std::cout<<" ENTRE EN EL VOXEL"<<0<<" "<<1<<std::endl;
                            return;
                        }
                        else if(depth.pixel(j,i)<(3*areaRegion/4+areaLowThreshold)){ //Voxel_col_3
                            output.addInt(0);
                            output.addInt(2);
                            pOutPort->write(output);
                            std::cout<<" ENTRE EN EL VOXEL"<<0<<" "<<2<<std::endl;
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion+areaLowThreshold)){ //Voxel_col_4
                            output.addInt(0);
                            output.addInt(3);
                            std::cout<<" ENTRE EN EL VOXEL"<<0<<" "<<3<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                    }
                    else if((lowXBox+ix)<x && x<(lowXBox+2*ix) && lowYBox<y && y<highYBox){ //Voxel_row_2
                        if(depth.pixel(j,i)<(areaRegion/4+areaLowThreshold)){ //Voxel_col_1
                            output.addInt(1);
                            output.addInt(0);
                            std::cout<<" ENTRE EN EL VOXEL"<<1<<" "<<0<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion/2+areaLowThreshold)){ //Voxel_col_2
                            output.addInt(1);
                            output.addInt(1);
                            std::cout<<" ENTRE EN EL VOXEL"<<1<<" "<<1<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(3*areaRegion/4+areaLowThreshold)){ //Voxel_col_3
                            output.addInt(1);
                            output.addInt(2);
                            std::cout<<" ENTRE EN EL VOXEL"<<1<<" "<<2<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion+areaLowThreshold)){ //Voxel_col_4
                            output.addInt(1);
                            output.addInt(3);
                            std::cout<<" ENTRE EN EL VOXEL"<<1<<" "<<3<<std::endl;
                            pOutPort->write(output);
                            return;
                        }

                    }
                    else if((lowXBox+2*ix)<x && x<(lowXBox+3*ix) && lowYBox<y && y<highYBox){ //Voxel_row_3
                        if(depth.pixel(j,i)<(areaRegion/4+areaLowThreshold)){ //Voxel_col_1
                            output.addInt(2);
                            output.addInt(0);
                            std::cout<<" ENTRE EN EL VOXEL"<<2<<" "<<0<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion/2+areaLowThreshold)){ //Voxel_col_2
                            output.addInt(2);
                            output.addInt(1);
                            std::cout<<" ENTRE EN EL VOXEL"<<2<<" "<<1<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(3*areaRegion/4+areaLowThreshold)){ //Voxel_col_3
                            output.addInt(2);
                            output.addInt(2);
                            std::cout<<" ENTRE EN EL VOXEL"<<2<<" "<<2<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion+areaLowThreshold)){ //Voxel_col_4
                            output.addInt(2);
                            output.addInt(3);
                            std::cout<<" ENTRE EN EL VOXEL"<<2<<" "<<3<<std::endl;
                            pOutPort->write(output);
                            return;
                        }

                    }
                    else if((lowXBox+3*ix)<x && x<(lowXBox+4*ix) && lowYBox<y && y<highYBox){ //Voxel_row_4
                        if(depth.pixel(j,i)<(areaRegion/4+areaLowThreshold)){ //Voxel_col_1
                            output.addInt(3);
                            output.addInt(0);
                            std::cout<<" ENTRE EN EL VOXEL"<<3<<" "<<0<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion/2+areaLowThreshold)){ //Voxel_col_2
                            output.addInt(3);
                            output.addInt(1);
                            std::cout<<" ENTRE EN EL VOXEL"<<3<<" "<<1<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(3*areaRegion/4+areaLowThreshold)){ //Voxel_col_3
                            output.addInt(3);
                            output.addInt(2);
                            std::cout<<" ENTRE EN EL VOXEL"<<3<<" "<<2<<std::endl;
                            pOutPort->write(output);
                            return;
                        }
                        else if(depth.pixel(j,i)<(areaRegion+areaLowThreshold)){ //Voxel_col_4
                            output.addInt(3);
                            output.addInt(3);
                            std::cout<<" ENTRE EN EL VOXEL"<<3<<" "<<3<<std::endl;
                            pOutPort->write(output);
                            return;
                        }

                    }
                    else{
                        std::cout<<"PIXEL OUT OF THE BOX"<<std::endl;
                    }
                }
                //If not enough occupancy pixels. The region is saved for later removal of search area.
                else{
                    //occupancy_indices.push_back(i);
                    occupancy_indices.push_back(j);
                }
            }

            if(lowXBox<x && x<(lowXBox+ix) && lowYBox<y && y<highYBox){ //CLEAN VOXEL
                if((5*areaRegion/4+areaLowThreshold)<depth.pixel(j,i) && depth.pixel(j,i)<(6*areaRegion/4+areaLowThreshold)){
                    yarp::os::Bottle output;
                    output.addInt(4);
                    output.addInt(4);
                    std::cout<<" ENTRE EN EL VOXEL"<<4<<" "<<4<<std::endl;
                    std::cout<<"BORRANDO PANTALLA"<<std::endl;
                    pOutPort->write(output);
                    return;
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

    //Delete followings lines???
//    //Lets check all the pixels in the area
//    for(int i=0; i<W;i++)
//    {
//        //printf(" The depth of the low pixel is %d\n", depth.pixel(H/10,W/10));
//        for(int j=floor(0.45*H); j<ceil(0.54*H); j++){
//        //printf("%d\n", depth.pixel(i,j));
//        if(depth.pixel(i,j)==0){
//            std::cout<<"The x pixel is"<<i<<std::endl;
//            std::cout<<"The y pixel is"<<j<<std::endl;
//            filter_list[j][i]++;
//            }
//         }
//    }


//     //Print array filter_list array
//     std::cout<<"**************************************************************************************************************************"<<std::endl;
//     for(int i=0; i<W;i++)
//     {
//        for(int j=floor(0.45*H); j<ceil(0.54*H); j++){
//            std::cout<<"Pixel number "<<i<<"  "<<j<<" Value "<< filter_list.at(j).at(i)<<std::endl;
//        }
//      }
//      std::cout<<"**************************************************************************************************************************"<<std::endl;


      /*yarp::sig::ImageOf<yarp::sig::PixelMono16> depth

      pOutImg->prepare() = outYarpImg;
      pOutImg->write();*/

      //The area pixels are (60cm from kinect) H:107 (20cm 2/5 Height) H:130 (25cm 1/2Height) Weidth:all (320 pix, 68cm).

//      yarp::os::Bottle output;
//      output.clear();
//      //output.addInt(depth.pixel(5,5));
//      //output.addInt(depth.pixel(5,6));
//      //output.addDouble(34.3);
//      pOutPort->write(output);
    }

}  // namespace roboticslab
