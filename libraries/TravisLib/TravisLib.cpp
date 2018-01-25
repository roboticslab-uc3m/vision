// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include "TravisLib.hpp"

namespace roboticslab
{

/************************************************************************/

bool Travis::setCvMat(const cv::Mat& image) {
    if (!_quiet) printf("[Travis] in: setCvMat(...)\n");
    if (!image.data) {
        fprintf(stderr,"[Travis] error: No image data.\n");
        return false;
    }
    if (!_overwrite) _img = image.clone();  // safer
    else _img = image;  // faster and less memory

    cvtColor(_img, _imgHsv, CV_BGR2HSV);

    return true;
}

/************************************************************************/

bool Travis::setBinCvMat(const cv::Mat& image) {
    if (!_quiet) printf("[Travis] in: setBinCvMat(...)\n");
    if (!image.data) {
        fprintf(stderr,"[Travis] error: No image data.\n");
        return false;
    }
    if (!_overwrite) _imgBin = image.clone();  // safer
    else _imgBin = image;  // faster and less memory

    // the result is bin but we store bin3 so we can colorfully paint on it
    cv::Mat outChannels[3];
    outChannels[0] = _imgBin;
    outChannels[1] = _imgBin;
    outChannels[2] = _imgBin;
    cv::merge(outChannels, 3, _imgBin3);

    //cvtColor(_img, _imgHsv, CV_BGR2HSV);

    return true;
}

/************************************************************************/

bool Travis::binarize(const char* algorithm) {
    if (!_quiet) printf("[Travis] in: binarize(%s)\n",algorithm);
    if (strcmp(algorithm,"canny")==0) {
        if (!_quiet) printf("[Travis] in: binarize(canny)\n");
        cvtColor(_img,_imgBin,CV_BGR2GRAY);
        Canny(_imgBin,_imgBin, 30,40);
    } else {
        fprintf(stderr,"[Travis] error: Unrecognized algorithm with 0 args: %s.\n",algorithm);
        return false;
    }
    // the result is bin but we store bin3 so we can colorfully paint on it
    cv::Mat outChannels[3];
    outChannels[0] = _imgBin;
    outChannels[1] = _imgBin;
    outChannels[2] = _imgBin;
    cv::merge(outChannels, 3, _imgBin3);
    return true;
}

/************************************************************************/

bool Travis::binarize(const char* algorithm, const double& threshold) {
    if (strcmp(algorithm,"redMinusGreen")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[2], bgrChannels[1], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else if (strcmp(algorithm,"redMinusBlue")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[2], bgrChannels[0], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else if (strcmp(algorithm,"greenMinusRed")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[1], bgrChannels[2], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else if (strcmp(algorithm,"greenMinusBlue")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[1], bgrChannels[0], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else if (strcmp(algorithm,"blueMinusRed")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[0], bgrChannels[2], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else if (strcmp(algorithm,"blueMinusGreen")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f)\n",algorithm,threshold);
        cv::Mat bgrChannels[3];
        cv::split(_img, bgrChannels);
        cv::subtract(bgrChannels[0], bgrChannels[1], _imgBin);  // BGR
        cv::threshold(_imgBin, _imgBin, threshold, 255, cv::THRESH_BINARY);
    } else {
        fprintf(stderr,"[Travis] error: Unrecognized algorithm with 1 arg: %s.\n",algorithm);
        return false;
    }
    // the result is bin but we store bin3 so we can colorfully paint on it
    cv::Mat outChannels[3];
    outChannels[0] = _imgBin;
    outChannels[1] = _imgBin;
    outChannels[2] = _imgBin;
    cv::merge(outChannels, 3, _imgBin3);
    return true;
}

/************************************************************************/

bool Travis::binarize(const char* algorithm, const double& min, const double& max) {
    if (strcmp(algorithm,"hue")==0) {
        if (!_quiet) printf("[Travis] in: binarize(%s, %f, %f)\n",algorithm,min,max);
        cv::Mat hsvChannels[3];
        split( _imgHsv, hsvChannels );
        cv::threshold(hsvChannels[0], hsvChannels[0], max, 255, cv::THRESH_TOZERO_INV);
        cv::threshold(hsvChannels[0], _imgBin, min, 255, CV_THRESH_BINARY);
        //cv::subtract(bgrChannels[2], bgrChannels[1], _imgBin);  // BGR

        // begin: extra V filter
        cv::threshold(hsvChannels[2], hsvChannels[2], 160, 255, cv::THRESH_TOZERO_INV);
        cv::threshold(hsvChannels[2], hsvChannels[2], 120, 255, CV_THRESH_BINARY);
        cv::bitwise_and(hsvChannels[2],_imgBin,_imgBin);
        // end: extra V filter
        
    } else {
        fprintf(stderr,"[Travis] error: Unrecognized algorithm with 2 args: %s.\n",algorithm);
        return false;
    }
    // the result is bin but we store bin3 so we can colorfully paint on it
    cv::Mat outChannels[3];
    outChannels[0] = _imgBin;
    outChannels[1] = _imgBin;
    outChannels[2] = _imgBin;
    cv::merge(outChannels, 3, _imgBin3);
    return true;
}

/************************************************************************/

void Travis::morphClosing(const int& closure) {
    if (!_quiet) printf("[Travis] in: morphClosing(%d)\n", closure);
    dilate(_imgBin, _imgBin, cv::Mat(), cv::Point(-1,-1), closure);
    erode(_imgBin, _imgBin, cv::Mat(), cv::Point(-1,-1), closure);
}

/************************************************************************/

void Travis::morphOpening(const int& opening) {
    if (!_quiet) printf("[Travis] in: morphOpening(%d)\n", opening);
    erode(_imgBin, _imgBin, cv::Mat(), cv::Point(-1,-1), opening);
    dilate(_imgBin, _imgBin, cv::Mat(), cv::Point(-1,-1), opening);
}

/************************************************************************/

int Travis::blobize(const int& maxNumBlobs) {
    if (!_quiet) printf("[Travis] in: blobize(%d)\n", maxNumBlobs);

    // [thanks getBiggestContour from smorante] note: here jgvictores decides to avoid Canny
    //findContours( _imgBin, _contours, CV_RETR_LIST, CV_CHAIN_APPROX_SIMPLE);
    findContours( _imgBin, _contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    //findContours( _imgBin, _contours, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE);

    if (!_quiet) printf("[Travis] # of found contours: %zd.\n", _contours.size());
    
    // [thanks http://stackoverflow.com/questions/13495207/opencv-c-sorting-contours-by-their-contourarea]
    // default to sort by size (to keep the biggest, xD)
    std::sort( _contours.begin(), _contours.end(), compareContourAreas);

    // Now truncate
    if (_contours.size() > maxNumBlobs)
        _contours.erase( _contours.begin()+maxNumBlobs, _contours.end() );

    return _contours.size();
}

/************************************************************************/

void Travis::pushContour(const std::vector <cv::Point>& contour) {
    if (!_quiet) printf("[Travis] in: pushContour()\n");
    _contours.push_back( contour );

}

/************************************************************************/
bool Travis::getBlobsXY(std::vector <cv::Point2d>& locations) {
    if (!_quiet) printf("[Travis] in: getBlobsXY(...)\n");

    // we have the number of actual blobs in _contours.size()

    // This method seems less accurate and less efficient, breaks if < 5 (px?)
    /*for ( int i = 0; i < _contours.size(); i++ ) {
        RotatedRect minEllipse = fitEllipse( Mat(_contours[i]) );
        locations.push_back( minEllipse.center );
    }*/

    // [thanks http://areshopencv.blogspot.com.es/2011/09/finding-center-of-gravity-in-opencv.html]
    std::vector<cv::Moments> mu( _contours.size() );

    for( int i = 0; i < _contours.size(); i++ ) {
        mu[i] = moments( cv::Mat(_contours[i]), false );
    }
    std::vector<cv::Point2d> mc( _contours.size() );
    for( int i = 0; i < _contours.size(); i++ ) {
        mc[i] = cv::Point2d( mu[i].m10/mu[i].m00 , mu[i].m01/mu[i].m00 );
        if( mc[i].x != mc[i].x )
            return false;
        if( mc[i].y != mc[i].y )
            return false;
        locations.push_back( mc[i] );
    }

    return true;
}

/************************************************************************/
bool Travis::getBlobsArea(std::vector <double>& areas) {
    if (!_quiet) printf("[Travis] in: getBlobsArea(...)\n");

    for( int i = 0; i < _contours.size(); i++ ) {
        areas.push_back( fabs(contourArea(cv::Mat(_contours[i]))) );
    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsPerimeter(std::vector <double>& perimeters) {
    if (!_quiet) printf("[Travis] in: getBlobsPerimeter(...)\n");

    for( int i = 0; i < _contours.size(); i++ ) {
        perimeters.push_back( arcLength(_contours[i],true) );
    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsSolidity(std::vector <double>& solidities) {
    if (!_quiet) printf("[Travis] in: getBlobsSolidity(...)\n");

    for( int i = 0; i < _contours.size(); i++ ) {

        double areaCont = contourArea(_contours[i]);

        std::vector <cv::Point> biggestCH;
        convexHull(_contours[i],biggestCH);
        double areaCH = contourArea(biggestCH);

        solidities.push_back( areaCont/areaCH );
    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsAngle(const int& method, std::vector <double>& angles) {
    if (!_quiet) printf("[Travis] in: getBlobsAngle(%d,...)\n", method);

    for( int i = 0; i < _contours.size(); i++ ) {
        //Rect sqCont = boundingRect( Mat(_contours[i]) );
        //RotatedRect sqCont = boundingRect( Mat(_contours[i]) );
        
        if (method == 0) {  // box
            // [thanks http://felix.abecassis.me/2011/10/opencv-bounding-box-skew-angle/]
            _minRotatedRects.push_back( minAreaRect( cv::Mat(_contours[i]) ) );
            /*double angle = minRotatedRect.angle;
            if (angle < -45.) angle += 90.;  // it just tends to go (-90,0)
            angles.push_back( angle );*/
            //j//angles.push_back( _minRotatedRects[_minRotatedRects.size()-1].angle+90.0 );

        } else if (method == 1) {  // ellipse
        // hopefully people will see this return false as a warning and treat before error.
            if (_contours[i].size() < 5) {
                fprintf(stderr,"[Travis] error: returning false as ellipse would break with < 5 points.\n");
                return false;  // else fitEllipse would cause break exit.
            }
            // [thanks smorante]
            _minRotatedRects.push_back( fitEllipse( cv::Mat(_contours[i]) ) );
            //?//if (angle < -45.) angle += 90.;
            //j//angles.push_back( _minRotatedRects[_minRotatedRects.size()-1].angle );        
        }
        cv::Point2f vertices[4];
        _minRotatedRects[_minRotatedRects.size()-1].points(vertices);
        cv::Point2f p_0_1 = vertices[1] - vertices[0];
        cv::Point2f p_0_3 = vertices[3] - vertices[0];
        if ( cv::norm(p_0_1) >  cv::norm(p_0_3) )
            angles.push_back( - atan2( p_0_3.y , p_0_3.x )*180.0/M_PI );
        else
            angles.push_back( - atan2( p_0_1.y , p_0_1.x )*180.0/M_PI );

    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsAspectRatio(std::vector <double>& aspectRatios, std::vector <double>& axisFirsts, std::vector <double>& axisSeconds) {
    if (!_quiet) printf("[Travis] in: getBlobsAspectRatio(...)\n");
    for( int i = 0; i < _minRotatedRects.size(); i++ ) {
        cv::Point2f vertices[4];
        _minRotatedRects[i].points(vertices);
        double length = cv::norm(vertices[1] - vertices[0]);
        double width = cv::norm(vertices[3] - vertices[0]);
        aspectRatios.push_back( width / length );
        axisFirsts.push_back( length );
        axisSeconds.push_back( width );
    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsRectangularity(std::vector <double>& rectangularities) {
    if (!_quiet) printf("[Travis] in: getBlobsRectangularity(...)\n");
    for( int i = 0; i < _minRotatedRects.size(); i++ ) {

        double areaObj = contourArea(_contours[i]);

        //double areaRect = _minRotatedRects[i].area();  // does not exist
        cv::Point2f vertices[4];
        _minRotatedRects[i].points(vertices);
        double length = cv::norm(vertices[1] - vertices[0]);
        double width = cv::norm(vertices[3] - vertices[0]);
        double areaRect = length * width;

        rectangularities.push_back( ( areaObj / areaRect ) - (M_PI/4.0) ); // subtract the ideal circ/square rel. 
    }
    return true;
}

/************************************************************************/
bool Travis::getBlobsHSV(std::vector <double>& hues, std::vector <double>& vals, std::vector <double>& sats,
        std::vector <double>& hueStdDevs, std::vector <double>& valStdDevs, std::vector <double>& satStdDevs) {
    if (!_quiet) printf("[Travis] in: getBlobsHSV(...)\n");
    cv::Mat hsvChannels[3];
    split( _imgHsv, hsvChannels );
    
    for( int i = 0; i < _contours.size(); i++ ) {

        // \begin{mask}
        std::vector <cv::Point> biggestCH;
        convexHull(_contours[i],biggestCH);

        std::vector < std::vector <cv::Point> > listCont;
        listCont.push_back(biggestCH);

        cv::Mat mask = cv::Mat::zeros(_img.rows, _img.cols, CV_8UC1);
        drawContours(mask, listCont,-1, cv::Scalar(255),CV_FILLED);
        // \end{mask}

        cv::Scalar h_mean, h_stddev;
        cv::meanStdDev(hsvChannels[0], h_mean, h_stddev, mask);
        cv::Scalar s_mean, s_stddev;
        cv::meanStdDev(hsvChannels[1], s_mean, s_stddev, mask);
        cv::Scalar v_mean, v_stddev;
        cv::meanStdDev(hsvChannels[2], v_mean, v_stddev, mask);

        hues.push_back( h_mean[0] );
        vals.push_back( v_mean[0] );
        sats.push_back( s_mean[0] );

        hueStdDevs.push_back( h_stddev[0] );
        valStdDevs.push_back( s_stddev[0] );
        satStdDevs.push_back( v_stddev[0] );

        // \begin{mask}
        mask.release();
        // \end{mask}

    }

    return true;
}

/************************************************************************/

cv::Mat& Travis::getCvMat(const int& image, const int& vizualization) {
    if (!_quiet) printf("[Travis] in: getCvMat(%d,%d)\n",image,vizualization);

    if (( vizualization == 2 )||( vizualization == 3 )) {  // Contour
        cv::RNG rng(12345);
        for( int i = 0; i < _contours.size(); i++ ) {
            cv::Scalar color = cv::Scalar( rng.uniform(0, 255), rng.uniform(0,255), rng.uniform(0,255) );
            if ( image == 1 )
                cv::drawContours( _imgBin3, _contours, i, color, 1, 8, CV_RETR_LIST, 0, cv::Point() );
            else
                cv::drawContours( _img, _contours, i, color, 1, 8, CV_RETR_LIST, 0, cv::Point() );
        }
    }

    if (( vizualization == 1 )||( vizualization == 3 )) {  // Box, computed in getBlobsAngle
        for(int i=0;i<_minRotatedRects.size();i++) {
            cv::Point2f vertices[4];
            _minRotatedRects[i].points(vertices);
            if ( image == 1 )
                for(int i = 0; i < 4; ++i)
                    cv::line( _imgBin3, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0), 1, CV_AA);
            else
                for(int i = 0; i < 4; ++i)
                    cv::line( _img, vertices[i], vertices[(i + 1) % 4], cv::Scalar(255, 0, 0), 1, CV_AA);
        }
    }

    if ( image == 1 )
        return _imgBin3;
    else
        return _img;  // image == 0, etc        
}

/************************************************************************/

void Travis::release() {
    _img.release();
    _imgHsv.release();
    _imgBin3.release();
    return;
}

/************************************************************************/
/************************************************************************/
/************************************************************************/

// comparison function object
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 ) {
    double i = fabs( contourArea(cv::Mat(contour1)) );
    double j = fabs( contourArea(cv::Mat(contour2)) );
    return ( i > j );
}

/************************************************************************/

bool travisCrop(const int x, const int y, const int width, const int height, cv::Mat& img) {
    printf("[Travis] in: travisCrop(%d,%d,%d,%d)\n",x,y,width,height);
    // Thanks: http://stackoverflow.com/questions/8267191/how-to-crop-a-cvmat-in-opencv
 
    // Set up a rectangle to define your region of interest
    cv::Rect myROI(x, y, width, height);
    img = img(myROI);
    return true;
}

/************************************************************************/

std::vector <cv::Point> getBiggestContour(const cv::Mat image){
    //variables
    cv::Mat grayImg, cannyImg;
    std::vector < std::vector <cv::Point> > contours;
    std::vector < std::vector <cv::Point> > biggest;

    //converting to grayscale
    cvtColor(image,grayImg,CV_BGR2GRAY);

    //canny filter and dilation to fill little holes
    Canny(grayImg,cannyImg, 30,100);
    dilate(cannyImg, cannyImg, cv::Mat(),cv::Point(-1,-1),1);

    //finding all contours
    findContours(cannyImg,contours,CV_RETR_LIST,CV_CHAIN_APPROX_SIMPLE);

    //finding biggest contour
    int maxSize=0;
    int indexCont=0;

    for(int i=0 ; i< contours.size() ; i++){
            if(contours[i].size() > maxSize){
                maxSize=contours[i].size();
                indexCont=i;
                }
     }

    biggest.push_back(contours[indexCont]);
    return biggest[0];
}

void calcLocationXY(float& locX, float& locY, const std::vector <cv::Point> biggestCont){

    cv::RotatedRect minEllipse;

    //fitting ellipse around contour
    minEllipse= fitEllipse(cv::Mat(biggestCont));

    //getting the center
    locX = minEllipse.center.x;
    locY = minEllipse.center.y;

}

void calcMask(cv::Mat& mask, const std::vector <cv::Point> biggestCont){

    std::vector < std::vector <cv::Point> > listCont;
    std::vector <cv::Point> biggestCH;

    //doing convexhull
    convexHull(biggestCont,biggestCH);
    listCont.push_back(biggestCH);

    //drawing in mask
    drawContours(mask, listCont,-1, cv::Scalar(255),CV_FILLED);
}

void calcArea(float& area, const std::vector <cv::Point> biggestCont){

    //setting area
    area = contourArea(biggestCont);
}

void calcRectangularity(float& rectangularity, const std::vector <cv::Point> biggestCont){
    //RotatedRect minEllipse;
    float areaObj;
    float areaRect;

    //calc area of contour
    areaObj = contourArea(biggestCont);

    cv::Rect sqCont = boundingRect(biggestCont);
    //calc area rect
    areaRect = sqCont.area();

    //setting parameter
    rectangularity = areaObj/areaRect  - (M_PI/4.0);  // subtract the ideal circ/square rel. 

}

void calcAngle(float& angle, const std::vector <cv::Point> biggestCont){
    cv::RotatedRect minRect = minAreaRect( cv::Mat(biggestCont));

    cv::Point2f vertices[4];
    minRect.points(vertices);
    cv::Point2f p_0_1 = vertices[1] - vertices[0];
    cv::Point2f p_0_3 = vertices[3] - vertices[0];
    if ( cv::norm(p_0_1) >  cv::norm(p_0_3) )
        angle = - atan2( p_0_3.y , p_0_3.x )*180.0/M_PI ;
    else
        angle = - atan2( p_0_1.y , p_0_1.x )*180.0/M_PI ;

}

void calcMassCenter(float& massCenterLocX, float& massCenterLocY , const std::vector <cv::Point> biggestCont){

    cv::Moments mu;
    cv::Point2f mc;

    //calc moments of contour
    mu = moments(biggestCont,false);

    //calc mass center with moments
    mc = cv::Point2f(mu.m10/mu.m00, mu.m01/mu.m00);

    massCenterLocX = mc.x;
    massCenterLocY = mc.y;
}

void calcAspectRatio(float& aspectRatio, float& axisFirst, float& axisSecond ,const std::vector <cv::Point> biggestCont){

    cv::RotatedRect minEllipse;
    cv::Point2f vertices[4];

    //extracting axis from vertices
    minEllipse = fitEllipse(cv::Mat(biggestCont));
    minEllipse.points(vertices);
    float dist[2];
    for (int i = 0; i < 2; i++){
        dist[i]=std::sqrt(pow((vertices[i].x - vertices[(i+1)%4].x),2)+pow((vertices[i].y - vertices[(i+1)%4].y),2));
    }

    //setting parameter
    aspectRatio = dist[0]/dist[1];
    axisFirst=dist[0];
    axisSecond=dist[1];
}


void calcSolidity(float& solidity, const std::vector <cv::Point> biggestCont){

    std::vector <cv::Point> biggestCH;
    float areaCont;
    float areaCH;

    //doing convexhull
    convexHull(biggestCont,biggestCH);

    areaCont= contourArea(biggestCont);
    areaCH = contourArea(biggestCH);
    solidity= areaCont/areaCH;

}

void calcHSVMeanStdDev(const cv::Mat image, const cv::Mat mask, float& hue_mean, float& hue_stddev,
                       float& saturation_mean, float& saturation_stddev,
                       float& value_mean, float& value_stddev){

    cv::Mat hsvImage;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // Separate the image in 3 places ( H - S - V )
     std::vector<cv::Mat> hsv_planes;
     split( hsvImage, hsv_planes );

   // calculating mean and stddev
     cv::Scalar h_mean, h_stddev;
     cv::meanStdDev(hsv_planes[0], h_mean, h_stddev,mask);

     cv::Scalar s_mean, s_stddev;
     cv::meanStdDev(hsv_planes[1], s_mean, s_stddev, mask);

     cv::Scalar v_mean, v_stddev;
     cv::meanStdDev(hsv_planes[2], v_mean, v_stddev, mask);

     //setting values
     hue_mean = h_mean[0];
     hue_stddev = h_stddev[0];
     saturation_mean = s_mean[0];
     saturation_stddev = s_stddev[0];
     value_mean = v_mean[0];
     value_stddev = v_stddev[0];

}


void calcHSVPeakColor(const cv::Mat image, const cv::Mat mask, float& hue_mode, float& hue_peak,
                       float& value_mode, float& value_peak) {

    cv::Mat hsvImage;
    cvtColor(image, hsvImage, CV_BGR2HSV);

    // Separate the image in 3 places ( H - S - V )
     std::vector<cv::Mat> hsv_planes;
     split( hsvImage, hsv_planes );

    // number of bins for each variable
    int h_bins = 180;
//    int s_bins = 255;
    int v_bins = 255;

    // hue varies from 0 to 180, the other 0-255. Ranges
    float h_ranges[] = { 0, 180 };
//    float s_ranges[] = { 0, 255 };
    float v_ranges[] = { 0, 255 };

    const float* h_histRange = { h_ranges };
//    const float* s_histRange = { s_ranges };
    const float* v_histRange = { v_ranges };

    // image to keep the histogram
    cv::MatND h_hist;
//    MatND s_hist;
    cv::MatND v_hist;

    //calculation
    cv::calcHist(&hsv_planes[0], 1, 0, mask, h_hist, 1, &h_bins, &h_histRange);
//    calcHist(&hsv_planes[1], 1, 0, mask, s_hist, 1, &s_bins, &s_histRange);
    cv::calcHist(&hsv_planes[2], 1, 0, mask, v_hist, 1, &v_bins, &v_histRange);

    // finding highest peak
    float h_mode= 0;
    int h_maxPixels = 0;

    for(int i=0; i< h_bins; i++){
        if(h_hist.at<float>(i) > h_maxPixels){
            h_mode=i;
            h_maxPixels=h_hist.at<float>(i);
        }
    }

    float h_peak= 0;

    for(int i=h_bins; i>0; i--){
        if((int)h_hist.at<float>(i) > 0){
            //cout << (int)h_hist.at<float>(i);
            h_peak=i;
            break;
        }
    }

//    float s_peak= 0;
//    int s_maxPixels = 0;

//    for(int i=0; i< s_bins; i++){
//        if(s_hist.at<float>(i) > s_maxPixels){
//            s_peak=i;
//            s_maxPixels=s_hist.at<float>(i);
//        }
//    }

    float v_mode= 0;
    int v_maxPixels = 0;

    for(int i=0; i< v_bins; i++){
        if(v_hist.at<float>(i) > v_maxPixels){
            v_mode=i;
            v_maxPixels=v_hist.at<float>(i);
        }
    }

    float v_peak= 0;

    for(int i=v_bins; i>0; i--){
        if((int)v_hist.at<float>(i) > 0){
            //cout << (int)v_hist.at<float>(i);
            v_peak=i;
            break;
        }
    }

    //setting
    hue_mode=h_mode;
    hue_peak=h_peak;
//    saturation_peak=s_peak;
    value_mode=v_mode;
    value_peak=v_peak;
}

void calcMoments(cv::Mat& theHuMoments, const std::vector <cv::Point> biggestCont ){

    cv::Moments mu;
    mu = cv::moments(biggestCont);
    HuMoments(mu,theHuMoments);

}

void calcArcLength(float& arc, const std::vector <cv::Point> biggestCont ){

    cv::Mat tranf = cv::Mat(biggestCont);
    arc=arcLength(tranf,true);
}

void calcCircle(float& radius, const std::vector <cv::Point> biggestCont ){

    cv::Point2f center;
    float rad=0;
    minEnclosingCircle(cv::Mat(biggestCont),center,rad);
    radius=rad;
}

}  // namespace roboticslab

