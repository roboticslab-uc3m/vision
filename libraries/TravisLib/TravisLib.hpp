// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#ifndef __TRAVIS_LIB_HPP__
#define __TRAVIS_LIB_HPP__

#include <stdio.h>  // just printf and fprintf

#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
//#include "highgui.h" // to show windows

/**
 * @ingroup vision_libraries
 * @defgroup TravisLib
 */

namespace roboticslab
{

/**
 * @ingroup TravisLib
 * @brief The Travis class implements all the algorithms on a single image.
 */
class Travis {
public:

    /**
     * Travis class constructor.
     * @param quiet suppress messages displayed upon success/failure.
     * @param overwrite will not make a copy (faster, less memory), but will overwrite the image you pass.
     */
    Travis(bool quiet=true, bool overwrite=true) : _quiet(quiet), _overwrite(overwrite) { }

    /**
     * Set the image in cv::Mat format.
     * @param image the image to set, in cv::Mat format.
     * @return true if the object was set successfully.
     */
    bool setCvMat(const cv::Mat& image);

    /**
     * Set the image in cv::Mat format.
     * @param image the image to set, in cv::Mat format.
     * @return true if the object was set successfully.
     */
    bool setBinCvMat(const cv::Mat& image);

    /**
     * Binarize the image.
     * @param algorithm implemented: "redMinusGreen", "greenMinusRed".
     */
    bool binarize(const char* algorithm);
    /**
     * Binarize the image.
     * @param algorithm implemented: "redMinusGreen", "greenMinusRed".
     * @param threshold i.e. 50.
     */
    bool binarize(const char* algorithm, const double& threshold);
    /**
     * Binarize the image.
     * @param algorithm implemented: "redMinusGreen", "greenMinusRed".
     * @param min i.e. 110.
     * @param max i.e. 130.
     */
    bool binarize(const char* algorithm, const double& min, const double& max);

    /**
     * Morphologically closing the binarized image.
     * @param closure i.e. 4 for a 100x100 image, 15 for higher resolution.
     */
    void morphClosing(const int& closure);

    /**
     * Morphologically opening the binarized image.
     * @param opening i.e. 4 for a 100x100 image, 15 for higher resolution.
     */
    void morphOpening(const int& opening);

    /**
     * Use findContours to get what we use as blobs.
     * @param maxNumBlobs the number of max blobs to keep, the rest get truncated.
     * @return number of blobs detected (can be 0), truncated by maxNumBlobs.
     */
    int blobize(const int& maxNumBlobs);

    /**
     * Push a contour.
     * @param contour to be pushed on to the stack. Use with care.
     */
    void pushContour(const std::vector <cv::Point>& contour);

    /**
     * This function calculates X and Y as moments directly extracted from the stored contours.
     * @param locations returned.
     */
    bool getBlobsXY(std::vector<cv::Point2d> &locations);

    /**
     * This function calculates the Area of the blobs (contours).
     * @param areas returned.
     */
    bool getBlobsArea(std::vector <double>& areas);

    /**
     * This function calculates the Solidity of the blobs (contours).
     * @param solidities returned.
     */
    bool getBlobsSolidity(std::vector <double>& solidities);

    /**
     * This function calculates ALPHA, and _minRotatedRects as a side effect.
     * @param method 0=box, 1=ellipse.
     * @param angles returned.
     */
    bool getBlobsAngle(const int& method, std::vector <double>& angles);

    /**
     * This function calculates the Aspect Ratios and Axes of the stored _minRotatedRects.
     * @param aspectRatios returned.
     * @param axisFirsts returned.
     * @param axisSeconds returned.
     */
    bool getBlobsAspectRatio(std::vector <double>& aspectRatios, std::vector <double>& axisFirsts, std::vector <double>& axisSeconds);

    bool getBlobsPerimeter(std::vector <double>& perimeters);

    /**
     * This function calculates the Rectangularities of the stored _minRotatedRects.
     * @param rectangularities returned.
     */
    bool getBlobsRectangularity(std::vector <double>& rectangularities);

    /**
     * This function calculates HSV Means and Standard Deviations.
     * @param hues returned.
     * @param vals returned.
     * @param sats returned.
     * @param hueStdDevs returned.
     * @param valStdDevs returned.
     * @param satStdDevs returned.
     */
    bool getBlobsHSV(std::vector <double>& hues, std::vector <double>& vals, std::vector <double>& sats,
        std::vector <double>& hueStdDevs, std::vector <double>& valStdDevs, std::vector <double>& satStdDevs);

    /**
     * Get the image in cv::Mat format.
     * @param image
     * @param vizualization param, 0=None,1=Box,2=Contour,3=Both.
     * @return the image, in cv::Mat format.
     */
    cv::Mat& getCvMat(const int& image, const int& vizualization);

    /**
     * Release _img and _imgBin3 to prevent memory leaks.
     */
    void release();

protected:
    /** Store the verbosity level. */
    bool _quiet;

    /** Store the overwrite parameter. */
    bool _overwrite;

    /** Store the image in cv::Mat format. */
    cv::Mat _img;

    /** Store the hsv image in cv::Mat format. */
    cv::Mat _imgHsv;

    /** Store the binary image in cv::Mat format. */
    cv::Mat _imgBin;

    /** Store the binary image fit for 3 layer sending in cv::Mat format. */
    cv::Mat _imgBin3;

    /** Store the contours (blob contours). */
    std::vector < std::vector <cv::Point> > _contours;

    /** Store the box. */
    std::vector < cv::RotatedRect > _minRotatedRects;

};

/**
 * @ingroup travis_functions
 * Can be used as a comparison function object for sorting.
 */
bool compareContourAreas ( std::vector<cv::Point> contour1, std::vector<cv::Point> contour2 );

/**
 * Crop the image.
 * @param image the image to set, in cv::Mat format.
 * @return true if the object was set successfully.
 */
bool travisCrop(const int x, const int y, const int width, const int height, cv::Mat& img);

/**
 * @ingroup travis_functions
 * This function gets the biggest contour.
 */
std::vector <cv::Point> getBiggestContour(const cv::Mat image);

/**
 * @ingroup travis_functions
 * This function calculates X and Y.
 */
void calcLocationXY(float& locX, float& locY, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the mask.
 */
void calcMask(cv::Mat& mask, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the area.
 */
void calcArea(float& area, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the rectangularity.
 */
void calcRectangularity(float& rectangularity, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the angle.
 */
void calcAngle(float& angle, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the mass center.
 */
void calcMassCenter(float& massCenterLocX, float& massCenterLocY , const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the aspect ratio.
 */
void calcAspectRatio(float& aspectRatio, float& axisFirst, float& axisSecond ,const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the solidity.
 */
void calcSolidity(float& solidity, const std::vector <cv::Point> biggestCont);

/**
 * @ingroup travis_functions
 * This function calculates the HSV mean and std deviation.
 */
void calcHSVMeanStdDev(const cv::Mat image, const cv::Mat mask, float& hue_mean, float& hue_stddev,
                       float& saturation_mean, float& saturation_stddev,
                       float& value_mean, float& value_stddev);

/**
 * @ingroup travis_functions
 * This function calculates the HSV peak color.
 */
void calcHSVPeakColor(const cv::Mat image, const cv::Mat mask, float& hue_mode, float& hue_peak,
                       float& value_mode, float& value_peak);

/**
 * @ingroup travis_functions
 * This function calculates the moments.
 */
void calcMoments(cv::Mat& theHuMoments, const std::vector <cv::Point> biggestCont );

/**
 * @ingroup travis_functions
 * This function calculates the arc length.
 */
void calcArcLength(float& arc, const std::vector <cv::Point> biggestCont );

/**
 * @ingroup travis_functions
 * This function calculates the circle.
 */
void calcCircle(float& radius, const std::vector <cv::Point> biggestCont );

}  // namespace roboticslab

#endif  // __TRAVIS_LIB_HPP__

