#pragma once

#include <opencv2/core/types.hpp>

// This Ellipse class is designed to work with the "Screen Coordinate System"
// which has the origin at the left-top corner.
// This is why the word "Pixel" is especially preferred instead of "Point".
// Thus, all points' y component are reflected for a convenient transform.
class customEllipse
{
public:
    // Computes and returns of a pixels' distance to the ellipse
    // overload for double version
    double GetDistance(double pX, double pY, double &estimation);

    customEllipse() {}
    customEllipse(double coefs[6]);
    customEllipse(cv::Point *points, int numPoints);
    customEllipse(double *pX, double *pY, int numPoints);
    ~customEllipse();

    cv::Point GetCenter();

    double GetCenterX() const;

    double GetCenterY() const;

    double GetRotation() const;

    double GetSemiMajorAxis() const;

    double GetSemiMinorAxis() const;

    // Gets the coefficients of the ellipse in conic form
    void GetCoefficients(double *);

    // Draws an ellipse with the desired resolution i.t.o. points
    // resolution may be selected wrt the perimeter
    cv::Point *DrawEllipse(int resolution);

    double GetPerimeter();

    // Computes and returns the fitting error of the points that
    // are used to generate the ellipse equation
    double GetAverageFittingError();

    // Computes and returns the fitting error of the points that
    // are used to generate the ellipse equation
    double GetRmsFittingError();

    // Computes and returns the closest points to the fitting points
    // GetFittingError has to be invoked before calling this fcn
    cv::Point *GetClosestPoints();

    // Gets the closest point to the test point and returns the distance
    double GetClosestPointAndDistance(cv::Point test, cv::Point &closest);

    // overload for the double version
    double GetClosestPointAndDistance(double testX, double testY,
                                      cv::Point &closest);

    void getEllipseSamples(int noOfSamples, std::vector<double> &xPoints,
                           std::vector<double> &yPoints);

private:
    // Ellipse coefficients
    double A1, B1, C1, D1, E1, F1; // ellipse coefficients
    double A2, B2, C2, D2, E2, F2;
    double A3, B3, C3, D3, E3, F3;
    double cX, cY;          // center of ellipse
    double a, b, perimeter; // semimajor and semiminor axes
    double rotation;        // rotation angle of the ellipse in radians
    double fitError;        // average fit error (per point)
    double rmsError;        // root mean square fitting error

    // variables that are being used multiple times
    double a2_b2; //(a^2 - b^2) is frequently used in point distance

    // points...
    int noPoints;
    double *xPoints;    // x coordinate of points in double
    double *yPoints;    // y coordinate of points in double
    double *estRadians; // angle (in radians) estimations for

    cv::Point *fitPoints;  // points that ellipse will be fit
    cv::Point *drawPoints; // points that are the polygonal approximation of the
                           // ellipse
    cv::Point *closePoints; // closest points to the fit points on the ellipse
                            // curve (with the same order, respectively)

    // initialize the parameters for safety
    void InitParams();

    // One step of NR method: f / fPrime
    inline double NewtonRaphsonIteration(double theta, double x, double y);

    // variable steps NR estimation for theta of the closest ellipse point
    // One iteration of NR method: f - fPrime
    double NewtonRaphsonThetaEstimation(double theta, double x, double y);

    // returns distance between two points
    inline double DistanceBwPoints(cv::Point p1, cv::Point p2);

    inline double DistanceBwPoints(double x1, double y1, double x2, double y2);

    // returns the SQUARED distance between two points
    inline double SquaredDistanceBwPoints(cv::Point p1, cv::Point p2);

    inline double SquaredDistanceBwPoints(double x1, double y1, double x2,
                                          double y2);

    // Computes and returns of a pixels' distance to the ellipse
    double GetDistance(cv::Point point, double &tmpEst);

    // Computes and returns of a pixels' squared distance to the ellipse
    double GetSquaredDistance(cv::Point point, double &tmpEst);

    // overload for double version
    double GetSquaredDistance(double pX, double pY, double &estimation);
};

bool CircleFit(const std::vector<double> &Xs, const std::vector<double> &Ys,
               double &centerX, double &centerY, double &radius);
