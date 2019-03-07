#include "KalmanFilter.h"
#include <cmath>
#include <limits>

using namespace std;

KalmanFilter::KalmanFilter() {
    thisA = 1;
    thisB = 0;
    thisC = 1;
}
/**
* Constructor
*
* @param R Process noise
* @param Q Measurement noise
*/
KalmanFilter::KalmanFilter(double R, double Q) { 
    thisA = 1;
    thisB = 0;
    thisC = 1;
    _R = R; 
    _Q = Q;
}
/**
* Constructor
*
* @param R Process noise
* @param Q Measurement noise
* @param A State vector
* @param B Control vector
* @param C Measurement vector
*/
KalmanFilter::KalmanFilter(double R, double Q, double A, double B, double C) { 
    _R = R; 
    _Q = Q;

    thisA = A; 
    thisB = B;
    thisC = C;

    cov = std::numeric_limits<double>::quiet_NaN();
    x = std::numeric_limits<double>::quiet_NaN();
}
/**
* Filters a measurement
*
* @param measurement The measurement value to be filtered
* @param u The controlled input value
* @return The filtered value
*/
double KalmanFilter::filter(double measurement, double u){
        if (isnan(x)) {
            x = (1 / thisC) * measurement;
            cov = (1 / thisC) * _Q * (1 / thisC);
        }else {
            double predX = (thisA * x) + (thisB * u);
            double predCov = ((thisA * cov) * thisA) + _R;

            // Kalman gain
            double K = predCov * thisC * (1 / ((thisC * predCov * thisC) + _Q));

            // Correction
            x = predX + K * (measurement - (thisC * predX));
            cov = predCov - (K * thisC * predCov);
        }
        return x;
}

/**
* Filters a measurement
*
* @param measurement The measurement value to be filtered
* @return The filtered value
*/
double KalmanFilter::filter(double measurement){
        double u = 0;
        if (isnan(x)) {
            x = (1 / thisC) * measurement;
            cov = (1 / thisC) * _Q * (1 / thisC);
        }else {
            double predX = (thisA * x) + (thisB * u);
            double predCov = ((thisA * cov) * thisA) + _R;

            // Kalman gain
            double K = predCov * thisC * (1 / ((thisC * predCov * thisC) + _Q));

            // Correction
            x = predX + K * (measurement - (thisC * predX));
            cov = predCov - (K * thisC * predCov);
        }
        return x;
}
/**
* Set the last measurement.
* @return The last measurement fed into the filter
*/
double KalmanFilter::lastMeasurement(){
    return x;
}

/**
* Sets measurement noise
*
* @param noise The new measurement noise
*/
void KalmanFilter::setMeasurementNoise(double noise){
   _Q = noise;
}

/**
* Sets process noise
*
* @param noise The new process noise
*/
void KalmanFilter::setProcessNoise(double noise){
   _R = noise;
}
