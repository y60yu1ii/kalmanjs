/**
 * Simple C++ implementation of the Kalman Filter for 1D data.
 * Originally written in JavaScript by Wouter Bulten
 *
 * Now rewritten into C++ 
 * 2019
 *
 * @license MIT License
 *
 * @author YaoYu Li
 *
 * @chekout https://github.com/y60yu1ii/kalmanjs
 *
 */
using namespace std;

class KalmanFilter { 
public: 
    KalmanFilter(); 
    KalmanFilter(double R, double Q); 
    KalmanFilter(double R, double Q, double A, double B, double C); 
    double filter(double measurement, double u);
    double filter(double measurement);
    double lastMeasurement();
    void setMeasurementNoise(double noise);
    void setProcessNoise(double noise);

private:
    //_A, _B, _C is not able to pass compile in Arduino
    double thisA;
    double thisB;
    double thisC;

    double _R;
    double _Q;

    double cov;
    double x;
};