#include "simulator3d.h"

#include "rand.h"

#include <map>
#include <iostream>
#include <cmath>
using namespace std;

namespace g2o {
namespace tutorial {

    using namespace Eigen;

Simulator3D::Simulator3D()
{

}

void Simulator3D::simulate(int numPoses, const SE2 &sensorOffset, bool sim_roll, bool sim_pitch)
{
    // simulate a robot observing landmarks while travelling on a grid
    int steps = 5;
    double stepLen = 1.0;
    int boundArea = 50;

    // each round has "steps" number move
    // at begin decide roll,pitch,yaw value of the round
    // at noise at each step

    double maxSensorRangeLandmarks = 2.5 * stepLen;

    int landMarksPerSquareMeter = 1;
    double observationProb = 0.8;

    int landmarksRange=2;

    Vector3d transNoise(0.05, 0.01, 0.02);
    Vector3d rotNoise(DEG2RAD(3.),DEG2RAD(1.),DEG2RAD(2.)); // roll,pitch,yaw
    Vector2d landmarkNoise(0.05, 0.05);

    Eigen::Matrix<double,6,6,Eigen::ColMajor> covariance;
    covariance.fill(0.);
    covariance(0, 0) = transNoise[0]*transNoise[0];
    covariance(1, 1) = transNoise[1]*transNoise[1];
    covariance(2, 2) = transNoise[2]*transNoise[2];
    covariance(3, 3) = rotNoise[0]*rotNoise[0];
    covariance(4, 4) = rotNoise[1]*rotNoise[1];
    covariance(5, 5) = rotNoise[2]*rotNoise[2];
    Eigen::Matrix<double,6,6,Eigen::ColMajor> information = covariance.inverse();

}

} // end namespace
} // end namespace
