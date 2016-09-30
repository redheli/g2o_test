#include "simulator3d.h"

#include "rand.h"

#include <map>
#include <iostream>
#include <cmath>
using namespace std;

namespace g2o {
namespace tutorial {

    using namespace Eigen;
typedef std::map<int, std::map<int, Simulator3D::LandmarkPtrVector> > LandmarkGrid;

Simulator3D::Simulator3D()
{

}

void Simulator3D::simulate(int numPoses, const Eigen::Vector3d &sensorOffset, bool sim_roll, bool sim_pitch)
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
    Vector3d rotNoise(DEG2RAD(3.),DEG2RAD(1.),DEG2RAD(2.)); // yaw pitch roll
    Vector3d landmarkNoise(0.05, 0.05, 0.05);

    Eigen::Matrix<double,6,6,Eigen::ColMajor> covariance;
    covariance.fill(0.);
    covariance(0, 0) = transNoise[0]*transNoise[0];
    covariance(1, 1) = transNoise[1]*transNoise[1];
    covariance(2, 2) = transNoise[2]*transNoise[2];
    covariance(3, 3) = rotNoise[0]*rotNoise[0];
    covariance(4, 4) = rotNoise[1]*rotNoise[1];
    covariance(5, 5) = rotNoise[2]*rotNoise[2];
    Eigen::Matrix<double,6,6,Eigen::ColMajor> information = covariance.inverse();

    int glb_id=0;
    // first pose
    Simulator3D::GridPose3D firstPose;
    firstPose.id = glb_id++;
    Eigen::Isometry3d t;
    Eigen::Matrix3d rot = Eigen::Matrix3d::Zero();
    t = rot;
    t.translation() = Eigen::Vector3d(0,0,0);
    firstPose.truePose = t;
    firstPose.simulatorPose = t;
    poses_.push_back(firstPose);

    // first 5 steps
    double droll = DEG2RAD(30.); // degree
    double dpitch = DEG2RAD(0.);
    double dyaw = DEG2RAD(30.); // degree
    for(int i=0;i<steps;++i){
        Simulator3D::GridPose3D nextGridPose;
        nextGridPose.id = glb_id++;
        // motion
        Eigen::Isometry3d true_motion;
        Eigen::Isometry3d noise_motion;
        // yaw
        Eigen::AngleAxisd rotz(dyaw / steps, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rotz_n(dyaw / steps + rotNoise[0], Eigen::Vector3d::UnitZ()); // noise
        // pitch
        Eigen::AngleAxisd roty(dpitch / steps, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roty_n(dpitch / steps + rotNoise[1], Eigen::Vector3d::UnitY());
        // roll
        Eigen::AngleAxisd rotx(droll / steps, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotx_n(droll / steps + rotNoise[2], Eigen::Vector3d::UnitX());
        // rotation
        Eigen::Matrix3d rot = (rotz * roty * rotx).toRotationMatrix();
        Eigen::Matrix3d rot_n = (rotz_n * roty_n * rotx_n).toRotationMatrix();
        true_motion = rot;
        noise_motion = rot_n;
        // transform
        Eigen::Vector3d t = Eigen::Vector3d(stepLen,0,0);
        Eigen::Vector3d t_n = Eigen::Vector3d(stepLen + transNoise[0],0 + transNoise[1],0 + transNoise[2]);
        true_motion.translation() = true_motion.linear() * t;
        noise_motion.translation() = noise_motion.linear() * t_n;
        // pre pose
        Simulator3D::GridPose3D &pre = poses_.back();
        // next true pose
        Eigen::Isometry3d new_p = pre.truePose * true_motion;
        // next noise pose
        Eigen::Isometry3d new_p_n = pre.simulatorPose * noise_motion;
        nextGridPose.truePose = new_p;
        nextGridPose.simulatorPose = new_p_n;
        poses_.push_back(nextGridPose);

    }

    // second 5 steps
    // droll = 0, dpitch = 0  , dyaw = -10 DEG
    droll = DEG2RAD(0.); // degree
    dpitch = DEG2RAD(0.);
    dyaw = DEG2RAD(-10.); // degree
    for(int i=0;i<steps;++i){
        Simulator3D::GridPose3D nextGridPose;
        nextGridPose.id = glb_id++;
        // motion
        Eigen::Isometry3d true_motion;
        Eigen::Isometry3d noise_motion;
        // yaw
        Eigen::AngleAxisd rotz(dyaw / steps, Eigen::Vector3d::UnitZ());
        Eigen::AngleAxisd rotz_n(dyaw / steps + rotNoise[0], Eigen::Vector3d::UnitZ()); // noise
        // pitch
        Eigen::AngleAxisd roty(dpitch / steps, Eigen::Vector3d::UnitY());
        Eigen::AngleAxisd roty_n(dpitch / steps + rotNoise[1], Eigen::Vector3d::UnitY());
        // roll
        Eigen::AngleAxisd rotx(droll / steps, Eigen::Vector3d::UnitX());
        Eigen::AngleAxisd rotx_n(droll / steps + rotNoise[2], Eigen::Vector3d::UnitX());
        // rotation
        Eigen::Matrix3d rot = (rotz * roty * rotx).toRotationMatrix();
        Eigen::Matrix3d rot_n = (rotz_n * roty_n * rotx_n).toRotationMatrix();
        true_motion = rot;
        noise_motion = rot_n;
        // transform
        Eigen::Vector3d t = Eigen::Vector3d(stepLen,0,0);
        Eigen::Vector3d t_n = Eigen::Vector3d(stepLen + transNoise[0],0 + transNoise[1],0 + transNoise[2]);
        true_motion.translation() = true_motion.linear() * t;
        noise_motion.translation() = noise_motion.linear() * t_n;
        // pre pose
        Simulator3D::GridPose3D &pre = poses_.back();
        // next true pose
        Eigen::Isometry3d new_p = pre.truePose * true_motion;
        // next noise pose
        Eigen::Isometry3d new_p_n = pre.simulatorPose * noise_motion;
        nextGridPose.truePose = new_p;
        nextGridPose.simulatorPose = new_p_n;

        poses_.push_back(nextGridPose);
    }

    // land mark
    // creating landmarks along the trajectory
    cerr << "Simulator: Creating landmarks ... ";
    LandmarkGrid grid;
    for (PosesVector::const_iterator it = poses_.begin(); it != poses_.end(); ++it) {
        const GridPose3D &pose = *it;
        Eigen::Quaterniond gtQuat = (Eigen::Quaterniond)pose.truePose.linear();
        Eigen::Vector3d gtTrans = pose.truePose.translation();
      int ccx = (int)round(gtTrans[0]); // x
      int ccy = (int)round(gtTrans[1]); // y
      int ccz = (int)round(gtTrans[2]); // z
      for (int a=-landmarksRange; a<=landmarksRange; a++)
        for (int b=-landmarksRange; b<=landmarksRange; b++){
          int cx=ccx+a;
          int cy=ccy+b;
          int cz=ccz+b;
          LandmarkPtrVector& landmarksForCell = grid[cx][cy];
          if (landmarksForCell.size() == 0) {
            for (int i = 0; i < landMarksPerSquareMeter; ++i) {
              Landmark* l = new Landmark();
              double offx, offy, offz;
              do {
                offx = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
                offy = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
                offz = Rand::uniform_rand(-0.5*stepLen, 0.5*stepLen);
              } while (hypot_sqr(offx, offy) < 0.25*0.25);
              l->truePose[0] = cx + offx;
              l->truePose[1] = cy + offy;
              l->truePose[2] = cz + offz;
              landmarksForCell.push_back(l);
            }
          }
        }
    }// end for poses
    cerr << "done." << endl;

    cerr << "Simulator: Simulating landmark observations for the poses ... ";
    double maxSensorSqr = maxSensorRangeLandmarks * maxSensorRangeLandmarks;
    int globalId = 0;
    for (PosesVector::iterator it = poses_.begin(); it != poses_.end(); ++it) {
      Simulator3D::GridPose3D& pv = *it;
      Eigen::Vector3d gtTrans = pv.truePose.translation();
      int cx = (int)round(gtTrans[0]); // x
      int cy = (int)round(gtTrans[1]); // y
      int cz = (int)round(gtTrans[2]); // z
      int numGridCells = (int)(maxSensorRangeLandmarks) + 1;

      pv.id = globalId++;
      Eigen::Isometry3d trueInv = pv.truePose.inverse();

      for (int xx = cx - numGridCells; xx <= cx + numGridCells; ++xx)
        for (int yy = cy - numGridCells; yy <= cy + numGridCells; ++yy) {
          LandmarkPtrVector& landmarksForCell = grid[xx][yy];
          if (landmarksForCell.size() == 0)
            continue;
          for (size_t i = 0; i < landmarksForCell.size(); ++i) {
            Landmark* l = landmarksForCell[i];
            double dSqr = hypot_sqr(pv.truePose.translation().x() - l->truePose.x(), pv.truePose.translation().y() - l->truePose.y());
            if (dSqr > maxSensorSqr)
              continue;
            double obs = Rand::uniform_rand(0.0, 1.0);
            if (obs > observationProb) // we do not see this one...
              continue;
            if (l->id < 0)
              l->id = globalId++;
            if (l->seenBy.size() == 0) {
              Vector3d trueObservation = trueInv * l->truePose;
              Vector3d observation = trueObservation;
              observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
              observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
              l->simulatedPose = pv.simulatorPose * observation;
            }
            l->seenBy.push_back(pv.id);
            pv.landmarks.push_back(l);
          } // end for i
        } // end for yy

    } // end for poses
    cerr << "done." << endl;

    // add the odometry measurements
    odometry_.clear();
    cerr << "Simulator: Adding odometry measurements ... ";
    for (size_t i = 1; i < poses_.size(); ++i) {
      const GridPose3D& prev = poses_[i-1];
      const GridPose3D& p = poses_[i];

      odometry_.push_back(GridEdge3D());
      GridEdge3D& edge = odometry_.back();

      edge.from = prev.id;
      edge.to = p.id;
      edge.trueTransf = prev.truePose.inverse() * p.truePose;
      edge.simulatorTransf = prev.simulatorPose.inverse() * p.simulatorPose;
      edge.information = information;
    }
    cerr << "done." << endl;

    landmarks_.clear();
    landmarkObservations_.clear();
    // add the landmark observations
    {
      cerr << "Simulator: add landmark observations ... ";
      Matrix2d covariance; covariance.fill(0.);
      covariance(0, 0) = landmarkNoise[0]*landmarkNoise[0];
      covariance(1, 1) = landmarkNoise[1]*landmarkNoise[1];
      covariance(2, 2) = landmarkNoise[2]*landmarkNoise[1];
      Matrix3d information = covariance.inverse();

      for (size_t i = 0; i < poses_.size(); ++i) {
        const GridPose3D& p = poses_[i];
        for (size_t j = 0; j < p.landmarks.size(); ++j) {
          Landmark* l = p.landmarks[j];
          if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) {
            landmarks_.push_back(*l);
          }
        }
      }

      for (size_t i = 0; i < poses_.size(); ++i) {
        const GridPose3D& p = poses_[i];
        // convert sensorOffset to Eigen::Isometry3d
        Eigen::Isometry3d sensor_offset;
        sensor_offset.translation() = sensorOffset;
        Eigen::Isometry3d trueInv = (p.truePose * sensor_offset).inverse();
        for (size_t j = 0; j < p.landmarks.size(); ++j) {
          Landmark* l = p.landmarks[j];
          Vector3d observation;
          Vector3d trueObservation = trueInv * l->truePose;
          observation = trueObservation;
          if (l->seenBy.size() > 0 && l->seenBy[0] == p.id) { // write the initial position of the landmark
            observation = (p.simulatorPose * sensor_offset).inverse() * l->simulatedPose;
          } else {
            // create observation for the LANDMARK using the true positions
            observation[0] += Rand::gauss_rand(0., landmarkNoise[0]);
            observation[1] += Rand::gauss_rand(0., landmarkNoise[1]);
          }

          landmarkObservations_.push_back(LandmarkEdge());
          LandmarkEdge& le = landmarkObservations_.back();

          le.from = p.id;
          le.to = l->id;
          le.trueMeas = trueObservation;
          le.simulatorMeas = observation;
          le.information = information;
        }
      }
      cerr << "done." << endl;
    }
}

} // end namespace
} // end namespace
