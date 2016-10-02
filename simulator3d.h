#ifndef SIMULATOR3D_H
#define SIMULATOR3D_H

#include "se2.h"
//#include "g2o_tutorial_slam2d_api.h"

#include <Eigen/StdVector>

#include <vector>
#include <map>

namespace g2o {
  namespace tutorial {

class Simulator3D
{
public:
    enum MotionType {
      MO_LEFT, MO_RIGHT,
      MO_NUM_ELEMS
    };

    /**
     * \brief simulated landmark
     */
    struct Landmark
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      int id;
      Eigen::Vector3d truePose;
      Eigen::Vector3d simulatedPose;
      std::vector<int> seenBy;
      Landmark() : id(-1) {}
    };
    typedef std::vector<Landmark, Eigen::aligned_allocator<Landmark> > LandmarkVector;
    typedef std::vector<Landmark*> LandmarkPtrVector;

    /**
     * simulated pose of the robot
     */
    struct GridPose3D
    {
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
      int id;
      Isometry3D truePose;
      Isometry3D simulatorPose;
      LandmarkPtrVector landmarks;      ///< the landmarks observed by this node
    };
    typedef std::vector<GridPose3D, Eigen::aligned_allocator<GridPose3D> >  PosesVector;

    /**
     * \brief odometry constraint
     */
    struct GridEdge3D
    {
      int from;
      int to;
      Isometry3D trueTransf;
      Isometry3D simulatorTransf;
      Eigen::Matrix<double,6,6,Eigen::ColMajor> information;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    typedef std::vector<GridEdge3D, Eigen::aligned_allocator<GridEdge3D> >  GridEdge3DVector;

    struct LandmarkEdge
    {
      int from;
      int to;
      Eigen::Vector3d trueMeas;
      Eigen::Vector3d simulatorMeas;
      Eigen::Matrix3d information;
      EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };
    typedef std::vector<LandmarkEdge, Eigen::aligned_allocator<LandmarkEdge> >  LandmarkEdgeVector;

public:
    Simulator3D();

    void simulate(int numPoses, const Eigen::Vector3d& sensorOffset = Eigen::Vector3d(0,0,0), bool sim_roll=true,
                  bool sim_pitch=true);

    GridPose3D generateNewPose(const GridPose3D& prev, const SE2& trueMotion, const Eigen::Vector2d& transNoise, double rotNoise);


public:
    PosesVector poses_;
    GridEdge3DVector odometry_;
    LandmarkVector landmarks_;
    LandmarkEdgeVector landmarkObservations_;
};

  } // end namespace
} // end namespace

#endif // SIMULATOR3D_H
