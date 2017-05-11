#ifndef _PATHPLANNING_LIBRARIES_HPP_
#define _PATHPLANNING_LIBRARIES_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>
#include <vector>
#include <fstream>

#define INF 100000000

namespace PathPlanning_lib
{

    enum locomotionMode
    {//Must be external
        DRIVING,
        WHEEL_WALKING
    };

    enum nodeState
    {
        OPEN,
        CLOSED,
        NARROW,
        FROZEN,
	OBSTACLE
    };

    struct terrainProperties
    {
        double elevation;
        double friction;
        double slip;
        double solarExposure;
    };

    struct riskProperties
    {
        double obstacle;
        double slope;
        double material;
        double uncertainty;
    };

    struct Node {
      //Node Parameters
        base::Pose2D pose; // X-Y-aspect
        double slope;
        terrainProperties soil;
        riskProperties risk;
	nodeState state;
        base::samples::RigidBodyState roverPose;
        Node *nodeParent;
  double cost;
  double work;
  double power;
  double dCostX;
  double dCostY;
  double heuristicCost;//for A*
  std::vector<Node*> nbList; //8-neighbour nodes List
  std::vector<Node*> nb4List; //4-Neighbourhood List

  double distanceCost;
  double headingCost;
  double materialCost;
  double riskCost;

  //Rover parameters
  double heading;
  double roll;
  double pitch;
  locomotionMode nodeLocMode;
  
  double aspect;


    Node()
    {
        cost = INF;
        state = OPEN;
        nodeParent = NULL;
    }

    Node(uint x_, uint y_, double e_, double f_, double s_, double r_)
    {
        pose.position[0] = (double)x_;
        pose.position[1] = (double)y_;
      // Calculate slope and aspect
        soil.elevation = e_;
        soil.friction = f_;
        soil.slip = s_;
        risk.obstacle = r_;
        work = INF;
    /*if ((risk>0.9)||(slip==1))
      state = CLOSED;
    else*/
    state = OPEN;
    //closed = (risk>0.7); // Close if NOT traversable
    nodeParent = NULL;
    }
};


/**
 * Base class for a motion planning library.
 */
    class PathPlanning
    {
        private:
            base::samples::RigidBodyState mStartPose;
            double pathCost;
            Node* nodeStart;
            Node* nodeGoal;
            std::vector< std::vector<double> > propagationMatrix;
            std::vector< std::vector<double> > propagationGXMatrix;
            std::vector< std::vector<double> > propagationGYMatrix;
        public:
            PathPlanning();
            ~PathPlanning();
            bool setStartNode(base::Waypoint wStart);
            std::vector< std::vector<Node*> > nodeMatrix;
    bool setGoal(double x, double y);
    void showStart();
    void initNodeMatrix(std::vector< std::vector<double> > elevation, std::vector< std::vector<double> > friction,
                            std::vector< std::vector<double> > slip, std::vector< std::vector<double> > risk);
    Node* getNode(uint x, uint y);
    void showNodeMatrix();
            std::vector<base::Waypoint> fastMarching(base::Waypoint wStart, base::Waypoint wGoal);
            double getPropagation(Node* nodeTarget);
            void setPropagation(Node* nodeTarget, double value);
            void calculateFieldGradient(std::vector< std::vector<double> > field,
                                        std::vector< std::vector<double> >& fieldGx,
                                        std::vector< std::vector<double> >& fieldGy);
            std::vector<base::Waypoint> gradientDescentTrajectory(base::Waypoint pStart, base::Waypoint wGoal,
                                                                  std::vector< std::vector<double> > field, double tau);
    void nodeUpdate(Node* node, Node* nodeParent, double cost, double heading, locomotionMode locMode);
    double costFunction(Node* nodeTarget);
    void heuristicCostFunction(Node* start, Node* goal);
    void calculatePitchRoll(double slope, double aspect, double yaw, double &roll, double &pitch);
            void propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand);
            Node* minCostNode(std::vector<Node*>& nodeList);
            void interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& height);
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
