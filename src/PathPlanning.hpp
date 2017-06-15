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

    enum terrainType
    {//Must be external
        OBSTACLE_SOIL,
        LOOSE_SOIL,
        COMPACT_SOIL
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

    struct soilType
    {
	double friction;
	double slip;
    };

    struct Node {
      //Node Parameters
        base::Pose2D pose; // X-Y-aspect
        double slope;
	double elevation;
        riskProperties risk;
	nodeState state;
        base::samples::RigidBodyState roverPose;
        Node *nodeParent;
        unsigned int terrain; //Index to terrainList

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

    Node(uint x_, uint y_, double e_, double c_, double r_)
    {
        pose.position[0] = (double)x_;
        pose.position[1] = (double)y_;
      // Calculate slope and aspect
        terrain = (unsigned int) c_;
	elevation = e_;
        risk.obstacle = r_;
        work = std::numeric_limits<double>::max();
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
            std::vector< std::vector<unsigned int*> > costMap;
	    std::vector< std::vector<double*> > riskMap;
	    std::vector< soilType* > terrainList;


	    void initTerrainList(std::vector< double > friction, std::vector< double > slip);
	    void costFunction(Node* nodeTarget, double& Power, locomotionMode& lM);

    bool setGoal(double x, double y);
    void showStart();
    void initNodeMatrix(std::vector< std::vector<double> > elevation, std::vector< std::vector<double> > cost, std::vector< std::vector<double> > risk);
    Node* getNode(double x, double y);
    void showNodeMatrix();
            void fastMarching(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector);
            double getPropagation(Node* nodeTarget);
            void setPropagation(Node* nodeTarget, double value);
            void calculateFieldGradient();
            void gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal,
					     std::vector< std::vector<double> > field, double tau,
 					     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector);
    void nodeUpdate(Node* node, Node* nodeParent, double cost, double heading, locomotionMode locMode);
    
    void heuristicCostFunction(Node* start, Node* goal);
    void calculatePitchRoll(double slope, double aspect, double yaw, double &roll, double &pitch);
            void propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand);
            Node* minCostNode(std::vector<Node*>& nodeList);
            void interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode);
            void showPropagationMatrix();
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
