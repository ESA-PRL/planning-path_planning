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

    enum planningMode
    {
        SHORTEST,
        SAFEST,
        BALANCED
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
        double friction;
        double slip;
    };

    struct costProperties
    {
        double costValue;
        double heuristicCost;
        base::Vector3d costGradient;
    };

    struct Node {
      //Node Parameters
        base::Pose2D pose; // X-Y-aspect
        double slope;
        terrainProperties soil;
        //costProperties cost;
	nodeState state;
        double risk;
        base::samples::RigidBodyState roverPose;
        
  bool closed;
  
  uint x; // X in grid
  uint y; // Y in grid
  double dx; //X in grid (double) for fastMarching
  double dy; //Y in grid (double) for fastMarching
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

  // Terrain properties
  double globalX;  // X in map
  double globalY;  // Y in map
  double height;
  double elevation;
  double friction;
  double slip;
  
  double aspect;
  double s; // slip ratio
  double mu;
  
  double solarExposure;


        Node()
        {
            cost = INF;
            closed = false;
            state = OPEN;
            nodeParent = NULL;
        }

        Node(uint x_, uint y_, double e_, double f_, double s_, double r_) : x(x_), y(y_), elevation(e_), friction(f_), slip(s_), risk(r_){
        work = INF;
    /*if ((risk>0.9)||(slip==1))
      state = CLOSED;
    else*/
    state = OPEN;
    //closed = (risk>0.7); // Close if NOT traversable
    nodeParent = NULL;
    dx = (double)x_;
    dy = (double)y_;
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
   //(Xd,Yd) Distance global frame -> Grid frame
   double offsetX;
   double offsetY;
   //Angle between global frame and grid frame
   double offsetTheta;
            std::vector< std::vector<double> > propagationMatrix;
            std::vector< std::vector<double> > propagationGXMatrix;
            std::vector< std::vector<double> > propagationGYMatrix;
        public:
            PathPlanning();
            ~PathPlanning();
            bool setStartNode(base::samples::RigidBodyState startPose);
            bool setStartNode(base::Pose2D startPose);
    double WD;
    double WO;
    double WM;
    double WR;
    std::vector< std::vector<Node*> > nodeMatrix;
    //void setOffset(double dx, double dy, double dtheta);
    //bool setStart(double x, double y, double heading);
    bool setGoal(double x, double y);
    void setPlanningMode(planningMode mode);
    void showStart();
    void initNodeMatrix(std::vector< std::vector<double> > elevation, std::vector< std::vector<double> > friction,
                            std::vector< std::vector<double> > slip, std::vector< std::vector<double> > risk);
    Node* getNode(uint x, uint y);
    void showNodeMatrix();
            std::vector<base::Waypoint> fastMarching(base::Waypoint wStart, base::Waypoint wGoal, std::vector< std::vector<Node*> > nodeMatrix);
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
    //void calculateCostGradient();
    Node* minCostNode(std::vector<Node*>& nodeList);
    void interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& L);
    std::vector<Node*> nodePath;
    std::vector<base::Waypoint> trajectory;
    void clearPath();
};

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
