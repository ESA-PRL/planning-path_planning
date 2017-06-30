#ifndef _PATHPLANNING_LIBRARIES_HPP_
#define _PATHPLANNING_LIBRARIES_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>
#include <vector>
#include <fstream>
#include <list>

#define INF 100000000

namespace PathPlanning_lib
{

    enum locomotionMode
    {//Must be external
        DRIVING,
        WHEEL_WALKING
    };

    enum nodeInterpType
    {//Must be external
        NON_INTERPOLATED,
        VERTICAL,
        HORIZONTAL
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
        uint index[2]; // Position in nodemap
        base::Pose2D pose; // X-Y-aspect
        double slope;
	double elevation;
        riskProperties risk;
	nodeState state;
        base::samples::RigidBodyState roverPose;
        Node *nodeParent;
        unsigned int terrain; //Index to terrainList
        double g;
        double rhs;
        double key[2];
        double power;
        double work;

  double cost;
  double dCostX;
  double dCostY;
  double heuristicCost;//for A*
        std::vector<Node*> nb8List; //8-neighbour nodes List
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
        work = INF;
        rhs = 0;
        g = std::numeric_limits<double>::max();
        key[0] = std::numeric_limits<double>::max();
        key[1] = std::numeric_limits<double>::max();
        state = OPEN;
        //closed = (risk>0.7); // Close if NOT traversable
        nodeParent = NULL;
    }
    };

    struct NodeMap
    {
        double nodeSize;
        base::Pose2D globalOriginPose;
        std::vector< std::vector<Node*> > nodeMatrix;
        Node* getNode(uint i, uint j);

        NodeMap(double size, base::Pose2D pos,
                std::vector< std::vector<double> > elevation,
		std::vector< std::vector<double> > cost,
		std::vector< std::vector<double> > risk);
    };

//__PATH_PLANNER_CLASS__
    class PathPlanning
    {
        private:
            base::samples::RigidBodyState mStartPose;
            double pathCost;

            std::vector< std::vector<double> > propagationMatrix;
            std::vector< std::vector<double> > propagationGXMatrix;
            std::vector< std::vector<double> > propagationGYMatrix;
            base::Time t1;
        public:
            PathPlanning();
            ~PathPlanning();
            bool setStartNode(base::Waypoint wStart);
            std::vector< std::vector<unsigned int*> > costMap;
	    std::vector< std::vector<double*> > riskMap;
	    std::vector< soilType* > terrainList;

	    void initTerrainList(std::vector< double > friction, std::vector< double > slip);
	    void costFunction(uint Terrain, double& Power, locomotionMode& lM);

            // Field D Star Functions
            void fieldDStar(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector, NodeMap * nodes);
            void setKey(Node * nodeTarget, base::Waypoint wStart);
            Node * getMinorKey( Node * nodeA, Node * nodeB);
            void updateState(Node * nodeTarget, Node * nodeGoal, base::Waypoint wStart, std::list<Node*>& openList);
            void computeCost(Node * nodeTarget, double& di, double& dj, double& cost);
            double getHeuristic(Node * nodeTarget, base::Waypoint wStart);
            void costInterpolation(double g1, double g2, uint cTerrain, uint bTerrain, double di, double dj, double& dk, double& vs);

    bool setGoal(double x, double y);
    void showStart();
    //Node* getNode(double x, double y);
    void showNodeMatrix();
            void fastMarching(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector, NodeMap * nodes);
            void calculateFieldGradient();
            void gradientNode(Node* nodeTarget, double& dx, double& dy);
            void gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal,
					     NodeMap * nodes, double tau,
 					     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector);

    void heuristicCostFunction(Node* start, Node* goal);
    void calculatePitchRoll(double slope, double aspect, double yaw, double &roll, double &pitch);
            void propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand);
            Node* minCostNode(std::vector<Node*>& nodeList);
            void interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode, NodeMap * nodes);
            void showPropagationMatrix();
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
