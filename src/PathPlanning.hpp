#ifndef _PATHPLANNING_LIBRARIES_HPP_
#define _PATHPLANNING_LIBRARIES_HPP_

#include <base/samples/RigidBodyState.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>
#include <vector>
#include <fstream>
#include <list>
#include <envire/core/Environment.hpp>
#include <envire/maps/TraversabilityGrid.hpp>
#include <envire/maps/ElevationGrid.hpp>
#include <envire/operators/SimpleTraversability.hpp>
//#include <orocos/envire/Orocos.hpp>

#define INF 100000000

namespace envire {
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace PathPlanning_lib
{
    /*enum local_state
    {
        HID,
        OBSTACLE,
        TRAVERSABLE_OPEN,
        TRAVERSABLE_CLOSED
    };*/

    enum node_state
    {
        OPEN,
        CLOSED,
        HIDDEN
    };

    struct terrainType
    {
        double cost;
        std::string optimalLM;
    };

    struct localNode
    {
        base::Pose2D pose; //In Local Units respect to Global Node
        base::Pose2D world_pose; //In physical Units respect to World Frame
        base::Pose2D parent_pose; // Position of Global Node Parent In Global Units
        base::Pose2D global_pose; // Position of this local node in Global Units respect to World Frame
        double total_cost;
        double cost;
        double risk;
        node_state state;
        std::vector<localNode*> nb4List;
        bool isObstacle;
        localNode(uint x_, uint y_, base::Pose2D _parent_pose)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            parent_pose = _parent_pose;
            state = OPEN;
            risk = 0;
            total_cost = INF;
            isObstacle = false;
        }
    };

    struct globalNode
    {
        base::Pose2D pose;
        base::Pose2D world_pose;
        double elevation;
        double slope;
        base::Vector2d aspect;
        node_state state;
        bool isObstacle;
        bool hasLocalMap; //Has it localmap?
        double cost;
        double total_cost;
        unsigned int terrain;
        std::vector< std::vector<localNode*> > localMap;
        std::vector<globalNode*> nb4List;
        std::vector<globalNode*> nb8List;
        std::string nodeLocMode;
        globalNode(uint x_, uint y_, double e_, double c_)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
          // Calculate slope and aspect
            terrain = (unsigned int) c_;
            elevation = e_;
            //risk.obstacle = r_;
            isObstacle = false;
            hasLocalMap = false;
            total_cost = INF;
            state = OPEN;
            nodeLocMode = "DONT_CARE";
        }
    };

//__PATH_PLANNER_CLASS__
    class PathPlanning
    {
        private:
            base::samples::RigidBodyState mStartPose;
            double pathCost;
            std::vector< std::vector<globalNode*> > globalMap;
            double global_cellSize;
            base::Pose2D global_offset;
            double local_cellSize;
            uint ratio_scale;
            globalNode* actualGlobalNodePos;
        public:
            PathPlanning(std::vector< terrainType* > _table);
            ~PathPlanning();
            bool setStartNode(base::Waypoint wStart);
            std::vector< std::vector<unsigned int*> > costMap;
	          std::vector< std::vector<double*> > riskMap;
            std::vector< terrainType* > terrainTable;
            std::vector<globalNode*> global_narrowBand;
            std::vector<globalNode*> global_propagatedNodes;
            std::vector<localNode*> local_narrowBand;
            std::vector<localNode*> localExpandableObstacles;
            std::vector<localNode*> horizonNodes;
            std::vector<localNode*> local_closedNodes;
            std::vector<base::Waypoint> trajectory;
            std::vector<bool> isGlobalWaypoint;

            globalNode * global_goalNode;
            localNode * local_goalNode;
            localNode * local_actualPose;

            base::Time t1;

            void initGlobalMap(double globalCellSize,  double localCellSize,
                               base::Pose2D offset,
                               std::vector< std::vector<double> > elevation,
                               std::vector< std::vector<double> > cost);

            globalNode* getGlobalNode(uint i, uint j);

            void calculateSlope(globalNode* nodeTarget);

            void setGoal(base::Waypoint wGoal);

            void calculateGlobalPropagation(base::Waypoint wPos);

            void calculateNominalCost(globalNode* nodeTarget);

            globalNode* minCostGlobalNode();

            void propagateGlobalNode(globalNode* nodeTarget);

            envire::ElevationGrid* getEnvireGlobalPropagation();

            envire::ElevationGrid* getLocalTotalCost(base::Waypoint wPos);

            void createLocalMap(globalNode* gNode);

            localNode* introducePixelInMap(base::Vector2d pos, bool& newVisible, std::vector<base::Waypoint>& trajectory);

            localNode* getLocalNode(base::Vector2d pos);
            localNode* getLocalNode(base::Waypoint wPos);

            void expandGlobalNode(globalNode* gNode);

            bool simUpdateVisibility(base::Waypoint wPos, std::vector< std::vector<double> >& costMatrix, double res, bool initializing, double camHeading, std::vector<base::Waypoint>& trajectory);

            globalNode* getNearestGlobalNode(base::Vector2d pos);
            globalNode* getNearestGlobalNode(base::Waypoint wPos);

            void updateLocalMap(base::Waypoint wPos);

            bool evaluateLocalMap(base::Waypoint wPos,
                                  std::vector< std::vector<double> >& costMatrix,
                                  double res,
                                  std::vector<base::Waypoint>& trajectory);

            void expandRisk();

            localNode* maxRiskNode();

            void propagateRisk(localNode* nodeTarget);

            envire::TraversabilityGrid* getEnvireLocalState(base::Waypoint wPos);

            envire::ElevationGrid* getEnvireRisk(base::Waypoint wPos);

            void setHorizonCost(localNode* horizonNode);

            double getInterpolatedCost(localNode* lNode);

            localNode * calculateLocalPropagation(base::Waypoint wInit, base::Waypoint wDest);

            bool propagateLocalNode(localNode* nodeTarget, double Tcatch, double Tstart);

            localNode* minCostLocalNode();

            std::vector<base::Waypoint> getLocalPath(localNode * lSetNode,
                                                     base::Waypoint wInit,
                                                     double tau);
            std::vector<base::Waypoint> getGlobalPath(base::Waypoint wPos,
                                                      double tau);

            bool calculateNextWaypoint(base::Waypoint& wPos, double tau);
            base::Waypoint calculateNextGlobalWaypoint(base::Waypoint& wPos, double tau);

            void gradientNode(localNode* nodeTarget, double& dnx, double& dny);
            void gradientNode(globalNode* nodeTarget, double& dnx, double& dny);

            double interpolate(double a, double b, double g00, double g01, double g10, double g11);

            std::string getLocomotionMode(base::Waypoint wPos);

            bool evaluatePath(localNode* lObs, std::vector<base::Waypoint>& trajectory, uint& maxIndex, uint& minIndex);

            bool isHorizon(localNode* lNode);
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
