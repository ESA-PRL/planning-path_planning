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
    enum local_state
    {
        HID,
        OBSTACLE,
        TRAVERSABLE_OPEN,
        TRAVERSABLE_CLOSED
    };

    enum global_state
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
        local_state state;
        std::vector<localNode*> nb4List;
        bool isObstacle;
        localNode(uint x_, uint y_, base::Pose2D _parent_pose)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            parent_pose = _parent_pose;
            state = HID;
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
        global_state state;
        bool isObstacle;
        bool hasLocalMap; //Has it localmap?
        double total_cost;
        unsigned int terrain;
        std::vector< std::vector<localNode*> > localMap;
        std::vector<globalNode*> nb4List;
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
            double global_scale;
            base::Pose2D global_offset;
            double local_scale;
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
            std::vector<localNode*> local_narrowBand;
            std::vector<localNode*> localExpandableObstacles;
            std::vector<localNode*> horizonNodes;
            std::vector<localNode*> local_closedNodes;
            globalNode * global_goalNode;
            localNode * local_goalNode;
            localNode * local_actualPose;

            base::Time t1;

            void initGlobalMap(double gScale, uint ratio,
                               base::Pose2D offset,
                               std::vector< std::vector<double> > elevation,
                               std::vector< std::vector<double> > cost);

            globalNode* getGlobalNode(uint i, uint j);

            void calculateSlope(globalNode* nodeTarget);

            void setGoal(base::Waypoint wGoal);

            void calculateGlobalPropagation();

            globalNode* minCostGlobalNode();

            void propagateGlobalNode(globalNode* nodeTarget);

            envire::ElevationGrid* getEnvireGlobalPropagation();

            envire::ElevationGrid* getLocalTotalCost(base::Waypoint wPos);

            void createLocalMap(globalNode* gNode);

            localNode* introducePixelInMap(base::Vector2d pos, local_state state, bool& newVisible);

            localNode* getLocalNode(base::Vector2d pos);
            localNode* getLocalNode(base::Waypoint wPos);

            void expandGlobalNode(globalNode* gNode);

            bool simUpdateVisibility(base::Waypoint wPos, std::vector< std::vector<double> >& costMatrix, double res, bool initializing, double camHeading);

            globalNode* getNearestGlobalNode(base::Vector2d pos);
            globalNode* getNearestGlobalNode(base::Waypoint wPos);

            void loadLocalArea(base::Waypoint wPos);

            void expandRisk();

            localNode* maxRiskNode();

            void propagateRisk(localNode* nodeTarget);

            envire::TraversabilityGrid* getEnvireLocalState(base::Waypoint wPos);

            envire::ElevationGrid* getEnvireRisk(base::Waypoint wPos);

            void setHorizonCost(localNode* horizonNode);

            void calculateLocalPropagation(base::Waypoint wGoal, base::Waypoint wPos);

            void propagateLocalNode(localNode* nodeTarget);

            localNode* minCostLocalNode();

            bool getPath(base::Waypoint wStart, double tau, std::vector<base::Waypoint>& trajectory);

            bool calculateNextWaypoint(base::Waypoint& wPos, double tau);

            void gradientNode(localNode* nodeTarget, double& dnx, double& dny);

            double interpolate(double a, double b, double g00, double g01, double g10, double g11);

            std::string getLocomotionMode(base::Waypoint wPos);

            bool isHorizon(localNode* lNode);
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
