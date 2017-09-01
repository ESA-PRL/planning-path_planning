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
#include <envire/operators/SimpleTraversability.hpp>
#include "NodeMap.hpp"
//#include <orocos/envire/Orocos.hpp>

#define INF 100000000

namespace envire {
    class Environment;
    class FrameNode;
    class TraversabilityGrid;
}

namespace PathPlanning_lib
{
    enum plannerType
    {//Must be external
        LOCAL_PLANNER,
        GLOBAL_PLANNER
    };
//__PATH_PLANNER_CLASS__
    class PathPlanning
    {
        private:
            base::samples::RigidBodyState mStartPose;
            double pathCost;
            plannerType type;
        public:
            PathPlanning(plannerType _type);
            ~PathPlanning();
            bool setStartNode(base::Waypoint wStart);
            std::vector< std::vector<unsigned int*> > costMap;
	          std::vector< std::vector<double*> > riskMap;
	          std::vector< soilType* > terrainList;
            std::vector<Node*> narrowBand;
            base::Time t1;

	          void initTerrainList(std::vector< std::vector<double> > soils);
	          void costFunction(uint Terrain, double& Power, locomotionMode& lM);

          // Field D Star Functions
            void fieldDStar(base::Waypoint wStart, base::Waypoint wGoal,
                            std::vector<base::Waypoint>& trajectory,
                            std::vector< short int >& locVector,
                            NodeMap * nodes);
            void getInterpolatedPath(Node * nodeStart, base::Waypoint wGoal,
                                     NodeMap nodes, double tau,
                                     std::vector<base::Waypoint>& trajectory,
                                     std::vector< short int >& locVector);
            void setKey(Node * nodeTarget, base::Waypoint wStart);
            double getHeuristic(Node * nodeTarget, base::Waypoint wStart);
            Node * getMinorKey( Node * nodeA, Node * nodeB);
            void updateState(Node * nodeTarget, Node * nodeGoal,
                             base::Waypoint wStart, std::list<Node*>& openList);
            void computeCost(Node * nodeTarget, double& di, double& dj,
                             double& cost);
            void costInterpolation(double g1, double g2, uint cTerrain,
                                   uint bTerrain, double di, double dj,
                                   double& dk, double& vs);

          // Fast Marching Functions
            void fastMarching(base::Waypoint wStart, base::Waypoint wGoal,
                              NodeMap * nodes, NodeMap * globalNodes);
            void initNarrowBand(NodeMap * nodes, base::Waypoint wGoal, NodeMap * globalNodes);
            void getHorizonCost(NodeMap* localMap, Node* horizonNode, NodeMap* globalMap);
            void propagationFunction(Node* nodeTarget, double scale);
            bool getPath(NodeMap * nodes, double tau,
 					                     std::vector<base::Waypoint>& trajectory,
                               std::vector< short int >& locVector, int currentSegment);
            Node* minCostNode();
            void gradientNode(Node* nodeTarget, double& dx, double& dy);
            bool calculateNextWaypoint(double x, double y, double& dCostX,
                                     double& dCostY, double& height,
                                     short int& locMode, NodeMap * nodes, double& risk);
            double interpolate(double a, double b, double g00, double g01, double g10, double g11);
            bool isHorizon(Node* n);
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_
