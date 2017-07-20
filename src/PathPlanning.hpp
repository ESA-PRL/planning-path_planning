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

//__PATH_PLANNER_CLASS__
    class PathPlanning
    {
        private:
            base::samples::RigidBodyState mStartPose;
            double pathCost;

        public:
            PathPlanning();
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
                              NodeMap * nodes);
            void initNarrowBand(NodeMap * nodes, base::Waypoint wGoal);
            void getHorizonCost(NodeMap* nodes, Node* horizonNode, base::Waypoint wGoal);
            void propagationFunction(Node* nodeTarget, double scale);
            void getPath(NodeMap * nodes, double tau,
 					                     std::vector<base::Waypoint>& trajectory,
                               std::vector< short int >& locVector);
            Node* minCostNode();
            void gradientNode(Node* nodeTarget, double& dx, double& dy);
            Node* calculateNextWaypoint(double x, double y, double& dCostX,
                                     double& dCostY, double& height,
                                     short int& locMode, NodeMap * nodes);
    };

} // end namespace motion_planning_libraries

#endif // _MOTION_PLANNING_LIBRARIES_HPP_