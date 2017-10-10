#ifndef _PATHPLANNING_LIBRARIES_NODEMAP_HPP_
#define _PATHPLANNING_LIBRARIES_NODEMAP_HPP_

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
  enum locomotionMode
  {//Must be external
      DRIVING,
      WHEEL_WALKING
  };

  enum nodeInterpType
  {
      NON_INTERPOLATED,
      VERTICAL,
      HORIZONTAL
  };

  enum nodeState
  {
      OPEN,
      CLOSED,
      HIDDEN
  };

  struct riskProperties
  {
      double obstacle;
      double slope;
      double material;
      double uncertainty;
  };

  struct terrainType
  {
      double cost;
      std::string optimalLM;
  };

    struct Node {
      //Node Parameters
        base::Pose2D pose; // X-Y-heading
        double slope;
        double aspect;
        double elevation;
        riskProperties risk;
        nodeState state;
        base::samples::RigidBodyState roverPose;
        Node *nodeParent;
        unsigned int terrain; //Index to terrainList
        //terrainType terrain;
        bool isObstacle;
        double total_cost;
        double slip_ratio;
        double cost_modifier;
        double dCostX;
        double dCostY;
        double heuristicCost;//for A*
        std::vector<Node*> nb8List; //8-neighbour nodes List
        std::vector<Node*> nb4List; //4-Neighbourhood List
        std::vector< std::vector<Node*> >* localNodeMatrix;
        std::string nodeLocMode;


      Node()
      {
          state = OPEN;
          nodeParent = NULL;
      }

      Node(uint x_, uint y_, double e_, double c_, nodeState s_)
      {
          pose.position[0] = (double)x_;
          pose.position[1] = (double)y_;
        // Calculate slope and aspect
          terrain = (unsigned int) c_;
          elevation = e_;
          //risk.obstacle = r_;
          risk.obstacle = 0;
          isObstacle = false;
          slip_ratio = 0;
          total_cost = INF;
          state = s_;
          nodeParent = NULL;
          localNodeMatrix = NULL;
          nodeLocMode = "DRIVING";
      }
  };

  class NodeMap
  {
      public:
          double scale;
          double minWork;
          base::Time t1;
          base::Pose2D globalOffset;
          std::vector< std::vector<Node*> > nodeMatrix;
          base::Waypoint actualPose;
          base::Waypoint goalPose;
          Node * nodeActualPos;
          Node * nodeGoal;
          Node * getNode(uint i, uint j);
          void setActualPos(base::Waypoint wPos);
          void setGoal(base::Waypoint wGoal);
          Node * getActualPos();
          Node * getGoal();
          std::vector<Node*> horizonNodes;
          std::vector<Node*> closedNodes;
          std::vector<Node*> obstacleNodes;
          NodeMap();
          NodeMap(envire::TraversabilityGrid* travGrid);
          NodeMap(double size, base::Pose2D pos,
                  std::vector< std::vector<double> > elevation,
                  std::vector< std::vector<double> > cost, nodeState state);
          void resetPropagation();
          void updateNodeMap(envire::TraversabilityGrid* travGrid);
          void makeNeighbourhood();
          void makeNeighbourhood(Node* n, uint i, uint j);
          Node * getNeighbour(Node* n, uint k);
          void createLocalNodeMap(envire::TraversabilityGrid* travGrid);
          void hidAll();
          bool isObstacle(Node* n);
          bool updateVisibility(base::Waypoint wPos, NodeMap* globalMap, bool initializing);
          void setHorizonCost(Node* horizonNode, NodeMap* globalMap);
          envire::ElevationGrid* getEnvirePropagation(base::Waypoint wPos, bool crop, double work_scale);
          envire::ElevationGrid* getEnvireRisk();
          envire::TraversabilityGrid* getLocalEnvireState(base::Waypoint wPos, bool crop);
          envire::TraversabilityGrid* getGlobalEnvireState();
          void expandRisk(std::vector<Node*>& expandableNodes);
          void propagateRisk(Node* nodeTarget, std::vector<Node*>& expandableNodes);
          Node * maxRiskNode(std::vector<Node*>& expandableNodes);
          std::string getLocomotionMode(base::Waypoint wPos);
          bool updateNodePower(double new_power, base::Waypoint wPos, bool value_inverted);
          bool updateNodeSlip(double dSlip, base::Waypoint wPos);
          void resetHorizonNodes(NodeMap* globalMap);
  };
} // end namespace motion_planning_libraries_nodemap

#endif // _MOTION_PLANNING_LIBRARIES_NODEMAP_HPP_
