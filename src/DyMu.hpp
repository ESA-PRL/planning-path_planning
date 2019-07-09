#ifndef _PATHPLANNING_LIBRARIES_HPP_
#define _PATHPLANNING_LIBRARIES_HPP_

/*********************************DyMu******************************************
                    -Dynamic Multilayered Path Planner-
                        ARES (ESA-UMA Collaboration)
                University of Malaga - European Space Agency
                                -Author-
                        J. Ricardo Sanchez Ibanez
                             -Contact mail-
                            ricardosan@uma.es
                              -Supervisors-
                        Carlos J. Perez del Pulgar
                             Martin Azkarate
*******************************************************************************/

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Frame.hpp>
#include <base/Waypoint.hpp>
#include <base/Trajectory.hpp>
#include <vector>
#include <fstream>

#include <base-logging/Logging.hpp>

#define INF 100000000 //Change this value to maximum available for float type

namespace PathPlanning_lib
{
    enum node_state
    {
        OPEN,
        CLOSED
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
        double deviation;
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
            deviation = INF;
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
        node_state state;
        bool isObstacle;
        bool hasLocalMap; //Has it localmap?
        double raw_cost;
        double cost;
        double obstacle_ratio; //Ratio of obstacle area in the global node area
        double total_cost;
        unsigned int terrain;
        std::vector< std::vector<localNode*> > localMap;
        std::vector<globalNode*> nb4List;
        std::vector<globalNode*> nb8List;
        std::string nodeLocMode;
        globalNode(uint x_, uint y_, double res, base::Pose2D offset)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            world_pose.position[0] = (double)x_*res + offset.position[0];
            world_pose.position[1] = (double)y_*res + offset.position[1];
            //terrain = (unsigned int) t_;
            //elevation = e_;

            //risk.obstacle = r_;
            isObstacle = false;
            hasLocalMap = false;
            raw_cost = 0.0;
            cost = 0.0;
            total_cost = INF;
            state = OPEN;
            nodeLocMode = "DONT_CARE";
            obstacle_ratio = 0.0;
        }
    };

//__DYMU_PATH_PLANNER_CLASS__
    class DyMuPathPlanner
    {
        private:
          // Global Layer
            std::vector< std::vector<globalNode*> > global_layer;
          // Dimensions
            uint num_nodes_X;
            uint num_nodes_Y;
          // Global Resolution (same for X and Y axii)
            double global_res;
          // Offset of Global Node (0,0) with respect to world frame
            base::Pose2D global_offset;
          // Local Resolution (same for X and Y axii)
            double local_res;
          // Local Nodes contained within a Global Node edge
            uint res_ratio;

          // Local Repairing Parameters
          // Risk Distance = Risky proximity to obstacles
            double risk_distance;
          // Distance to reconnect after first safe waypoint
            // - Only relevant for Conservative Approach (Hazard Avoidance mode)
            double reconnect_distance;
          // Parameter that controls curvature of resulting repaired paths
            double risk_ratio;
          // Vector of slope values from 0 to the maximum feasible slope
            std::vector<double> slope_range;
          // Vector of available locomotion modes
            std::vector<std::string> locomotion_modes;

        public:
          // -- PARAMETERS --

          // The narrow band is formed by those open nodes that have been
          // already visited by the FMM solver
            std::vector<globalNode*> global_narrowband;
          // Compilation of all visited Global Nodes
            std::vector<globalNode*> global_propagated_nodes;
          // Same concept of narrow band as in the global computation
            std::vector<localNode*> local_narrowband;
          // Obstacle Local Nodes whose associated risk must be computed
            std::vector<localNode*> local_expandable_obstacles;
          // Same concept as in global
            std::vector<localNode*> local_propagated_nodes;
          // The last computed path
            std::vector<base::Waypoint> current_path;
          // LookUp Table containing cost values per terrain and slope value
            std::vector<double> cost_lutable;
          // Global Node containing the goal
            globalNode * global_goal;
          // Local Node containing the position of the agent
            localNode * local_agent;
          // Total Cost needed by the agent to reach the goal
            double remaining_total_cost;

          // -- FUNCTIONS --
          // Class Constructor
            DyMuPathPlanner(std::vector<double> costData,
                         std::vector<double> slope_values,
                         std::vector<std::string> locomotion_modes,
                           double risk_distance,
                           double reconnect_distance,
                           double risk_ratio);
          // Class Destructor
            ~DyMuPathPlanner();
          // Initialization of Global Layer using Elevation and Terrain Maps
            bool initGlobalLayer(double globalres,  double localres,
                                 uint num_nodes_X, uint num_nodes_Y,
                                 base::Pose2D offset);

            bool setCostMap(std::vector< std::vector<double> > cost_map);

            bool computeCostMap(std::vector< std::vector<double> > elevation,
                                std::vector< std::vector<double> > terrainMap,
                                bool to_be_smoothed);

          // Returns global node (i,j)
            globalNode* getGlobalNode(uint i, uint j);
          // Slope is calculated for nodeTarget
            void calculateSlope(globalNode* nodeTarget);

            bool setGoal(base::Waypoint wGoal);

            void calculateNominalCost(globalNode* nodeTarget,
                                      bool to_be_smoothed);

            void smoothCost(globalNode* nodeTarget);

            bool computeTotalCostMap(base::Waypoint wPos);
            bool computeEntireTotalCostMap();

            void resetTotalCostMap();

            globalNode* minCostGlobalNode();

            void propagateGlobalNode(globalNode* nodeTarget);

            void createLocalMap(globalNode* gNode);

            localNode* getLocalNode(base::Pose2D pos);
            localNode* getLocalNode(base::Waypoint wPos);

            void expandGlobalNode(globalNode* gNode);

            globalNode* getNearestGlobalNode(base::Pose2D pos);
            globalNode* getNearestGlobalNode(base::Waypoint wPos);

            bool computeLocalPlanning(base::Waypoint wPos,
                                  base::samples::frame::Frame traversabilityMap,
                                  double res,
                                  std::vector<base::Waypoint>& trajectory,
                                  bool keepOldWaypoints, base::Time &localTime);

            void expandRisk();

            localNode* maxRiskNode();

            void propagateRisk(localNode* nodeTarget);

            void setHorizonCost(localNode* horizonNode);

            double getTotalCost(localNode* lNode);

            double getTotalCost(base::Waypoint wInt);

            localNode * computeLocalPropagation(base::Waypoint wInit, base::Waypoint wOvertake, bool keepOldWaypoints);

            void propagateLocalNode(localNode* nodeTarget);

            localNode* minCostLocalNode(double Tovertake, double minC);

            localNode* minCostLocalNode(localNode* reachNode);

            std::vector<base::Waypoint> getLocalPath(localNode * lSetNode,
                                                     base::Waypoint wInit,
                                                     double tau);
            std::vector<base::Waypoint> getGlobalPath(base::Waypoint wPos);

            std::vector<base::Waypoint> getNewPath(base::Waypoint wPos);

            bool computeLocalWaypointGDM(base::Waypoint& wPos, double tau);
            base::Waypoint computeLocalWaypointDijkstra(localNode * lNode);
            base::Waypoint calculateNextGlobalWaypoint(base::Waypoint& wPos, double tau);

            void gradientNode(localNode* nodeTarget, double& dnx, double& dny);
            void gradientNode(globalNode* nodeTarget, double& dnx, double& dny);

            double interpolate(double a, double b, double g00, double g01, double g10, double g11);

            std::string getLocomotionMode(base::Waypoint wPos);

            void evaluatePath(std::vector<base::Waypoint>& trajectory);

            bool isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex, std::vector<base::Waypoint> trajectory);

            void repairPath(std::vector<base::Waypoint>& trajectory, uint minIndex, uint maxIndex);
            void repairPath(std::vector<base::Waypoint>& trajectory, base::Waypoint wInit, std::vector<base::Waypoint>& current_path, uint index, bool keepOldWaypoints);
    };

} // end namespace

#endif // _PATHPLANNING_LIBRARIES_HPP_
