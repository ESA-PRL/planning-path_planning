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
#include <vector>

#include <base-logging/Logging.hpp>

//#define INF 100000000 //Change this value to maximum available for float type

namespace PathPlanning_lib
{
    //double INF = std::numeric_limits<double>::infinity();
    enum node_state
    {
        OPEN,
        CLOSED
    };

    enum repairingAproach
    {
        CONSERVATIVE, // Hazard Avoidance - FM*
        SWEEPING // multiBiFM*
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
            deviation = std::numeric_limits<double>::infinity();
            total_cost = std::numeric_limits<double>::infinity();
            isObstacle = false;
            risk = 0.0;
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
        double hazard_density; //Ratio of obstacle area in the global node area
        double trafficability;
        double total_cost;
        unsigned int terrain;
        std::vector< std::vector<localNode*> > localMap;
        std::vector<globalNode*> nb4List;
        std::vector<globalNode*> nb8List;
        std::string nodeLocMode;
        globalNode(uint x_, uint y_, double res, std::vector<double>  offset)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            world_pose.position[0] = (double)x_*res + offset[0];
            world_pose.position[1] = (double)y_*res + offset[1];
            //terrain = (unsigned int) t_;
            //elevation = e_;

            //risk.obstacle = r_;
            isObstacle = false;
            hasLocalMap = false;
            raw_cost = 0.0;
            cost = 0.0;
            total_cost = std::numeric_limits<double>::infinity();
            state = OPEN;
            nodeLocMode = "DONT_CARE";
            hazard_density = 0.0;
            trafficability = 1.0;
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
            std::vector<double> global_offset;
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
          // Approach chosen to do the repairings
            repairingAproach repairing_approach;

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

          //Index to current path in which the local path meets global
            int reconnecting_index;

          // -- FUNCTIONS --
          // Class Constructor
            DyMuPathPlanner(double risk_distance,
                            double reconnect_distance,
                            double risk_ratio,
                            repairingAproach input_approach);
          // Class Destructor
            ~DyMuPathPlanner();
          // Initialization of Global Layer using Elevation and Terrain Maps
            bool initGlobalLayer(double globalres,  double localres,
                                 uint num_nodes_X, uint num_nodes_Y,
                                 std::vector<double> offset);

            bool setCostMap(std::vector< std::vector<double> > cost_map);

            bool computeCostMap(std::vector<double> costData,
                            std::vector<double> slope_values,
                            std::vector<std::string> locomotionModes,
                            std::vector< std::vector<double> > elevation,
                                std::vector< std::vector<double> > terrainMap);

          // Slope is calculated for nodeTarget
            void calculateSlope(globalNode* nodeTarget);

            void calculateNominalCost(globalNode* nodeTarget, int range,
                                      int numLocs);

            void smoothCost(globalNode* nodeTarget);

          // Returns global node (i,j)
            globalNode* getGlobalNode(uint i, uint j);

            bool setGoal(base::Waypoint wGoal);

            bool computeTotalCostMap(base::Waypoint wPos);
            bool computeEntireTotalCostMap();

            void resetTotalCostMap();
            void resetGlobalNarrowBand();

            void propagateGlobalNode(globalNode* nodeTarget);

            globalNode* minCostGlobalNode();

            globalNode* getNearestGlobalNode(base::Pose2D pos);
            globalNode* getNearestGlobalNode(base::Waypoint wPos);

            std::vector<base::Waypoint> getPath(base::Waypoint wPos);

            bool computeGlobalPath(base::Waypoint wPos);

            base::Waypoint computeNextGlobalWaypoint(base::Waypoint& wPos,
                                                       double tau);

            void gradientNode(globalNode* nodeTarget, double& dnx, double& dny);

            double interpolate(double a, double b, double g00, double g01,
                               double g10, double g11);

            std::string getLocomotionMode(base::Waypoint wPos);

            std::vector< std::vector<double> > getTotalCostMatrix();
            std::vector< std::vector<double> > getGlobalCostMatrix();
            std::vector< std::vector<double> > getHazardDensityMatrix();
            std::vector< std::vector<double> > getTrafficabilityMatrix();

            double getTotalCost(base::Waypoint wInt);

          // LOCAL PATH REPAIRING

            void createLocalMap(globalNode* gNode);

            localNode* getLocalNode(base::Pose2D pos);
            localNode* getLocalNode(base::Waypoint wPos);

            void subdivideGlobalNode(globalNode* gNode);

            bool computeLocalPlanning(base::Waypoint wPos,
                                  base::samples::frame::Frame traversabilityMap,
                                  double res,
                                  std::vector<base::Waypoint>& trajectory,
                                  base::Time &localTime);

            void expandRisk();

            localNode* maxRiskNode();

            void propagateRisk(localNode* nodeTarget);

            void setHorizonCost(localNode* horizonNode);

            double getTotalCost(localNode* lNode);

            localNode * computeLocalPropagation(base::Waypoint wInit, base::Waypoint wOvertake);

            void propagateLocalNode(localNode* nodeTarget);

            localNode* minCostLocalNode(double Tovertake, double minC);

            localNode* minCostLocalNode(localNode* reachNode);

            std::vector<base::Waypoint> getLocalPath(localNode * lSetNode,
                                                     base::Waypoint wInit,
                                                     double tau);




            bool computeLocalWaypointGDM(base::Waypoint& wPos, double tau);
            base::Waypoint computeLocalWaypointDijkstra(localNode * lNode);


            void gradientNode(localNode* nodeTarget, double& dnx, double& dny);





            bool evaluatePath(uint starting_index);

            bool isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex, std::vector<base::Waypoint> trajectory);

            //void repairPath(std::vector<base::Waypoint>& trajectory, uint minIndex, uint maxIndex);
            int repairPath(base::Waypoint wInit, uint index);

            //TODO: Make function to change repairing approach anytime

            std::vector< std::vector<double> > getRiskMatrix(base::Waypoint rover_pos);
            std::vector< std::vector<double> > getDeviationMatrix(base::Waypoint rover_pos);
    };

} // end namespace

#endif // _PATHPLANNING_LIBRARIES_HPP_
