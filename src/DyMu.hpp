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

#include <base/Waypoint.hpp>
#include <base/samples/Frame.hpp>
#include <base/samples/RigidBodyState.hpp>
#include <vector>

#include <base-logging/Logging.hpp>

//#define INF 100000000 //Change this value to maximum available for float type

namespace PathPlanning_lib
{
// double INF = std::numeric_limits<double>::infinity();
enum node_state
{
    OPEN,
    CLOSED
};

enum repairingAproach
{
    CONSERVATIVE,  // Hazard Avoidance - FM*
    SWEEPING       // multiBiFM*
};

struct localNode
{
    base::Pose2D pose;         // In Local Units respect to Global Node
    base::Pose2D world_pose;   // In physical Units respect to World Frame
    base::Pose2D parent_pose;  // Position of Global Node Parent In Global Units
    base::Pose2D global_pose;  // Position of this local node in Global Units respect to World Frame

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
    bool hasLocalMap;  // Has it localmap?
    double raw_cost;
    double cost;
    double hazard_density;  // Ratio of obstacle area in the global node area
    double trafficability;
    double total_cost;
    unsigned int terrain;
    std::vector<std::vector<localNode*>> localMap;
    std::vector<globalNode*> nb4List;
    std::vector<globalNode*> nb8List;
    std::string nodeLocMode;
    globalNode(uint x_, uint y_, double res, std::vector<double> offset)
    {
        pose.position[0] = (double)x_;
        pose.position[1] = (double)y_;
        world_pose.position[0] = (double)x_ * res + offset[0];
        world_pose.position[1] = (double)y_ * res + offset[1];
        // terrain = (unsigned int) t_;
        // elevation = e_;

        // risk.obstacle = r_;
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

struct costCriteria
{
    int num_samples;
    double mean;
    double std_deviation;
    bool empty;
    costCriteria(int num_samples_, double mean_, double std_deviation_)
    {
        num_samples = num_samples_;
        mean = mean_;
        std_deviation = std_deviation_;
        empty = false;
    }
    costCriteria()
    {
        num_samples = 0;
        mean = 0;
        std_deviation = 0;
        empty = true;
    }

    void addData(std::vector<double> new_samples)
    {
        int n = new_samples.size();
        if (n != 0)
        {
            double sum = 0;
            for (int i = 0; i < n; i++)
            {
                sum += new_samples[i];
            }
            double new_mean = (mean * num_samples + sum) / (num_samples + n);

            if (num_samples + n - 2 > 0)
            {
                double acc_diff = 0;
                for (int i = 0; i < n; i++)
                {
                    if (!empty)
                        acc_diff += (new_samples[i] - mean) * (new_samples[i] - new_mean);
                    else
                        acc_diff += pow(new_samples[i] - new_mean, 2);
                }
                std_deviation = sqrt((pow(std_deviation, 2) * (num_samples - 1) + acc_diff)
                                     / (num_samples + n - 2));
            }
            else
                LOG_ERROR_S << "ERROR: not enough samples to obtain standard deviation.";

            num_samples += n;
            mean = new_mean;
            empty = false;
        }
    }

    void addData(int num_samples_, double mean_, double std_deviation_)
    {
        if (num_samples_ != 0)
        {
            double new_mean =
                (mean * num_samples + mean_ * num_samples_) / (num_samples + num_samples_);

            std_deviation = sqrt((pow(std_deviation, 2) * (num_samples - 1)
                                  + pow(std_deviation_, 2) * (num_samples_ - 1))
                                 / (num_samples + num_samples_ - 2));

            num_samples += num_samples_;
            mean = new_mean;
            empty = false;
        }
    }

    void addData(double new_sample)
    {
        double new_mean = (mean * num_samples + new_sample) / (num_samples + 1);

        double acc_diff;
        if (!empty) acc_diff = (new_sample - mean) * (new_sample - new_mean);
        std_deviation =
            sqrt((pow(std_deviation, 2) * (num_samples - 1) + acc_diff) / (num_samples + 1 - 2));

        num_samples += 1;
        mean = new_mean;
        empty = false;
    }

    void erase()
    {
        num_samples = 0;
        mean = 0;
        std_deviation = 0;
        empty = true;
    }
};

struct segmentedTerrain
{
    double cost;
    double slope_ratio;  // Ratio of increasing cost per degree (u/°)
    std::vector<costCriteria> criteria_info, traverse_info, rejected_info;
    std::vector<std::vector<double>> data_samples;
    bool traversed;
    segmentedTerrain()
    {
        cost = 1;
        slope_ratio = 1;
        traversed = false;
    }
    segmentedTerrain(double cost_, double slope_ratio_)
    {
        cost = cost_;
        slope_ratio = slope_ratio_;
        traversed = false;
    }
    segmentedTerrain(std::vector<costCriteria> criteria_info_)
    {
        criteria_info.resize(criteria_info_.size());
        traverse_info.resize(criteria_info_.size());
        rejected_info.resize(criteria_info_.size());
        data_samples.resize(criteria_info_.size());
        criteria_info = criteria_info_;
        traversed = true;
    }

    // Procedure to include or not new samples in the accumulated info of a criteria
    void dataAnalysis()
    {
        if (!traversed)
        {
            for (int i = 0; i < criteria_info.size(); i++)
            {
                if (data_samples[i].size() > 2)
                {
                    criteria_info[i].addData(data_samples[i]);
                    data_samples[i].erase(data_samples[i].begin(), data_samples[i].end());
                }

                if (criteria_info[i].num_samples > 29)
                {
                    traversed = true;
                    std::cout << "\033[1;32mNow we have gathered enough info of the "
                                 "current terrain.\033[0m"
                              << std::endl;
                }
            }
        }
        else
        {
            for (int i = 0; i < criteria_info.size(); i++)
            {
                if (criteria_info[i].num_samples > 29)
                {
                    if (data_samples[i].size() > 9)
                    {
                        traverse_info[i].addData(data_samples[i]);
                        if (FTest(i))
                            criteria_info[i].addData(traverse_info[i].num_samples,
                                                     traverse_info[i].mean,
                                                     traverse_info[i].std_deviation);

                        data_samples[i].erase(data_samples[i].begin(), data_samples[i].end());
                        traverse_info[i].erase();
                    }
                    if (rejected_info[i].num_samples > 29)
                    {
                        if (TTest(i))
                            criteria_info[i].addData(rejected_info[i].num_samples,
                                                     rejected_info[i].mean,
                                                     rejected_info[i].std_deviation);
                        else if (rejected_info[i].num_samples >= criteria_info[i].num_samples
                                 && rejected_info[i].std_deviation < criteria_info[i].std_deviation)
                        {
                            LOG_WARN_S << "\033[1;35mWARNING: [Criteria " << i + 1
                                       << "] The amount and quality of the rejected samples is "
                                          "greater than the saved ones, now we are using the "
                                          "rejected info as the correct one. \033[0m";
                            traverse_info[i].erase();
                            traverse_info[i].addData(criteria_info[i].num_samples,
                                                     criteria_info[i].mean,
                                                     criteria_info[i].std_deviation);
                            criteria_info[i].erase();
                            criteria_info[i].addData(rejected_info[i].num_samples,
                                                     rejected_info[i].mean,
                                                     rejected_info[i].std_deviation);
                            rejected_info[i].erase();
                            rejected_info[i].addData(traverse_info[i].num_samples,
                                                     traverse_info[i].mean,
                                                     traverse_info[i].std_deviation);
                            traverse_info[i].erase();
                        }
                    }
                }
                else
                {
                    criteria_info[i].addData(data_samples[i]);
                    data_samples[i].erase(data_samples[i].begin(), data_samples[i].end());
                }
            }
        }
    }

    // Comparison between groups with big number of samples
    bool TTest(int i)
    {
        double n1, n2, s1, s2, x1, x2, t;
        n1 = criteria_info[i].num_samples;
        n2 = rejected_info[i].num_samples;
        s1 = criteria_info[i].std_deviation;
        s2 = rejected_info[i].std_deviation;
        x1 = criteria_info[i].mean;
        x2 = rejected_info[i].mean;

        bool result = false;
        t = abs(x1 - x2) / sqrt(pow(s1, 2) / n1 + pow(s2, 2) / n2);
        if (t < 2.00) result = true;
        return result;
    }

    // Comparison between standard deviation of two groups
    bool FTest(int i)
    {
        double s1, s2, F;
        bool result;
        s1 = traverse_info[i].std_deviation;
        s2 = criteria_info[i].std_deviation;

        F = pow(s1, 2) / pow(s2, 2);
        if (F < 2.05)
            result = studentTTest(i);
        else
            result = cochranTTest(i);
        return result;
    }

    // Comparison of std deviation of the mean between two groups with similar std deviation
    bool studentTTest(int i)
    {
        double n1, n2, s1, s2, x1, x2, sp, t;
        n1 = criteria_info[i].num_samples;
        n2 = traverse_info[i].num_samples;
        s1 = criteria_info[i].std_deviation;
        s2 = traverse_info[i].std_deviation;
        x1 = criteria_info[i].mean;
        x2 = traverse_info[i].mean;

        sp = sqrt(((n1 - 1) * pow(s1, 2) + (n2 - 1) * pow(s2, 2)) / (n1 + n2 - 2));
        t = sqrt(n1 * n2 / (n1 + n2)) * (x1 - x2) / sp;
        if (t < 2.02)
            return true;
        else
        {
            LOG_WARN_S << "\033[1;35mWARNING: [Criteria " << i + 1
                       << "] Sample rejected after Student T test.\033[0m";
            rejected_info[i].addData(traverse_info[i].num_samples,
                                     traverse_info[i].mean,
                                     traverse_info[i].std_deviation);

            return false;
        }
    }

    // Comparison of std deviation of the mean between two groups with different std deviation
    bool cochranTTest(int i)
    {
        double n1, n2, s1, s2, x1, x2, tcal, ttab;
        n1 = criteria_info[i].num_samples;
        n2 = traverse_info[i].num_samples;
        s1 = criteria_info[i].std_deviation;
        s2 = traverse_info[i].std_deviation;
        x1 = criteria_info[i].mean;
        x2 = traverse_info[i].mean;

        tcal = (x1 - x2) / sqrt(pow(s1, 2) / n1 + pow(s2, 2) / n2);
        ttab =
            (2.02 * pow(s1, 2) / n1 + 2.22 * pow(s2, 2) / n2) / (pow(s1, 2) / n1 + pow(s2, 2) / n2);
        if (tcal < ttab)
            return true;
        else
        {
            LOG_WARN_S << "\033[1;35mWARNING: [Criteria " << i + 1
                       << "] Sample rejected after Cochran T test.\033[0m";
            return false;
        }
    }
};

//__DYMU_PATH_PLANNER_CLASS__
class DyMuPathPlanner
{
  private:
    // Global Layer
    std::vector<std::vector<globalNode*>> global_layer;
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

    // Cost ratio updating
    // Vector of terrain segmentation info
    std::vector<segmentedTerrain> terrain_vector;
    // Vector of criteria weights
    std::vector<double> weights;
    // Number of segmented terrains
    int num_terrains;
    // Number of parameters used for cost updating
    int num_criteria;
    // Base speed to obtain costs
    double base_speed;

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
    globalNode* global_goal;
    // Local Node containing the position of the agent
    localNode* local_agent;
    // Total Cost needed by the agent to reach the goal
    double remaining_total_cost;

    // Index to current path in which the local path meets global
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
    bool initGlobalLayer(double globalres,
                         double localres,
                         uint num_nodes_X,
                         uint num_nodes_Y,
                         std::vector<double> offset);

    bool setCostMap(std::vector<std::vector<double>> cost_map);

    bool computeCostMap(std::vector<double> cost_data,
                        std::vector<double> slope_values,
                        std::vector<std::string> locomotionModes,
                        std::vector<std::vector<double>> elevation,
                        std::vector<std::vector<double>> terrainMap);

    // Slope is calculated for nodeTarget
    void calculateSlope(globalNode* nodeTarget);

    void calculateNominalCost(globalNode* nodeTarget, int range, int numLocs);

    void smoothCost(globalNode* nodeTarget);

    // Returns global node (i,j)
    globalNode* getGlobalNode(uint i, uint j);

    bool setGoal(base::Waypoint wGoal);

    bool computeTotalCostMap(base::Waypoint wPos);
    bool computeEntireTotalCostMap();

    bool isSafeNode(globalNode* global_node);
    bool isFullyClosedNode(globalNode* global_node);

    void resetTotalCostMap();
    void resetGlobalNarrowBand();

    void propagateGlobalNode(globalNode* nodeTarget);

    globalNode* minCostGlobalNode();

    globalNode* getNearestGlobalNode(base::Pose2D pos);
    globalNode* getNearestGlobalNode(base::Waypoint wPos);

    std::vector<base::Waypoint> getPath(base::Waypoint wPos);

    bool computeGlobalPath(base::Waypoint wPos);

    base::Waypoint computeNextGlobalWaypoint(base::Waypoint& wPos, double tau);

    void gradientNode(globalNode* nodeTarget, double& dnx, double& dny);

    double interpolate(double a, double b, double g00, double g01, double g10, double g11);

    std::string getLocomotionMode(base::Waypoint wPos);

    std::vector<std::vector<double>> getTotalCostMatrix();
    std::vector<std::vector<double>> getGlobalCostMatrix();
    std::vector<std::vector<double>> getHazardDensityMatrix();
    std::vector<std::vector<double>> getTrafficabilityMatrix();

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
                              base::Time& localTime);

    void expandRisk();

    localNode* maxRiskNode();

    void propagateRisk(localNode* nodeTarget);

    void setHorizonCost(localNode* horizonNode);

    double getTotalCost(localNode* lNode);

    localNode* computeLocalPropagation(base::Waypoint wInit, base::Waypoint wOvertake);

    void propagateLocalNode(localNode* nodeTarget);

    localNode* minCostLocalNode(double Tovertake, double minC);

    localNode* minCostLocalNode(localNode* reachNode);

    std::vector<base::Waypoint> getLocalPath(localNode* lSetNode, base::Waypoint wInit, double tau);

    bool computeLocalWaypointGDM(base::Waypoint& wPos, double tau);
    base::Waypoint computeLocalWaypointDijkstra(localNode* lNode);

    void gradientNode(localNode* nodeTarget, double& dnx, double& dny);

    bool evaluatePath(uint starting_index);

    bool isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex);

    // void repairPath(std::vector<base::Waypoint>& trajectory, uint minIndex, uint maxIndex);
    int repairPath(base::Waypoint wInit, uint index);

    // TODO: Make function to change repairing approach anytime

    std::vector<std::vector<double>> getRiskMatrix(base::Waypoint rover_pos);
    std::vector<std::vector<double>> getDeviationMatrix(base::Waypoint rover_pos);

    int getReconnectingIndex();

    // COST RATIO UPDATING AFTER TRAVERSE (CoRa)

    // Initialize the algorithm including criteria vectors and other variables
    bool initCoRaMethod(int num_terrains_, int num_criteria_, std::vector<double> weights_);

    // Obtain the terrain currently being traversed
    int getTerrain(base::samples::RigidBodyState current_pos);

    // Add a sample to the corresponding terrain and criteria
    bool fillTerrainInfo(int terrain_id, std::vector<double> data);

    // Get a new cost_data vector using the accumulated info of the traverse
    std::vector<double> updateCost();

    // Obtain ratios about which terrain is harder to traverse
    std::vector<double> computeCostRatio();
};

}  // end namespace

#endif  // _PATHPLANNING_LIBRARIES_HPP_
