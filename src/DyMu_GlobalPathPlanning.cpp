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

#include "DyMu.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

/*****************************CONSTRUCTOR**************************************/
DyMuPathPlanner::DyMuPathPlanner(double risk_distance,
                                 double reconnect_distance,
                                 double risk_ratio,
                                 repairingAproach input_approach):
                           risk_distance(risk_distance),
                           reconnect_distance(reconnect_distance),
                           risk_ratio(risk_ratio),
                           repairing_approach(input_approach)
{
    global_goal = NULL;
    reconnecting_index = 0;
}


/******************************DESTRUCTOR**************************************/
DyMuPathPlanner::~DyMuPathPlanner()
{
}



/*******************GLOBAL_LAYER_INITIALIZATION********************************/
bool DyMuPathPlanner::initGlobalLayer(double globalres, double localres,
                                      uint numnodesX, uint numnodesY,
                                      std::vector<double> offset)
{
    global_res = globalres;
    local_res = localres;
    num_nodes_X = numnodesX;
    num_nodes_Y = numnodesY;
    res_ratio = (uint)(global_res/local_res);
    global_offset = offset;
    std::vector<globalNode*> nodeRow;
    for (uint j = 0; j < num_nodes_Y; j++)
    {
        for (uint i = 0; i < num_nodes_X; i++)
            nodeRow.push_back(new globalNode(i, j, global_res, offset));
        global_layer.push_back(nodeRow);
        nodeRow.clear();
    }

  // NEIGHBOURHOOD
    for (uint j = 0; j < num_nodes_Y; j++)
    {
        for (uint i = 0; i < num_nodes_X; i++)
        {

            //                 4 - Neighbourhood
            //                     nb4List[3]
            //                      (i, j+1)
            //                         ||
            //         nb4List[1] __ target __ nb4List[2]
            //          (i-1, j)  __ (i, j) __  (i+1, j)
            //                         ||
            //                     nb4List[0]
            //                      (i, j-1)

            global_layer[j][i]->nb4List.clear();
            global_layer[j][i]->nb4List.push_back(getGlobalNode(i,j-1));
            global_layer[j][i]->nb4List.push_back(getGlobalNode(i-1,j));
            global_layer[j][i]->nb4List.push_back(getGlobalNode(i+1,j));
            global_layer[j][i]->nb4List.push_back(getGlobalNode(i,j+1));

            //                 8 - Neighbourhood
            //       nb4List[3] __ nb4List[2] __ nb4List[1]
            //       (i-1, j+1) __  (i, j+1)  __ (i+1, j+1)
            //           ||            ||            ||
            //       nb4List[4] __   target   __ nb4List[0]
            //        (i-1, j)  __   (i, j)   __  (i+1, j)
            //           ||            ||            ||
            //       nb4List[5] __ nb4List[6] __ nb4List[7]
            //       (i-1, j-1) __  (i, j-1)  __ (i+1, j-1)

           global_layer[j][i]->nb8List.push_back(getGlobalNode(i+1,j));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i+1,j+1));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i,  j+1));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i-1,j+1));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i-1,j));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i-1,j-1));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i,  j-1));
           global_layer[j][i]->nb8List.push_back(getGlobalNode(i+1,j-1));

        }
    }

    return true;
}

/*******************USING PREVIOUSLY DEFINED COST MAP**************************/
// Straightforward way to assign cost values to the global Layer
// A cost map is provided by the user, having the same size as the global layer
bool DyMuPathPlanner::setCostMap(std::vector< std::vector<double> > cost_map)
{
    double cost;
    if ((cost_map.size() != num_nodes_Y)||(cost_map[0].size() != num_nodes_X))
        return false;
    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
        {
            cost = cost_map[j][i];
            global_layer[j][i]->cost = cost;
            if (cost<=0)
            {
                global_layer[j][i]->isObstacle = true;
                global_layer[j][i]->trafficability = 0.0;
                global_layer[j][i]->hazard_density = 1.0;
            }
        }
    return true;
}

/*******************COMPUTE COST MAP FROM SLOPE MAP****************************/
// - cost_lutable = LookUp Table of cost in the form of an array
// - slope_range = discrete series of slope values
// - locomotion_modes = strings indicating the locomotion mode of the vehicle
// Example:
//     Slope = [S1, S2, S3] <-- This is a sorted array!
//                Locomotion 1  -    Locomotion 2
// Terrain 1      C1 - C2 - C3  -    C4 - C5 - C6
// Terrain 2      C7 - C8 - C9  -  C10 - C11 - C12
// Then:
// - cost_lutable = [Cost1,Cost2,Cost3,Cost4,Cost5,Cost6,Cost7,Cost8,Cost9]
// - slope_range = [Slope1,Slope2,Slope3,Slope4,Slope5]
// - locomotion_modes = [Locomotion1]

// First, slope values are computed
// Then, by means of the Cost LookUp Table, cost values are assigned
// Cost may be smoothed to eliminate heavy discontinuities
bool DyMuPathPlanner::computeCostMap( std::vector<double> costData,
                                      std::vector<double> slope_values,
                                      std::vector<std::string> locomotionModes,
                                  std::vector< std::vector<double> > elevation,
                                 std::vector< std::vector<double> > terrainMap)
{
    this->cost_lutable = costData;
    this->slope_range = slope_values;
    this->locomotion_modes = locomotionModes;

    int range = slope_range.size();
    int numLocs = locomotion_modes.size();
    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
        {
            global_layer[j][i]->elevation = elevation[j][i];
            if ((i == 0)||(j==0)||(i==num_nodes_X-1)||(j==num_nodes_Y-1))
                global_layer[j][i]->terrain = 0; //Ensures borders are obstacles
            else
                global_layer[j][i]->terrain = terrainMap[j][i];
        }
    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
        {
            calculateSlope(global_layer[j][i]);
            calculateNominalCost(global_layer[j][i], range, numLocs);
            if (global_layer[j][i]->isObstacle)
            {
                global_layer[j][i]->trafficability = 0.0;
                global_layer[j][i]->hazard_density = 1.0;
            }
        }
    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
            smoothCost(global_layer[j][i]);
    return true;
}

/*************************COMPUTATION OF SLOPE*********************************/
// Slope is computed for a Global Node based on the values of elevation assigned
// to itself and its neighbours
void DyMuPathPlanner::calculateSlope(globalNode* nodeTarget)
{
    double dx, dy;
    if (nodeTarget->nb4List[1] == NULL)
        dx = (nodeTarget->nb4List[2]->elevation - nodeTarget->elevation)/
             global_res;
    else
    {
        if (nodeTarget->nb4List[2] == NULL)
            dx = (nodeTarget->elevation - nodeTarget->nb4List[1]->elevation)/
                 global_res;
        else
            dx = (nodeTarget->nb4List[2]->elevation -
                  nodeTarget->nb4List[1]->elevation)*0.5/global_res;
    }
    if (nodeTarget->nb4List[0] == NULL)
        dy = (nodeTarget->nb4List[3]->elevation - nodeTarget->elevation)/
             global_res;
    else
    {
        if (nodeTarget->nb4List[3] == NULL)
            dy = (nodeTarget->elevation - nodeTarget->nb4List[0]->elevation)/
                 global_res;
        else
            dy = (nodeTarget->nb4List[3]->elevation -
                  nodeTarget->nb4List[0]->elevation)*0.5/global_res;
    }
    nodeTarget->slope = atan(sqrt(pow(dx,2)+pow(dy,2)));
}


/*********************CALCULATION OF GLOBAL NODE COST**************************/
// It is computed the value of raw cost of a global nodeEnd
// In case the cost will not be later smoothed, that value is final
//   (cost = raw_cost)

void DyMuPathPlanner::calculateNominalCost(globalNode* nodeTarget,
                                           int range, int numLocs)
{
    double Cdefinitive, Ccandidate, C1, C2;

    double Cmax = *std::max_element(cost_lutable.begin(), cost_lutable.end());


  // Obstacles defined by input terrain map
    if(nodeTarget->terrain == 0) //Global Obstacle
    {
        nodeTarget->raw_cost = Cmax;
        nodeTarget->isObstacle = true;

        for (uint i = 0; i<4; i++)
            if (!nodeTarget->isObstacle)
            {
                nodeTarget->nb4List[i]->raw_cost = Cmax;
            }
    }
    else if(range == 1) //Slopes are not taken into account
    {
        Cdefinitive = cost_lutable[nodeTarget->terrain*numLocs];
        for(uint i = 0; i<locomotion_modes.size(); i++)
        {
            Ccandidate = cost_lutable[nodeTarget->terrain*numLocs + i];
            if (Ccandidate < Cdefinitive)
                Cdefinitive = Ccandidate;
        }
        nodeTarget->raw_cost = std::max(nodeTarget->raw_cost,Cdefinitive);
    }
    else
    {
        double slope_index = (nodeTarget->slope)*180/M_PI/(slope_range.back()-
                            slope_range.front())*(slope_range.size()-1);
        // In case the slope is higher than the maximum, it is an obstacle
        if(slope_index > (slope_range.size()-1))
        {
            nodeTarget->raw_cost = Cmax;
            nodeTarget->isObstacle = true;
            for (uint i = 0; i<4; i++)
                if (!nodeTarget->isObstacle)
                {
                    nodeTarget->nb4List[i]->raw_cost = Cmax;
                }
        }
        else
        {
            double slope_min_index = std::floor(slope_index);
            double slope_max_index = std::ceil(slope_index);
            Cdefinitive = Cmax;
            if (numLocs > 1)
            {
                for(uint i = 1; i<locomotion_modes.size();i++)
                {
                    C1 = cost_lutable[nodeTarget->terrain*range*numLocs +
                                      i*range + (int)slope_min_index];
                    C2 = cost_lutable[nodeTarget->terrain*range*numLocs +
                                      i*range + (int)slope_max_index];
                    Ccandidate = C1 + (C2-C1)*(slope_index-slope_min_index);
                    if (Ccandidate < Cdefinitive)
                    {
                        Cdefinitive = Ccandidate;
                        nodeTarget->raw_cost = std::max(nodeTarget->raw_cost,
                                                   Cdefinitive);
                        nodeTarget->nodeLocMode = locomotion_modes[i];
                    }
                }
            }
            else
            {
                C1 = cost_lutable[nodeTarget->terrain*range +
                                  (int)slope_min_index];
                C2 = cost_lutable[nodeTarget->terrain*range +
                                  (int)slope_max_index];
                Cdefinitive = C1 + (C2-C1)*(slope_index-slope_min_index);
                nodeTarget->raw_cost = std::max(nodeTarget->raw_cost,
                                           Cdefinitive);
                nodeTarget->nodeLocMode = locomotion_modes[0];
            }

        }
    }
}


/**************************SMOOTHING THE COST MAP******************************/
// Cost Map averaging to minimize discontinuities
void DyMuPathPlanner::smoothCost(globalNode* nodeTarget)
{
    double Csum = nodeTarget->cost, n = 5;
    for (uint i = 0; i<4; i++)
    {
        if (nodeTarget->nb4List[i] == NULL)
            n--;
        else
            Csum += nodeTarget->nb4List[i]->raw_cost;
    }
    nodeTarget->cost = Csum/n;
}


/*****************************GET GLOBAL NODE**********************************/
// Returns Global Node Nij

globalNode* DyMuPathPlanner::getGlobalNode(uint i, uint j)
{
    if ((i >= num_nodes_X)||(j >= num_nodes_Y))
        return NULL;
    return global_layer[j][i];
}


/**************************PLACING THE GOAL************************************/
// The goal must be a global node. Therefore, in this function the global node
// closest to wGoal is the one chosen.
bool DyMuPathPlanner::setGoal(base::Waypoint wGoal)
{
  // We transform the position from global coordinates to grid coordinates
    wGoal.position[0] = (wGoal.position[0] - global_offset[0])/global_res;
    wGoal.position[1] = (wGoal.position[1] - global_offset[1])/global_res;

  // In case the wGoal is out of boundaries
    if ((wGoal.position[0] < 0)||(wGoal.position[1] < 0))
        return false;

  // We get the closest Global Node
    uint scaledX = (uint)(wGoal.position[0] + 0.5);
    uint scaledY = (uint)(wGoal.position[1] + 0.5);
    globalNode * candidateGoal = getGlobalNode(scaledX, scaledY);

  // Check if it is out of boundaries or degenerated
    if ((candidateGoal == NULL)||
        (candidateGoal->nb4List[0] == NULL)||
        (candidateGoal->nb4List[1] == NULL)||
        (candidateGoal->nb4List[2] == NULL)||
        (candidateGoal->nb4List[3] == NULL))
        return false;

  // Check whether it is valid (is not placed next to an obstacle Global Node)
    if ((candidateGoal->isObstacle)||
        (candidateGoal->nb4List[0]->isObstacle)||
        (candidateGoal->nb4List[1]->isObstacle)||
        (candidateGoal->nb4List[2]->isObstacle)||
        (candidateGoal->nb4List[3]->isObstacle))
        return false;

  // At this point, the chosen goal is valid
    global_goal = candidateGoal;
    global_goal->pose.orientation = wGoal.heading;
    return true;
}


/***********************COMPUTATION OF TOTAL COST******************************/
// Total Cost is the metric used to define the amount of cost required to reach
// the goal from a certain location. It is computed by means of the Fast
// Marching Method (FMM). In this particular case, the computation stops when
// the value of total cost of the GLobal node next to the robot pose is computed
bool DyMuPathPlanner::computeTotalCostMap(base::Waypoint wPos)
{

  // Check validity of Goal
    if ((global_goal == NULL)||(global_goal->isObstacle))
    {
        LOG_WARN_S << "The goal is not valid";
        return false;
    }

    globalNode * startNode = getNearestGlobalNode(wPos);

  // Check validity of Global Node closest to wPos
    if (!isSafeNode(startNode))
    {
        LOG_ERROR_S << "PLANNER: The rover is located too close to an obstacle";
        return false;
    }

  // In case it is not the first run, all previous computation is resetted
    resetTotalCostMap();
    resetGlobalNarrowBand();

    globalNode * nodeTarget = global_goal;
    while ((!global_narrowband.empty())&&(!isFullyClosedNode(startNode)))
    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)&&
                !(nodeTarget->nb4List[i]->isObstacle))
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    if (global_narrowband.empty())
    {
        LOG_ERROR_S << "The goal is unreachable";
        return false;
    }
    else
    {
        return true;
    }
}

bool DyMuPathPlanner::isSafeNode(globalNode* global_node)
{
    if (global_node->isObstacle)
    {
        return false;
    }
    for (uint i = 0; i<8; i++)
        if (global_node->nb8List[i]->isObstacle)
        {
            return false;
        }
    return true;
}

bool DyMuPathPlanner::isFullyClosedNode(globalNode* global_node)
{
    if (global_node->state == OPEN)
    {
        return false;
    }
    for (uint i = 0; i<4; i++)
        if (global_node->nb4List[i]->state==OPEN)
        {
            return false;
        }
    return true;
}

/***********************COMPUTATION OF TOTAL COST******************************/
// Total Cost is the metric used to define the amount of cost required to reach
// the goal from a certain location. It is computed by means of the Fast
// Marching Method (FMM). Unlike in the previous case, here the total cost is
// computed for all non-obstacle nodes.
bool DyMuPathPlanner::computeEntireTotalCostMap()
{

  // Check validity of Goal
    if ((global_goal == NULL)||(global_goal->isObstacle))
    {
        LOG_WARN_S << "The goal is not valid";
        return false;
    }

  // In case it is not the first run, all previous computation is resetted
    resetTotalCostMap();
    resetGlobalNarrowBand();

    globalNode * nodeTarget = global_goal;
    while (!global_narrowband.empty())
    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)&&
                !(nodeTarget->nb4List[i]->isObstacle))
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    return true;
}


/***********************RESET TOTAL COST VALUES********************************/
// This is done to leave the global nodes at an initial state for another path
// planning computation
void DyMuPathPlanner::resetTotalCostMap()
{
    if (!global_propagated_nodes.empty())
    {
        LOG_DEBUG_S << "resetting global nodes for new goal";
        for(uint i = 0; i<global_propagated_nodes.size(); i++)
        {
            global_propagated_nodes[i]->state = OPEN;
            global_propagated_nodes[i]->total_cost =
                                        std::numeric_limits<double>::infinity();
        }
        global_propagated_nodes.clear();
    }
}


/***********************RESET GLOBAL NARROW BAND*******************************/
// The narrow band is cleared and will only contain the global node considered
// as goal
void DyMuPathPlanner::resetGlobalNarrowBand()
{
    global_narrowband.clear();
    global_narrowband.push_back(global_goal);
    global_propagated_nodes.push_back(global_goal);
    global_goal->total_cost = 0;
}


/**********************PROPAGATION TO GLOBAL NODES*****************************/
// The value of total cost is computed based on the Eikonal equation.
void DyMuPathPlanner::propagateGlobalNode(globalNode* nodeTarget)
{
    double Tx,Ty,T,C;
  // Total Cost of vertical neighbour
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
    {
        Ty = fmin(nodeTarget->nb4List[3]->total_cost,
                  nodeTarget->nb4List[0]->total_cost);
    }
    else if (nodeTarget->nb4List[0] == NULL)
    {
        Ty = nodeTarget->nb4List[3]->total_cost;
    }
    else
    {
        Ty = nodeTarget->nb4List[0]->total_cost;
    }

  // Total Cost of horizontal neighbour
    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = fmin(nodeTarget->nb4List[1]->total_cost,
                  nodeTarget->nb4List[2]->total_cost);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->total_cost;
    else
        Tx = nodeTarget->nb4List[1]->total_cost;

  // Here, the cost also depends on feedback from the local layer, in the form
  // of hazard density and trafficability
    C = global_res*(nodeTarget->cost)*(2 +
                       nodeTarget->hazard_density - nodeTarget->trafficability);

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < std::numeric_limits<double>::infinity())&&
        (Ty < std::numeric_limits<double>::infinity()))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == std::numeric_limits<double>::infinity())
        {
            global_propagated_nodes.push_back(nodeTarget);
            global_narrowband.push_back(nodeTarget);
        }
        nodeTarget->total_cost = T;
    }
}


/*******************GET THE NODE WITH LOWEST COST******************************/
// Returns the global node contained in the narrow band with the lowest value of
// total cost
globalNode* DyMuPathPlanner::minCostGlobalNode()
{
    globalNode* nodePointer = global_narrowband.front();
    uint index = 0;
    uint i;
    double minCost = global_narrowband.front()->total_cost;
    for (i =0; i < global_narrowband.size(); i++)
    {
        if (global_narrowband[i]->total_cost < minCost)
        {
            minCost = global_narrowband[i]->total_cost;
            nodePointer = global_narrowband[i];
            index = i;
        }
    }
    global_narrowband.erase(global_narrowband.begin() + index);
    return nodePointer;
}


/***************************GET NEAREST NODE***********************************/
// Returns the global node closest to pos
globalNode* DyMuPathPlanner::getNearestGlobalNode(base::Pose2D pos)
{
    return getGlobalNode((uint)(pos.position[0]/global_res + 0.5),
                         (uint)(pos.position[1]/global_res + 0.5));
}


/***************************GET NEAREST NODE***********************************/
// Returns the global node closest to wPos
globalNode* DyMuPathPlanner::getNearestGlobalNode(base::Waypoint wPos)
{
    return getGlobalNode((uint)(wPos.position[0]/global_res + 0.5),
                         (uint)(wPos.position[1]/global_res + 0.5));
}


/****************************GET THE PATH**************************************/
// The global path, after being evaluated in case of passing close to local
// obstacles, is returned
std::vector<base::Waypoint> DyMuPathPlanner::getPath(base::Waypoint wPos)
{
    LOG_DEBUG_S << "PLANNER: the robot is at (" << wPos.position[0] <<
                   ", " << wPos.position[1] << ") ";
    computeGlobalPath(wPos);
    LOG_DEBUG_S << "PLANNER: resulting path size = " << current_path.size() <<
                   " waypoints, BEFORE evaluation";
    evaluatePath(0);
    LOG_DEBUG_S << "PLANNER: resulting path size = " << current_path.size() <<
                   " waypoints, AFTER evaluation";
    return current_path;
}


/*********************COMPUTE GLOBAL PATH**************************************/
// Path is extracted from the total cost values of the global layer
bool DyMuPathPlanner::computeGlobalPath(base::Waypoint wPos)
{
      base::Waypoint sinkPoint;
      base::Waypoint wNext;
      sinkPoint.position[0] = global_res*global_goal->pose.position[0];
      sinkPoint.position[1] = global_res*global_goal->pose.position[1];
      sinkPoint.position[2] = global_goal->elevation;
      sinkPoint.heading = global_goal->pose.orientation;

      current_path.clear();
      double tau = std::min(0.4,risk_distance);
      wNext = computeNextGlobalWaypoint(wPos, tau);

      if (std::isnan(wNext.position[0])||std::isnan(wNext.position[1]))
      {
          LOG_ERROR_S << "PLANNER: Gradient Descent Method failed at (" <<
                         wNext.position[0] << ", " << wNext.position[1] << ") ";
          return false;
      }

      current_path.push_back(wPos);

      wPos = wNext;
      LOG_DEBUG_S << "PLANNER: the robot is at (" << wPos.position[0] <<
                     ", " << wPos.position[1] << ") ";

      LOG_DEBUG_S << "PLANNER: the goal is at (" << sinkPoint.position[0] <<
                     ", " << sinkPoint.position[1] << ") ";

      while(sqrt(pow((wPos.position[0] - sinkPoint.position[0]),2) +
               pow((wPos.position[1] - sinkPoint.position[1]),2)) >
                2.0*global_res)
      {
          wNext = computeNextGlobalWaypoint(wPos, tau);
          current_path.push_back(wPos);
          if(sqrt(pow((wPos.position[0] - wNext.position[0]),2) +
               pow((wPos.position[1] - wNext.position[1]),2)) <
               0.01*tau*global_res)
          {
              LOG_ERROR_S << "ERROR in trajectory";
              return false;
          }
          wPos = wNext;
      }
      LOG_DEBUG_S << "Adding final waypoint with heading" << sinkPoint.heading;
      current_path.push_back(sinkPoint);
      return true;
}


/*********************COMPUTE NEXT GLOBAL WAYPOINT*****************************/
// The next waypoint is computed according to the gradient of total cost
base::Waypoint DyMuPathPlanner::computeNextGlobalWaypoint(base::Waypoint& wPos,
                                                          double tau)
{
    base::Waypoint wNext;

  // Position of wPos in terms of global units
    double globalXpos = (wPos.position[0]-global_offset[0])/global_res;
    double globalYpos = (wPos.position[1]-global_offset[1])/global_res;

  // Position of the global Node placed next to wPos in the downleft corner
    uint globalCornerX = (uint)(globalXpos);
    uint globalCornerY = (uint)(globalYpos);

  // Distance wPos - globalCorner
    double globalDistX = globalXpos - (double)(globalCornerX);
    double globalDistY = globalYpos - (double)(globalCornerY);

  // Take pointers to global Nodes - corners of cell where wPos is
    globalNode * gNode00 = getGlobalNode(globalCornerX, globalCornerY);
    globalNode * gNode10 = gNode00->nb4List[2];
    globalNode * gNode01 = gNode00->nb4List[3];
    globalNode * gNode11 = gNode10->nb4List[3];

    double gx00, gx10, gx01, gx11;
    double gy00, gy10, gy01, gy11;

    gradientNode( gNode00, gx00, gy00);
    gradientNode( gNode10, gx10, gy10);
    gradientNode( gNode01, gx01, gy01);
    gradientNode( gNode11, gx11, gy11);

    double dCostX = interpolate(globalDistX,globalDistY,gx00,gx01,gx10,gx11);
    double dCostY = interpolate(globalDistX,globalDistY,gy00,gy01,gy10,gy11);

    wPos.position[2] = interpolate(globalDistX,globalDistY,
                                   gNode00->elevation, gNode10->elevation,
                                   gNode01->elevation, gNode11->elevation);

    wNext.position[0] = wPos.position[0] - global_res*tau*dCostX;
    wNext.position[1] = wPos.position[1] - global_res*tau*dCostY;

    wNext.heading = atan2(-dCostY,-dCostX);

    /*if ((dCostX)||(dCostY))
        return NULL;*/
    return wNext;
}


/*************************GRADIENT NODE****************************************/
// Computes the gradient of total cost of a node
void DyMuPathPlanner::gradientNode(globalNode* nodeTarget, double& dnx,
                                   double& dny)
{
    double dx, dy;

      if (((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))||
          ((nodeTarget->nb4List[1] != NULL)&&(nodeTarget->nb4List[2] != NULL)&&
           (nodeTarget->nb4List[1]->total_cost ==
            std::numeric_limits<double>::infinity())&&
           (nodeTarget->nb4List[2]->total_cost ==
            std::numeric_limits<double>::infinity())))
          dx = 0;
      else
      {
          if ((nodeTarget->nb4List[1] == NULL)||
              (nodeTarget->nb4List[1]->total_cost ==
               std::numeric_limits<double>::infinity()))
              dx = nodeTarget->nb4List[2]->total_cost - nodeTarget->total_cost;
          else
          {
              if ((nodeTarget->nb4List[2] == NULL)||
                  (nodeTarget->nb4List[2]->total_cost ==
                   std::numeric_limits<double>::infinity()))
                  dx = nodeTarget->total_cost -
                       nodeTarget->nb4List[1]->total_cost;
              else
                  dx = (nodeTarget->nb4List[2]->total_cost -
                        nodeTarget->nb4List[1]->total_cost)*0.5;
          }
      }
      if (((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))||
          ((nodeTarget->nb4List[0] != NULL)&&(nodeTarget->nb4List[3] != NULL)&&
           (nodeTarget->nb4List[0]->total_cost ==
            std::numeric_limits<double>::infinity())&&
           (nodeTarget->nb4List[3]->total_cost ==
            std::numeric_limits<double>::infinity())))
          dy = 0;
      else
      {
          if ((nodeTarget->nb4List[0] == NULL)||
              (nodeTarget->nb4List[0]->total_cost ==
                std::numeric_limits<double>::infinity()))
              dy = nodeTarget->nb4List[3]->total_cost - nodeTarget->total_cost;
          else
          {
              if ((nodeTarget->nb4List[3] == NULL)||
                  (nodeTarget->nb4List[3]->total_cost ==
                    std::numeric_limits<double>::infinity()))
                  dy = nodeTarget->total_cost -
                       nodeTarget->nb4List[0]->total_cost;
              else
                  dy = (nodeTarget->nb4List[3]->total_cost -
                        nodeTarget->nb4List[0]->total_cost)*0.5;
          }
      }
      if ((dx == 0)&&(dy==0))
      {
          dnx = 0;
          dny = 0;
      }
      else
      {
          dnx = dx/sqrt(pow(dx,2)+pow(dy,2));
          dny = dy/sqrt(pow(dx,2)+pow(dy,2));
      }
}


/*************************INTERPOLATION FUNCTION*******************************/
// Returns value interpolated
double DyMuPathPlanner::interpolate(double a, double b, double g00, double g01,
                                    double g10, double g11)
{
    return g00 + (g10 - g00)*a + (g01 - g00)*b + (g11 + g00 - g10 - g01)*a*b;
}


/*************************GET LOCOMOTION MODE**********************************/
// The locomotion mode that requires lower cost in that location is indicated
std::string DyMuPathPlanner::getLocomotionMode(base::Waypoint wPos)
{
    globalNode * gNode = getNearestGlobalNode(wPos);
    return gNode->nodeLocMode;
}


/***********************GET TOTAL COST MATRIX**********************************/
// A matrix with only total cost values is provided
std::vector< std::vector<double> > DyMuPathPlanner::getTotalCostMatrix()
{
    std::vector< std::vector<double> > total_cost_matrix(num_nodes_Y);
    for (uint i = 0; i < num_nodes_Y; i++)
        total_cost_matrix[i].resize(num_nodes_X);

    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
            if (global_layer[j][i]->total_cost ==
                                        std::numeric_limits<double>::infinity())
                total_cost_matrix[j][i] = -1.0;
            else
                total_cost_matrix[j][i] = global_layer[j][i]->total_cost;
    return total_cost_matrix;
}


/**********************GET GLOBAL COST MATRIX**********************************/
// A matrix with only cost values is provided
std::vector< std::vector<double> > DyMuPathPlanner::getGlobalCostMatrix()
{
    std::vector< std::vector<double> > global_cost_matrix(num_nodes_Y);
    for (uint i = 0; i < num_nodes_Y; i++)
        global_cost_matrix[i].resize(num_nodes_X);

    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
          if (global_layer[j][i]->isObstacle)
              global_cost_matrix[j][i] = -1.0;
          else
              global_cost_matrix[j][i] = global_layer[j][i]->cost*(2 +
                global_layer[j][i]->hazard_density -
                global_layer[j][i]->trafficability);
    return global_cost_matrix;
}


/**********************GET HAZARD DENSITY MATRIX*******************************/
// A matrix with only hazard density values is provided
std::vector< std::vector<double> > DyMuPathPlanner::getHazardDensityMatrix()
{
    std::vector< std::vector<double> > hazard_density_matrix(num_nodes_Y);
    for (uint i = 0; i < num_nodes_Y; i++)
        hazard_density_matrix[i].resize(num_nodes_X);

    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
            hazard_density_matrix[j][i] = global_layer[j][i]->hazard_density;
    return hazard_density_matrix;
}


/**********************GET TRAFFICABILITY MATRIX*******************************/
// A matrix with only trafficability values is provided
std::vector< std::vector<double> > DyMuPathPlanner::getTrafficabilityMatrix()
{
    std::vector< std::vector<double> > trafficability_matrix(num_nodes_Y);
    for (uint i = 0; i < num_nodes_Y; i++)
        trafficability_matrix[i].resize(num_nodes_X);

    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
            trafficability_matrix[j][i] = global_layer[j][i]->trafficability;
    return trafficability_matrix;
}


/*******************GET THE TOTAL COST OF A LOCATION***************************/
// Here, the local cost corresponding to the location of wInt is provided,
// according to the values of the nearby global nodes.
double DyMuPathPlanner::getTotalCost(base::Waypoint wInt)
{
    uint i = (uint)(wInt.position[0]/global_res);
    uint j = (uint)(wInt.position[1]/global_res);
    double a = wInt.position[0] - (double)(i);
    double b = wInt.position[1] - (double)(j);

    globalNode * node00 = global_layer[j][i];
    globalNode * node10 = node00->nb4List[2];
    globalNode * node01 = node00->nb4List[3];
    globalNode * node11 = node00->nb4List[2]->nb4List[3];

    if (((node00==NULL)||(node10==NULL)||(node01==NULL)||(node11==NULL))||
       ((node00->state==OPEN)||(node10->state==OPEN)||(node01->state==OPEN)||
       (node11->state==OPEN)))
    {
        globalNode* nearestNode = getNearestGlobalNode(wInt);
        return nearestNode->total_cost;
    }
    else
    {
        double w00 = node00->total_cost;
        double w10 = node10->total_cost;
        double w01 = node01->total_cost;
        double w11 = node11->total_cost;
        return w00 + (w10 - w00)*a + (w01 - w00)*b +
               (w11 + w00 - w10 - w01)*a*b;
    }
}
