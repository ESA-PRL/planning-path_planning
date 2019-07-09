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

DyMuPathPlanner::DyMuPathPlanner(std::vector<double> costData,
                           std::vector<double> slope_values,
                           std::vector<std::string> locomotion_modes,
                           double risk_distance,
                           double reconnect_distance,
                           double risk_ratio):
                           cost_lutable(costData),
                           slope_range(slope_values),locomotion_modes(locomotion_modes),
                           risk_distance(risk_distance), reconnect_distance(reconnect_distance),risk_ratio(risk_ratio)
{
    global_goal = NULL;
    std::cout << "Cost data is [ ";
    for(uint i = 0; i<cost_lutable.size(); i++)
        std::cout << cost_lutable[i] << " ";
    std::cout << " ]" << std::endl;;
}

DyMuPathPlanner::~DyMuPathPlanner()
{
}



/*******************GLOBAL_LAYER_INITIALIZATION********************************/
bool DyMuPathPlanner::initGlobalLayer(double globalres, double localres,
                                      uint numnodesX, uint numnodesY,
                                      base::Pose2D offset)
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
    printf("Building Global Map Neighbourhood\n");
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

  // COST is computed
    /*printf("Calculating Nominal Cost, Slope and Aspect values\n");
    for (j = 0; j < global_layer.size(); j++)
        for (i = 0; i < global_layer[0].size(); i++)
        {
            calculateSlope(global_layer[j][i]);
            calculateNominalCost(global_layer[j][i], to_be_smoothed);
        }
    if (to_be_smoothed)
        for (j = 0; j < global_layer.size(); j++)
            for (i = 0; i < global_layer[0].size(); i++)
                smoothCost(global_layer[j][i]);*/

    return true;
}


bool DyMuPathPlanner::setCostMap(std::vector< std::vector<double> > cost_map)
{
    double cost;
    for (uint j = 0; j < num_nodes_Y; j++)
        for (uint i = 0; i < num_nodes_X; i++)
        {
            cost = cost_map[j][i];
            global_layer[j][i]->cost = cost;
            if (cost<=0)
                global_layer[j][i]->isObstacle = true;
        }
    return true;
}


/*****************************GET GLOBAL NODE*******************************/
// Returns Global Node Nij

globalNode* DyMuPathPlanner::getGlobalNode(uint i, uint j)
{
    if ((i >= global_layer[0].size())||(j >= global_layer.size()))
        return NULL;
    return global_layer[j][i];
}


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
                                           bool to_be_smoothed)
{
    double Cdefinitive, Ccandidate, C1, C2;
    int range = slope_range.size();
    int numLocs = locomotion_modes.size();

  // Obstacles defined by input terrain map
    if(nodeTarget->terrain == 0) //Global Obstacle
    {
        nodeTarget->raw_cost = cost_lutable[0];
        nodeTarget->isObstacle = true;
        if (!to_be_smoothed)
            nodeTarget->cost = cost_lutable[0];

        for (uint i = 0; i<4; i++)
            if (!nodeTarget->isObstacle)
            {
                nodeTarget->nb4List[i]->raw_cost = cost_lutable[0];
                if (!to_be_smoothed)
                    nodeTarget->nb4List[i]->cost = cost_lutable[0];
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
        if (!to_be_smoothed)
            nodeTarget->cost = nodeTarget->raw_cost;
    }
    else
    {
        double slope_index = (nodeTarget->slope)*180/M_PI/(slope_range.back()-
                            slope_range.front())*(slope_range.size()-1);
        // In case the slope is higher than the maximum, it is an obstacle
        if(slope_index > (slope_range.size()-1))
        {
            nodeTarget->raw_cost = cost_lutable[0];
            nodeTarget->isObstacle = true;
            if (!to_be_smoothed)
                nodeTarget->cost = cost_lutable[0];
            for (uint i = 0; i<4; i++)
                if (!nodeTarget->isObstacle)
                {
                    nodeTarget->nb4List[i]->raw_cost = cost_lutable[0];
                    if (!to_be_smoothed)
                        nodeTarget->nb4List[i]->cost = cost_lutable[0];
                }
        }
        else
        {
            double slope_min_index = std::floor(slope_index);
            double slope_max_index = std::ceil(slope_index);
            Cdefinitive = cost_lutable[0];
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
                if (!to_be_smoothed)
                    nodeTarget->cost = nodeTarget->raw_cost;
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
                if (!to_be_smoothed)
                    nodeTarget->cost = nodeTarget->raw_cost;
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


bool DyMuPathPlanner::setGoal(base::Waypoint wGoal)
{
  // This function sets the Global Node nearest to the goal location
  // as the Goal node

    wGoal.position[0] = wGoal.position[0]/global_res; //TODO: Take into account offset!!
    wGoal.position[1] = wGoal.position[1]/global_res;
    uint scaledX = (uint)(wGoal.position[0] + 0.5);
    uint scaledY = (uint)(wGoal.position[1] + 0.5);
    globalNode * candidateGoal = getGlobalNode(scaledX, scaledY);

  // Check whether it is the same Global Node
    if ((candidateGoal->pose.position[0] == global_goal->pose.position[0])&&
        (candidateGoal->pose.position[1] == global_goal->pose.position[1]))
        return false;

  // Check whether it is valid (is not placed next to an obstacle Global Node)
    if ((candidateGoal->terrain == 0)||
        (candidateGoal->nb4List[0]->terrain == 0)||
        (candidateGoal->nb4List[1]->terrain == 0)||
        (candidateGoal->nb4List[2]->terrain == 0)||
        (candidateGoal->nb4List[3]->terrain == 0))
    {
        LOG_DEBUG_S << "Goal NOT valid, nearest global node is (" << global_goal->pose.position[0]
                << "," << global_goal->pose.position[1] << ") and is forbidden area";
        return false;
    }

    global_goal = candidateGoal;
    global_goal->pose.orientation = wGoal.heading;
    LOG_DEBUG_S << "Goal is global node (" << global_goal->pose.position[0]
              << "," << global_goal->pose.position[1] << ")";
    return true;
}


bool DyMuPathPlanner::computeTotalCostMap(base::Waypoint wPos)
{
    LOG_DEBUG_S << "Computing Global Propagation";

    if (global_goal == NULL)
        return false;
  // To recompute the Global Propagation, first the Global Nodes used before
  // are reset
    resetTotalCostMap();

    global_narrowband.clear();
    global_narrowband.push_back(global_goal);
    global_propagated_nodes.push_back(global_goal);
    global_goal->total_cost = 0;

    globalNode * nodeTarget = global_goal;
    globalNode * startNode = getNearestGlobalNode(wPos);

    LOG_DEBUG_S << "starting global propagation loop ";
    while ((!global_narrowband.empty())&&(startNode->state == OPEN))
    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)&&
                !(nodeTarget->nb4List[i]->isObstacle)) //TODO: take care of this for degenerated fields!!
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    if (global_narrowband.empty())
        return false;
    else
        return true;
}

bool DyMuPathPlanner::computeEntireTotalCostMap()
{
    LOG_DEBUG_S << "Computing Global Propagation";

    if (global_goal == NULL)
        return false;
  // To recompute the Global Propagation, first the Global Nodes used before
  // are reset
    resetTotalCostMap();

    global_narrowband.clear();
    global_narrowband.push_back(global_goal);
    global_propagated_nodes.push_back(global_goal);
    global_goal->total_cost = 0;

    globalNode * nodeTarget = global_goal;

    LOG_DEBUG_S << "starting global propagation loop ";
    while (!global_narrowband.empty()) //TODO: if narrow band is empty, maybe nodeGoal is unreachable, fix this!!

    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)&&
                !(nodeTarget->nb4List[i]->isObstacle)) //TODO: take care of this for degenerated fields!!
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    return true;
}



void DyMuPathPlanner::resetTotalCostMap()
{
    if (!global_propagated_nodes.empty())
    {
        LOG_DEBUG_S << "resetting global nodes for new goal";
        for(uint i = 0; i<global_propagated_nodes.size(); i++)
        {
            global_propagated_nodes[i]->state = OPEN;
            global_propagated_nodes[i]->total_cost = INF;
        }
        global_propagated_nodes.clear();
    }
}


void DyMuPathPlanner::propagateGlobalNode(globalNode* nodeTarget)
{
    double Tx,Ty,T,C;
    std::string L;
  // Neighbor Propagators Tx and Ty
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
    {
        Ty = fmin(nodeTarget->nb4List[3]->total_cost, nodeTarget->nb4List[0]->total_cost);
    }
    else if (nodeTarget->nb4List[0] == NULL)
    {
        Ty = nodeTarget->nb4List[3]->total_cost;
    }
    else
    {
        Ty = nodeTarget->nb4List[0]->total_cost;
    }

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = fmin(nodeTarget->nb4List[1]->total_cost, nodeTarget->nb4List[2]->total_cost);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->total_cost;
    else
        Tx = nodeTarget->nb4List[1]->total_cost;

  // Cost Function to obtain optimal power and locomotion mode
    C = global_res*(nodeTarget->cost);

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
        {
            global_propagated_nodes.push_back(nodeTarget);
            global_narrowband.push_back(nodeTarget);
        }
        nodeTarget->total_cost = T;
    }
}


globalNode* DyMuPathPlanner::minCostGlobalNode()
{
  // This can be improved by using binary search methods and a sorted narrowband
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

globalNode* DyMuPathPlanner::getNearestGlobalNode(base::Pose2D pos)
{
    return getGlobalNode((uint)(pos.position[0]/global_res + 0.5), (uint)(pos.position[1]/global_res + 0.5));
}

globalNode* DyMuPathPlanner::getNearestGlobalNode(base::Waypoint wPos)
{
    return getGlobalNode((uint)(wPos.position[0]/global_res + 0.5), (uint)(wPos.position[1]/global_res + 0.5));
}


std::vector<base::Waypoint> DyMuPathPlanner::getNewPath(base::Waypoint wPos)
{
    current_path.clear();
    return getGlobalPath(wPos);
}

std::vector<base::Waypoint> DyMuPathPlanner::getGlobalPath(base::Waypoint wPos)
{
      base::Waypoint sinkPoint;
      base::Waypoint wNext;
      sinkPoint.position[0] = global_res*global_goal->pose.position[0];
      sinkPoint.position[1] = global_res*global_goal->pose.position[1];
      sinkPoint.position[2] = global_goal->elevation;
      sinkPoint.heading = global_goal->pose.orientation;

      std::vector<base::Waypoint> trajectory;

      trajectory.clear();
      double tau = std::min(0.4,risk_distance);
      wNext = calculateNextGlobalWaypoint(wPos, tau);
      trajectory.push_back(wPos);

      wPos = wNext;
      LOG_DEBUG_S << "trajectory initialized with tau = " << tau;


      while(sqrt(pow((wPos.position[0] - sinkPoint.position[0]),2) +
               pow((wPos.position[1] - sinkPoint.position[1]),2)) > 2.0*global_res)
      {
          wNext = calculateNextGlobalWaypoint(wPos, tau);
          /*if (wNext == NULL)
          {
              LOG_WARN_S << "global waypoint (" << wNext.position[0] << "," << wNext.position[1] << ") is degenerate (nan gradient)";
              return trajectory;
          }*/
          trajectory.push_back(wPos);
          if(sqrt(pow((wPos.position[0] - wNext.position[0]),2) +
               pow((wPos.position[1] - wNext.position[1]),2)) < 0.01*tau*global_res)
          {
              LOG_ERROR_S << "ERROR in trajectory";
              return trajectory;
          }
          wPos = wNext;
      }
      LOG_DEBUG_S << "Adding final waypoint with heading" << sinkPoint.heading;
      trajectory.push_back(sinkPoint);

      for(uint i = 0; i<trajectory.size(); i++)
      {
          current_path.push_back(trajectory[i]);
      }
      return trajectory;
}

base::Waypoint DyMuPathPlanner::calculateNextGlobalWaypoint(base::Waypoint& wPos, double tau)
{

    base::Waypoint wNext;

  // Position of wPos in terms of global units
    double globalXpos = (wPos.position[0]-global_offset.position[0])/global_res;
    double globalYpos = (wPos.position[1]-global_offset.position[1])/global_res;

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


void DyMuPathPlanner::gradientNode(globalNode* nodeTarget, double& dnx, double& dny)
{
    double dx, dy;

      if (((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))||
          ((nodeTarget->nb4List[1] != NULL)&&(nodeTarget->nb4List[2] != NULL)&&
           (nodeTarget->nb4List[1]->total_cost == INF)&&(nodeTarget->nb4List[2]->total_cost == INF)))
          dx = 0;
      else
      {
          if ((nodeTarget->nb4List[1] == NULL)||(nodeTarget->nb4List[1]->total_cost == INF))
              dx = nodeTarget->nb4List[2]->total_cost - nodeTarget->total_cost;
          else
          {
              if ((nodeTarget->nb4List[2] == NULL)||(nodeTarget->nb4List[2]->total_cost == INF))
                  dx = nodeTarget->total_cost - nodeTarget->nb4List[1]->total_cost;
              else
                  dx = (nodeTarget->nb4List[2]->total_cost -
                        nodeTarget->nb4List[1]->total_cost)*0.5;
          }
      }
      if (((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))||
          ((nodeTarget->nb4List[0] != NULL)&&(nodeTarget->nb4List[3] != NULL)&&
           (nodeTarget->nb4List[0]->total_cost == INF)&&(nodeTarget->nb4List[3]->total_cost == INF)))
          dy = 0;
      else
      {
          if ((nodeTarget->nb4List[0] == NULL)||(nodeTarget->nb4List[0]->total_cost == INF))
              dy = nodeTarget->nb4List[3]->total_cost - nodeTarget->total_cost;
          else
          {
              if ((nodeTarget->nb4List[3] == NULL)||(nodeTarget->nb4List[3]->total_cost == INF))
                  dy = nodeTarget->total_cost - nodeTarget->nb4List[0]->total_cost;
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

double DyMuPathPlanner::interpolate(double a, double b, double g00, double g01, double g10, double g11)
{
    return g00 + (g10 - g00)*a + (g01 - g00)*b + (g11 + g00 - g10 - g01)*a*b;
}


std::string DyMuPathPlanner::getLocomotionMode(base::Waypoint wPos)
{
    globalNode * gNode = getNearestGlobalNode(wPos);

    if(locomotion_modes.size() > 1)
    {
        double Cdefinitive, Ccandidate, C1, C2;
        int range = slope_range.size();
        int numLocs = locomotion_modes.size();
        int locIndex;

        globalNode * gNode = getNearestGlobalNode(wPos);

        if(range == 1) //Slopes are not taken into account
        {
            Cdefinitive = cost_lutable[gNode->terrain*numLocs];
            locIndex = 0;
            for(uint i = 1; i<locomotion_modes.size(); i++)
            {
                Ccandidate = cost_lutable[gNode->terrain*numLocs + i];
                if (Ccandidate < Cdefinitive)
                {
                    Cdefinitive = Ccandidate;
                    locIndex = i;
                }
            }
            return locomotion_modes[locIndex];
        }
        else
        {
            double slope_index = (gNode->slope)*180/M_PI/(slope_range.back()-slope_range.front())*(slope_range.size()-1);
            double slope_min_index = std::floor(gNode->slope*180/M_PI);
            double slope_max_index = std::ceil(gNode->slope*180/M_PI);
            C1 = cost_lutable[gNode->terrain*range*numLocs + (int)slope_min_index];
            C2 = cost_lutable[gNode->terrain*range*numLocs + (int)slope_max_index];
            Cdefinitive = C1 + (C2-C1)*(slope_index-slope_min_index);
            locIndex = 0;
                /*if((nodeTarget->terrain>0)&&((nodeTarget->slope)*180/M_PI<20.0)&&((nodeTarget->slope)*180/M_PI>10.0))
                    LOG_DEBUG_S << "checking cost -> " << Cdefinitive  << "and slope_index is" << slope_index << " and slope_min_index is " << slope_min_index <<
                      " and slope_max_index is " << slope_max_index
                      << " and C1 = " << C1 << " and C2 = " << C2;*/
            for(uint i = 1; i<locomotion_modes.size();i++)
            {
                C1 = cost_lutable[gNode->terrain*range*numLocs + i*range + (int)slope_min_index];
                C2 = cost_lutable[gNode->terrain*range*numLocs + i*range + (int)slope_max_index];
                Ccandidate = C1 + (C2-C1)*(slope_index-slope_min_index);
                if (Ccandidate < Cdefinitive)
                {
                    Cdefinitive = Ccandidate;
                    locIndex = i;
                }
            }
            return locomotion_modes[locIndex];
        }
    }
    else
        return locomotion_modes[0];
}
