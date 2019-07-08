/*********************DyMu**************************
        -Dynamic Multilayered Path Planning-
            ARES (ESA-UMA Collaboration)
    University of Malaga - European Space Agency
                    -Author-
            J. Ricardo Sanchez Ibanez
                 -Contact mail-
                ricardosan@uma.es
                  -Supervisors-
            Carlos J. Perez del Pulgar
                 Martin Azkarate
****************************************************/

#include "DyMu.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning(std::vector<double> costData,
                           std::vector<double> slope_values,
                           std::vector<std::string> locomotion_modes,
                           double risk_distance,
                           double reconnect_distance,
                           double risk_ratio):
                           cost_data(costData),
                           slope_range(slope_values),locomotion_modes(locomotion_modes),
                           risk_distance(risk_distance), reconnect_distance(reconnect_distance),risk_ratio(risk_ratio)
{
    global_goalNode = NULL;
    std::cout << "Cost data is [ ";
    for(uint i = 0; i<cost_data.size(); i++)
        std::cout << cost_data[i] << " ";
    std::cout << " ]" << std::endl;;
}

PathPlanning::~PathPlanning()
{
}



/*******************GLOBAL_MAP_INITIALIZATION************************************/
void PathPlanning::initGlobalMap(double globalCellSize,  double localCellSize,
                                 base::Pose2D offset,
                                 std::vector< std::vector<double> > elevation,
                                 std::vector< std::vector<double> > terrainMap)
{
    t1 = base::Time::now();
    global_cellSize = globalCellSize;
    local_cellSize = localCellSize;
    ratio_scale = (uint)(global_cellSize/local_cellSize);

    LOG_DEBUG_S << "Creating Global Map using scale " <<
                 global_cellSize << " m";
    LOG_DEBUG_S << "Each global edge is composed by " <<
                 ratio_scale << " local nodes, having a local resolution of " <<
                 local_cellSize << " m";
    global_offset = offset;
    std::vector<globalNode*> nodeRow;
    uint i,j;
    for (j = 0; j < terrainMap.size(); j++)
    {
        for (i = 0; i < terrainMap[0].size(); i++)
        {
            nodeRow.push_back(new globalNode(i, j, elevation[j][i],
                                           terrainMap[j][i]));
            nodeRow.back()->world_pose.position[0] = i*global_cellSize;
            nodeRow.back()->world_pose.position[1] = j*global_cellSize;
        }
        globalMap.push_back(nodeRow);
        nodeRow.clear();
    }
    LOG_DEBUG_S << "Global Map of "<< globalMap[0].size() << " x "
              << globalMap.size() << " nodes created in "
              << (base::Time::now()-t1) << " s";

  // NEIGHBOURHOOD
    printf("Building Global Map Neighbourhood\n");
    t1 = base::Time::now();
    for (uint j = 0; j < globalMap.size(); j++)
    {
        for (uint i = 0; i < globalMap[0].size(); i++)
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

            globalMap[j][i]->nb4List.clear();
            globalMap[j][i]->nb4List.push_back(getGlobalNode(i,j-1));
            globalMap[j][i]->nb4List.push_back(getGlobalNode(i-1,j));
            globalMap[j][i]->nb4List.push_back(getGlobalNode(i+1,j));
            globalMap[j][i]->nb4List.push_back(getGlobalNode(i,j+1));

            //                 8 - Neighbourhood
            //       nb4List[3] __ nb4List[2] __ nb4List[1]
            //       (i-1, j+1) __  (i, j+1)  __ (i+1, j+1)
            //           ||            ||            ||
            //       nb4List[4] __   target   __ nb4List[0]
            //        (i-1, j)  __   (i, j)   __  (i+1, j)
            //           ||            ||            ||
            //       nb4List[5] __ nb4List[6] __ nb4List[7]
            //       (i-1, j-1) __  (i, j-1)  __ (i+1, j-1)

           globalMap[j][i]->nb8List.push_back(getGlobalNode(i+1,j));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i+1,j+1));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i,  j+1));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i-1,j+1));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i-1,j));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i-1,j-1));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i,  j-1));
           globalMap[j][i]->nb8List.push_back(getGlobalNode(i+1,j-1));

        }
    }
    LOG_DEBUG_S << "Neighbourhood made in " << (base::Time::now()-t1)
              << " s";

  // SLOPE AND ASPECT
    printf("Calculating Nominal Cost, Slope and Aspect values\n");
    t1 = base::Time::now();
    for (j = 0; j < globalMap.size(); j++)
        for (i = 0; i < globalMap[0].size(); i++)
        {
            calculateSlope(globalMap[j][i]);
            calculateNominalCost(globalMap[j][i]);
        }

    LOG_DEBUG_S << "Nominal Cost, Slope and Aspect calculated in " <<
                 (base::Time::now()-t1) << " s";
    for (j = 0; j < globalMap.size(); j++)

        for (i = 0; i < globalMap[0].size(); i++)
        {
            calculateSmoothCost(globalMap[j][i]);
        }

    LOG_DEBUG_S << "cost is smoothed in " <<
          (base::Time::now()-t1) << " s";

  // Initialize goal global node
    global_goalNode = getGlobalNode(0,0);
}


/*****************************GET GLOBAL NODE*******************************/
// Returns Global Node Nij

globalNode* PathPlanning::getGlobalNode(uint i, uint j)
{
    if ((i >= globalMap[0].size())||(j >= globalMap.size()))
        return NULL;
    return globalMap[j][i];
}


void PathPlanning::calculateSlope(globalNode* nodeTarget)
{
    double dx, dy;
    if (nodeTarget->nb4List[1] == NULL)
        dx = (nodeTarget->nb4List[2]->elevation - nodeTarget->elevation)/global_cellSize;
    else
    {
        if (nodeTarget->nb4List[2] == NULL)
            dx = (nodeTarget->elevation - nodeTarget->nb4List[1]->elevation)/global_cellSize;
        else
            dx = (nodeTarget->nb4List[2]->elevation -
                  nodeTarget->nb4List[1]->elevation)*0.5/global_cellSize;
    }
    if (nodeTarget->nb4List[0] == NULL)
        dy = (nodeTarget->nb4List[3]->elevation - nodeTarget->elevation)/global_cellSize;
    else
    {
        if (nodeTarget->nb4List[3] == NULL)
            dy = (nodeTarget->elevation - nodeTarget->nb4List[0]->elevation)/global_cellSize;
        else
            dy = (nodeTarget->nb4List[3]->elevation -
                  nodeTarget->nb4List[0]->elevation)*0.5/global_cellSize;
    }
    nodeTarget->slope = atan(sqrt(pow(dx,2)+pow(dy,2)));
    // In this case, aspect points to the direction of maximum positive slope
    if ((dx == 0) && (dy == 0))
        nodeTarget->aspect = 0;
    else
        nodeTarget->aspect = atan2(dy,dx);
}


void PathPlanning::calculateSmoothCost(globalNode* nodeTarget)
{
    double Csum = nodeTarget->cost, n = 5;
    for (uint i = 0; i<4; i++)
    {
        if (nodeTarget->nb4List[i] == NULL)
            n--;
        else
            Csum += nodeTarget->nb4List[i]->cost;
    }
    nodeTarget->cost = std::max(nodeTarget->cost,Csum/n);
}


void PathPlanning::calculateNominalCost(globalNode* nodeTarget)
{
    double Cdefinitive, Ccandidate, C1, C2;
    int range = slope_range.size();
    int numLocs = locomotion_modes.size();

    if(nodeTarget->terrain == 0) //Global Obstacle
    {
        Cdefinitive = cost_data[0];
        nodeTarget->isObstacle = true;
    }
    else if(range == 1) //Slopes are not taken into account
    {
        Cdefinitive = cost_data[nodeTarget->terrain*numLocs];
        for(uint i = 0; i<locomotion_modes.size(); i++)
        {
            Ccandidate = cost_data[nodeTarget->terrain*numLocs + i];
            if (Ccandidate < Cdefinitive)
                Cdefinitive = Ccandidate;
        }
    }
    else
    {
        double slopeIndex = (nodeTarget->slope)*180/M_PI/(slope_range.back()-slope_range.front())*(slope_range.size()-1);
        if(slopeIndex > (slope_range.size()-1))
        {
            Cdefinitive = cost_data[0]; //TODO: here is obstacle, change this to vary k instead
            nodeTarget->isObstacle = true;
        }
        else
        {
            double slopeMinIndex = std::floor(slopeIndex);
            double slopeMaxIndex = std::ceil(slopeIndex);
            C1 = cost_data[nodeTarget->terrain*range*numLocs + (int)slopeMinIndex];
            C2 = cost_data[nodeTarget->terrain*range*numLocs + (int)slopeMaxIndex];
            Cdefinitive = C1 + (C2-C1)*(slopeIndex-slopeMinIndex);
            nodeTarget->nodeLocMode = locomotion_modes[0];
            /*if((nodeTarget->terrain>0)&&((nodeTarget->slope)*180/M_PI<20.0)&&((nodeTarget->slope)*180/M_PI>10.0))
                LOG_DEBUG_S << "checking cost -> " << Cdefinitive  << "and slopeIndex is" << slopeIndex << " and slopeMinIndex is " << slopeMinIndex <<
                  " and slopeMaxIndex is " << slopeMaxIndex
                  << " and C1 = " << C1 << " and C2 = " << C2;*/
            if (locomotion_modes.size()>1)
                for(uint i = 1; i<locomotion_modes.size();i++)
                {
                    C1 = cost_data[nodeTarget->terrain*range*numLocs + i*range + (int)slopeMinIndex];
                    C2 = cost_data[nodeTarget->terrain*range*numLocs + i*range + (int)slopeMaxIndex];
                    Ccandidate = C1 + (C2-C1)*(slopeIndex-slopeMinIndex);
                    if (Ccandidate < Cdefinitive)
                    {
                        Cdefinitive = Ccandidate;
                        nodeTarget->nodeLocMode = locomotion_modes[i];
                    }
                }

        }
    }

    //Csum = (terrainTable[nodeTarget->terrain]->cost);
    //Cdefinitive = cost_data[nodeTarget->terrain*10 + 0 + (int)(std::min(nodeTarget->terrain*180.0/3.1416,20.0))];
    /*for (uint i = 0; i<4; i++)
    {
        if (nodeTarget->nb4List[i] == NULL)
            n--;
        else
            Csum += (terrainTable[nodeTarget->nb4List[i]->terrain]->cost);
    }
    nodeTarget->cost = std::max(terrainTable[nodeTarget->terrain]->cost,Csum/n);*/
    nodeTarget->cost = Cdefinitive;
}


bool PathPlanning::setGoal(base::Waypoint wGoal)
{
  // This function sets the Global Node nearest to the goal location
  // as the Goal node

    wGoal.position[0] = wGoal.position[0]/global_cellSize;
    wGoal.position[1] = wGoal.position[1]/global_cellSize;
    uint scaledX = (uint)(wGoal.position[0] + 0.5);
    uint scaledY = (uint)(wGoal.position[1] + 0.5);
    globalNode * candidateGoal = getGlobalNode(scaledX, scaledY);

  // Check whether it is the same Global Node
    if ((candidateGoal->pose.position[0] == global_goalNode->pose.position[0])&&
        (candidateGoal->pose.position[1] == global_goalNode->pose.position[1]))
        return false;

  // Check whether it is valid (is not placed next to an obstacle Global Node)
    if ((candidateGoal->terrain == 0)||
        (candidateGoal->nb4List[0]->terrain == 0)||
        (candidateGoal->nb4List[1]->terrain == 0)||
        (candidateGoal->nb4List[2]->terrain == 0)||
        (candidateGoal->nb4List[3]->terrain == 0))
    {
        LOG_DEBUG_S << "Goal NOT valid, nearest global node is (" << global_goalNode->pose.position[0]
                << "," << global_goalNode->pose.position[1] << ") and is forbidden area";
        return false;
    }

    global_goalNode = candidateGoal;
    global_goalNode->pose.orientation = wGoal.heading;
    LOG_DEBUG_S << "Goal is global node (" << global_goalNode->pose.position[0]
              << "," << global_goalNode->pose.position[1] << ")";
    return true;
}


void PathPlanning::calculateGlobalPropagation(base::Waypoint wPos)
{
    LOG_DEBUG_S << "Computing Global Propagation";

  // To recompute the Global Propagation, first the Global Nodes used before
  // are reset
    if (!global_propagatedNodes.empty())
    {
        LOG_DEBUG_S << "resetting global nodes for new goal";
        for(uint i = 0; i<global_propagatedNodes.size(); i++)
        {
            global_propagatedNodes[i]->state = OPEN;
            global_propagatedNodes[i]->total_cost = INF;
        }
        global_propagatedNodes.clear();
    }

    global_narrowBand.clear();
    global_narrowBand.push_back(global_goalNode);
    global_propagatedNodes.push_back(global_goalNode);
    global_goalNode->total_cost = 0;

    globalNode * nodeTarget = global_goalNode;
    globalNode * startNode = getNearestGlobalNode(wPos);

    t1 = base::Time::now();
    LOG_DEBUG_S << "starting global propagation loop ";
    while ((!global_narrowBand.empty())&&(startNode->state == OPEN)) //TODO: if narrow band is empty, maybe nodeGoal is unreachable, fix this!!

    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)&&
                !(nodeTarget->nb4List[i]->isObstacle)) //TODO: take care of this for degenerated fields!!
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    t1 = base::Time::now() - t1;
    LOG_DEBUG_S << "ended global propagation loop in " << t1 << " seconds";

    expectedCost = getTotalCost(wPos);
    LOG_DEBUG_S << "expected total cost required to reach the goal: " << expectedCost;
}


void PathPlanning::propagateGlobalNode(globalNode* nodeTarget)
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
    C = global_cellSize*(nodeTarget->cost);

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
        {
            global_propagatedNodes.push_back(nodeTarget);
            global_narrowBand.push_back(nodeTarget);
        }
        nodeTarget->total_cost = T;
    }
}


globalNode* PathPlanning::minCostGlobalNode()
{
    globalNode* nodePointer = global_narrowBand.front();
    uint index = 0;
    uint i;
    double minCost = global_narrowBand.front()->total_cost;
    for (i =0; i < global_narrowBand.size(); i++)
    {
        if (global_narrowBand[i]->total_cost < minCost)
        {
            minCost = global_narrowBand[i]->total_cost;
            nodePointer = global_narrowBand[i];
            index = i;
        }
    }
    global_narrowBand.erase(global_narrowBand.begin() + index);
    return nodePointer;
}

globalNode* PathPlanning::getNearestGlobalNode(base::Pose2D pos)
{
    return getGlobalNode((uint)(pos.position[0]/global_cellSize + 0.5), (uint)(pos.position[1]/global_cellSize + 0.5));
}

globalNode* PathPlanning::getNearestGlobalNode(base::Waypoint wPos)
{
    return getGlobalNode((uint)(wPos.position[0]/global_cellSize + 0.5), (uint)(wPos.position[1]/global_cellSize + 0.5));
}




    /************************
     **** LOCAL PLANNING ****
     ************************/


std::vector<base::Waypoint> PathPlanning::getNewPath(base::Waypoint wPos)
{
    globalPath.clear();
    return getGlobalPath(wPos);
}

std::vector<base::Waypoint> PathPlanning::getGlobalPath(base::Waypoint wPos)
{
      base::Waypoint sinkPoint;
      base::Waypoint wNext;
      sinkPoint.position[0] = global_cellSize*global_goalNode->pose.position[0];
      sinkPoint.position[1] = global_cellSize*global_goalNode->pose.position[1];
      sinkPoint.position[2] = global_goalNode->elevation;
      sinkPoint.heading = global_goalNode->pose.orientation;

      std::vector<base::Waypoint> trajectory;

      trajectory.clear();
      double tau = std::min(0.4,risk_distance);
      wNext = calculateNextGlobalWaypoint(wPos, tau);
      trajectory.push_back(wPos);

      wPos = wNext;
      LOG_DEBUG_S << "trajectory initialized with tau = " << tau;


      while(sqrt(pow((wPos.position[0] - sinkPoint.position[0]),2) +
               pow((wPos.position[1] - sinkPoint.position[1]),2)) > 2.0*global_cellSize)
      {
          wNext = calculateNextGlobalWaypoint(wPos, tau);
          /*if (wNext == NULL)
          {
              LOG_WARN_S << "global waypoint (" << wNext.position[0] << "," << wNext.position[1] << ") is degenerate (nan gradient)";
              return trajectory;
          }*/
          trajectory.push_back(wPos);
          if(sqrt(pow((wPos.position[0] - wNext.position[0]),2) +
               pow((wPos.position[1] - wNext.position[1]),2)) < 0.01*tau*global_cellSize)
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
          globalPath.push_back(trajectory[i]);
      }
      return trajectory;
}

base::Waypoint PathPlanning::calculateNextGlobalWaypoint(base::Waypoint& wPos, double tau)
{

    base::Waypoint wNext;

  // Position of wPos in terms of global units
    double globalXpos = (wPos.position[0]-global_offset.position[0])/global_cellSize;
    double globalYpos = (wPos.position[1]-global_offset.position[1])/global_cellSize;

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

    wNext.position[0] = wPos.position[0] - global_cellSize*tau*dCostX;///sqrt(pow(dCostX,2) + pow(dCostY,2));
    wNext.position[1] = wPos.position[1] - global_cellSize*tau*dCostY;///sqrt(pow(dCostX,2) + pow(dCostY,2));

    wNext.heading = atan2(-dCostY,-dCostX);

    /*if ((dCostX)||(dCostY))
        return NULL;*/
    return wNext;
}

/*
 - Compute Local Waypoint Dijkstra
   -- Computation of next local waypoint using
      Dijkstra method
   -- This is unlikely to be called, just whenever
      a corridor surrounded by obstacles is met
*/



void PathPlanning::gradientNode(globalNode* nodeTarget, double& dnx, double& dny)
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

double PathPlanning::interpolate(double a, double b, double g00, double g01, double g10, double g11)
{
    return g00 + (g10 - g00)*a + (g01 - g00)*b + (g11 + g00 - g10 - g01)*a*b;
}


std::string PathPlanning::getLocomotionMode(base::Waypoint wPos)
{
    if(locomotion_modes.size() > 1)
    {
        double Cdefinitive, Ccandidate, C1, C2;
        int range = slope_range.size();
        int numLocs = locomotion_modes.size();
        int locIndex;

        globalNode * gNode = getNearestGlobalNode(wPos);

        if(range == 1) //Slopes are not taken into account
        {
            Cdefinitive = cost_data[gNode->terrain*numLocs];
            locIndex = 0;
            for(uint i = 1; i<locomotion_modes.size(); i++)
            {
                Ccandidate = cost_data[gNode->terrain*numLocs + i];
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
            double slopeEq, omega;

            omega = acos(cos(gNode->aspect)*cos(wPos.heading)+sin(gNode->aspect)*sin(wPos.heading));
            slopeEq = acos(sqrt(pow(cos(omega),2)*pow(cos(gNode->slope),2)+pow(sin(omega),2)));

            LOG_DEBUG_S << "equivalent slope is " << slopeEq << " with omega = " << omega << " and heading = " << wPos.heading << " and aspect = " << gNode->aspect;
            double slopeIndex = slopeEq*180/M_PI/(slope_range.back()-slope_range.front())*(slope_range.size()-1);
            if(slopeIndex > (slope_range.size()-1))
                slopeIndex = (slope_range.size()-1);

            double slopeMinIndex = std::floor(slopeIndex);
            double slopeMaxIndex = std::ceil(slopeIndex);
            C1 = cost_data[gNode->terrain*range*numLocs + (int)slopeMinIndex];
            C2 = cost_data[gNode->terrain*range*numLocs + (int)slopeMaxIndex];
            Cdefinitive = C1 + (C2-C1)*(slopeIndex-slopeMinIndex);
            locIndex = 0;
                /*if((nodeTarget->terrain>0)&&((nodeTarget->slope)*180/M_PI<20.0)&&((nodeTarget->slope)*180/M_PI>10.0))
                    LOG_DEBUG_S << "checking cost -> " << Cdefinitive  << "and slopeIndex is" << slopeIndex << " and slopeMinIndex is " << slopeMinIndex <<
                      " and slopeMaxIndex is " << slopeMaxIndex
                      << " and C1 = " << C1 << " and C2 = " << C2;*/
            for(uint i = 1; i<locomotion_modes.size();i++)
            {
                C1 = cost_data[gNode->terrain*range*numLocs + i*range + (int)slopeMinIndex];
                C2 = cost_data[gNode->terrain*range*numLocs + i*range + (int)slopeMaxIndex];
                Ccandidate = C1 + (C2-C1)*(slopeIndex-slopeMinIndex);
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





// Output Total Cost Map
base::samples::DistanceImage PathPlanning::getGlobalTotalCostMap()
{
    base::samples::DistanceImage globalTotalCostMap;
    globalTotalCostMap.setSize(globalMap[0].size(),globalMap.size());
    for (uint j = 0; j < globalMap.size(); j++)
    {
        for (uint i = 0; i < globalMap[0].size(); i++)
        {
                globalTotalCostMap.data[i + (globalMap.size()-j-1)*globalMap[0].size()] = (globalMap[j][i]->total_cost);
        }
    }
    globalTotalCostMap.scale_x = global_cellSize;
    globalTotalCostMap.scale_y = global_cellSize;
    globalTotalCostMap.center_x = global_offset.position[0] + global_cellSize*0.5*globalMap[0].size();
    globalTotalCostMap.center_y = global_offset.position[1] + global_cellSize*0.5*globalMap.size();
    return globalTotalCostMap;
}


// Output Cost Map
base::samples::DistanceImage PathPlanning::getGlobalCostMap()
{
    base::samples::DistanceImage globalCostMap;
    globalCostMap.setSize(globalMap[0].size(),globalMap.size());
    for (uint j = 0; j < globalMap.size(); j++)
    {
        for (uint i = 0; i < globalMap[0].size(); i++)
        {
                globalCostMap.data[i + (globalMap.size()-j-1)*globalMap[0].size()] = (globalMap[j][i]->cost);
        }
    }
    globalCostMap.scale_x = global_cellSize;
    globalCostMap.scale_y = global_cellSize;
    globalCostMap.center_x = global_offset.position[0] + global_cellSize*0.5*globalMap[0].size();
    globalCostMap.center_y = global_offset.position[1] + global_cellSize*0.5*globalMap.size();
    return globalCostMap;
}
