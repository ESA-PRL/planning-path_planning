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


/***************************CREATE LOCAL MAP***********************************/
// The global node is subdivided into local nodes, which are connected to the
// rest forming the local layer
void DyMuPathPlanner::createLocalMap(globalNode* gNode)
{
  // We set that, effectively, this global node is subdivided
    gNode->hasLocalMap = true;

  // This square portion of local layer is created as a matrix
    std::vector<localNode*> nodeRow;
    for (uint j = 0; j < res_ratio; j++)
    {
        for (uint i = 0; i < res_ratio; i++)
        {
            nodeRow.push_back(new localNode(i, j, gNode->pose));
            nodeRow.back()->global_pose.position[0] =
                nodeRow.back()->parent_pose.position[0] - 0.5 +
                (0.5/(double)res_ratio) +
                nodeRow.back()->pose.position[0]*(1/(double)res_ratio);
            nodeRow.back()->global_pose.position[1] =
                nodeRow.back()->parent_pose.position[1] - 0.5 +
                (0.5/(double)res_ratio) +
                nodeRow.back()->pose.position[1]*(1/(double)res_ratio);
            nodeRow.back()->world_pose.position[0] =
                             nodeRow.back()->global_pose.position[0]/global_res;
            nodeRow.back()->world_pose.position[1] =
                             nodeRow.back()->global_pose.position[1]/global_res;
        }
        gNode->localMap.push_back(nodeRow);
        nodeRow.clear();
    }

  // NEIGHBOURHOOD is created, considering local nodes of neighbouring global
  // nodes
    for (uint j = 0; j < res_ratio; j++)
    {
        for (uint i = 0; i < res_ratio; i++)
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

            gNode->localMap[j][i]->nb4List.clear();
            if (j==0)
            {
                if (gNode->nb4List[0]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[0]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(
                                   gNode->nb4List[0]->localMap[res_ratio-1][i]);
                        gNode->nb4List[0]->localMap[res_ratio-1][i]->nb4List[3]
                                                        = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(
                                                       gNode->localMap[j-1][i]);

            if (i==0)
            {
                if (gNode->nb4List[1]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[1]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(
                                   gNode->nb4List[1]->localMap[j][res_ratio-1]);
                        gNode->nb4List[1]->localMap[j][res_ratio-1]->nb4List[2]
                                                        = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(
                                                       gNode->localMap[j][i-1]);

            if (i==res_ratio-1)
            {
                if (gNode->nb4List[2]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[2]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(
                                             gNode->nb4List[2]->localMap[j][0]);
                        gNode->nb4List[2]->localMap[j][0]->nb4List[1] =
                                                          gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(
                                                       gNode->localMap[j][i+1]);

            if (j==res_ratio-1)
            {
                if (gNode->nb4List[3]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[3]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(
                                             gNode->nb4List[3]->localMap[0][i]);
                        gNode->nb4List[3]->localMap[0][i]->nb4List[0] =
                                                          gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(
                                                       gNode->localMap[j+1][i]);
        }
    }
}

/**********************SUBDIVIDE THE GLOBAL NODE*******************************/
// If the global node was not already subdivided, do it now (and its
// 8-neighbours just in case)
void DyMuPathPlanner::subdivideGlobalNode(globalNode* gNode)
{
    if(!gNode->hasLocalMap) createLocalMap(gNode);
    for (int i = 0; i<8; i++)
        if(!(gNode->nb8List[i]==NULL))
            if(!(gNode->nb8List[i]->hasLocalMap))
                createLocalMap(gNode->nb8List[i]);
}

/*************************GET CLOSEST LOCAL NODE*******************************/
// The global node is subdivided into local nodes, which are connected to the
// rest forming the local layer
localNode* DyMuPathPlanner::getLocalNode(base::Pose2D pos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(pos);

    subdivideGlobalNode(nearestNode);

    double cornerX = nearestNode->pose.position[0] - global_res/2;
    double cornerY = nearestNode->pose.position[1] - global_res/2;
    double a = pos.position[0] - cornerX;
    double b = pos.position[1] - cornerY;
    return nearestNode->localMap[(uint)(b*res_ratio)][(uint)(a*res_ratio)];
}


localNode* DyMuPathPlanner::getLocalNode(base::Waypoint wPos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(wPos);
    subdivideGlobalNode(nearestNode);

    double cornerX = nearestNode->pose.position[0] - global_res/2;
    double cornerY = nearestNode->pose.position[1] - global_res/2;
    double a = wPos.position[0] - cornerX;
    double b = wPos.position[1] - cornerY;
    return nearestNode->localMap[(uint)(b*res_ratio)][(uint)(a*res_ratio)];
}


/*************************GET CLOSEST LOCAL NODE*******************************/
// The global node is subdivided into local nodes, which are connected to the
// rest forming the local layer
bool DyMuPathPlanner::computeLocalPlanning(base::Waypoint wPos,
                                  base::samples::frame::Frame traversabilityMap,
                                  double res,
                                  std::vector<base::Waypoint>& trajectory,
                                  bool keepOldWaypoints, base::Time &localTime)
{

    localNode* lNode;
    globalNode* gNode;

    uint height = traversabilityMap.getHeight();
    uint width = traversabilityMap.getWidth();


  // Create new Local Nodes if necessary
    uint a = (uint)(fmax(0,((wPos.position[1] -
                            (double)height/2*res)/global_res)));
    uint b = (uint)(fmin(global_layer.size(),
                       ((wPos.position[1] + (double)height/2*res)/global_res)));
    uint c = (uint)(fmax(0,((wPos.position[0] -
                            (double)width/2*res)/global_res)));
    uint d = (uint)(fmin(global_layer[0].size(),
                        ((wPos.position[0] + (double)width/2*res)/global_res)));
    for (uint j = a; j < b; j++)
        for (uint i = c; i < d; i++)
            subdivideGlobalNode(global_layer[j][i]);

  //Indexes of the minimum and maximum waypoints affected in the trajectory by obstacles
    uint minIndex = current_path.size(), maxIndex = 0;
    bool isBlocked = false;
    bool pathBlocked = false;

    double offsetX = wPos.position[0] - res*(double)width/2;
    double offsetY = wPos.position[1] + res*(double)height/2; // Image convention (Y pointing down)

    double globalSizeX = global_res*global_layer[0].size()-0.5;
    double globalSizeY = global_res*global_layer.size()-0.5;


    base::Pose2D pos;
    for (uint j = 0; j < height; j++)
        for (uint i = 0; i < width; i++)
        {
            pos.position[0] = offsetX + i*res;
            pos.position[1] = offsetY - j*res; // Image convention (Y pointing down)
            if ((pos.position[0] > -0.5)&&(pos.position[0] < globalSizeX)&&
                (pos.position[1] > -0.5)&&(pos.position[1] < globalSizeY))
            {
                uint8_t value = traversabilityMap.image[
                                            j*traversabilityMap.getRowSize() +
                                            i*traversabilityMap.getPixelSize()];
                lNode = getLocalNode(pos);
                gNode = getNearestGlobalNode(lNode->parent_pose);
                if ((!lNode->isObstacle)&&((value != 0)||(gNode->isObstacle))) //If pixel is obstacle (value == 1)
                {
                    lNode->isObstacle = true;
                    local_expandable_obstacles.push_back(lNode);
                    lNode->risk = 1.0;
                    isBlocked = isBlockingObstacle(lNode, maxIndex, minIndex, trajectory);//See here if its blocking (and which waypoint)
                    pathBlocked = (pathBlocked)?true:isBlocked;// Path is blocked if isBlocked is true at least once
                    gNode->hazard_density = std::min(1.0,
                            gNode->hazard_density + 1.0/(res_ratio*res_ratio)) ;
                    /*if(gNode->hazard_density < 1.0)
                    {
                        gNode->hazard_density += 1.0/(res_ratio*res_ratio);
                    }*/
                    for (uint k = 0; k<8; k++)
                    {
                        if (gNode->nb8List[k] != NULL)
                        {
                            gNode->nb8List[k]->hazard_density = std::min(1.0, gNode->nb8List[k]->hazard_density + 0.1/(res_ratio*res_ratio));
                        }
                    }
                }
            }
        }

    std::cout << " Max index = " << maxIndex << "; Min index = " << minIndex << std::endl;
    if((pathBlocked)&&(maxIndex>minIndex))
    {
        std::cout  << "-- LOCAL REPAIRING STARTED--" << std::endl;
        base::Time tInit = base::Time::now();
        std::cout << " Path Size: " << current_path.size() << std::endl;
        std::cout <<  "Max Index: " << maxIndex << std::endl;
        expandRisk();
        trajectory.clear();
        reconnecting_index = repairPath(wPos, maxIndex, keepOldWaypoints);
        trajectory = current_path;
        localTime = base::Time::now() - tInit;
        std::cout << "Local Time in library: " << (base::Time::now()-tInit)  << std::endl;
        return true;
    }
    return false;
}

/*****************************PATH REPAIRING***********************************/
// current_path is modified according to the following input:
//  - wayp_start = waypoint from which the wave will propagate
//  - index = points to the position in current_path where the Overtake Waypoint
//    is located
//  - keepOldWaypoints = if yes, conservative approach is used (aka Hazard Avoi-
//    dance), otherwise it is used the sweeping approach (aka multiBiFM)
int DyMuPathPlanner::repairPath(base::Waypoint wayp_start, uint index,
                                bool keepOldWaypoints)
{
    //std::cout << "NEW REPAIRING" << std::endl;
    //std::cout << "Initial Waypoint is " << wayp_start.position[0] << "," << wayp_start.position[1] << std::endl;
    //std::cout << "Input index is " << index << std::endl;
    //std::cout << "Size of current path is " << current_path.size() << std::endl;
	// It may happen a path with no Global Waypoints is intended to be repaired
  // (since trajectory was cleared in previous repairing)
	  if (current_path.empty())
	  {
		    std::cout << "Repairing aborted." << std::endl;
		    return -1;
	  }

  // overtake_index is pointing to the Global Waypoint placed after risk area,
  //   which is initially pointed by index as well
	// index increases to point to another Global Waypoint placed further than
  //   reconnect_distance to the one pointed by overtake_index
    double overtake_index;
    if(keepOldWaypoints)
    {
      // In case of using the conservative approach, the propagation wave will
      // reach either the Overtake waypoint or another placed further
        overtake_index = (reconnecting_index>index)?reconnecting_index:index;
        index = overtake_index;
    }
    else
        overtake_index = index;

    // TODO: maybe this should be set outside
    while((index < current_path.size())&&
	      (sqrt(pow(current_path[index].position[0] -
         current_path[overtake_index].position[0],2) +
         pow(current_path[index].position[1] -
         current_path[overtake_index].position[1],2)) < reconnect_distance))
        index++;

  // If index overflows current_path, it means there is no Global Waypoint to
  // reconnect/overtake
    if(index >= current_path.size())
    {
        //trajectory.push_back(wayp_start);
        /*std::cout << "initial number of Global Waypoints was " << current_path.size() << std::endl;
        std::cout << "Global Waypoint to reconnect does not exist, variable index is " << index << std::endl;*/
        current_path.clear();
        current_path.push_back(wayp_start);
        return -1;
    }
    else
    {
      // Last Waypoint is the goal, only Conservative Approach is permitted
        if (index == current_path.size()-1)
            keepOldWaypoints = 1;
        localNode * lSet = computeLocalPropagation(wayp_start, current_path[index],
                                                   keepOldWaypoints);
        if (lSet == NULL) //Local Planning is aborted because of having entered obstacle area
        {
            std::cout << "repairing aborted" << std::endl;
            LOG_DEBUG_S << "index = " << index;
            std::cout << "global stops at (" << current_path[index].position[0]
                  << "," << current_path[index].position[1] << ")" << std::endl;
            //trajectory.push_back(wayp_start);
            current_path.clear();
            current_path.push_back(wayp_start);
            return -1;
        }
        else
        {
          // Look for the waypoint closest to the rover position
            //std::cout << "Index = " << index << std::endl;
            double proximity, candidate_proximity, original_distance = 0, new_distance = 0;
            uint closest_index = 0;
            proximity = sqrt(pow(current_path[0].position[0] - wayp_start.position[0],2) + pow(current_path[0].position[1] - wayp_start.position[1],2));
            for (uint k = 1; k<index; k++)
            {
                candidate_proximity = sqrt(pow(current_path[k].position[0] - wayp_start.position[0],2) + pow(current_path[k].position[1] - wayp_start.position[1],2));
                if (candidate_proximity < proximity)
                    closest_index = k;
            }
            for (uint k = closest_index; k < index; k++)
                original_distance += sqrt(pow(current_path[k+1].position[0] - current_path[k].position[0],2) + pow(current_path[k+1].position[1] - current_path[k].position[1],2));
            std::cout << "The length of the old segment is " << original_distance << " meters" << std::endl;
            std::vector<base::Waypoint> localPath = getLocalPath(lSet,wayp_start,0.4);
            std::cout << "Local Path size is " << localPath.size() << " meters" << std::endl;
            if (localPath.size()>1)
            {
                for (uint k = 0; k < localPath.size()-1; k++)
                {
                    new_distance += sqrt(pow(localPath[k+1].position[0] - localPath[k].position[0],2) + pow(localPath[k+1].position[1] - localPath[k].position[1],2));
                    /*std::cout << "Local Waypoint " << k << " is " << localPath[k].position[0] << "," << localPath[k].position[1] << std::endl;
                    std::cout << "Distance is " << new_distance << std::endl;*/
                }
                std::cout << "The length of the new segment is " << new_distance << " meters" << std::endl;
                std::cout << "Trafficability is " << (original_distance/new_distance) << std::endl;
                /*LOG_DEBUG_S << "local path starts at (" << localPath[0].position[0] << "," << localPath[0].position[1] << ")";*/
                globalNode* gNode;
                for (uint k = closest_index; k < index; k++)
                {
                    gNode = getNearestGlobalNode(current_path[k]);
                    gNode->trafficability = std::min(original_distance/new_distance,gNode->trafficability);
                }
                if (keepOldWaypoints)
                {
                    // Remove old trajectory, including one extra global waypoint, which is coincident with last local waypoint
                    current_path.erase(current_path.begin(), current_path.begin() + index);
                    localPath.pop_back();
                    current_path.insert(current_path.begin(), localPath.begin(),localPath.end());
                    return localPath.size();
                    //trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                    //trajectory.insert(trajectory.end(),current_path.begin(),current_path.end());
                }
                else
                {
                    base::Waypoint newWaypoint;
                    newWaypoint.position[0] = lSet->global_pose.position[0];
                    newWaypoint.position[1] = lSet->global_pose.position[1];
                    current_path.clear();
                    current_path = getGlobalPath(newWaypoint);
                    localPath.pop_back();
                    current_path.insert(current_path.begin(), localPath.begin(),localPath.end());
                    return localPath.size();
                    //current_path.erase(current_path.begin());
                    //trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                    //trajectory.insert(trajectory.end(),current_path.begin(),current_path.end());
                }
            }
            else
            {
              if (keepOldWaypoints)
              {
                  // Remove old trajectory, including one extra global waypoint, which is coincident with last local waypoint
                  current_path.erase(current_path.begin(), current_path.begin() + index);
                  return 0;
                  //trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                  //trajectory.insert(trajectory.end(),current_path.begin(),current_path.end());
              }
              else
              {
                  base::Waypoint newWaypoint;
                  newWaypoint.position[0] = lSet->global_pose.position[0];
                  newWaypoint.position[1] = lSet->global_pose.position[1];
                  current_path.clear();
                  current_path = getGlobalPath(newWaypoint);
                  return 0;
                  //current_path.erase(current_path.begin());
                  //trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                  //trajectory.insert(trajectory.end(),current_path.begin(),current_path.end());
              }
            }
        }
    }
}

bool DyMuPathPlanner::isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex, std::vector<base::Waypoint> trajectory)
{
    bool isBlocked = false;
    for (uint i = 0; i < current_path.size(); i++)
    {
        /*LOG_DEBUG_S << "distance = " << sqrt(
            pow(obNode->global_pose.position[0]-current_path[i].position[0],2) +
            pow(obNode->global_pose.position[1]-current_path[i].position[1],2));
        LOG_DEBUG_S << "global pose = " << obNode->global_pose.position[0] << "," << obNode->global_pose.position[1]
                  << " and global path position is " << current_path[i].position[0] << "," << current_path[i].position[1];*/
        if(sqrt(
            pow(obNode->world_pose.position[0]-current_path[i].position[0],2) +
            pow(obNode->world_pose.position[1]-current_path[i].position[1],2))
            < risk_distance)
        {
            if(!isBlocked)
            {
                isBlocked = true;
                minIndex = (i<minIndex)?i:minIndex;
		//minIndex = max(i,minIndex);
            }
            else
                maxIndex = (i>maxIndex)?i:maxIndex;
		//maxIndex = min(i,minIndex);
        }
        else if (isBlocked)
        {
            maxIndex = (i>maxIndex)?i:maxIndex;
            return isBlocked;
        }
    }
    if (isBlocked) // In case it arrives here, last Global Waypoint cannot be reconnected (impossible solution)
        maxIndex = current_path.size();

    // At this point, it seems is no blocking obstacle, but it could block a local waypoint yet
    /*for (uint k = 0; k < trajectory.size(); k++)
        if(sqrt(
            pow(obNode->world_pose.position[0]-trajectory[k].position[0],2) +
            pow(obNode->world_pose.position[1]-trajectory[k].position[1],2))
            < risk_distance)
            return true;*/

    return isBlocked;
}


double DyMuPathPlanner::getTotalCost(localNode* lNode)
{
    uint i = (uint)(lNode->global_pose.position[0]);
    uint j = (uint)(lNode->global_pose.position[1]);
    double a = lNode->global_pose.position[0] - (double)(i);
    double b = lNode->global_pose.position[1] - (double)(j);

    globalNode * node00 = getNearestGlobalNode(lNode->parent_pose);
    globalNode * node10 = node00->nb4List[2];
    globalNode * node01 = node00->nb4List[3];
    globalNode * node11 = node00->nb4List[2]->nb4List[3];

    double w00 = (node00==NULL)?INF:node00->total_cost;
    double w10 = (node10==NULL)?INF:node10->total_cost;
    double w01 = (node01==NULL)?INF:node01->total_cost;
    double w11 = (node11==NULL)?INF:node11->total_cost;

    return w00 + (w10 - w00)*a + (w01 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
}

void DyMuPathPlanner::expandRisk()
{
    localNode * nodeTarget;
    globalNode* gNode;
    base::Time tInit = base::Time::now(), tCurrent;
    while(!local_expandable_obstacles.empty())
    {
        nodeTarget = maxRiskNode();
        for (uint i = 0; i<4; i++)
        {
            if ((nodeTarget->nb4List[i] != NULL)&&(!nodeTarget->nb4List[i]->isObstacle))
            {
                gNode = getNearestGlobalNode(nodeTarget->nb4List[i]->parent_pose);
                if (getNearestGlobalNode(nodeTarget->parent_pose) != gNode)
                    subdivideGlobalNode(gNode);
            }
            if ((nodeTarget->nb4List[i] != NULL)&&(!nodeTarget->nb4List[i]->isObstacle))
                propagateRisk(nodeTarget->nb4List[i]);
        }
        tCurrent = base::Time::now() - tInit;
        /*if (tCurrent.toSeconds() > 1.0)
        {
            std::cout << "ERROR propagating risk" << std::endl;
            std::cout << "Parent is " << gNode->pose.position[0] << "," << gNode->pose.position[1] << std::endl;
            std::cout << "Neighbor parent is " <<nodeTarget->nb4List[3]->parent_pose.position[0] << "," << nodeTarget->nb4List[3]->parent_pose.position[1] << std::endl;
            break;
        }*/
    }
}

localNode* DyMuPathPlanner::maxRiskNode()
{
    if (local_expandable_obstacles.empty())
        return NULL;
    localNode* nodePointer = local_expandable_obstacles.front();
    uint index = 0;
    double maxRisk = local_expandable_obstacles.front()->risk;
    // LOG_DEBUG_S << "Size of Narrow Band is: " << this->narrowBand.size();
    for (uint i =0; i < local_expandable_obstacles.size(); i++)
    {
        if (maxRisk == 1)
            break;
        if (local_expandable_obstacles[i]->risk > maxRisk)
        {
            maxRisk = local_expandable_obstacles[i]->risk;
            nodePointer = local_expandable_obstacles[i];
            index = i;
            break;
        }
    }
    /* LOG_DEBUG_S << "next expandable node is  (" <<
        nodePointer->pose.position[0] << "," <<
        nodePointer->pose.position[1] << ")";*/
    local_expandable_obstacles.erase(local_expandable_obstacles.begin() + index);
    return nodePointer;
}

void DyMuPathPlanner::propagateRisk(localNode* nodeTarget)
{
    double Ry,Rx;
    localNode * Ny0 = nodeTarget->nb4List[0];
    localNode * Ny1 = nodeTarget->nb4List[3];
    Ry = fmax(Ny0 == NULL?0:Ny0->risk, Ny1 == NULL?0:Ny1->risk);
    localNode * Nx0 = nodeTarget->nb4List[1];
    localNode * Nx1 = nodeTarget->nb4List[2];
    Rx = fmax(Nx0 == NULL?0:Nx0->risk, Nx1 == NULL?0:Nx1->risk);

    double Sx = 1 - Rx;
    double Sy = 1 - Ry;
    double C = local_res/risk_distance;
    double S;

    if (fabs(Sx-Sy)<C)
        S = (Sx+Sy+sqrt(2*pow(C,2.0) - pow((Sx-Sy),2.0)))/2;
    else
        S = fmin(Sx,Sy) + C;

    double R = std::max(1 - S,0.0);
    if ((R>0)&&(R>nodeTarget->risk))
    {
        nodeTarget->risk = R;
        local_expandable_obstacles.push_back(nodeTarget);
    }
}


localNode * DyMuPathPlanner::computeLocalPropagation(base::Waypoint wayp_start, base::Waypoint wOvertake, bool keepOldWaypoints)
{
  //wayp_start is the waypoint from which the path is repaired

    double Tovertake = getTotalCost(wOvertake);
    std::cout << "Overtake is " << wOvertake.position[0] << "," << wOvertake.position[1] << " and T associated is " << Tovertake << std::endl;
    double distReference = sqrt(
              pow(wayp_start.position[0]-wOvertake.position[0],2)
            + pow(wayp_start.position[1]-wOvertake.position[1],2));

    if(!local_propagated_nodes.empty())
    {
    LOG_DEBUG_S << "resetting previous closed nodes";
        for (uint i = 0; i < local_propagated_nodes.size(); i++)
        {
            local_propagated_nodes[i]->state = OPEN;
            local_propagated_nodes[i]->deviation = INF;
            local_propagated_nodes[i]->total_cost = INF;
        }
        local_propagated_nodes.clear();
    }

  // Initializing the Narrow Band
    std::cout << "Init Waypoint is " << wayp_start.position[0] << "," << wayp_start.position[1] << std::endl;

    local_agent = getLocalNode(wayp_start);

    std::cout << "Local Agent is " << local_agent->pose.position[0] << "," << local_agent->pose.position[1] << std::endl;


    if(local_agent->isObstacle)
    {
        std::cout << "actual pose is within obstacle area, local repairing aborted" << std::endl;
        return NULL;
    }

    local_agent->deviation = 0;
    local_agent->total_cost = getTotalCost(local_agent);
    local_agent->state = CLOSED;

    local_narrowband.clear();
    local_narrowband.push_back(local_agent);

    local_propagated_nodes.push_back(local_agent);

    localNode * nodeTarget;
    localNode * nodeEnd = NULL;
    if (keepOldWaypoints)
    {
        nodeEnd = getLocalNode(wOvertake);
        if(nodeEnd->isObstacle)
        {
            std::cout << "nodeEnd is obstacle" << std::endl;
            return NULL;
        }
    }
    globalNode* gNode;


    double minC = (getTotalCost(wayp_start)-Tovertake)/distReference;


  // Propagation Loop
    std::cout << "starting local propagation loop" << std::endl;
    std::cout << "initial risk is " << local_agent->risk << std::endl;
    std::cout << "minC is " << minC << std::endl;

    base::Time tInit = base::Time::now();
    base::Time tCurrent = base::Time::now();
    while(true)//TODO: Control this
    {
        try
        {
            if (keepOldWaypoints)
                nodeTarget = minCostLocalNode(nodeEnd);
            else
                nodeTarget = minCostLocalNode(Tovertake, minC);
        }
        catch(int n)
        {
            std::cout << "PLANNING_ERROR: minCostLocalNode failed" << std::endl;
        }
        nodeTarget->state = CLOSED;
        //LOG_DEBUG_S << "nodeTarget " << nodeTarget->global_pose.position[0] << "," << nodeTarget->global_pose.position[1];
        for (uint i = 0; i<4; i++)
        {
            if (nodeTarget->nb4List[i] != NULL)
            {
                try
                {
                    gNode = getNearestGlobalNode(nodeTarget->nb4List[i]->parent_pose);
                }
                catch(int n)
                {
                    std::cout << "PLANNING_ERROR: getNearestGlobalNode of neighbour failed" << std::endl;
                }
                try
                {
                    if (getNearestGlobalNode(nodeTarget->parent_pose) != gNode)
                        subdivideGlobalNode(gNode);
                }
                catch(int n)
                {
                    std::cout << "PLANNING_ERROR: subdivideGlobalNode failed" << std::endl;
                }
            }
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)
                &&(!nodeTarget->nb4List[i]->isObstacle))
            {
                try
                {
                    propagateLocalNode(nodeTarget->nb4List[i]);
                }
                catch(int n)
                {
                    std::cout << "PLANNING_ERROR: propagateLocalNode failed" << std::endl;
                }
                try
                {
                    if (nodeEnd == NULL)
                        if ((nodeTarget->nb4List[i]->total_cost < Tovertake)&&(nodeTarget->nb4List[i]->risk == 0))
                            nodeEnd = nodeTarget->nb4List[i];
                }
                catch(int n)
                {
                    std::cout << "PLANNING_ERROR: nodeEnd assignment failed" << std::endl;
                }
            }
        }
        if ((nodeEnd != NULL)&&(nodeEnd->state == CLOSED)&&
            (nodeEnd->nb4List[0]->state == CLOSED)&&(nodeEnd->nb4List[1]->state == CLOSED)&&
            (nodeEnd->nb4List[2]->state == CLOSED)&&(nodeEnd->nb4List[3]->state == CLOSED))
        {
            std::cout << "Node End is " << nodeEnd->pose.position[0] << "," << nodeEnd->pose.position[1] << "; Risk associated is " << nodeEnd->risk << "; T associated is " << nodeEnd->total_cost << std::endl;
            return nodeEnd;
        }
        tCurrent = base::Time::now() - tInit;
        if (tCurrent.toSeconds() > 5.0)
        {
            std::cout << "ERROR" << std::endl;
            //std::cout << "Node Target is " << nodeEnd->pose.position[0] << "," << nodeEnd->pose.position[1] << "; Risk associated is " << nodeEnd->risk << "; T associated is " << nodeEnd->total_cost << std::endl;
            std::cout << "The local narrowband size is " << local_narrowband.size() << std::endl;
            std::cout << "The local propagated nodes size is " << local_propagated_nodes.size() << std::endl;
            return NULL;
        }
    }

}

void DyMuPathPlanner::propagateLocalNode(localNode* nodeTarget)
{
    double Tx,Ty,T,R,C;

  // Neighbor Propagators Tx and Ty
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Ty = fmin(nodeTarget->nb4List[3]->deviation, nodeTarget->nb4List[0]->deviation);
    else if (nodeTarget->nb4List[0] == NULL)
        Ty = nodeTarget->nb4List[3]->deviation;
    else
        Ty = nodeTarget->nb4List[0]->deviation;

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = fmin(nodeTarget->nb4List[1]->deviation, nodeTarget->nb4List[2]->deviation);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->deviation;
    else
        Tx = nodeTarget->nb4List[1]->deviation;

  //Cost Function
    R = nodeTarget->risk;

    if (nodeTarget->total_cost == INF)
        nodeTarget->total_cost = getTotalCost(nodeTarget);

    C = local_res*(risk_ratio*R + 1);

    if(C <= 0) //TODO: make this function return a false bool
        LOG_ERROR_S << "C is not positive";

    if(C > INF)
        LOG_ERROR_S << "C is higher than INF";

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->deviation)
    {
        if (nodeTarget->deviation == INF) //It is not in narrowband
        {
            local_narrowband.push_back(nodeTarget);
            local_propagated_nodes.push_back(nodeTarget);
        }
        nodeTarget->deviation = T;
    }
}

localNode* DyMuPathPlanner::minCostLocalNode(double Tovertake, double minC)
{
    localNode* nodePointer = local_narrowband.front();
    uint index = 0;
    uint i;
    double minH = local_narrowband.front()->deviation;// + fmax(0, local_narrowband.front()->total_cost - Tovertake)/minC;
    double currentH;
    // LOG_DEBUG_S << "Size of Narrow Band is: " << local_narrowband.size();
    for (i =0; i < local_narrowband.size(); i++)
    {
        currentH = local_narrowband[i]->deviation;// + fmax(0, local_narrowband[i]->total_cost - Tovertake)/minC;
        if (currentH < minH)
        {
            minH = currentH;
            nodePointer = local_narrowband[i];
            index = i;
        }
    }
    local_narrowband.erase(local_narrowband.begin() + index);
    return nodePointer;
}


localNode* DyMuPathPlanner::minCostLocalNode(localNode* reachNode)
{
    localNode* nodePointer = local_narrowband.front();
    uint index = 0;
    uint i;
    double distance = sqrt(
              pow(local_narrowband.front()->world_pose.position[0]-reachNode->world_pose.position[0],2)
            + pow(local_narrowband.front()->world_pose.position[1]-reachNode->world_pose.position[1],2));
    double minH = local_narrowband.front()->deviation + distance;
    double currentH;
    //LOG_DEBUG_S << "Size of Narrow Band is: " << local_narrowband.size();
    for (i =0; i < local_narrowband.size(); i++)
    {
        distance = sqrt(
              pow(local_narrowband[i]->world_pose.position[0]-reachNode->world_pose.position[0],2)
            + pow(local_narrowband[i]->world_pose.position[1]-reachNode->world_pose.position[1],2));
        currentH = local_narrowband[i]->deviation + distance;
        if (currentH < minH)
        {
            minH = currentH;
            nodePointer = local_narrowband[i];
            index = i;
        }
    }
    local_narrowband.erase(local_narrowband.begin() + index);
    return nodePointer;
}


std::vector<base::Waypoint> DyMuPathPlanner::getLocalPath(localNode * lSetNode,
                                                       base::Waypoint wayp_start,
                                                       double tau)
{
    base::Waypoint wPos;
    bool newWaypoint;
    wPos.position[0] = lSetNode->global_pose.position[0];
    wPos.position[1] = lSetNode->global_pose.position[1];
    wPos.heading = lSetNode->global_pose.orientation;

    tau = 0.5*local_res;
    std::vector<base::Waypoint> trajectory;
    newWaypoint = computeLocalWaypointGDM(wPos, tau*local_res);
    trajectory.insert(trajectory.begin(),wPos);
    LOG_DEBUG_S << "repairing trajectory initialized";
    LOG_DEBUG_S << "lSetNode at " << wPos.position[0] << ", " << wPos.position[1];
    LOG_DEBUG_S << "wayp_start at " << wayp_start.position[0] << ", " << wayp_start.position[1];

    while(sqrt(pow((trajectory.front().position[0] - wayp_start.position[0]),2) +
             pow((trajectory.front().position[1] - wayp_start.position[1]),2)) > 1.5*local_res)
    {
        newWaypoint = computeLocalWaypointGDM(wPos, tau);
        if (newWaypoint)
            trajectory.insert(trajectory.begin(),wPos);
        else
        {
            LOG_WARN_S << "local trajectory is degenerated due to obstacles";
            localNode * lNode = getLocalNode(trajectory[0]);
            while(lNode->deviation == INF)
            {
                trajectory.erase(trajectory.begin());
                lNode = getLocalNode(trajectory[0]);
            }
            wPos = computeLocalWaypointDijkstra(lNode);
            trajectory.insert(trajectory.begin(),wPos);
        }
         //   return trajectory;
        /*if (trajectory.size() > 999)//TODO: quit this
        {
            LOG_ERROR_S << "computing local trajectory";
            return trajectory;
        }*/
    }

    LOG_DEBUG_S << "trajectory front at " << trajectory.front().position[0] << ", " << trajectory.front().position[1];
    LOG_DEBUG_S << "Local cell size is " << local_res;
    return trajectory;
}

base::Waypoint DyMuPathPlanner::computeLocalWaypointDijkstra(localNode * lNode)
{
    double newX, newY, t = INF;
    base::Waypoint wPos;

    for (uint i = 1; i<4; i++)
        if((lNode->nb4List[i] != NULL)&&(lNode->nb4List[i]->deviation < t))
        {
            t = lNode->nb4List[i]->deviation;
            newX = lNode->nb4List[i]->world_pose.position[0];
            newY = lNode->nb4List[i]->world_pose.position[1];
        }

    wPos.position[0] = newX;
    wPos.position[1] = newY;
    wPos.heading = atan2(newY-lNode->world_pose.position[1],newX-lNode->world_pose.position[0]);
    return wPos;
}


/*
  - Compute Local Waypoint GDM
    -- Computation of next local waypoint using
       gradient descent method
*/

bool DyMuPathPlanner::computeLocalWaypointGDM(base::Waypoint& wPos, double tau)
{
    double a,b;

    double gx00, gx10, gx01, gx11;
    double gy00, gy10, gy01, gy11;

    localNode * lNode = getLocalNode(wPos);
    localNode * node00;
    localNode * node10;
    localNode * node01;
    localNode * node11;

    double globalXpos = (wPos.position[0]-global_offset.position[0]);
    double globalYpos = (wPos.position[1]-global_offset.position[1]);
    uint globalCornerX = (uint)(globalXpos/global_res);
    uint globalCornerY = (uint)(globalYpos/global_res);
    double globalDistX = globalXpos - (double)(globalCornerX);
    double globalDistY = globalYpos - (double)(globalCornerY);
    globalNode * gNode00 = getGlobalNode(globalCornerX, globalCornerY);
    globalNode * gNode10 = gNode00->nb4List[2];
    globalNode * gNode01 = gNode00->nb4List[3];
    globalNode * gNode11 = gNode10->nb4List[3];
    wPos.position[2] = interpolate(globalDistX,globalDistY,
                                   gNode00->elevation, gNode10->elevation,
                                   gNode01->elevation, gNode11->elevation);

    if (lNode->world_pose.position[0] < wPos.position[0])
    {
        if (lNode->world_pose.position[1] < wPos.position[1])
        {
            node00 = lNode;
            node10 = lNode->nb4List[2];
            node01 = lNode->nb4List[3];
            node11 = lNode->nb4List[2]->nb4List[3];
            a = (wPos.position[0] - lNode->world_pose.position[0])/local_res;
            b = (wPos.position[1] - lNode->world_pose.position[1])/local_res;
        }
        else
        {
            node00 = lNode->nb4List[0];
            node10 = lNode->nb4List[2];
            node01 = lNode;
            node11 = lNode->nb4List[0]->nb4List[2];
            a = (wPos.position[0] - lNode->world_pose.position[0])/local_res;
            b = 1+(wPos.position[1] - lNode->world_pose.position[1])/local_res;
        }
    }
    else
    {
        if (lNode->world_pose.position[1] < wPos.position[1])
        {
            node00 = lNode->nb4List[1];
            node10 = lNode;
            node01 = lNode->nb4List[3];
            node11 = lNode->nb4List[3]->nb4List[1];
            a = 1+(wPos.position[0] - lNode->world_pose.position[0])/local_res;
            b = (wPos.position[1] - lNode->world_pose.position[1])/local_res;
        }
        else
        {
            node00 = lNode->nb4List[1]->nb4List[0];
            node10 = lNode->nb4List[0];
            node01 = lNode->nb4List[1];
            node11 = lNode;
            a = 1+(wPos.position[0] - lNode->world_pose.position[0])/local_res;
            b = 1+(wPos.position[1] - lNode->world_pose.position[1])/local_res;
        }
    }

    gradientNode( node00, gx00, gy00);
    gradientNode( node10, gx10, gy10);
    gradientNode( node01, gx01, gy01);
    gradientNode( node11, gx11, gy11);

    double dCostX = interpolate(a,b,gx00,gx01,gx10,gx11);
    double dCostY = interpolate(a,b,gy00,gy01,gy10,gy11);

    if ((std::isnan(dCostX))||(std::isnan(dCostY)))
    {
        LOG_WARN_S << "local waypoint (" << wPos.position[0] << "," << wPos.position[1] << ") is degenerate (nan gradient)";
        return false;
    }

    if (sqrt(pow(dCostX,2) + pow(dCostY,2)) < 0.001)
    {
        LOG_WARN_S << "local waypoint (" << wPos.position[0] << "," << wPos.position[1] << ") is degenerate (near 0 gradient)";
        return false;
    }

    wPos.position[0] = wPos.position[0] - tau*dCostX;
    wPos.position[1] = wPos.position[1] - tau*dCostY;
    wPos.heading = atan2(dCostY,dCostX);

    return true;
}


void DyMuPathPlanner::gradientNode(localNode* nodeTarget, double& dnx, double& dny)
{
    double dx, dy;

      if (((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))||
          ((nodeTarget->nb4List[1] != NULL)&&(nodeTarget->nb4List[2] != NULL)&&
           (nodeTarget->nb4List[1]->deviation == INF)&&(nodeTarget->nb4List[2]->deviation == INF)))
          dx = 0;
      else
      {
          if ((nodeTarget->nb4List[1] == NULL)||(nodeTarget->nb4List[1]->deviation == INF))
              dx = nodeTarget->nb4List[2]->deviation - nodeTarget->deviation;
          else
          {
              if ((nodeTarget->nb4List[2] == NULL)||(nodeTarget->nb4List[2]->deviation == INF))
                  dx = nodeTarget->deviation - nodeTarget->nb4List[1]->deviation;
              else
                  dx = (nodeTarget->nb4List[2]->deviation -
                        nodeTarget->nb4List[1]->deviation)*0.5;
          }
      }
      if (((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))||
          ((nodeTarget->nb4List[0] != NULL)&&(nodeTarget->nb4List[3] != NULL)&&
           (nodeTarget->nb4List[0]->deviation == INF)&&(nodeTarget->nb4List[3]->deviation == INF)))
          dy = 0;
      else
      {
          if ((nodeTarget->nb4List[0] == NULL)||(nodeTarget->nb4List[0]->deviation == INF))
              dy = nodeTarget->nb4List[3]->deviation - nodeTarget->deviation;
          else
          {
              if ((nodeTarget->nb4List[3] == NULL)||(nodeTarget->nb4List[3]->deviation == INF))
                  dy = nodeTarget->deviation - nodeTarget->nb4List[0]->deviation;
              else
                  dy = (nodeTarget->nb4List[3]->deviation -
                        nodeTarget->nb4List[0]->deviation)*0.5;
          }
      }
      dnx = dx/sqrt(pow(dx,2)+pow(dy,2));
      dny = dy/sqrt(pow(dx,2)+pow(dy,2));
}


/*************EVALUATE IF NEW PATH PASSES THROUGH UNDESIRED AREAS**************/
//
bool DyMuPathPlanner::evaluatePath(std::vector<base::Waypoint>& trajectory, bool keepOldWaypoints)
{

    LOG_DEBUG_S << "Path is evaluated again";

    uint minIndex = 0, maxIndex = 0, rectifiedIndex = 0;
    bool isBlocked = false;
    localNode* closest_local_node;
    globalNode* closest_goal_node;
    std::vector<base::Waypoint> final_path;
    final_path.clear();
    uint index_waypoint = 0;
    reconnecting_index = 0;
    /*TODO:
     - Convert this into a while loop, in which an iterator is going through the path
     - After each reparation, the previous portion must be maintained
     -
    */
    // Current path is the trajectory starting from minIndex
    // Final path collects the portion of path before minIndex each time a
    //  repairing is computed
    while( index_waypoint<current_path.size() )
    {
        closest_goal_node = getNearestGlobalNode(current_path[index_waypoint]);
        if (closest_goal_node->hasLocalMap)
        {
            closest_local_node = getLocalNode(current_path[index_waypoint]);
            if(closest_local_node->risk > 0.0)
            {
                if(!isBlocked)
                {
                    isBlocked = true;
                    minIndex = index_waypoint;
                }
            }
            else if (isBlocked) //We enter a safe area after having been blocked
            {
                std::cout << std::endl << " LOOP: still in subdivided nodes" << std::endl;
                std::cout << " LOOP: index waypoint is " << index_waypoint << std::endl;
                std::cout << " LOOP: minIndex is " << minIndex << std::endl;
                std::cout << " LOOP: prior path size is " << current_path.size() << std::endl;
                rectifiedIndex = minIndex;
                while (rectifiedIndex > 0)
                {
                    if (sqrt(pow(current_path[minIndex].position[0]-
                      current_path[rectifiedIndex].position[0],2) +
                      pow(current_path[minIndex].position[1]-current_path[rectifiedIndex].position[1],2)) > 2.0)
                        break;
                    rectifiedIndex--;
                }
                std::cout << " LOOP: rectifiedIndex is " << rectifiedIndex << std::endl;
                final_path.insert(final_path.end(), current_path.begin(),current_path.begin()+rectifiedIndex);
                index_waypoint = repairPath(current_path[rectifiedIndex], index_waypoint, keepOldWaypoints);
                std::cout << " LOOP: resulting index waypoint is " << index_waypoint << std::endl;
                std::cout << " LOOP: current path size is " << current_path.size() << std::endl;
                isBlocked = false;
                minIndex = 0;
            }
        }
        else if (isBlocked)
        {
            std::cout << std::endl << " LOOP: in non subdivided nodes" << std::endl;
            std::cout << " LOOP: index waypoint is " << index_waypoint << std::endl;
            std::cout << " LOOP: minIndex is " << minIndex << std::endl;
            std::cout << " LOOP: path size is " << current_path.size() << std::endl;
            rectifiedIndex = minIndex;
            while (rectifiedIndex > 0)
            {
                if (sqrt(pow(current_path[minIndex].position[0]-
                  current_path[rectifiedIndex].position[0],2) +
                  pow(current_path[minIndex].position[1]-current_path[rectifiedIndex].position[1],2)) > 2.0)
                      break;
                rectifiedIndex--;
            }
            final_path.insert(final_path.end(), current_path.begin(),current_path.begin()+rectifiedIndex);
            index_waypoint = repairPath(current_path[rectifiedIndex], index_waypoint, keepOldWaypoints);
            std::cout << " LOOP: resulting index waypoint is " << index_waypoint << std::endl;
            std::cout << " LOOP: current path size is " << current_path.size() << std::endl;
            isBlocked = false;
            minIndex = 0;
        }
        if (index_waypoint == -1)
            return false;
        else
            index_waypoint++;
    }
    std::cout << " EVALUATION: min Index is " << minIndex << std::endl;
    std::cout << " EVALUATION: current path size is " << current_path.size() << std::endl;
    if (isBlocked)
    {
        std::cout << "Goal is occluded by local obstacles" << std::endl;
        final_path.insert(final_path.end(), current_path.begin(),current_path.begin()+minIndex);
    }
    else
    {
        std::cout << std::endl << "FINAL" << std::endl;
        std::cout << " FINAL: index waypoint is " << index_waypoint << std::endl;
        std::cout << " FINAL: minIndex is " << minIndex << std::endl;
        std::cout << " FINAL: path size is " << current_path.size() << std::endl;
        final_path.insert(final_path.end(), current_path.begin()+minIndex,current_path.end());
    }
    trajectory.clear();
    trajectory = final_path;
    return true;
}

std::vector< std::vector<double> > DyMuPathPlanner::getRiskMatrix(base::Waypoint rover_pos)
{
    // Here we cover a square portion of the global layer, centered at the
    // global node closest to the robot
    // global_side_num is the number of global nodes at a side of the square
    uint half_num= 10;
    uint global_side_num = 2*half_num + 1;
    // local_side_num is the number of local nodes at the side
    uint local_side_num = global_side_num*res_ratio;

    globalNode* gNode = getNearestGlobalNode(rover_pos);
    globalNode* node_target;
    int coord_x;
    int coord_y;

    std::vector< std::vector<double> > risk_matrix(local_side_num);
    for (uint i = 0; i < local_side_num; i++)
        risk_matrix[i].resize(local_side_num);

    /*std::cout << "LocalSideNum = " << local_side_num << std::endl;
    std::cout << "Resolution Ratio = " << res_ratio << std::endl;
    std::cout << "Global Node = " << gNode->pose.position[0] << "," << gNode->pose.position[1] << std::endl;*/

    for (uint j = 0; j < global_side_num; j++)
        for (uint i = 0; i < global_side_num; i++)
        {
            coord_x = gNode->pose.position[0] - half_num + i;
            coord_y = gNode->pose.position[1] - half_num + j;
            node_target = getGlobalNode(coord_x,coord_y);
            //std::cout << "Target Node Coordinates = " << coord_x << "," << coord_y << std::endl;
            //std::cout << "Global Node Position = " << node_target->pose.position[0] << "," << node_target->pose.position[1] << std::endl;
            if ((node_target!=NULL)&&(node_target->hasLocalMap))
            {
                //std::cout << "It is expandable" << std::endl;
                for (uint l = 0; l<res_ratio; l++)
                    for(uint k = 0; k<res_ratio; k++)
                    {
                        risk_matrix[l+j*res_ratio][k+i*res_ratio] =
                                                    node_target->localMap[l][k]->risk;
                        //std::cout << "Risk Matrix coordinates " << (k+i*res_ratio) << "," << (l+j*res_ratio) << std::endl;
                    }
            }
        }
    //std::cout << "Done" << std::endl;
    return risk_matrix;
}

std::vector< std::vector<double> > DyMuPathPlanner::getDeviationMatrix(base::Waypoint rover_pos)
{
    // Here we cover a square portion of the global layer, centered at the
    // global node closest to the robot
    // global_side_num is the number of global nodes at a side of the square
    uint half_num= 10;
    uint global_side_num = 2*half_num + 1;
    // local_side_num is the number of local nodes at the side
    uint local_side_num = global_side_num*res_ratio;

    globalNode* gNode = getNearestGlobalNode(rover_pos);
    globalNode* node_target;
    int coord_x;
    int coord_y;

    std::vector< std::vector<double> > dev_matrix(local_side_num);
    for (uint i = 0; i < local_side_num; i++)
        dev_matrix[i].resize(local_side_num);

    /*std::cout << "LocalSideNum = " << local_side_num << std::endl;
    std::cout << "Resolution Ratio = " << res_ratio << std::endl;
    std::cout << "Global Node = " << gNode->pose.position[0] << "," << gNode->pose.position[1] << std::endl;*/

    for (uint j = 0; j < global_side_num; j++)
        for (uint i = 0; i < global_side_num; i++)
        {
            coord_x = gNode->pose.position[0] - half_num + i;
            coord_y = gNode->pose.position[1] - half_num + j;
            node_target = getGlobalNode(coord_x,coord_y);
            //std::cout << "Target Node Coordinates = " << coord_x << "," << coord_y << std::endl;
            //std::cout << "Global Node Position = " << node_target->pose.position[0] << "," << node_target->pose.position[1] << std::endl;
            if ((node_target!=NULL)&&(node_target->hasLocalMap))
            {
                //std::cout << "It is expandable" << std::endl;
                for (uint l = 0; l<res_ratio; l++)
                    for(uint k = 0; k<res_ratio; k++)
                    {
                        if (node_target->localMap[l][k]->deviation == INF)
                            dev_matrix[l+j*res_ratio][k+i*res_ratio] = -1;
                        else
                            dev_matrix[l+j*res_ratio][k+i*res_ratio] =
                                                    node_target->localMap[l][k]->deviation;
                        //std::cout << "Risk Matrix coordinates " << (k+i*res_ratio) << "," << (l+j*res_ratio) << std::endl;
                    }
            }
        }
    //std::cout << "Done" << std::endl;
    return dev_matrix;
}
