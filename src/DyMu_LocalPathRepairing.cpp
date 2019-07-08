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

void PathPlanning::createLocalMap(globalNode* gNode) //TODO: Create nb8List
{
    // LOG_DEBUG_S << "creating Local Map of " << gNode->pose.position[0] << "," << gNode->pose.position[1];
    gNode->hasLocalMap = true;
    std::vector<localNode*> nodeRow;
    for (uint j = 0; j < ratio_scale; j++)
    {
        for (uint i = 0; i < ratio_scale; i++)
        {
            nodeRow.push_back(new localNode(i, j, gNode->pose));
            nodeRow.back()->global_pose.position[0] =
                nodeRow.back()->parent_pose.position[0] - 0.5 +
                (0.5/(double)ratio_scale) +
                nodeRow.back()->pose.position[0]*(1/(double)ratio_scale);
            nodeRow.back()->global_pose.position[1] =
                nodeRow.back()->parent_pose.position[1] - 0.5 +
                (0.5/(double)ratio_scale) +
                nodeRow.back()->pose.position[1]*(1/(double)ratio_scale);
            nodeRow.back()->world_pose.position[0] = nodeRow.back()->global_pose.position[0]/global_cellSize;
            nodeRow.back()->world_pose.position[1] = nodeRow.back()->global_pose.position[1]/global_cellSize;
        }
        gNode->localMap.push_back(nodeRow);
        nodeRow.clear();
    }
  // NEIGHBOURHOOD
    for (uint j = 0; j < ratio_scale; j++)
    {
        for (uint i = 0; i < ratio_scale; i++)
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
                        gNode->localMap[j][i]->nb4List.push_back(gNode->nb4List[0]->localMap[ratio_scale-1][i]);
                        gNode->nb4List[0]->localMap[ratio_scale-1][i]->nb4List[3] = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(gNode->localMap[j-1][i]);

            if (i==0)
            {
                if (gNode->nb4List[1]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[1]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(gNode->nb4List[1]->localMap[j][ratio_scale-1]);
                        gNode->nb4List[1]->localMap[j][ratio_scale-1]->nb4List[2] = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(gNode->localMap[j][i-1]);

            if (i==ratio_scale-1)
            {
                if (gNode->nb4List[2]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[2]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(gNode->nb4List[2]->localMap[j][0]);
                        gNode->nb4List[2]->localMap[j][0]->nb4List[1] = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(gNode->localMap[j][i+1]);

            if (j==ratio_scale-1)
            {
                if (gNode->nb4List[3]==NULL)
                    gNode->localMap[j][i]->nb4List.push_back(NULL);
                else
                {
                    if(gNode->nb4List[3]->hasLocalMap)
                    {
                        gNode->localMap[j][i]->nb4List.push_back(gNode->nb4List[3]->localMap[0][i]);
                        gNode->nb4List[3]->localMap[0][i]->nb4List[0] = gNode->localMap[j][i];
                    }
                    else
                        gNode->localMap[j][i]->nb4List.push_back(NULL);
                }
            }
            else
                gNode->localMap[j][i]->nb4List.push_back(gNode->localMap[j+1][i]);
        }
    }
}


void PathPlanning::expandGlobalNode(globalNode* gNode)
{
    if(!gNode->hasLocalMap) createLocalMap(gNode);
    for (int i = 0; i<8; i++)
        if(!(gNode->nb8List[i]==NULL))
            if(!(gNode->nb8List[i]->hasLocalMap)) 
                createLocalMap(gNode->nb8List[i]);
}

localNode* PathPlanning::getLocalNode(base::Pose2D pos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(pos);
    // LOG_DEBUG_S << "NearestNode is " << nearestNode->pose.position[0] << ", " << nearestNode->pose.position[1];

    expandGlobalNode(nearestNode);

    double cornerX = nearestNode->pose.position[0] - global_cellSize/2;
    double cornerY = nearestNode->pose.position[1] - global_cellSize/2;
    double a = pos.position[0] - cornerX;//fmod(pos.position[0]/global_cellSize, cornerX);
    double b = pos.position[1] - cornerY;//fmod(pos.position[1]/global_cellSize, cornerY);
    /* LOG_DEBUG_S << "cornerX = " << cornerX << ", cornerY = " << cornerY;
    LOG_DEBUG_S << "a = " << a << ", b = " << b;*/
    return nearestNode->localMap[(uint)(b*ratio_scale)][(uint)(a*ratio_scale)];
}


localNode* PathPlanning::getLocalNode(base::Waypoint wPos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(wPos);
    // LOG_DEBUG_S << "NearestNode is " << nearestNode->pose.position[0] << ", " << nearestNode->pose.position[1];

    expandGlobalNode(nearestNode);

    double cornerX = nearestNode->pose.position[0] - global_cellSize/2;
    double cornerY = nearestNode->pose.position[1] - global_cellSize/2;
    double a = wPos.position[0] - cornerX;//fmod(wPos.position[0]/global_cellSize, cornerX);
    double b = wPos.position[1] - cornerY;//fmod(wPos.position[1]/global_cellSize, cornerY);
    // LOG_DEBUG_S << "a = " << a << ", b = " << b;
    //expandGlobalNode(nearestNode);
    return nearestNode->localMap[(uint)(b*ratio_scale)][(uint)(a*ratio_scale)];
}

bool PathPlanning::computeLocalPlanning(base::Waypoint wPos,
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
    uint a = (uint)(fmax(0,((wPos.position[1] - (double)height/2*res)/global_cellSize)));
    uint b = (uint)(fmin(globalMap.size(),((wPos.position[1] + (double)height/2*res)/global_cellSize)));
    uint c = (uint)(fmax(0,((wPos.position[0] - (double)width/2*res)/global_cellSize)));
    uint d = (uint)(fmin(globalMap[0].size(),((wPos.position[0]  + (double)width/2*res)/global_cellSize)));
    for (uint j = a; j < b; j++)
        for (uint i = c; i < d; i++)
            expandGlobalNode(globalMap[j][i]);

  //Indexes of the minimum and maximum waypoints affected in the trajectory by obstacles
    uint minIndex = globalPath.size(), maxIndex = 0;
    bool isBlocked = false;
    bool pathBlocked = false;

    double offsetX = wPos.position[0] - res*(double)width/2;
    double offsetY = wPos.position[1] + res*(double)height/2; // Image convention (Y pointing down)

    double globalSizeX = global_cellSize*globalMap[0].size()-0.5;
    double globalSizeY = global_cellSize*globalMap.size()-0.5;
    
    /*LOG_DEBUG_S << "received map ";
    LOG_DEBUG_S << "height = " << height <<" and width = "<< width;
    LOG_DEBUG_S << "offsetX = " << offsetX <<" and offsetY = "<< offsetY;
    LOG_DEBUG_S << "globalSizeX = " << globalSizeX <<" and globalSizeY = "<< globalSizeY;*/
    
    base::Pose2D pos;
    //t1 = base::Time::now();
    for (uint j = 0; j < height; j++)
    {
        for (uint i = 0; i < width; i++)
        {
            pos.position[0] = offsetX + i*res;
            pos.position[1] = offsetY - j*res; // Image convention (Y pointing down)
            if((pos.position[0] > -0.5)&&(pos.position[0] < globalSizeX)&&(pos.position[1] > -0.5)&&(pos.position[1] < globalSizeY))
            {
                uint8_t value = traversabilityMap.image[j*traversabilityMap.getRowSize()+i*traversabilityMap.getPixelSize()]; //TODO: check if this is correct!!
                //LOG_DEBUG_S << "pos = "  << pos.position[0] << "," << pos.position[1];
                lNode = getLocalNode(pos);
                //LOG_DEBUG_S << "local node pos = "  << lNode->global_pose.position[0] << "," << lNode->global_pose.position[1];
                gNode = getNearestGlobalNode(lNode->parent_pose);
                //LOG_DEBUG_S << "global node pos = "  << gNode->pose.position[0] << "," << gNode->pose.position[1];
                if ((!lNode->isObstacle)&&((value != 0)||(gNode->isObstacle))) //If pixel is obstacle (value == 1)
                {
                lNode->isObstacle = true;
                localExpandableObstacles.push_back(lNode);
                lNode->risk = 1.0;
                isBlocked = isBlockingObstacle(lNode, maxIndex, minIndex, trajectory);//See here if its blocking (and which waypoint)
                pathBlocked = (pathBlocked)?true:isBlocked;// Path is blocked if isBlocked is true at least once
                //pathBlocked |= isBlocked;
                }
            }
        }
    }
    //t1 = base::Time::now() - t1;

    if(pathBlocked)
    {
        LOG_DEBUG_S << std::endl << "-- LOCAL REPAIRING STARTED--";
        base::Time tInit = base::Time::now();
        std::cout <<  "Time now in library: " << tInit << std::endl;
        expandRisk();
        repairPath(trajectory, wPos, globalPath, maxIndex, keepOldWaypoints);
        localTime = base::Time::now() - tInit;
        std::cout << "Local Time in library: " << (base::Time::now()-tInit)  << std::endl;
        return true;
    }
    return false;
}



bool PathPlanning::computeLocalPlanning(base::Waypoint wPos,
                                       std::vector< std::vector<double> >& costMatrix,
                                       double res,
                                       std::vector<base::Waypoint>& trajectory,
                                       bool keepOldWaypoints, base::Time& localTime)
{
    localNode* lNode;
    globalNode* gNode;

    
    uint a = (uint)(fmax(0,((wPos.position[1] - 4.0)/res)));
    uint b = (uint)(fmin(costMatrix.size(),((wPos.position[1] + 4.0)/res)));
    uint c = (uint)(fmax(0,((wPos.position[0] - 4.0)/res)));
    uint d = (uint)(fmin(costMatrix[0].size(),((wPos.position[0] + 4.0)/res)));


  //Indexes of the minimum and maximum waypoints affected in the trajectory by obstacles
    uint minIndex = globalPath.size(), maxIndex = 0;
    bool isBlocked = false;

    base::Pose2D pos;

    for (uint j = a; j < b; j++)
    {
        for (uint i = c; i < d; i++)
        {
            double dx = i*res - wPos.position[0];
            double dy = j*res - wPos.position[1];
            //if (initializing)
            if (sqrt(pow(dx,2) + pow(dy,2)) < 3.0)
            {
                pos.position[0] = i*res;
                pos.position[1] = j*res;
                lNode = getLocalNode(pos);
                gNode = getNearestGlobalNode(lNode->parent_pose);
                if ((!lNode->isObstacle)&&((costMatrix[j][i] == 0)||(gNode->isObstacle)))
                {
                    lNode->isObstacle = true;
                    localExpandableObstacles.push_back(lNode);
                    lNode->risk = 1.0;
                    isBlocked = isBlockingObstacle(lNode, maxIndex, minIndex, trajectory);
                }
            }
        }
    }
    //In case an obstacle is blocking, expand the Risk and start repairing

    if(isBlocked)
    {
        t1 = base::Time::now();
        expandRisk();
        repairPath(trajectory, wPos, globalPath, maxIndex, keepOldWaypoints);
        localTime = base::Time::now() - t1;
        return true;
    }

    return false;
}

/* REPAIR PATH function
   This function is in charge of computing a series of Local Waypoints that are connected to Global Waypoints placed further than the obstacle area of effect (risky area)
*/
void PathPlanning::repairPath(std::vector<base::Waypoint>& trajectory, base::Waypoint wInit, std::vector<base::Waypoint>& globalPath, uint index, bool keepOldWaypoints)
{   
    // Previous trajectory is erased, information about previous Global Waypoints is stored in globalPath 
    trajectory.clear();
	
	// It may happen a path with no Global Waypoints is intended to be repaired (since trajectory was cleared in previous repairing)
	if (globalPath.empty())
	{
		LOG_DEBUG_S << "repairing operation is called, but there are not Global Waypoints in the trajectory. Repairing aborted.";
		return;
	}
	
    //oldIndex is pointing to the Global Waypoint placed after risk area, which is initially pointed by index as well
	//index increases to point to another Global Waypoint placed further than reconnect_distance to the one pointed by oldIndex
    double oldIndex = index;
    while((index < globalPath.size())&&
	      (sqrt(pow(globalPath[index].position[0]-globalPath[oldIndex].position[0],2)
              + pow(globalPath[index].position[1]-globalPath[oldIndex].position[1],2)) < reconnect_distance))
        index++;
			
			
    if(index >= globalPath.size()) //This means last waypoint is on forbidden area (either on risky area or under reconnect_distance from a previous Global Waypoint)
    {
        trajectory.push_back(wInit);
        LOG_DEBUG_S << "initial number of Global Waypoints was " << globalPath.size();
        LOG_DEBUG_S << "Global Waypoint to reconnect does not exist, variable index is " << index;
        globalPath.clear();
        LOG_DEBUG_S << "trajectory is cleared due to goal placed on forbidden area";
    }
    else
    {
        localNode * lSet = computeLocalPropagation(wInit, globalPath[index], keepOldWaypoints);
        if (lSet == NULL) //Local Planning is aborted because of having entered obstacle area
        {
            LOG_DEBUG_S << "repairing aborted";
            LOG_DEBUG_S << "index = " << index;
            LOG_DEBUG_S << "global stops at (" << globalPath[index].position[0] << "," << globalPath[index].position[1] << ")";
            trajectory.push_back(wInit);
            globalPath.clear();
        }
        else
        {
            std::vector<base::Waypoint> localPath = getLocalPath(lSet,wInit,0.4);
            LOG_DEBUG_S << "local path starts at (" << localPath[0].position[0] << "," << localPath[0].position[1] << ")";
            if (keepOldWaypoints)
            {        
                // Remove old trajectory, including one extra global waypoint, which is coincident with last local waypoint
                globalPath.erase(globalPath.begin(), globalPath.begin() + index + 1);
                globalPath.insert(globalPath.begin(), localPath.back());
                localPath.pop_back();
                trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                trajectory.insert(trajectory.end(),globalPath.begin(),globalPath.end());
            }
            else
            {
                base::Waypoint newWaypoint;
                newWaypoint.position[0] = lSet->global_pose.position[0];
                newWaypoint.position[1] = lSet->global_pose.position[1];
                globalPath.clear();
                globalPath = getGlobalPath(newWaypoint);
                // Remove first global waypoint, which is coincident with last local waypoint
                globalPath.erase(globalPath.begin());
                trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
                trajectory.insert(trajectory.end(),globalPath.begin(),globalPath.end());
            }
        }
    }
}



void PathPlanning::repairPath(std::vector<base::Waypoint>& trajectory, uint minIndex, uint maxIndex)
{
    LOG_DEBUG_S << "trajectory from waypoint " << minIndex << " to waypoint " << maxIndex << " must be repaired";
    LOG_DEBUG_S << "size of globalPath is " << globalPath.size();
    uint indexLim = 0;

    for(uint i = minIndex; i>0;i--)
    {
        if (sqrt(
              pow(globalPath[i].position[0]-globalPath[minIndex].position[0],2)
            + pow(globalPath[i].position[1]-globalPath[minIndex].position[1],2)
          ) > 2*risk_distance)
        {
            indexLim = i;
            break;
        }
    }

    if(maxIndex >= globalPath.size()-1) //This means last waypoint is on forbidden area
    {
        globalPath.resize(indexLim+1);
        for(uint i = 0; i<trajectory.size(); i++)
        {
            if ((trajectory[i].position[0] == globalPath.back().position[0])&&
                (trajectory[i].position[1] == globalPath.back().position[1])&&
                (trajectory[i].heading == globalPath.back().heading))
            {
                trajectory.resize(i+1);
                break;
            }
        }
        LOG_DEBUG_S << "trajectory is shortened due to goal placed on forbidden area";
    }
    else
    {
        double Treach = getTotalCost(globalPath[maxIndex]);
        double distReference = sqrt(
              pow(globalPath[indexLim].position[0]-globalPath[maxIndex].position[0],2)
            + pow(globalPath[indexLim].position[1]-globalPath[maxIndex].position[1],2));
        //Resize trajectory to eliminate non safe part of the trajectory
        globalPath.resize(indexLim+1);
        //Resize as well the globalPath pointers
        for(uint i = 0; i<trajectory.size(); i++)
        {
            if ((trajectory[i].position[0] == globalPath.back().position[0])&&
                (trajectory[i].position[1] == globalPath.back().position[1])&&
                (trajectory[i].heading == globalPath.back().heading))
            {
                trajectory.resize(i+1);
                break;
            }
        }
        LOG_DEBUG_S << "global Path is repaired from " << indexLim;
        LOG_DEBUG_S << "global Path size is " << globalPath.size();
        LOG_DEBUG_S << "trajectory is repaired from " << trajectory.size();
      //Trajectory is repaired from indexLim
        
        localNode * lSet = computeLocalPropagation(trajectory.back(), globalPath[maxIndex], true); //TODO: Change the true by a boolean input variable
        std::vector<base::Waypoint> localPath = getLocalPath(lSet,trajectory[indexLim],0.4);
        base::Waypoint newWaypoint;
        newWaypoint.position[0] = lSet->global_pose.position[0];
        newWaypoint.position[1] = lSet->global_pose.position[1];
        std::vector<base::Waypoint> restPath = getGlobalPath(newWaypoint);
        trajectory.insert(trajectory.end(),localPath.begin(),localPath.end());
        trajectory.insert(trajectory.end(),restPath.begin(),restPath.end());
    }
}

bool PathPlanning::isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex, std::vector<base::Waypoint> trajectory)
{
    bool isBlocked = false;
    for (uint i = 0; i < globalPath.size(); i++)
    {
        /*LOG_DEBUG_S << "distance = " << sqrt(
            pow(obNode->global_pose.position[0]-globalPath[i].position[0],2) +
            pow(obNode->global_pose.position[1]-globalPath[i].position[1],2));
        LOG_DEBUG_S << "global pose = " << obNode->global_pose.position[0] << "," << obNode->global_pose.position[1]
                  << " and global path position is " << globalPath[i].position[0] << "," << globalPath[i].position[1];*/
        if(sqrt(
            pow(obNode->world_pose.position[0]-globalPath[i].position[0],2) +
            pow(obNode->world_pose.position[1]-globalPath[i].position[1],2))
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
        maxIndex = globalPath.size();

    // At this point, it seems is no blocking obstacle, but it could block a local waypoint yet
    for (uint k = 0; k < trajectory.size(); k++)
        if(sqrt(
            pow(obNode->world_pose.position[0]-trajectory[k].position[0],2) +
            pow(obNode->world_pose.position[1]-trajectory[k].position[1],2))
            < risk_distance)
            return true;

    return isBlocked;
}


double PathPlanning::getTotalCost(localNode* lNode)
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

double PathPlanning::getTotalCost(base::Waypoint wInt)
{
    uint i = (uint)(wInt.position[0]);
    uint j = (uint)(wInt.position[1]);
    double a = wInt.position[0] - (double)(i);
    double b = wInt.position[1] - (double)(j);

    globalNode * node00 = globalMap[j][i];
    globalNode * node10 = node00->nb4List[2];
    globalNode * node01 = node00->nb4List[3];
    globalNode * node11 = node00->nb4List[2]->nb4List[3];

    double w00 = (node00==NULL)?INF:node00->total_cost;
    double w10 = (node10==NULL)?INF:node10->total_cost;
    double w01 = (node01==NULL)?INF:node01->total_cost;
    double w11 = (node11==NULL)?INF:node11->total_cost;

    /* LOG_DEBUG_S << "debugging";
    LOG_DEBUG_S << " - w00 = " << w00;
    LOG_DEBUG_S << " - w00 = " << w01;
    LOG_DEBUG_S << " - w00 = " << w00;
    LOG_DEBUG_S << " - w00 = " << w00;*/

    return w00 + (w10 - w00)*a + (w01 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
}

void PathPlanning::expandRisk()
{
    LOG_DEBUG_S << "starting risk expansion";
    t1 = base::Time::now();
    localNode * nodeTarget;
    globalNode* gNode;
    while(!localExpandableObstacles.empty())
    {
        nodeTarget = maxRiskNode();
        // LOG_DEBUG_S << "number of expandable nodes is " << localExpandableObstacles.size() <<" and current risk is " << nodeTarget->risk;
        // LOG_DEBUG_S << "expanding node " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1];
        for (uint i = 0; i<4; i++)
        {
            if (nodeTarget->nb4List[i] != NULL)
            {
                gNode = getNearestGlobalNode(nodeTarget->nb4List[i]->parent_pose);
                if (getNearestGlobalNode(nodeTarget->parent_pose) != gNode)
                    expandGlobalNode(gNode);
            }
            if (nodeTarget->nb4List[i] != NULL)
                propagateRisk(nodeTarget->nb4List[i]);
        }
    }
    t1 = base::Time::now() - t1;
    LOG_DEBUG_S << "ended risk expansion in " << t1 << " seconds";
}

localNode* PathPlanning::maxRiskNode()
{
    if (localExpandableObstacles.empty())
        return NULL;
    localNode* nodePointer = localExpandableObstacles.front();
    uint index = 0;
    double maxRisk = localExpandableObstacles.front()->risk;
    // LOG_DEBUG_S << "Size of Narrow Band is: " << this->narrowBand.size();
    for (uint i =0; i < localExpandableObstacles.size(); i++)
    {
        if (maxRisk == 1)
            break;
        if (localExpandableObstacles[i]->risk > maxRisk)
        {
            maxRisk = localExpandableObstacles[i]->risk;
            nodePointer = localExpandableObstacles[i];
            index = i;
            break;
        }
    }
    /* LOG_DEBUG_S << "next expandable node is  (" <<
        nodePointer->pose.position[0] << "," <<
        nodePointer->pose.position[1] << ")";*/
    localExpandableObstacles.erase(localExpandableObstacles.begin() + index);
    return nodePointer;
}

void PathPlanning::propagateRisk(localNode* nodeTarget)
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
    double C = local_cellSize/risk_distance;
    double S;

    if (fabs(Sx-Sy)<C)
        S = (Sx+Sy+sqrt(2*pow(C,2.0) - pow((Sx-Sy),2.0)))/2;
    else
        S = fmin(Sx,Sy) + C;

    double R = std::max(1 - S,0.0);
    if ((R>0)&&(R>nodeTarget->risk))
    {
        nodeTarget->risk = R;
        localExpandableObstacles.push_back(nodeTarget);
    }
}


localNode * PathPlanning::computeLocalPropagation(base::Waypoint wInit, base::Waypoint wOvertake, bool keepOldWaypoints)
{
  //wInit is the waypoint from which the path is repaired

    double Tovertake = getTotalCost(wOvertake);
    double distReference = sqrt(
              pow(wInit.position[0]-wOvertake.position[0],2)
            + pow(wInit.position[1]-wOvertake.position[1],2));

    if(!local_propagatedNodes.empty())
    {
    LOG_DEBUG_S << "resetting previous closed nodes";
        for (uint i = 0; i < local_propagatedNodes.size(); i++)
        {
            local_propagatedNodes[i]->state = OPEN;
            local_propagatedNodes[i]->deviation = INF;
            local_propagatedNodes[i]->total_cost = INF;
        }
        local_propagatedNodes.clear();
    }

  // Initializing the Narrow Band
    LOG_DEBUG_S << "initializing Narrow Band";

    local_actualPose = getLocalNode(wInit);
    if(local_actualPose->isObstacle)
    {
        LOG_DEBUG_S << "actual pose is within obstacle area, local repairing aborted";
        return NULL;
    }
    local_actualPose->deviation = 0;
    local_actualPose->total_cost = getTotalCost(local_actualPose);
    local_actualPose->state = CLOSED;

    local_narrowBand.clear();
    local_narrowBand.push_back(local_actualPose);

    local_propagatedNodes.push_back(local_actualPose);

    localNode * nodeTarget;
    localNode * nodeEnd = NULL;
    if (keepOldWaypoints)
        nodeEnd = getLocalNode(wOvertake);

    if(nodeEnd->isObstacle)
    {
        LOG_DEBUG_S << "nodeEnd is obstacle";
        return NULL;
    }

    globalNode* gNode;


    double minC = (getTotalCost(wInit)-Tovertake)/distReference;
    

  // Propagation Loop
    t1 = base::Time::now();
    LOG_DEBUG_S << "starting local propagation loop";
    LOG_DEBUG_S << "initial risk is " << local_actualPose->risk;

    while(true)//TODO: Control this
    {
        if (keepOldWaypoints)
            nodeTarget = minCostLocalNode(nodeEnd);
        else
            nodeTarget = minCostLocalNode(Tovertake, minC);
        nodeTarget->state = CLOSED;
        //LOG_DEBUG_S << "nodeTarget " << nodeTarget->global_pose.position[0] << "," << nodeTarget->global_pose.position[1];
        for (uint i = 0; i<4; i++)
        {    
            if (nodeTarget->nb4List[i] != NULL)
            {
                // LOG_DEBUG_S << "-- neighbour "  << i << " is " << nodeTarget->nb4List[i]->global_pose.position[0] << "," << nodeTarget->nb4List[i]->global_pose.position[1];
                // LOG_DEBUG_S << "-- global parent of neighbour is "  << nodeTarget->nb4List[i]->parent_pose.position[0] << "," << nodeTarget->nb4List[i]->parent_pose.position[1];
                gNode = getNearestGlobalNode(nodeTarget->nb4List[i]->parent_pose);
                // LOG_DEBUG_S << "-- global parent of neighbour is "  << gNode->pose.position[0] << "," << gNode->pose.position[1];
                if (getNearestGlobalNode(nodeTarget->parent_pose) != gNode)
                    expandGlobalNode(gNode);
            }
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN)
                &&(!nodeTarget->nb4List[i]->isObstacle))
            {
                propagateLocalNode(nodeTarget->nb4List[i]);
                if (nodeEnd == NULL)
                    if ((nodeTarget->nb4List[i]->total_cost < Tovertake)&&(nodeTarget->nb4List[i]->risk == 0))
                        nodeEnd = nodeTarget->nb4List[i];
            }
        }
        if ((nodeEnd != NULL)&&(nodeEnd->state == CLOSED)&&
            (nodeEnd->nb4List[0]->state == CLOSED)&&(nodeEnd->nb4List[2]->state == CLOSED)&&
            (nodeEnd->nb4List[2]->state == CLOSED)&&(nodeEnd->nb4List[3]->state == CLOSED))
        {
            t1 = base::Time::now() - t1;
            LOG_DEBUG_S << "ended local propagation loop in " << t1 << " seconds";
            return nodeEnd;
        }
    }

}

void PathPlanning::propagateLocalNode(localNode* nodeTarget)
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

    C = local_cellSize*(risk_ratio*R + 1);

    if(C <= 0)
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
            local_narrowBand.push_back(nodeTarget);
            local_propagatedNodes.push_back(nodeTarget);
        }
        nodeTarget->deviation = T;
    }
}

localNode* PathPlanning::minCostLocalNode(double Tovertake, double minC)
{
    localNode* nodePointer = local_narrowBand.front();
    uint index = 0;
    uint i;
    double minH = local_narrowBand.front()->deviation;// + fmax(0, local_narrowBand.front()->total_cost - Tovertake)/minC;
    double currentH;
    // LOG_DEBUG_S << "Size of Narrow Band is: " << local_narrowBand.size();
    for (i =0; i < local_narrowBand.size(); i++)
    {
        currentH = local_narrowBand[i]->deviation;// + fmax(0, local_narrowBand[i]->total_cost - Tovertake)/minC;
        if (currentH < minH)
        {
            minH = currentH;
            nodePointer = local_narrowBand[i];
            index = i;
        }
    }
    local_narrowBand.erase(local_narrowBand.begin() + index);
    return nodePointer;
}


localNode* PathPlanning::minCostLocalNode(localNode* reachNode)
{
    localNode* nodePointer = local_narrowBand.front();
    uint index = 0;
    uint i;
    double distance = sqrt(
              pow(local_narrowBand.front()->world_pose.position[0]-reachNode->world_pose.position[0],2)
            + pow(local_narrowBand.front()->world_pose.position[1]-reachNode->world_pose.position[1],2));
    double minH = local_narrowBand.front()->deviation + distance;
    double currentH;
    //LOG_DEBUG_S << "Size of Narrow Band is: " << local_narrowBand.size();
    for (i =0; i < local_narrowBand.size(); i++)
    {
        distance = sqrt(
              pow(local_narrowBand[i]->world_pose.position[0]-reachNode->world_pose.position[0],2)
            + pow(local_narrowBand[i]->world_pose.position[1]-reachNode->world_pose.position[1],2));
        currentH = local_narrowBand[i]->deviation + distance;
        if (currentH < minH)
        {
            minH = currentH;
            nodePointer = local_narrowBand[i];
            index = i;
        }
    }
    local_narrowBand.erase(local_narrowBand.begin() + index);
    return nodePointer;
}


std::vector<base::Waypoint> PathPlanning::getLocalPath(localNode * lSetNode,
                                                       base::Waypoint wInit,
                                                       double tau)
{
    base::Waypoint wPos;
    bool newWaypoint;
    wPos.position[0] = lSetNode->global_pose.position[0];
    wPos.position[1] = lSetNode->global_pose.position[1];
    wPos.heading = lSetNode->global_pose.orientation;

    tau = 0.5*local_cellSize;
    std::vector<base::Waypoint> trajectory;
    newWaypoint = computeLocalWaypointGDM(wPos, tau*local_cellSize);
    trajectory.insert(trajectory.begin(),wPos);
    LOG_DEBUG_S << "repairing trajectory initialized";
    LOG_DEBUG_S << "lSetNode at " << wPos.position[0] << ", " << wPos.position[1];
    LOG_DEBUG_S << "wInit at " << wInit.position[0] << ", " << wInit.position[1];

    while(sqrt(pow((trajectory.front().position[0] - wInit.position[0]),2) +
             pow((trajectory.front().position[1] - wInit.position[1]),2)) > 1.5*local_cellSize)
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
    LOG_DEBUG_S << "Local cell size is " << local_cellSize;
    return trajectory;
}

base::Waypoint PathPlanning::computeLocalWaypointDijkstra(localNode * lNode)
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

bool PathPlanning::computeLocalWaypointGDM(base::Waypoint& wPos, double tau)
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
    uint globalCornerX = (uint)(globalXpos/global_cellSize);
    uint globalCornerY = (uint)(globalYpos/global_cellSize);
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
            a = (wPos.position[0] - lNode->world_pose.position[0])/local_cellSize;
            b = (wPos.position[1] - lNode->world_pose.position[1])/local_cellSize;
        }
        else
        {
            node00 = lNode->nb4List[0];
            node10 = lNode->nb4List[2];
            node01 = lNode;
            node11 = lNode->nb4List[0]->nb4List[2];
            a = (wPos.position[0] - lNode->world_pose.position[0])/local_cellSize;
            b = 1+(wPos.position[1] - lNode->world_pose.position[1])/local_cellSize;
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
            a = 1+(wPos.position[0] - lNode->world_pose.position[0])/local_cellSize;
            b = (wPos.position[1] - lNode->world_pose.position[1])/local_cellSize;
        }
        else
        {
            node00 = lNode->nb4List[1]->nb4List[0];
            node10 = lNode->nb4List[0];
            node01 = lNode->nb4List[1];
            node11 = lNode;
            a = 1+(wPos.position[0] - lNode->world_pose.position[0])/local_cellSize;
            b = 1+(wPos.position[1] - lNode->world_pose.position[1])/local_cellSize;
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


void PathPlanning::gradientNode(localNode* nodeTarget, double& dnx, double& dny)
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

void PathPlanning::evaluatePath(std::vector<base::Waypoint>& trajectory)
{

    LOG_DEBUG_S << "Path is evaluated again";

    uint minIndex = 0, maxIndex = 0;
    bool isBlocked = false;
    localNode* nearestNode;

        for (uint i = 0; i < globalPath.size(); i++)
        {
            nearestNode = getLocalNode(globalPath[i]);
            if(nearestNode->risk > 0.0)
            {
                if(!isBlocked)
                {
                    isBlocked = true;
                    minIndex = (i<minIndex)?i:minIndex;
                }
                else
                    maxIndex = (i>maxIndex)?i:maxIndex;
            }
            else if (isBlocked)
            {
                maxIndex = (i>maxIndex)?i:maxIndex;
                repairPath(trajectory, minIndex, maxIndex);
                isBlocked = false;
                i = minIndex;
            }
        }
        if (isBlocked)
        {
            maxIndex = globalPath.size();
            repairPath(trajectory, minIndex, maxIndex);
        }

}


// Output Local Risk Map
base::samples::DistanceImage PathPlanning::getLocalRiskMap(base::Waypoint wPos)
{
    base::samples::DistanceImage localRiskMap;
    double R;
    
    uint a = (uint)(fmax(0,wPos.position[1] - 3.0));
    uint b = (uint)(fmin(globalMap.size(),wPos.position[1] + 3.0));
    uint c = (uint)(fmax(0,wPos.position[0] - 3.0));
    uint d = (uint)(fmin(globalMap[0].size(),wPos.position[0] + 3.0));

    localRiskMap.setSize(ratio_scale*(1+d-c),ratio_scale*(1+b-a));

        for (uint j = 0; j <= b-a; j++)
            for (uint i = 0; i <= d-c; i++)
                for (uint l = 0; l < ratio_scale; l++)
                    for (uint k = 0; k < ratio_scale; k++)
                    {
                        //expandGlobalNode(globalMap[j+a][i+c]);
                        R = globalMap[j+a][i+c]->localMap[l][k]->risk;
                        if (R == 1)
                            localRiskMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j-1)*ratio_scale*(1+d-c)] = 255;
                        if ((R > 0)&&(R<1))
                            localRiskMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j-1)*ratio_scale*(1+d-c)] = 32+R*(150-16);
                        if (R==0)
                            localRiskMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j-1)*ratio_scale*(1+d-c)] = 16;
                        if (local_actualPose == globalMap[j+a][i+c]->localMap[l][k])
                            localRiskMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j-1)*ratio_scale*(1+d-c)] = 0;
                    }

    localRiskMap.scale_x = local_cellSize;
    localRiskMap.scale_y = local_cellSize;
    localRiskMap.center_x = wPos.position[0];
    localRiskMap.center_y = wPos.position[1];
    return localRiskMap;
}


/*
 - Get Local Propagation Map
  -- Returns an image with the propagation computed
     on the local map
*/ 
base::samples::DistanceImage PathPlanning::getLocalPropagationMap(base::Waypoint wPos)
{
    base::samples::DistanceImage localPropagationMap;
    double t;

    uint a = (uint)(fmax(0,wPos.position[1] - 2.0));
    uint b = (uint)(fmin(globalMap.size(),wPos.position[1] + 2.0));
    uint c = (uint)(fmax(0,wPos.position[0] - 2.0));
    uint d = (uint)(fmin(globalMap[0].size(),wPos.position[0] + 2.0));

    localPropagationMap.setSize(ratio_scale*(1+d-c),ratio_scale*(1+b-a));

        for (uint j = 0; j <= b-a; j++)
            for (uint i = 0; i <= d-c; i++)
                for (uint l = 0; l < ratio_scale; l++)
                    for (uint k = 0; k < ratio_scale; k++)
                    {
                        t = globalMap[j+a][i+c]->localMap[l][k]->deviation;
                        if (t == INF)
                            localPropagationMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j -1)*ratio_scale*(1+d-c)] = 0;
                        else
                            localPropagationMap.data[k + ratio_scale*i + (ratio_scale*(1+b-a) - l - ratio_scale*j -1)*ratio_scale*(1+d-c)] = fmin(t*40.0,255);
                    }

    localPropagationMap.scale_x = local_cellSize;
    localPropagationMap.scale_y = local_cellSize;
    localPropagationMap.center_x = wPos.position[0];
    localPropagationMap.center_y = wPos.position[1];
    return localPropagationMap;
}
