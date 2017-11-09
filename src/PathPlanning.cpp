#include "PathPlanning.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning(std::vector< terrainType* > _table):terrainTable(_table)
{
    this->local_narrowBand.clear();
    global_goalNode = NULL;
}

PathPlanning::~PathPlanning()
{
}


void PathPlanning::initGlobalMap(double globalCellSize,  double localCellSize,
                                 base::Pose2D offset,
                                 std::vector< std::vector<double> > elevation,
                                 std::vector< std::vector<double> > cost)
{
    t1 = base::Time::now();
    global_cellSize = globalCellSize;
    local_cellSize = localCellSize;
    ratio_scale = (uint)(global_cellSize/local_cellSize);

    std::cout << "PLANNER: Creating Global Map using scale " <<
                 global_cellSize << " m" << std::endl;
    std::cout << "PLANNER: Each global edge is composed by " <<
                 ratio_scale << " local nodes, having a local resolution of " << local_cellSize << " m" << std::endl;
    global_offset = offset;
    std::vector<globalNode*> nodeRow;
    uint i,j;
    for (j = 0; j < cost.size(); j++)
    {
        for (i = 0; i < cost[0].size(); i++)
        {
            nodeRow.push_back(new globalNode(i, j, elevation[j][i],
                                           cost[j][i]));
            nodeRow.back()->world_pose.position[0] = i*global_cellSize;
            nodeRow.back()->world_pose.position[1] = j*global_cellSize;
        }
        globalMap.push_back(nodeRow);
        nodeRow.clear();
    }
    std::cout << "PLANNER: Global Map of "<< globalMap[0].size() << " x "
              << globalMap.size() << " nodes created in "
              << (base::Time::now()-t1) << " s" << std::endl;

  // NEIGHBOURHOOD
    printf("PLANNER: Building Global Map Neighbourhood\n");
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
        }
    }
    std::cout << "PLANNER: 4 - Neighbourhood made in " << (base::Time::now()-t1)
              << " s" << std::endl;

  // SLOPE AND ASPECT
    printf("PLANNER: Calculating Slope and Aspect values\n");
    t1 = base::Time::now();
    for (j = 0; j < globalMap.size(); j++)
        for (i = 0; i < globalMap[0].size(); i++)
            calculateSlope(globalMap[j][i]);
    std::cout << "PLANNER: Slope and Aspect calculated in " << (base::Time::now()-t1)
              << " s" << std::endl;
}

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
        dx = nodeTarget->nb4List[2]->elevation - nodeTarget->elevation;
    else
    {
        if (nodeTarget->nb4List[2] == NULL)
            dx = nodeTarget->elevation - nodeTarget->nb4List[1]->elevation;
        else
            dx = (nodeTarget->nb4List[2]->elevation -
                  nodeTarget->nb4List[1]->elevation)*0.5;
    }
    if (nodeTarget->nb4List[0] == NULL)
        dy = nodeTarget->nb4List[3]->elevation - nodeTarget->elevation;
    else
    {
        if (nodeTarget->nb4List[3] == NULL)
            dy = nodeTarget->elevation - nodeTarget->nb4List[0]->elevation;
        else
            dy = (nodeTarget->nb4List[3]->elevation -
                  nodeTarget->nb4List[0]->elevation)*0.5;
    }
    nodeTarget->slope = sqrt(pow(dx,2)+pow(dy,2));
    nodeTarget->aspect.x() = -dx/nodeTarget->slope;
    nodeTarget->aspect.y() = -dy/nodeTarget->slope;
}


void PathPlanning::setGoal(base::Waypoint wGoal)
{
    wGoal.position[0] = wGoal.position[0]/global_cellSize;
    wGoal.position[1] = wGoal.position[1]/global_cellSize;
    uint scaledX = (uint)(wGoal.position[0] + 0.5);
    uint scaledY = (uint)(wGoal.position[1] + 0.5);
    global_goalNode = getGlobalNode(scaledX, scaledY);
    global_goalNode->pose.orientation = wGoal.heading;
    std::cout << "PLANNING: Goal is global node (" << global_goalNode->pose.position[0]
              << "," << global_goalNode->pose.position[1] << ")" << std::endl;
}


void PathPlanning::calculateGlobalPropagation()
{
    global_narrowBand.clear();
    global_narrowBand.push_back(global_goalNode);
    global_goalNode->total_cost = 0;
    global_goalNode->nodeLocMode = terrainTable[global_goalNode->terrain]->optimalLM;
    globalNode * nodeTarget;
    t1 = base::Time::now();
    std::cout<< "PLANNER: starting global propagation loop " << std::endl;
    while (!global_narrowBand.empty())
    {
        nodeTarget = minCostGlobalNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN))
                    propagateGlobalNode(nodeTarget->nb4List[i]);
    }
    std::cout<< "PLANNER: ended global propagation loop" << std::endl;
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
}

void PathPlanning::propagateGlobalNode(globalNode* nodeTarget)
{
    double Tx,Ty,T,C, aspect_x, aspect_y;
    std::string L;

  // Neighbor Propagators Tx and Ty
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
    {
        if (nodeTarget->nb4List[3]->total_cost < nodeTarget->nb4List[0]->total_cost)
        {
            Ty = nodeTarget->nb4List[3]->total_cost;
            aspect_y = - nodeTarget->aspect.y();
        }
        else
        {
            Ty = nodeTarget->nb4List[0]->total_cost;
            aspect_y = nodeTarget->aspect.y();
        }
    }
    else if (nodeTarget->nb4List[0] == NULL)
    {
        Ty = nodeTarget->nb4List[3]->total_cost;
        aspect_y = - nodeTarget->aspect.y();
    }
    else
    {
        Ty = nodeTarget->nb4List[0]->total_cost;
        aspect_y = nodeTarget->aspect.y();
    }


    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        if (nodeTarget->nb4List[2]->total_cost < nodeTarget->nb4List[1]->total_cost)
        {
             Tx = nodeTarget->nb4List[2]->total_cost;
             aspect_x = - nodeTarget->aspect.x();
        }
        else
        {
            Tx = nodeTarget->nb4List[1]->total_cost;
            aspect_x = nodeTarget->aspect.x();
        }
    else if (nodeTarget->nb4List[1] == NULL)
    {
        Tx = nodeTarget->nb4List[2]->total_cost;
        aspect_x = - nodeTarget->aspect.x();
    }
    else
    {
        Tx = nodeTarget->nb4List[1]->total_cost;
        aspect_x = nodeTarget->aspect.x();
    }


  //Cost Function to obtain optimal power and locomotion mode
    double C0, Cs, Cv, beta, deltaW, ref, error, Cmax;

    C0 = global_cellSize*(terrainTable[nodeTarget->terrain]->cost);
    Cs = C0/cos(nodeTarget->slope);

    Cv = 2*Cs; //TODO: this has to be an input

    C = Cv;

    deltaW = std::abs(Tx-Ty);


  // Equation






        if (nodeTarget->slope == 0)
        { // Scalar Case -> Eikonal Equation
            if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
                T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
            else
                T = fmin(Tx,Ty) + C;
        }
        else
        { // Vectorial Case -> Hamilton Jacobi Equation
            if (Ty > Tx)
            {
                Cmax = Cs + Cv*(1-aspect_x)/2;
                beta = atan2(aspect_y, aspect_x);
            }
            else
            {
                Cmax = Cs + Cv*(1-aspect_y)/2;
                beta = atan2(aspect_x, aspect_y);
            }
            if (Cmax <= deltaW)
            {
                T = std::min(Tx,Ty) + Cmax;
            }
            else
            {
                if (beta >= 0)
                {
                    ref = deltaW/Cmax;
                    C = Cs + Cv*(1-cos(beta-ref*M_PI/4))/2;
                    error = C*(cos(ref*M_PI/4)-sin(ref*M_PI/4))/Cmax - deltaW/Cmax;
                    while(std::abs(error) > 0.0001)
                    {
                        ref += error/2;
                        C = Cs + Cv*(1-cos(beta-ref*M_PI/4))/2;
                        error = C*(cos(ref*M_PI/4)-sin(ref*M_PI/4))/Cmax - deltaW/Cmax;
                    }
                }
                else
                {
                    ref = deltaW/Cmax;
                    C = Cs + Cv*(1-cos(-beta+ref*M_PI/4))/2;
                    error = C*(cos(ref*M_PI/4)-sin(ref*M_PI/4))/Cmax - deltaW/Cmax;
                    while(std::abs(error) > 0.0001)
                    {
                        ref += error/2;
                        C = Cs + Cv*(1-cos(-beta+ref*M_PI/4))/2;
                        error = C*(cos(ref*M_PI/4)-sin(ref*M_PI/4))/Cmax - deltaW/Cmax;
                    }
                }
                T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
           }
    }

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
            global_narrowBand.push_back(nodeTarget);
        nodeTarget->total_cost = T;
        nodeTarget->nodeLocMode = terrainTable[nodeTarget->terrain]->optimalLM;
    }
}

/*void PathPlanning::propagateGlobalNode(globalNode* nodeTarget)
{
    double Tx,Ty,T,C;
    std::string L;
  // Neighbor Propagators Tx and Ty
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Ty = fmin(nodeTarget->nb4List[3]->total_cost, nodeTarget->nb4List[0]->total_cost);
    else if (nodeTarget->nb4List[0] == NULL)
        Ty = nodeTarget->nb4List[3]->total_cost;
    else
        Ty = nodeTarget->nb4List[0]->total_cost;

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = fmin(nodeTarget->nb4List[1]->total_cost, nodeTarget->nb4List[2]->total_cost);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->total_cost;
    else
        Tx = nodeTarget->nb4List[1]->total_cost;

  //Cost Function to obtain optimal power and locomotion mode
    C = global_cellSize*(terrainTable[nodeTarget->terrain]->cost);

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
            global_narrowBand.push_back(nodeTarget);
        nodeTarget->total_cost = T;
        nodeTarget->nodeLocMode = terrainTable[nodeTarget->terrain]->optimalLM;
    }
}*/

globalNode* PathPlanning::minCostGlobalNode()
{
    globalNode* nodePointer = global_narrowBand.front();
    uint index = 0;
    uint i;
    double minCost = global_narrowBand.front()->total_cost;
    //std::cout << "Size of Narrow Band is: " << this->narrowBand.size() << std::endl;
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

envire::ElevationGrid* PathPlanning::getEnvireGlobalPropagation()
{
    std::cout<< "PLANNER: building elevation grid for representing total global cost" << std::endl;
    envire::ElevationGrid* elevGrid = new envire::ElevationGrid(
    globalMap[0].size(), globalMap.size(),
    global_cellSize, global_cellSize);
    for (uint j = 0; j < globalMap.size(); j++)
    {
        for (uint i = 0; i < globalMap[0].size(); i++)
        {
            if((globalMap[j][i]->total_cost < 20) && (globalMap[j][i]->total_cost > 0)) //Threshold here!!
                elevGrid->get((double)(i)*global_cellSize,(double)(j)*global_cellSize) = globalMap[j][i]->total_cost;// (nodeMatrix[j][i]->work-minWork) / (nodeActualPos->work-minWork);
            else
                elevGrid->get((double)(i)*global_cellSize,(double)(j)*global_cellSize) = 0;
        }
    }
    std::cout<< "PLANNER: elevation grid built" << std::endl;
    return elevGrid;
}


void PathPlanning::createLocalMap(globalNode* gNode)
{
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
    if(!gNode->hasLocalMap)
    {
      gNode->hasLocalMap = true;
      createLocalMap(gNode);
    }
}

localNode* PathPlanning::introducePixelInMap(base::Vector2d pos, local_state state, bool& newVisible)
{
  // Introduce information on the local node
    localNode* lNode = getLocalNode(pos);
    //std::cout<< "PLANNER: localNode is node " << lNode->pose.position[0] << ", " << lNode->pose.position[1] << " of global node"<< std::endl;
    if (lNode->state == HID)
        newVisible = true;
    lNode->state = state; //TODO: affect neighbours as well!!
    if (state == OBSTACLE)
        if (!lNode->isObstacle)
            {
                lNode->isObstacle = true;
                localExpandableObstacles.push_back(lNode);
                lNode->risk = 1.0;
            }
    return lNode;
    //std::cout<< "PLANNER: local node with state " << lNode->state << std::endl;
}

localNode* PathPlanning::getLocalNode(base::Vector2d pos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(pos);
    //std::cout<< "PLANNER: NearestNode is " << nearestNode->pose.position[0] << ", " << nearestNode->pose.position[1] << std::endl;

    double cornerX = nearestNode->pose.position[0] - global_cellSize/2;
    double cornerY = nearestNode->pose.position[1] - global_cellSize/2;
    double a = fmod(pos.x()/global_cellSize, cornerX);
    double b = fmod(pos.y()/global_cellSize, cornerY);
    //std::cout<< "PLANNER: a = " << a << ", b = " << b << std::endl;
    return nearestNode->localMap[(uint)(b*ratio_scale)][(uint)(a*ratio_scale)];
}

localNode* PathPlanning::getLocalNode(base::Waypoint wPos)
{

  // Locate to which global node belongs that point
    globalNode* nearestNode = getNearestGlobalNode(wPos);
    //std::cout<< "PLANNER: NearestNode is " << nearestNode->pose.position[0] << ", " << nearestNode->pose.position[1] << std::endl;

    double cornerX = nearestNode->pose.position[0] - global_cellSize/2;
    double cornerY = nearestNode->pose.position[1] - global_cellSize/2;
    double a = fmod(wPos.position[0]/global_cellSize, cornerX);
    double b = fmod(wPos.position[1]/global_cellSize, cornerY);
    //std::cout<< "PLANNER: a = " << a << ", b = " << b << std::endl;
    return nearestNode->localMap[(uint)(b*ratio_scale)][(uint)(a*ratio_scale)];
}

globalNode* PathPlanning::getNearestGlobalNode(base::Vector2d pos)
{
    return getGlobalNode((uint)(pos.x()/global_cellSize + 0.5), (uint)(pos.y()/global_cellSize + 0.5));
}

globalNode* PathPlanning::getNearestGlobalNode(base::Waypoint wPos)
{
    return getGlobalNode((uint)(wPos.position[0]/global_cellSize + 0.5), (uint)(wPos.position[1]/global_cellSize + 0.5));
}

void PathPlanning::updateLocalMap(base::Waypoint wPos)
{
    globalNode* nearestNode = getNearestGlobalNode(wPos);
    if (actualGlobalNodePos != nearestNode)
    {
        actualGlobalNodePos = nearestNode;
        std::cout << "PLANNER: Building new local maps" << std::endl;
        uint a = (uint)(fmax(0,((wPos.position[1] - 6.0)/global_cellSize)));
        uint b = (uint)(fmin(globalMap.size(),((wPos.position[1] + 6.0)/global_cellSize)));
        uint c = (uint)(fmax(0,((wPos.position[0] - 6.0)/global_cellSize)));
        uint d = (uint)(fmin(globalMap[0].size(),((wPos.position[0]  + 6.0)/global_cellSize)));
        for (uint j = a; j < b; j++)
            for (uint i = c; i < d; i++)
                expandGlobalNode(globalMap[j][i]);
    }
}

bool PathPlanning::simUpdateVisibility(base::Waypoint wPos, std::vector< std::vector<double> >& costMatrix, double res, bool initializing, double camHeading)
{
    t1 = base::Time::now();
    uint a = (uint)(fmax(0,((wPos.position[1] - 4.0)/res)));
    uint b = (uint)(fmin(costMatrix.size(),((wPos.position[1] + 4.0)/res)));
    uint c = (uint)(fmax(0,((wPos.position[0] - 4.0)/res)));
    uint d = (uint)(fmin(costMatrix[0].size(),((wPos.position[0] + 4.0)/res)));
    bool newVisibleArea;
    std::vector<localNode*> localNodesToUpdate;
    localNode* toUpdate;
    localNode* nodeTarget;
    double alpha;

    localExpandableObstacles.clear();

    for (uint j = a; j < b; j++)
    {
        for (uint i = c; i < d; i++)
        {
            double dx = i*res - wPos.position[0];
            double dy = j*res - wPos.position[1];
            if (initializing)
                alpha = acos((dx*cos(wPos.heading) + dy*sin(wPos.heading))/sqrt(pow(dx,2) + pow(dy,2)));
            else
                alpha = acos((dx*cos(camHeading) + dy*sin(camHeading))/sqrt(pow(dx,2) + pow(dy,2)));
            //if (initializing)
            if(true)
            {
                if (sqrt(pow(dx,2) + pow(dy,2)) < 3.0)
                {
                    base::Vector2d pos(i*res,j*res);
                    if (costMatrix[j][i] == 0)
                        toUpdate = introducePixelInMap(pos, OBSTACLE, newVisibleArea); //TODO: make this function return a local node
                    else
                        toUpdate = introducePixelInMap(pos, TRAVERSABLE_OPEN, newVisibleArea);
                    localNodesToUpdate.push_back(toUpdate);
                }
            }
            else
            {
                if ((sqrt(pow(dx,2) + pow(dy,2)) > 0.5)&&(sqrt(pow(dx,2) + pow(dy,2)) < 3.0)&&
                    (fabs(alpha) < 3.1416*25.0/180.0))
                {
                    base::Vector2d pos(i*res,j*res);
                    if (costMatrix[j][i] == 0)
                        toUpdate = introducePixelInMap(pos, OBSTACLE, newVisibleArea);
                    else
                        toUpdate = introducePixelInMap(pos, TRAVERSABLE_OPEN, newVisibleArea);
                    localNodesToUpdate.push_back(toUpdate);
                }
            }
        }
    }
    expandRisk();
    if(newVisibleArea == true) //CONTINUE HERE!!!
    {
        std::cout << "PLANNER: new visible nodes" << std::endl;
        if (!horizonNodes.empty())
        {
            std::cout << "PLANNER: checking previous horizon nodes, number = "<< horizonNodes.size() << std::endl;
            bool isHorizon = false;
            for (uint i = horizonNodes.size()-1; i > 0; i--)
            {
                for (uint k = 0; k < 4; k++)
                {
                    if ((horizonNodes[i]->nb4List[k] != NULL) && ((horizonNodes[i]->nb4List[k]->state == HID)))
                        isHorizon = true;
                }
                if(!isHorizon)
                {
                    horizonNodes[i]->total_cost = INF;
                    horizonNodes[i]->state = TRAVERSABLE_OPEN;
                    horizonNodes.erase(horizonNodes.begin() + i);
                }
                isHorizon = false;
            }
            for (uint k = 0; k < 4; k++)
            {
                if ((horizonNodes[0]->nb4List[k] != NULL) && ((horizonNodes[0]->nb4List[k]->state == HID)))
                    isHorizon = true;
            }
            if(!isHorizon)
            {
                horizonNodes[0]->total_cost = INF;
                horizonNodes[0]->state = TRAVERSABLE_OPEN;
                horizonNodes.erase(horizonNodes.begin());
                //isHorizon = false;
            }
            isHorizon = false;
        }

        std::cout << "PLANNER: adding new horizon nodes" << std::endl;
        while(!localNodesToUpdate.empty())
        {
            nodeTarget = localNodesToUpdate.back();
            localNodesToUpdate.pop_back();
            if(nodeTarget->state == TRAVERSABLE_OPEN)
                for (uint k = 0; k < 4; k++)
                {
                    if((nodeTarget->nb4List[k] != NULL) &&
                       (nodeTarget->nb4List[k]->state ==
                        HID))
                    {
                        horizonNodes.push_back(nodeTarget);
                        setHorizonCost(nodeTarget);
                        break;
                    }
                }
        }
        std::cout << "PLANNER: Visibility is updated in " << (base::Time::now()-t1) << " s" << std::endl;
        //std::cout<< 100*visibleNodes.size()/(nodeMatrix.size()*nodeMatrix[0].size()) << "% of map is visible " << std::endl;
    }
    return newVisibleArea;
}


void PathPlanning::setHorizonCost(localNode* horizonNode)
{
    uint i = (uint)(horizonNode->global_pose.position[0]);
    uint j = (uint)(horizonNode->global_pose.position[1]);
    double a = horizonNode->global_pose.position[0] - (double)(i);
    double b = horizonNode->global_pose.position[1] - (double)(j);

    globalNode * node00 = globalMap[j][i];
    globalNode * node10 = node00->nb4List[2];
    globalNode * node01 = node00->nb4List[3];
    globalNode * node11 = node00->nb4List[2]->nb4List[3];

    double w00 = node00->total_cost;
    double w10 = node10->total_cost;
    double w01 = node01->total_cost;
    double w11 = node11->total_cost;

    horizonNode->total_cost = w00 + (w01 - w00)*a + (w10 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
}


void PathPlanning::expandRisk()
{
    localNode * nodeTarget;
    while(!localExpandableObstacles.empty())
    {
        nodeTarget = maxRiskNode();
        //std::cout << "PLANNER: number of expandable nodes is " << localExpandableObstacles.size() <<" and current risk is " << nodeTarget->risk << std::endl;
        //std::cout << "PLANNER: expanding node " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
        for (uint i = 0; i<4; i++)
            if (nodeTarget->nb4List[i] != NULL)
                propagateRisk(nodeTarget->nb4List[i]);
    }
}

localNode* PathPlanning::maxRiskNode()
{
    if (localExpandableObstacles.empty())
        return NULL;
    localNode* nodePointer = localExpandableObstacles.front();
    uint index = 0;
    double maxRisk = localExpandableObstacles.front()->risk;
    //std::cout << "Size of Narrow Band is: " << this->narrowBand.size() << std::endl;
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
    /*std::cout << "PLANNER: next expandable node is  (" <<
        nodePointer->pose.position[0] << "," <<
        nodePointer->pose.position[1] << ")" << std::endl;*/
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
    double C = 0.1;
    double S;

    if (fabs(Sx-Sy)<C)
        S = (Sx+Sy+sqrt(2*pow(C,2.0) - pow((Sx-Sy),2.0)))/2;
    else
        S = fmin(Sx,Sy) + C;

    double R = 1 - S;
    if ((R>0.1)&&(R>nodeTarget->risk))
    {
        nodeTarget->risk = R;
        localExpandableObstacles.push_back(nodeTarget);
    }
}

envire::TraversabilityGrid* PathPlanning::getEnvireLocalState(base::Waypoint wPos)
{
    uint a = (uint)(fmax(0,wPos.position[1] - 4.0+0.5));
    uint b = (uint)(fmin(globalMap.size(),wPos.position[1] + 4.0+0.5));
    uint c = (uint)(fmax(0,wPos.position[0] - 4.0+0.5));
    uint d = (uint)(fmin(globalMap[0].size(),wPos.position[0] + 4.0+0.5));

    envire::TraversabilityGrid* travGrid = new envire::TraversabilityGrid(
        ratio_scale*(1+d-c), ratio_scale*(1+b-a),
        local_cellSize, local_cellSize, (double)(c)*global_cellSize, (double)(a)*global_cellSize);

    std::cout << "PLANNER: travGrid created" << std::endl;
    for (uint j = 0; j <= b-a; j++)
    {
        for (uint i = 0; i <= d-c; i++)
        {
            for (uint l = 0; l < ratio_scale; l++)
            {
                for (uint k = 0; k < ratio_scale; k++)
                {
                    travGrid->setProbability(1.0, (k + ratio_scale*i), (l + ratio_scale*j));
                    if (globalMap[j+a][i+c]->localMap[l][k]->state == HID)
                        travGrid->setTraversability(0,(k + ratio_scale*i), (l + ratio_scale*j));
                    else if (globalMap[j+a][i+c]->localMap[l][k]->state == OBSTACLE)
                        travGrid->setTraversability(1,(k + ratio_scale*i), (l + ratio_scale*j));
                    else if (globalMap[j+a][i+c]->localMap[l][k]->risk != 0)
                        travGrid->setTraversability(2,(k + ratio_scale*i), (l + ratio_scale*j));
                    else
                        travGrid->setTraversability(3,(k + ratio_scale*i), (l + ratio_scale*j));
                }
            }
        }
    }
    travGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.2)); // Hidden Area
    travGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0)); // Obstacle Area
    travGrid->setTraversabilityClass(2, envire::TraversabilityClass(0.5)); // Hazardous Area
    travGrid->setTraversabilityClass(3, envire::TraversabilityClass(1.0)); // Traversable Area

    std::cout << "PLANNER: State Map is updated" << std::endl;
    return travGrid;
}

envire::ElevationGrid* PathPlanning::getEnvireRisk(base::Waypoint wPos)
{
    uint a = (uint)(fmax(0,wPos.position[1] - 4.0+0.5));
    uint b = (uint)(fmin(globalMap.size(),wPos.position[1] + 4.0+0.5));
    uint c = (uint)(fmax(0,wPos.position[0] - 4.0+0.5));
    uint d = (uint)(fmin(globalMap[0].size(),wPos.position[0] + 4.0+0.5));
    envire::ElevationGrid* riskGrid = new envire::ElevationGrid(
        ratio_scale*(1+d-c), ratio_scale*(1+b-a),
        local_cellSize, local_cellSize);
    for (uint j = 0; j <= b-a; j++)
        for (uint i = 0; i <= d-c; i++)
            for (uint l = 0; l < ratio_scale; l++)
                for (uint k = 0; k < ratio_scale; k++)
                    riskGrid->get(((double)(k + ratio_scale*i))*local_cellSize,
                                  ((double)(l + ratio_scale*j))*local_cellSize) =
                                  globalMap[j+a][i+c]->localMap[l][k]->risk;
    return riskGrid;
}

envire::ElevationGrid* PathPlanning::getLocalTotalCost(base::Waypoint wPos)
{
    uint a = (uint)(fmax(0,wPos.position[1] - 4.0+0.5));
    uint b = (uint)(fmin(globalMap.size(),wPos.position[1] + 4.0+0.5));
    uint c = (uint)(fmax(0,wPos.position[0] - 4.0+0.5));
    uint d = (uint)(fmin(globalMap[0].size(),wPos.position[0] + 4.0+0.5));
    envire::ElevationGrid* totalCostGrid = new envire::ElevationGrid(
        ratio_scale*(1+d-c), ratio_scale*(1+b-a),
        local_cellSize, local_cellSize);
    double totalCost;
    for (uint j = 0; j <= b-a; j++)
        for (uint i = 0; i <= d-c; i++)
            for (uint l = 0; l < ratio_scale; l++)
                for (uint k = 0; k < ratio_scale; k++)
                {
                    totalCost = globalMap[j+a][i+c]->localMap[l][k]->total_cost;
                    if (totalCost == INF)
                        totalCostGrid->get(((double)(k + ratio_scale*i))*local_cellSize,
                                  ((double)(l + ratio_scale*j))*local_cellSize) = 0;
                    else
                        totalCostGrid->get(((double)(k + ratio_scale*i))*local_cellSize,
                              ((double)(l + ratio_scale*j))*local_cellSize) = totalCost;
                }

    return totalCostGrid;
}


void PathPlanning::calculateLocalPropagation(base::Waypoint wGoal, base::Waypoint wPos)
{
    localNode * nodeTarget;
  // Initializing the Narrow Band
    std::cout << "PLANNER: initializing Narrow Band" << std::endl;

    local_narrowBand.clear();
    local_narrowBand = horizonNodes;
    if(global_goalNode->hasLocalMap)
    {
        std::cout << "PLANNER: global_goalNode size of local map = " << global_goalNode->localMap.size() << " x " << global_goalNode->localMap[0].size() << std::endl;
        local_goalNode = global_goalNode->localMap[ratio_scale/2][ratio_scale/2];
        if (local_goalNode->state != HID)
        {
            local_goalNode->total_cost = 0;
            local_narrowBand.push_back(local_goalNode);
        }
    }

    if(!local_closedNodes.empty())
    {
    std::cout << "PLANNER: resetting previous closed nodes" << std::endl;
        for (uint i = 0; i < local_closedNodes.size(); i++)
        {
            local_closedNodes[i]->state = TRAVERSABLE_OPEN;
            local_closedNodes[i]->total_cost = INF;
        }
        local_closedNodes.clear();
    }

  // Propagation Loop
    t1 = base::Time::now();
    std::cout << "PLANNER: starting local propagation loop" << std::endl;

    local_actualPose = getLocalNode(wPos);

    localNode * nodeN0 = local_actualPose->nb4List[2];
    std::cout<< "PLANNER: nodeN0 is " << nodeN0->pose.position[0] << "," << nodeN0->pose.position[1] << " with state = " << nodeN0->state << std::endl;
    localNode * nodeN1 = nodeN0->nb4List[3];
    std::cout<< "PLANNER: nodeN1 is " << nodeN1->pose.position[0] << "," << nodeN1->pose.position[1] << " with state = " << nodeN1->state << std::endl;
    localNode * nodeN2 = local_actualPose->nb4List[3];
    std::cout<< "PLANNER: nodeN2 is " << nodeN2->pose.position[0] << "," << nodeN2->pose.position[1] << " with state = " << nodeN2->state << std::endl;
    localNode * nodeN3 = nodeN2->nb4List[1];
    std::cout<< "PLANNER: nodeN3 is " << nodeN3->pose.position[0] << "," << nodeN3->pose.position[1] << " with state = " << nodeN3->state << std::endl;
    localNode * nodeN4 = local_actualPose->nb4List[1];
    std::cout<< "PLANNER: nodeN4 is " << nodeN4->pose.position[0] << "," << nodeN4->pose.position[1] << " with state = " << nodeN4->state << std::endl;
    localNode * nodeN5 = nodeN4->nb4List[0];
    std::cout<< "PLANNER: nodeN5 is " << nodeN5->pose.position[0] << "," << nodeN5->pose.position[1] << " with state = " << nodeN5->state << std::endl;
    localNode * nodeN6 = local_actualPose->nb4List[0];
    std::cout<< "PLANNER: nodeN6 is " << nodeN6->pose.position[0] << "," << nodeN6->pose.position[1] << " with state = " << nodeN6->state << std::endl;
    localNode * nodeN7 = nodeN6->nb4List[2];
    std::cout<< "PLANNER: nodeN7 is " << nodeN7->pose.position[0] << "," << nodeN7->pose.position[1] << " with state = " << nodeN7->state << std::endl;

    std::cout << "PLANNER: local neighbours defined" << std::endl;

    while ((local_actualPose->state != TRAVERSABLE_CLOSED)||
           (nodeN0->state != TRAVERSABLE_CLOSED)||
           (nodeN1->state != TRAVERSABLE_CLOSED)||
           (nodeN2->state != TRAVERSABLE_CLOSED)||
           (nodeN3->state != TRAVERSABLE_CLOSED)||
           (nodeN4->state != TRAVERSABLE_CLOSED)||
           (nodeN5->state != TRAVERSABLE_CLOSED)||
           (nodeN6->state != TRAVERSABLE_CLOSED)||
           (nodeN7->state != TRAVERSABLE_CLOSED))
    {
            nodeTarget = minCostLocalNode();
            nodeTarget->state = TRAVERSABLE_CLOSED;
            /*if (nodeTarget == local_actualPose)
                std::cout << "PLANNER: now considering actualPose" << std::endl;*/
            //std::cout<< "PLANNER: next nodeTarget is " << nodeTarget->pose.position[0] << "," << nodeTarget->pose.position[1] << std::endl;
            for (uint i = 0; i<4; i++)
            {
                if (nodeTarget->nb4List[i] == NULL)
                    std::cout<< "PLANNER: nodeTarget " << nodeTarget->pose.position[0] << "," << nodeTarget->pose.position[1] << "has a null neighbour " << i << std::endl;
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (nodeTarget->nb4List[i]->state == TRAVERSABLE_OPEN))
                {
                    propagateLocalNode(nodeTarget->nb4List[i]);
                    local_closedNodes.push_back(nodeTarget->nb4List[i]);//To later erase total cost value of non close nodes as well!!!
                }
            }
    }
    std::cout<< "PLANNER: ended local propagation loop" << std::endl;
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
}

void PathPlanning::propagateLocalNode(localNode* nodeTarget)
{
    double Tx,Ty,P,T,R,C;

  // Neighbor Propagators Tx and Ty
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Ty = fmin(nodeTarget->nb4List[3]->total_cost, nodeTarget->nb4List[0]->total_cost);
    else if (nodeTarget->nb4List[0] == NULL)
        Ty = nodeTarget->nb4List[3]->total_cost;
    else
        Ty = nodeTarget->nb4List[0]->total_cost;

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = fmin(nodeTarget->nb4List[1]->total_cost, nodeTarget->nb4List[2]->total_cost);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->total_cost;
    else
        Tx = nodeTarget->nb4List[1]->total_cost;

  //Cost Function to obtain optimal power and locomotion mode
    R = nodeTarget->risk;
    if (nodeTarget->state == OBSTACLE)
        C = local_cellSize*(terrainTable[0]->cost)*(1+9*R);
    else
        C = local_cellSize*(terrainTable[1]->cost)*(1+9*R)/2;  //TODO: Change this

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
            local_narrowBand.push_back(nodeTarget);
        nodeTarget->total_cost = T;
    }
}

localNode* PathPlanning::minCostLocalNode()
{
    localNode* nodePointer = local_narrowBand.front();
    uint index = 0;
    uint i;
    double minCost = local_narrowBand.front()->total_cost;
    //std::cout << "PLANNER: Size of Narrow Band is: " << local_narrowBand.size() << std::endl;
    for (i =0; i < local_narrowBand.size(); i++)
    {
        if (local_narrowBand[i]->total_cost < minCost)
        {
            minCost = local_narrowBand[i]->total_cost;
            nodePointer = local_narrowBand[i];
            index = i;
        }
    }
    local_narrowBand.erase(local_narrowBand.begin() + index);
    return nodePointer;
}


bool PathPlanning::getPath(base::Waypoint wPos, double tau,
                           std::vector<base::Waypoint>& trajectory)
{
    base::Waypoint sinkPoint;
    bool newWaypoint;
    sinkPoint.position[0] = global_goalNode->pose.position[0];
    sinkPoint.position[1] = global_goalNode->pose.position[1];
    sinkPoint.heading = global_goalNode->pose.orientation;

    trajectory.clear();
        //trajectory.insert(trajectory.begin(), startPoint);
    trajectory.push_back(wPos);
    std::cout << "PLANNER: trajectory initialized" << std::endl;

    while(sqrt(pow((trajectory.back().position[0] - sinkPoint.position[0]),2) +
             pow((trajectory.back().position[1] - sinkPoint.position[1]),2)) > (2*tau*local_cellSize))
    {
        newWaypoint = calculateNextWaypoint(wPos, tau*local_cellSize);
        if (newWaypoint)
            trajectory.push_back(wPos);
        else
            return false;
        if (trajectory.size() > 99)
        {
            std::cout << "PLANNER: ERROR in trajectory" << std::endl;
            return false;
        }
    }
    trajectory.push_back(sinkPoint);
    std::cout<< "PLANNER: Adding final waypoint with heading" << sinkPoint.heading << " "<< trajectory.back().heading<<  std::endl;
    return true;
}


bool PathPlanning::calculateNextWaypoint(base::Waypoint& wPos, double tau)
{
    double a,b;

    double gx00, gx10, gx01, gx11;
    double gy00, gy10, gy01, gy11;

    localNode * lNode = getLocalNode(wPos);
    localNode * node00;
    localNode * node10;
    localNode * node01;
    localNode * node11;

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

    if ((isHorizon(node00))||(isHorizon(node01))||
       (isHorizon(node10))||(isHorizon(node11)))
        return false;

    gradientNode( node00, gx00, gy00);
    gradientNode( node10, gx10, gy10);
    gradientNode( node01, gx01, gy01);
    gradientNode( node11, gx11, gy11);

    double dCostX = interpolate(a,b,gx00,gx01,gx10,gx11);
    double dCostY = interpolate(a,b,gy00,gy01,gy10,gy11);

    wPos.position[0] = wPos.position[0] - tau*dCostX/sqrt(pow(dCostX,2) + pow(dCostY,2));
    wPos.position[1] = wPos.position[1] - tau*dCostY/sqrt(pow(dCostX,2) + pow(dCostY,2));
    wPos.heading = atan2(-dCostY,-dCostX);

    if ((std::isnan(wPos.position[0]))||(std::isnan(wPos.position[1])))
        return false;
    return true;
}


void PathPlanning::gradientNode(localNode* nodeTarget, double& dnx, double& dny)
{
    double dx, dy;

      if (((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))||
          ((nodeTarget->nb4List[1]->total_cost == INF)&&(nodeTarget->nb4List[2]->total_cost == INF)))
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
          ((nodeTarget->nb4List[0]->total_cost == INF)&&(nodeTarget->nb4List[3]->total_cost == INF)))
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
      dnx = dx/sqrt(pow(dx,2)+pow(dy,2));
      dny = dy/sqrt(pow(dx,2)+pow(dy,2));
}

double PathPlanning::interpolate(double a, double b, double g00, double g01, double g10, double g11)
{
    return g00 + (g10 - g00)*a + (g01 - g00)*b + (g11 + g00 - g10 - g01)*a*b;
}


std::string PathPlanning::getLocomotionMode(base::Waypoint wPos)
{
    globalNode * gNode = getNearestGlobalNode(wPos);
    return gNode->nodeLocMode;
}

bool PathPlanning::isHorizon(localNode* lNode)
{
    for (uint k = 0; k < 4; k++)
        if ((lNode->nb4List[k] != NULL) && ((lNode->nb4List[k]->state == HID)))
            return true;
    return false;
}
