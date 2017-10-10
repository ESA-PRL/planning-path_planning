#include "NodeMap.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

//__NODEMAP_CONSTRUCTORS__
NodeMap::NodeMap()
{
}

NodeMap::NodeMap(envire::TraversabilityGrid* travGrid)
{
    printf("Creating NodeMap from envire data\n");
    t1 = base::Time::now();
    scale = travGrid->getScaleX();
    std::vector<Node*> nodeRow;
    uint klass;
    for (uint j = 0; j < travGrid->getCellSizeY(); j++)
    {
        for (uint i = 0; i < travGrid->getCellSizeX(); i++)
        {
            klass = travGrid->getGridData()[j][i];
            nodeRow.push_back(new Node(i, j, 0, klass==1?0:1,OPEN));
            switch(klass)
            {
                case 0: //Unknown node
                    nodeRow.back()->state = HIDDEN;
                    break;
                case 1: //Obstacle node
                    //nodeRow.back()->state = OBSTACLE;
                    break;
                default:
                    break;
            }
        }
        nodeMatrix.push_back(nodeRow);
        nodeRow.clear();
    }
    std::cout<<"NodeMap created in " << (base::Time::now()-t1) << " s" << std::endl;
    t1 = base::Time::now();
    makeNeighbourhood();
    std::cout<<"Neighbourhood made in " << (base::Time::now()-t1) << " s" << std::endl;
}

void NodeMap::createLocalNodeMap(envire::TraversabilityGrid* travGrid)
{
    double globalNodePosX, globalNodePosY;
    uint klass;
    for (uint j = 0; j < travGrid->getCellSizeY(); j++)
    {
        for (uint i = 0; i < travGrid->getCellSizeX(); i++)
        {
            klass = travGrid->getGridData()[j][i];
            if(klass != 0) //Node not hidden
            {
                // Check in which global cell is
                // Here Im not taking into account relative translation and rotation between maps!!
                globalNodePosX = ((double)i) * travGrid->getScaleX()/this->scale;
                globalNodePosY = ((double)j) * travGrid->getScaleY()/this->scale;
                /*std::cout<<" Hi, Im local node (" << i << "," <<
                           j << ") at global node (" << (uint)globalNodePosX << "," << (uint)globalNodePosY << ")" << std::endl;*/
                if(this->nodeMatrix[(uint)globalNodePosY][(uint)globalNodePosX]->localNodeMatrix == NULL)
                {
                    this->nodeMatrix[(uint)globalNodePosY][(uint)globalNodePosX]->localNodeMatrix = new std::vector< std::vector<Node*> >;
                    std::cout<<" Hi, Im local node (" << i << "," <<
                               j << ") at global node (" << (uint)globalNodePosX << "," << (uint)globalNodePosY << ")" << std::endl;
                }
            }

        }
    }
}

NodeMap::NodeMap(double size, base::Pose2D pos,
                std::vector< std::vector<double> > elevation,
		            std::vector< std::vector<double> > cost, nodeState state)
{
    t1 = base::Time::now();
    scale = size;
    std::cout<<"PLANNER: Creating nodemap using scale " << scale << " m" << std::endl;
    globalOffset = pos;
    std::vector<Node*> nodeRow;
    double numNodes = cost.size()*cost[0].size();
    uint i,j;
    for (j = 0; j < cost.size(); j++)
    {
        for (i = 0; i < cost[0].size(); i++)
            nodeRow.push_back(new Node(i, j, elevation[j][i], cost[j][i], state));
        nodeMatrix.push_back(nodeRow);
        nodeRow.clear();
        std::cout << "PLANNER: loaded " << ((i+1)*(j+1)/numNodes)*100 << " %" <<std::endl;
    }
    std::cout<<"NodeMap of "<< nodeMatrix[0].size() << " x " << nodeMatrix.size() << " nodes created in " << (base::Time::now()-t1) << " s" << std::endl;
}

void NodeMap::makeNeighbourhood()
{
  printf("Making Neighbourhood\n");
    t1 = base::Time::now();
  for (uint j = 0; j < nodeMatrix.size(); j++)
  {
      for (uint i = 0; i < nodeMatrix[0].size(); i++)
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

          nodeMatrix[j][i]->nb4List.clear();
          nodeMatrix[j][i]->nb4List.push_back(getNode(i,j-1));
          nodeMatrix[j][i]->nb4List.push_back(getNode(i-1,j));
          nodeMatrix[j][i]->nb4List.push_back(getNode(i+1,j));
          nodeMatrix[j][i]->nb4List.push_back(getNode(i,j+1));
      }
  }
    std::cout<<"PLANNER: 4 - Neighbourhood made in " << (base::Time::now()-t1) << " s" << std::endl;
}

void NodeMap::makeNeighbourhood(Node* n, uint i, uint j)
{
    //n->nb4List.clear();
    if (n->nb4List.empty())
    {
        n->nb4List.push_back(getNode(i,j-1));
        n->nb4List.push_back(getNode(i-1,j));
        n->nb4List.push_back(getNode(i+1,j));
        n->nb4List.push_back(getNode(i,j+1));
    }
}

Node * NodeMap::getNeighbour(Node* n, uint k)
{
    if (n->nb4List.empty())
    {
        uint i = (uint)(n->pose.position[0]);
        uint j = (uint)(n->pose.position[1]);
        n->nb4List.push_back(getNode(i,j-1));
        n->nb4List.push_back(getNode(i-1,j));
        n->nb4List.push_back(getNode(i+1,j));
        n->nb4List.push_back(getNode(i,j+1));
    }
    return n->nb4List[k];
}

void NodeMap::updateNodeMap(envire::TraversabilityGrid* travGrid)
{
    t1 = base::Time::now();
    printf("Updating NodeMap\n");
    uint klass;
    uint counter = 0;
    uint sizeY = travGrid->getCellSizeY();
    uint sizeX = travGrid->getCellSizeX();
    for (uint j = 0; j < sizeY; j++)
    {
        for (uint i = 0; i < sizeX; i++)
        {
            //klass = travGrid->getGridData()[j][i];
            /*switch(klass)
            {
                case 0: //Unknown node
                    nodeMatrix[j][i]->state = HIDDEN;
                    break;
                case 1: //Obstacle node
                    nodeMatrix[j][i]->state = OBSTACLE;
                    counter++;
                    break;
                default:
                    nodeMatrix[j][i]->state = FAR;
                    counter++;
                    break;
            }*/
        }
    }
    std::cout<<"NodeMap updated in " << (base::Time::now()-t1) << " s" << std::endl;
    std::cout<< 100*counter/(travGrid->getCellSizeY()*travGrid->getCellSizeX()) << "% of map is visible " << std::endl;
}

void NodeMap::resetPropagation()
{
    if(!closedNodes.empty())
    {
    std::cout << "PLANNER: making closed nodes open again" << std::endl;
        for (uint i = 0; i < closedNodes.size(); i++)
        {
            closedNodes[i]->state = OPEN;
            closedNodes[i]->total_cost = INF;
        }
        closedNodes.clear();
    }
    /*for (uint j = 0; j < nodeMatrix.size(); j++)
    {
        for (uint i = 0; i < nodeMatrix[0].size(); i++)
        {
            if (nodeMatrix[j][i]->state == CLOSED)
                nodeMatrix[j][i]->state = OPEN;
            nodeMatrix[j][i]->work = INF;
        }
    }*/
}

void NodeMap::setActualPos(base::Waypoint wPos)
{
    wPos.position[0] = wPos.position[0]/(this->scale);
    wPos.position[1] = wPos.position[1]/(this->scale);
    uint scaledX = (uint)(wPos.position[0] + 0.5);
    uint scaledY = (uint)(wPos.position[1] + 0.5);
    nodeActualPos = this->getNode(scaledX, scaledY);
    nodeActualPos->pose.orientation = wPos.heading;
    std::cout << "Actual position near node (" <<
        nodeActualPos->pose.position[0] << "," <<
        nodeActualPos->pose.position[1] << ")" << std::endl;
    this->actualPose = wPos;
}

void NodeMap::setGoal(base::Waypoint wGoal)
{
    wGoal.position[0] = wGoal.position[0]/(this->scale);
    wGoal.position[1] = wGoal.position[1]/(this->scale);
    uint scaledX = (uint)(wGoal.position[0] + 0.5);
    uint scaledY = (uint)(wGoal.position[1] + 0.5);
    nodeGoal = this->getNode(scaledX, scaledY);
    nodeGoal->pose.orientation = wGoal.heading;
    std::cout << "Goal near node (" << nodeGoal->pose.position[0] <<
        "," << nodeGoal->pose.position[1] << ")" << std::endl;
    this->goalPose = wGoal;
}

Node * NodeMap::getActualPos()
{
    return this->nodeActualPos;
}

Node * NodeMap::getGoal()
{
    return this->nodeGoal;
}

//__GETNODE_FUNCTION__

Node* NodeMap::getNode(uint i, uint j)
{
    if ((i >= nodeMatrix[0].size())||(j >= nodeMatrix.size()))
        return NULL;
    return nodeMatrix[j][i];
}

// Simulation function
void NodeMap::hidAll()
{
    for (uint j = 0; j < nodeMatrix.size(); j++)
        for (uint i = 0; i < nodeMatrix[0].size(); i++)
            nodeMatrix[j][i]->state = HIDDEN;
    std::cout << "All nodes are hidden" << std::endl;
}


bool NodeMap::isObstacle(Node* n)
{
    return n->isObstacle;
}

// Simulation function
bool NodeMap::updateVisibility(base::Waypoint wPos, NodeMap* globalMap, bool initializing)
{
    t1 = base::Time::now();
    double dx,dy,alpha,counter = 0;
    bool flag = false;
    std::cout << "PLANNER: updating visibility" << std::endl;
    //Simulating what ExoTeR sees, a circular area of 1.4m radius
    uint a = (uint)(fmax(0,((wPos.position[1] - 4.0)/scale)));
    uint b = (uint)(fmin(nodeMatrix.size(),((wPos.position[1] + 4.0)/scale)));
    uint c = (uint)(fmax(0,((wPos.position[0] - 4.0)/scale)));
    uint d = (uint)(fmin(nodeMatrix[0].size(),((wPos.position[0] + 4.0)/scale)));
    std::cout << "PLANNER: a = " << a << ", b = " << b << ", c = "<< c << ", d = " << d << std::endl;
    Node* nodeTarget;
    Node* nDummy;
    obstacleNodes.clear();
    for (uint j = a; j < b; j++)
    {
        for (uint i = c; i < d; i++)
        {
            nodeTarget = nodeMatrix[j][i];
            if(nodeTarget->state != HIDDEN)
            {
               /* if ((((nodeTarget->nb4List[1] != NULL) &&
                          (nodeTarget->nb4List[2] != NULL)) &&
                         ((nodeTarget->nb4List[1]->terrain == 0) &&
                          (nodeTarget->nb4List[2]->terrain == 0))) ||
                        ((nodeTarget->nb4List[0] != NULL) &&
                          (nodeTarget->nb4List[3] != NULL)) &&
                        ((nodeTarget->nb4List[0]->terrain == 0) &&
                          (nodeTarget->nb4List[3]->terrain == 0)))
                {
                  // Obstacle nodes are too close
                     nodeTarget->terrain = 0;
                     obstacleNodes.push_back(nodeTarget);
                     nodeTarget->risk.obstacle = 1;
                }*/
                /*else if (nodeTarget->state == CLOSED)
                {
                  // Just making traversable nodes OPEN again
                    nodeTarget->state = OPEN;
                }*/
            }
            else
            {
                makeNeighbourhood(nodeTarget,i,j);
                dx = i*scale + globalOffset.position[0] - wPos.position[0];
                dy = j*scale + globalOffset.position[1] - wPos.position[1];
                alpha = acos((dx*cos(wPos.heading) + dy*sin(wPos.heading))/sqrt(pow(dx,2) + pow(dy,2)));
                if (initializing)
                {
                    if ((sqrt(pow(dx,2) + pow(dy,2)) < 3.0)&&(nodeTarget->state == HIDDEN))
                    {
                        flag = true;
                        nodeTarget->state = OPEN;
                        if (nodeTarget->terrain == 0)
                        {
                            //std::cout << "PLANNER: Its an obstacle" << std::endl;
                            obstacleNodes.push_back(nodeTarget);
                            nodeTarget->risk.obstacle = 1;
                        }
                    }
                }
                else
                {
                    if ((sqrt(pow(dx,2) + pow(dy,2)) > 0.5)&&(sqrt(pow(dx,2) + pow(dy,2)) < 3.0)&&
                        (fabs(alpha) < 3.1416*25.0/180.0)&&(nodeTarget->state == HIDDEN))
                    {
                        flag = true;
                        nodeTarget->state = OPEN;
                        if (nodeTarget->terrain == 0)
                        {
                            //std::cout << "PLANNER: Its an obstacle" << std::endl;
                            obstacleNodes.push_back(nodeTarget);
                            nodeTarget->risk.obstacle = 1;
                        }
                    }
                }
            }
        }
    }

    expandRisk(obstacleNodes);
    std::cout<< "PLANNER: Risk expanded" << std::endl;
    /*for (uint j = a; j < b; j++)
    {
        for (uint i = c; i < d; i++)
            std::cout << nodeMatrix[j][i]->risk.obstacle << " ";
        std::cout << std::endl;
    }*/
    obstacleNodes.clear(); // DEBUGGING PURPOSES

    // Conservative method to avoid passing through corridors of length 1 node

    /*if ((nodeTarget->nb4List[1] != NULL)&&(nodeTarget->nb4List[2] != NULL)&&
        (nodeTarget->nb4List[1]->state == OBSTACLE)&&
         (nodeTarget->nb4List[2]->state == OBSTACLE))
        nodeTarget->state = OBSTACLE;*/

    // Check actual horizon nodes
    if(flag == true) //CONTINUE HERE!!!
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
                    if ((horizonNodes[i]->nb4List[k] != NULL) && ((horizonNodes[i]->nb4List[k]->state == HIDDEN)))
                        isHorizon = true;
                }
                if(!isHorizon)
                {
                    horizonNodes[i]->total_cost = INF;
                    horizonNodes[i]->state = OPEN;
                    horizonNodes.erase(horizonNodes.begin() + i);
                    //isHorizon = false;
                }
                isHorizon = false;
            }
            for (uint k = 0; k < 4; k++)
            {
                if ((horizonNodes[0]->nb4List[k] != NULL) && ((horizonNodes[0]->nb4List[k]->state == HIDDEN)))
                    isHorizon = true;
            }
            if(!isHorizon)
            {
                horizonNodes[0]->total_cost = INF;
                horizonNodes[0]->state = OPEN;
                horizonNodes.erase(horizonNodes.begin());
                //isHorizon = false;
            }
            isHorizon = false;
        }

        std::cout << "PLANNER: adding new horizon nodes" << std::endl;
        for (uint j = a; j < b; j++)
            for (uint i = c; i < d; i++)
            {
                //Here Horizon Nodes must be updated
                nodeTarget = nodeMatrix[j][i];
                if((nodeTarget->state == OPEN) &&//Being OPEN it cannot be horizon
                   (nodeTarget->terrain != 0))
                    for (uint k = 0; k < 4; k++)
                    {
                        if((nodeTarget->nb4List[k] != NULL) &&
                           (nodeTarget->nb4List[k]->state ==
                            HIDDEN))
                        {
                            horizonNodes.push_back(nodeTarget);
                            setHorizonCost(nodeTarget, globalMap);
                            nodeTarget->state = CLOSED;
                            break;
                        }
                    }
            }
        std::cout << "Visibility is updated in " << (base::Time::now()-t1) << " s" << std::endl;
        //std::cout<< 100*visibleNodes.size()/(nodeMatrix.size()*nodeMatrix[0].size()) << "% of map is visible " << std::endl;
    }
    return flag;
}

void NodeMap::setHorizonCost(Node* horizonNode, NodeMap* globalMap)
{
    double x = (horizonNode->pose.position[0]*this->scale +
               this->globalOffset.position[0]) / globalMap->scale +
               globalMap->globalOffset.position[0];
    double y = (horizonNode->pose.position[1]*this->scale +
               this->globalOffset.position[1]) / globalMap->scale +
               globalMap->globalOffset.position[1];

    uint i = (uint)(x);
    uint j = (uint)(y);
    double a = x - (double)(i);
    double b = y - (double)(j);

    Node * node00 = (globalMap->getNode(i,j));
    Node * node10 = node00->nb4List[2];
    Node * node01 = node00->nb4List[3];
    Node * node11 = node00->nb4List[2]->nb4List[3];

    double w00 = node00->total_cost;
    double w10 = node10->total_cost;
    double w01 = node01->total_cost;
    double w11 = node11->total_cost;

    horizonNode->total_cost = w00 + (w10 - w00)*a + (w01 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
    if ((horizonNode->total_cost < 0)||(horizonNode->total_cost == INF))
    {
        std::cout << "ERROR: Horizon Node " <<
          horizonNode->pose.position[0] << "," <<
          horizonNode->pose.position[1] << ")" << std::endl;
        std::cout << " - total_cost = " << horizonNode->total_cost << std::endl;
        std::cout << " - w00 =  " << w00 << std::endl;
        std::cout << " - w10 =  " << w10 << std::endl;
        std::cout << " - w01 =  " << w01 << std::endl;
        std::cout << " - w11 =  " << w11 << std::endl;
        std::cout << " - a =  " << a << std::endl;
        std::cout << " - b =  " << b << std::endl;
        std::cout << " - i =  " << i << std::endl;
        std::cout << " - j =  " << j << std::endl;
    }
}

void NodeMap::expandRisk(std::vector<Node*>& expandableNodes)
{
    Node * nodeTarget;
    Node * nodeNeighbour;
    /*uint kmax = 10;
    std::vector<Node*> nextExpandableNodes;
    for(uint k = 0; k<kmax; k++)
    {
        nextExpandableNodes.clear();
        if (expandableNodes.size() == 0)
            break;
        for(uint j = 0; j < expandableNodes.size()-1;j++)
        {
            //std::cout << "k = " << k << ", j = " << j << std::endl;
            nodeTarget = expandableNodes[j];
            //std::cout << "PLANNER: expanding node " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
            //std::cout << "PLANNER: expandableNodes -> " << expandableNodes.size() << std::endl;
            //expandableNodes.erase(expandableNodes.begin()+j);
            for (uint i = 0; i<4; i++)
            {
                //std::cout << "Number of neighbors " << nodeTarget->nb4List.size() << std::endl;
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (1 - k*(1/(double)kmax) > nodeTarget->nb4List[i]->risk.obstacle))
                {
                    //std::cout << "Im here" << std::endl;
                    nodeTarget->nb4List[i]->risk.obstacle = 1 - k*(1/(double)kmax);
                    nextExpandableNodes.push_back(nodeTarget->nb4List[i]);
                }
                //std::cout << "k = " << k << ", j = " << j << ", i = " << i << std::endl;
            }
        }
        expandableNodes = nextExpandableNodes;
    }
    expandableNodes.clear();*/
    //uint counter = 0;
    //std::cout << "PLANNER: expanding risk of " << expandableNodes.size() << " new obstacle nodes" << std::endl;
    while(!expandableNodes.empty())
    {
        //std::cout << "PLANNER: number of expandable nodes is " << expandableNodes.size() << std::endl;
        nodeTarget = maxRiskNode(expandableNodes);
        //std::cout << "PLANNER: number of expandable nodes is " << expandableNodes.size() <<" and current risk is " << nodeTarget->risk.obstacle << std::endl;
        //std::cout << "PLANNER: expanding node " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
        for (uint i = 0; i<4; i++)
        {
            nodeNeighbour = getNeighbour(nodeTarget,i);
            if (nodeNeighbour != NULL)
                propagateRisk(nodeNeighbour, expandableNodes);
        }/*counter++;
        if (counter > 1000)
            break;*/
    }
}

void NodeMap::propagateRisk(Node* nodeTarget, std::vector<Node*>& expandableNodes)
{
    double Ry,Rx;
    /*if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Ry = fmax(nodeTarget->nb4List[3]->risk.obstacle, nodeTarget->nb4List[0]->risk.obstacle);
    else if (nodeTarget->nb4List[0] == NULL)
        Ry = nodeTarget->nb4List[3]->risk.obstacle;
    else
        Ry = nodeTarget->nb4List[0]->risk.obstacle;*/
    Node * Ny0 = getNeighbour(nodeTarget,0);
    Node * Ny1 = getNeighbour(nodeTarget,3);
    Ry = fmax(Ny0 == NULL?0:Ny0->risk.obstacle, Ny1 == NULL?0:Ny1->risk.obstacle);

   /* if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Rx = fmax(nodeTarget->nb4List[1]->risk.obstacle, nodeTarget->nb4List[2]->risk.obstacle);
    else if (nodeTarget->nb4List[1] == NULL)
        Rx = nodeTarget->nb4List[2]->risk.obstacle;
    else
        Rx = nodeTarget->nb4List[1]->risk.obstacle;*/
    Node * Nx0 = getNeighbour(nodeTarget,1);
    Node * Nx1 = getNeighbour(nodeTarget,2);
    Rx = fmax(Nx0 == NULL?0:Nx0->risk.obstacle, Nx1 == NULL?0:Nx1->risk.obstacle);

    double Sx = 1 - Rx;
    double Sy = 1 - Ry;
    double C = 0.1;
    double S;

    //std::cout << "PLANNER: Sx = " << Sx << std::endl;
    //std::cout << "PLANNER: Sy = " << Sy << std::endl;

    if (fabs(Sx-Sy)<C)
        S = (Sx+Sy+sqrt(2*pow(C,2.0) - pow((Sx-Sy),2.0)))/2;
    else
        S = fmin(Sx,Sy) + C;

    //std::cout << "PLANNER: S = " << S << std::endl;

    double R = 1 - S;
    if ((R>0.1)&&(R>nodeTarget->risk.obstacle))
    {
        nodeTarget->risk.obstacle = R;
        expandableNodes.push_back(nodeTarget);
    }
}

Node* NodeMap::maxRiskNode(std::vector<Node*>& expandableNodes)
{
    if (expandableNodes.empty())
        return NULL;
    Node* nodePointer = expandableNodes.front();
    uint index = 0;
    double maxRisk = expandableNodes.front()->risk.obstacle;
    //std::cout << "Size of Narrow Band is: " << this->narrowBand.size() << std::endl;
    for (uint i =0; i < expandableNodes.size(); i++)
    {
        if (maxRisk == 1)
            break;
        if (expandableNodes[i]->risk.obstacle > maxRisk)
        {
            maxRisk = expandableNodes[i]->risk.obstacle;
            nodePointer = expandableNodes[i];
            index = i;
            break;
        }
    }
    /*std::cout << "PLANNER: next expandable node is  (" <<
        nodePointer->pose.position[0] << "," <<
        nodePointer->pose.position[1] << ")" << std::endl;*/
    expandableNodes.erase(expandableNodes.begin() + index);
    return nodePointer;
}


envire::ElevationGrid* NodeMap::getEnvirePropagation(base::Waypoint wPos, bool crop, double work_scale)
{
    if (crop)
    {
        uint a = (uint)(fmax(0,((wPos.position[1] - 4.0)/scale)));
        uint b = (uint)(fmin(nodeMatrix.size(),((wPos.position[1] + 4.0)/scale)));
        uint c = (uint)(fmax(0,((wPos.position[0] - 4.0)/scale)));
        uint d = (uint)(fmin(nodeMatrix[0].size(),((wPos.position[0] + 4.0)/scale)));
        envire::ElevationGrid* elevGrid = new envire::ElevationGrid(
            d-c, b-a,
            this->scale, this->scale);
        for (uint j = 0; j < b-a; j++)
        {
            for (uint i = 0; i < d-c; i++)
            {
                if(nodeMatrix[j+a][i+c]->total_cost != INF)
                    elevGrid->get((double)(i)*scale,(double)(j)*scale) = (nodeMatrix[j+a][i+c]->total_cost-minWork)/work_scale;// / (nodeActualPos->work-minWork);
                else
                    elevGrid->get((double)(i)*scale,(double)(j)*scale) = 0;
            }
        }
        return elevGrid;
    }
    else
    {
        envire::ElevationGrid* elevGrid = new envire::ElevationGrid(
            this->nodeMatrix[0].size(), this->nodeMatrix.size(),
            this->scale, this->scale);
        for (uint j = 0; j < nodeMatrix.size(); j++)
        {
            for (uint i = 0; i < nodeMatrix[0].size(); i++)
            {
                if(nodeMatrix[j][i]->total_cost != INF)
                    elevGrid->get((double)(i)*scale,(double)(j)*scale) = nodeMatrix[j][i]->total_cost/work_scale;// (nodeMatrix[j][i]->work-minWork) / (nodeActualPos->work-minWork);
                else
                    elevGrid->get((double)(i)*scale,(double)(j)*scale) = 0;
            }
        }
        return elevGrid;
    }
}

envire::ElevationGrid* NodeMap::getEnvireRisk()
{

    envire::ElevationGrid* elevGrid = new envire::ElevationGrid(
            this->nodeMatrix[0].size(), this->nodeMatrix.size(),
            this->scale, this->scale);
    for (uint j = 0; j < nodeMatrix.size(); j++)
    {
        for (uint i = 0; i < nodeMatrix[0].size(); i++)
        {
            if(nodeMatrix[j][i]->total_cost != INF)
                elevGrid->get((double)(i)*scale,(double)(j)*scale) = nodeMatrix[j][i]->risk.obstacle;
            else
                elevGrid->get((double)(i)*scale,(double)(j)*scale) = nodeMatrix[j][i]->risk.obstacle;
        }
    }
    return elevGrid;
}

envire::TraversabilityGrid* NodeMap::getGlobalEnvireState()
{
    envire::TraversabilityGrid* travGrid = new envire::TraversabilityGrid(
          nodeMatrix[0].size(), nodeMatrix.size(),
          this->scale, this->scale, -0.5, -0.5);
    travGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.2)); // Obstacle Area
    travGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0)); // Driving Area
    travGrid->setTraversabilityClass(2, envire::TraversabilityClass(1.0)); // Wheel-walking Area
    travGrid->setTraversabilityClass(3, envire::TraversabilityClass(0.5)); // Transition Area
    /*
    for(uint i = 0; i < 1; i++) //TODO: put here number of terrains
        travGrid->setTraversabilityClass(i+2, envire::TraversabilityClass(1 - 2*(double)i/10));
    for(uint i = 0; i < 4; i++)
        travGrid->setTraversabilityClass(i+3, envire::TraversabilityClass(0.9-0.2 * (double)i));
   */
    for (uint j = 0; j < nodeMatrix.size(); j++)
    {
        for (uint i = 0; i < nodeMatrix[0].size(); i++)
        {
            travGrid->setProbability(1.0, i,j);
            if (nodeMatrix[j][i]->terrain == 0)
                travGrid->setTraversability(0, i,j);
            else if (nodeMatrix[j][i]->terrain == 1)
            {
                travGrid->setTraversability(1, i,j);
            }
            else
            {
                travGrid->setTraversability(2, i,j);
            }
        }
    }
    std::cout << "Global State Map is updated" << std::endl;
    return travGrid;
}

envire::TraversabilityGrid* NodeMap::getLocalEnvireState(base::Waypoint wPos, bool crop)
{
    if (crop)
    {
        uint a = (uint)(fmax(0,((wPos.position[1] - 4.0)/scale)));
        uint b = (uint)(fmin(nodeMatrix.size(),((wPos.position[1] + 4.0)/scale)));
        uint c = (uint)(fmax(0,((wPos.position[0] - 4.0)/scale)));
        uint d = (uint)(fmin(nodeMatrix[0].size(),((wPos.position[0] + 4.0)/scale)));
        envire::TraversabilityGrid* travGrid = new envire::TraversabilityGrid(
            d-c, b-a,
            this->scale, this->scale, (double)(c)*scale,(double)(a)*scale);
        std::cout << "PLANNER: giving local envire state, a = " << a << ", b = " << b << ", c = " << c << ", d =" << d << std::endl;
        for (uint j = 0; j < b-a; j++)
        {
            for (uint i = 0; i < d-c; i++)
            {
                travGrid->setProbability(1.0, i,j);
                if (nodeMatrix[j+a][i+c]->state == HIDDEN)
                    travGrid->setTraversability(0,i,j);
                else if (nodeMatrix[j+a][i+c]->terrain == 0)
                    travGrid->setTraversability(1, i,j);
                else if (nodeMatrix[j+a][i+c]->risk.obstacle == 0)
                {
                    travGrid->setTraversability(3, i,j);
                }
                else
                {
                    travGrid->setTraversability(2, i,j);
                }
            }
        }
        travGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.2)); // Hidden Area
        travGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0)); // Obstacle Area
        travGrid->setTraversabilityClass(2, envire::TraversabilityClass(0.5)); // Hazardous Area
        travGrid->setTraversabilityClass(3, envire::TraversabilityClass(1.0)); // Traversable Area

        std::cout << "State Map is updated" << std::endl;
        return travGrid;

    }
    else
    {
        envire::TraversabilityGrid* travGrid = new envire::TraversabilityGrid(
            nodeMatrix[0].size(), nodeMatrix.size(),
            this->scale, this->scale);
        for (uint j = 0; j < nodeMatrix.size(); j++)
        {
            for (uint i = 0; i < nodeMatrix[0].size(); i++)
            {
                travGrid->setProbability(1.0, i,j);
                if (nodeMatrix[j][i]->state == HIDDEN)
                    travGrid->setTraversability(0,i,j);
                else if (nodeMatrix[j][i]->terrain == 0)
                    travGrid->setTraversability(1, i,j);
                else if (nodeMatrix[j][i]->risk.obstacle == 0)
                {
                    travGrid->setTraversability(3, i,j);
                }
                else
                {
                    travGrid->setTraversability(2, i,j);
                }
            }
        }
        travGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.2)); // Hidden Area
        travGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0)); // Obstacle Area
        travGrid->setTraversabilityClass(2, envire::TraversabilityClass(0.5)); // Hazardous Area
        travGrid->setTraversabilityClass(3, envire::TraversabilityClass(1.0)); // Traversable Area

        std::cout << "State Map is updated" << std::endl;
        return travGrid;
    }
}

std::string NodeMap::getLocomotionMode(base::Waypoint wPos)
{
    wPos.position[0] = wPos.position[0]/(this->scale);
    wPos.position[1] = wPos.position[1]/(this->scale);
    uint scaledX = (uint)(wPos.position[0] + 0.5);
    uint scaledY = (uint)(wPos.position[1] + 0.5);
    Node * n = this->getNode(scaledX, scaledY);
    return n->nodeLocMode;
}

bool NodeMap::updateNodePower(double new_power, base::Waypoint wPos, bool value_inverted)
{
    /*wPos.position[0] = wPos.position[0]/(this->scale);
    wPos.position[1] = wPos.position[1]/(this->scale);
    uint scaledX = (uint)(wPos.position[0] + 0.5);
    uint scaledY = (uint)(wPos.position[1] + 0.5);
    Node * n = this->getNode(scaledX, scaledY);
    if (value_inverted)
        new_power = 1/new_power;
    if (new_power > n->power)
    {
        n->power = new_power;
        return true; //In this case, replanning of global is needed
    }*/
    return false; //Global replanning not needed
}

bool NodeMap::updateNodeSlip(double dSlip, base::Waypoint wPos)
{
    wPos.position[0] = wPos.position[0]/(this->scale);
    wPos.position[1] = wPos.position[1]/(this->scale);
    uint scaledX = (uint)(wPos.position[0] + 0.5);
    uint scaledY = (uint)(wPos.position[1] + 0.5);
    Node * n = this->getNode(scaledX, scaledY);

    std::cout << "PLANNER: Updating Node " <<
          n->pose.position[0] << "," <<
          n->pose.position[1] << ")" << std::endl;

    if (dSlip > n->slip_ratio)
    {
        std::cout << "PLANNER: previous slip ratio was " << n->slip_ratio << " and new slip ratio is " << dSlip << std::endl;
        n->slip_ratio = dSlip;
        return true; //In this case, replanning of global is needed
    }
    std::cout << "PLANNER: new slip ratio is lower than the previous one, no update" << std::endl;
    return false; //Global replanning not needed
}


void NodeMap::resetHorizonNodes(NodeMap* globalMap)
{
    if (!horizonNodes.empty())
    {
        std::cout << "PLANNER: recalculating cost of "<< horizonNodes.size() << " horizon nodes" << std::endl;
        for (uint i = 0; i < horizonNodes.size(); i++)
            setHorizonCost(horizonNodes[i], globalMap);
    }
}
