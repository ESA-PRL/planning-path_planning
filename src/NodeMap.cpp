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
    std::cout<<"Creating nodemap with scale " << scale << " m" << std::endl;
    globalOffset = pos;
    std::vector<Node*> nodeRow;
    for (uint j = 0; j < elevation.size(); j++)
    {
        for (uint i = 0; i < elevation[0].size(); i++)
            nodeRow.push_back(new Node(i, j, elevation[j][i], cost[j][i], state));
        nodeMatrix.push_back(nodeRow);
        nodeRow.clear();
        //std::cout << "PLANNER: row number " << j << std::endl;
    }
    std::cout<<"NodeMap created in " << (base::Time::now()-t1) << " s" << std::endl;
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


  //                 8 - Neighbourhood
  //       nb4List[3] __ nb4List[2] __ nb4List[1]
  //       (i-1, j+1) __  (i, j+1)  __ (i+1, j+1)
  //           ||            ||            ||
  //       nb4List[4] __   target   __ nb4List[0]
  //        (i-1, j)  __   (i, j)   __  (i+1, j)
  //           ||            ||            ||
  //       nb4List[5] __ nb4List[6] __ nb4List[7]
  //       (i-1, j-1) __  (i, j-1)  __ (i+1, j-1)

          /*nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i,  j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j-1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i,  j-1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j-1));*/
      }
  }
    std::cout<<"4 - Neighbourhood made in " << (base::Time::now()-t1) << " s" << std::endl;
}

void NodeMap::makeNeighbourhood(Node* n, uint i, uint j)
{
    n->nb4List.clear();
    n->nb4List.push_back(getNode(i,j-1));
    n->nb4List.push_back(getNode(i-1,j));
    n->nb4List.push_back(getNode(i+1,j));
    n->nb4List.push_back(getNode(i,j+1));
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
            closedNodes[i]->work = INF;
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


// Simulation function
bool NodeMap::updateVisibility(base::Waypoint wPos, NodeMap* globalMap)
{
    t1 = base::Time::now();
    double rx,ry,x,y,counter = 0;
    bool flag = false;
    std::cout << "PLANNER: updating visibility" << std::endl;
    //Simulating what ExoTeR sees, a circular area of 1.4m radius
    uint a = (uint)(fmax(0,((wPos.position[1] - 2.0)/scale)));
    uint b = (uint)(fmin(nodeMatrix.size(),((wPos.position[1] + 2.0)/scale)));
    uint c = (uint)(fmax(0,((wPos.position[0] - 2.0)/scale)));
    uint d = (uint)(fmin(nodeMatrix[0].size(),((wPos.position[0] + 2.0)/scale)));
    std::cout << "PLANNER: a = " << a << ", b = " << b << ", c = "<< c << ", d = " << d << std::endl;
    Node* nodeTarget;
    obstacleNodes.clear();
    for (uint j = a; j < b; j++)
    {
        for (uint i = c; i < d; i++)
        {
            nodeTarget = nodeMatrix[j][i];
            makeNeighbourhood(nodeTarget,i,j); //TODO: this is making neighbours all the time...
            //std::cout << "PLANNER: analyzing node " << i << ", " << j << std::endl;
            if(nodeTarget->state != HIDDEN)
            {
                if ((((nodeTarget->nb4List[1] != NULL) &&
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
                }
                /*else if (nodeTarget->state == CLOSED)
                {
                  // Just making traversable nodes OPEN again
                    nodeTarget->state = OPEN;
                }*/
            }
            else
            {
                rx = i*scale + globalOffset.position[0];
                ry = j*scale + globalOffset.position[1];
                x = wPos.position[0];// + 0.4*cos(wPos.heading);
                y = wPos.position[1];// + 0.4*sin(wPos.heading);
                if ((sqrt(pow(rx-x,2) +
                             pow(ry-y,2)) < 1.5)&&(nodeTarget->state == HIDDEN))
                {
                    flag = true;
                    nodeTarget->state = OPEN;
                    if (nodeTarget->terrain == 0)
                    {
                        //std::cout << "PLANNER: Its an obstacle" << std::endl;
                        obstacleNodes.push_back(nodeTarget);
                    }
                }
            }
        }
    }

    //expandRisk(obstacleNodes);
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
                    horizonNodes[i]->work = INF;
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
                horizonNodes[0]->work = INF;
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

    double w00 = node00->work;
    double w10 = node10->work;
    double w01 = node01->work;
    double w11 = node11->work;

    horizonNode->work = w00 + (w10 - w00)*a + (w01 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
    if ((horizonNode->work < 0)||(horizonNode->work == INF))
    {
        std::cout << "ERROR: Horizon Node " <<
          horizonNode->pose.position[0] << "," <<
          horizonNode->pose.position[1] << ")" << std::endl;
        std::cout << " - work = " << horizonNode->work << std::endl;
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

void NodeMap::expandRisk(std::vector<Node*> expandableNodes)
{
    Node * nodeTarget;
    uint kmax = 4;
    for(uint k = 0; k<kmax; k++)
    {
        if (expandableNodes.size() == 0)
            break;
        for(int j = expandableNodes.size()-1; j>=0; j--)
        {
            //std::cout << "k = " << k << ", j = " << j << std::endl;
            nodeTarget = expandableNodes[j];
            //std::cout << "PLANNER: expanding node " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
            //std::cout << "PLANNER: expandableNodes -> " << expandableNodes.size() << std::endl;
            expandableNodes.erase(expandableNodes.begin()+j);
            for (uint i = 0; i<4; i++)
            {
                //std::cout << "Number of neighbors " << nodeTarget->nb4List.size() << std::endl;
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (1 - k*0.25 > nodeTarget->nb4List[i]->risk.obstacle))
                {
                    //std::cout << "Im here" << std::endl;
                    nodeTarget->nb4List[i]->risk.obstacle = 1 - k*(1/(double)kmax);
                    expandableNodes.push_back(nodeTarget->nb4List[i]);
                }
                //std::cout << "k = " << k << ", j = " << j << ", i = " << i << std::endl;
            }
        }
    }

}


envire::ElevationGrid* NodeMap::getEnvirePropagation()
{

    envire::ElevationGrid* elevGrid = new envire::ElevationGrid(
            this->nodeMatrix[0].size(), this->nodeMatrix.size(),
            this->scale, this->scale);
    for (uint j = 0; j < nodeMatrix.size(); j++)
    {
        for (uint i = 0; i < nodeMatrix[0].size(); i++)
        {
            if(nodeMatrix[j][i]->work != INF)
                elevGrid->get((double)(i)*scale,(double)(j)*scale) = nodeMatrix[j][i]->work;
            else
                elevGrid->get((double)(i)*scale,(double)(j)*scale) = 0;
        }
    }
    return elevGrid;
}

envire::TraversabilityGrid* NodeMap::getEnvireState()
{
    envire::TraversabilityGrid* travGrid = new envire::TraversabilityGrid(
          nodeMatrix[0].size(), nodeMatrix.size(),
          this->scale, this->scale);
    travGrid->setTraversabilityClass(0, envire::TraversabilityClass(0.2));
    travGrid->setTraversabilityClass(1, envire::TraversabilityClass(0.0));
    for(uint i = 0; i < 1; i++) //TODO: put here number of terrains
        travGrid->setTraversabilityClass(i+2, envire::TraversabilityClass(1 - 2*(double)i/10));
    for(uint i = 0; i < 4; i++)
        travGrid->setTraversabilityClass(i+3, envire::TraversabilityClass(0.9-0.2 * (double)i));
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
                travGrid->setTraversability(2, i,j);
            }
            else
            {
                travGrid->setTraversability(2 + (uint)((nodeMatrix[j][i]->risk.obstacle)/0.25), i,j);
            }
        }
    }
    std::cout << "State Map is updated" << std::endl;
    return travGrid;
}

double NodeMap::getLocomotionMode(double x, double y)
{
    uint i = (uint)(x);
    uint j = (uint)(y);
    double a = x - (double)(i);
    double b = y - (double)(j);

    Node * node00 = getNode(i, j);
    Node * node10 = node00->nb4List[2];
    Node * node01 = node00->nb4List[3];
    Node * node11 = node00->nb4List[2]->nb4List[3];

    //std::cout << "NodeLocModes: " << node00->nodeLocMode << "-" << node01->nodeLocMode << "-" << node10->nodeLocMode << "-" << node11->nodeLocMode << std::endl;

    return (double)ceil((node00->nodeLocMode + node10->nodeLocMode + node01->nodeLocMode + node11->nodeLocMode)/4.0);
}
