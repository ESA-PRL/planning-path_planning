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
    t1 = base::Time::now();
    printf("Creating NodeMap from envire data\n");
    scale = travGrid->getScaleX();
    std::vector<Node*> nodeRow;
    uint klass;
    for (uint j = 0; j < travGrid->getCellSizeY(); j++)
    {
        for (uint i = 0; i < travGrid->getCellSizeX(); i++)
        {
            klass = travGrid->getGridData()[j][i];
            nodeRow.push_back(new Node(i, j, 0, klass==1?0:1, 0));
            switch(klass)
            {
                case 0: //Unknown node
                    nodeRow.back()->state = HIDDEN;
                    break;
                case 1: //Obstacle node
                    nodeRow.back()->state = OBSTACLE;
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

NodeMap::NodeMap(double size, base::Pose2D pos,
                std::vector< std::vector<double> > elevation,
		std::vector< std::vector<double> > cost,
		std::vector< std::vector<double> > risk)
{
    t1 = base::Time::now();
    printf("Creating NodeMap\n");
    scale = size;
    globalOriginPose = pos;
    std::vector<Node*> nodeRow;
    for (uint j = 0; j < elevation[0].size(); j++)
    {
        for (uint i = 0; i < elevation.size(); i++)
            nodeRow.push_back(new Node(i, j, elevation[j][i], cost[j][i],
                               risk[j][i]));
        nodeMatrix.push_back(nodeRow);
        nodeRow.clear();
    }
    std::cout<<"NodeMap created in " << (base::Time::now()-t1) << " s" << std::endl;
    t1 = base::Time::now();
    makeNeighbourhood();
    std::cout<<"Neighbourhood made in " << (base::Time::now()-t1) << " s" << std::endl;
}

void NodeMap::makeNeighbourhood()
{
  printf("Making Neighbourhood\n");
  for (uint j = 0; j < nodeMatrix[0].size(); j++)
  {
      for (uint i = 0; i < nodeMatrix.size(); i++)
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

          nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i,  j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j+1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i-1,j-1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i,  j-1));
          nodeMatrix[j][i]->nb8List.push_back(getNode(i+1,j-1));
      }
  }
}

void NodeMap::resetPropagation()
{
    for (uint j = 0; j < nodeMatrix[0].size(); j++)
    {
        for (uint i = 0; i < nodeMatrix.size(); i++)
        {
            nodeMatrix[j][i]->state = FAR;
            nodeMatrix[j][i]->work = INF;
        }
    }
}

void NodeMap::setActualPos(base::Waypoint wPos)
{
    wPos.position[0] = wPos.position[0]/(this->scale);
    wPos.position[1] = wPos.position[1]/(this->scale);
    uint scaledX = (uint)(wPos.position[0] + 0.5);
    uint scaledY = (uint)(wPos.position[1] + 0.5);
    nodeActualPos = this->getNode(scaledX, scaledY);
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
    if (nodeMatrix[j][i]->state != OBSTACLE)
        return nodeMatrix[j][i];
    else
        return NULL;
}
