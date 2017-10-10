#include "PathPlanning.hpp"
#include "NodeMap.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning(plannerType _type, std::vector< terrainType* > _table):type(_type),costTable(_table)
{
    std::cout<< "Created planner type: " << type << std::endl;
    this->narrowBand.clear();
}

PathPlanning::~PathPlanning()
{
}

//__COST_FUNCTION__

void PathPlanning::costFunction(uint Terrain, double& Power, locomotionMode& lM)
{
    // Here a Look-Up Table should be built
    Power = costTable[Terrain]->cost;

    switch(Terrain)
    {
	      case 0: lM = DRIVING; break;
	      //case 1: Power = 0.088; lM = DRIVING; break;
              //case 2: Power = 1.074; lM = WHEEL_WALKING; break;
	      //case 2: Power = 0.236; lM = DRIVING; break;
         // SLOWNESS POWER
              case 1: lM = DRIVING; break;
              case 2: lM = WHEEL_WALKING; break;
	      //case 2: Power = 1.074; lM = DRIVING; break;
    }
}


//__FAST_MARCHING_ALGORITHM__

void PathPlanning::fastMarching(base::Waypoint wGoal, NodeMap * nodes)
{
    nodes->setGoal(wGoal);
    Node * nodeTarget;
    initNarrowBand(nodes, wGoal);
  // Propagation Loop
    t1 = base::Time::now();
    std::cout<< "PLANNER: starting propagation loop " << std::endl;
    nodes->minWork = 0;
    while (!narrowBand.empty())
    {
        nodeTarget = minCostNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN))
                {
                    scalarPropagation(nodeTarget->nb4List[i], nodes->scale);
                    nodes->closedNodes.push_back(nodeTarget->nb4List[i]);
                }
    }
    std::cout<< "PLANNER: ended propagation loop" << std::endl;
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
}

void PathPlanning::fastMarching(base::Waypoint wGoal, NodeMap * nodes,
                                NodeMap * globalNodes, base::Waypoint wStart)
{
  // Initialize nodeStart, nodeGoal and nodeTarget
    std::cout << "PLANNER: rover is at " << wStart.position[0] << "," << wStart.position[1] << std::endl;
    nodes->setActualPos(wStart);
    std::cout << "PLANNER: actual node is " << nodes->actualPose.position[0] << "," << nodes->actualPose.position[1] << std::endl;
    nodes->setGoal(wGoal);
    std::cout << "PLANNER: goal pose is " << nodes->getGoal()->pose.position[0] << "," << nodes->getGoal()->pose.position[1] << std::endl;
    Node * nodeTarget;
  // Initializing the Narrow Band
    std::cout << "PLANNER: initializing Narrow Band" << std::endl;
    initNarrowBand(nodes, wGoal);

  // Propagation Loop
    t1 = base::Time::now();
    std::cout << "PLANNER: starting local propagation loop" << std::endl;

    Node * node00 = nodes->nodeActualPos;
    Node * nodeN0 = nodes->getNeighbour(node00,2);
    Node * nodeN1 = nodes->getNeighbour(nodeN0,3);
    Node * nodeN2 = nodes->getNeighbour(node00,3);
    Node * nodeN3 = nodes->getNeighbour(nodeN2,1);
    Node * nodeN4 = nodes->getNeighbour(node00,1);
    Node * nodeN5 = nodes->getNeighbour(nodeN4,0);
    Node * nodeN6 = nodes->getNeighbour(node00,0);
    Node * nodeN7 = nodes->getNeighbour(nodeN6,2);

    std::cout<< "PLANNER: created LOCAL Final Nodes" << std::endl;
    if(node00->terrain == 0)
            std::cout << "PLANNER: rover is on obstacle area??" << std::endl;
    if(node00->state == HIDDEN)
            std::cout << "PLANNER: for some reason node00 is hidden... " << std::endl;
    std::cout << "PLANNER: node00 pose is " << node00->pose.position[0] << "," << node00->pose.position[1] << std::endl;
    bool first_iteration = true;
    while ((node00->state != CLOSED)||(nodeN0->state != CLOSED)||
           (nodeN1->state != CLOSED)||(nodeN2->state != CLOSED)||
           (nodeN3->state != CLOSED)||(nodeN4->state != CLOSED)||
           (nodeN5->state != CLOSED)||(nodeN6->state != CLOSED)||
           (nodeN7->state != CLOSED))
    {
            nodeTarget = minCostNode();
            if (first_iteration)
            {
                first_iteration = false;
                nodes->minWork = nodeTarget->total_cost;
            }
            nodeTarget->state = CLOSED;
            for (uint i = 0; i<4; i++)
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (nodeTarget->nb4List[i]->state == OPEN))
                {
                    scalarPropagation(nodeTarget->nb4List[i], nodes->scale);
                    if(nodeTarget->nb4List[i] == node00)
                       std::cout << "PLANNER: total cost of node00 is " << node00->total_cost << std::endl;
                    nodes->closedNodes.push_back(nodeTarget->nb4List[i]);//To later erase total cost value of non close nodes as well!!!
                }
    }
    std::cout<< "PLANNER: ended local propagation loop" << std::endl;
    std::cout<< "PLANNER: value of local total cost = " << node00->total_cost << std::endl;
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
}


void PathPlanning::initNarrowBand(NodeMap * nodes, base::Waypoint wGoal)
{
    std::cout << "PLANNER: Initializing NarrowBand" << std::endl;
    Node * nodeGoal = nodes->getGoal();

    nodes->resetPropagation();
    narrowBand.clear();
    narrowBand = nodes->horizonNodes;
    if(nodeGoal->state != HIDDEN)
    {
        nodeGoal->total_cost = 0;
        narrowBand.push_back(nodeGoal);
        if (this->type == LOCAL_PLANNER)
            std::cout << "PLANNER: goal node is visible" << std::endl;
    }
    else
    {
        if (this->type == LOCAL_PLANNER)
            std::cout << "PLANNER: goal node is not visible" << std::endl;
    }
    std::cout << "PLANNER: number of horizon nodes = " << narrowBand.size() << std::endl;
}

void PathPlanning::getHorizonCost(NodeMap* localMap, Node* horizonNode, NodeMap* globalMap)
{
    double x = (horizonNode->pose.position[0]*localMap->scale +
               localMap->globalOffset.position[0]) / globalMap->scale +
               globalMap->globalOffset.position[0];
    double y = (horizonNode->pose.position[1]*localMap->scale +
               localMap->globalOffset.position[1]) / globalMap->scale +
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

    horizonNode->total_cost = w00 + (w01 - w00)*a + (w10 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
    if (horizonNode->total_cost < 0)
    {
        std::cout << "ERROR: Horizon Node " <<
          horizonNode->pose.position[0] << "," <<
          horizonNode->pose.position[1] << ")" << std::endl;
        std::cout << " - total cost = " << horizonNode->total_cost << std::endl;
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

//__ISOTROPIC_PROPAGATION_FUNCTION_USING_EIKONAL_EQUATION__

void PathPlanning::scalarPropagation(Node* nodeTarget, double scale)
{
    double Tx,Ty,P,T,R,C;
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
    if(this->type == LOCAL_PLANNER)
    {
        R = nodeTarget->risk.obstacle;
        if (nodeTarget->terrain == 0)
            C = scale*(costTable[nodeTarget->terrain]->cost)*(1+9*R);
        else
            C = scale*(costTable[1]->cost)*(1+9*R);
    }
    else
    {
        C = scale*(costTable[nodeTarget->terrain]->cost);
        //L =
        /*if (nodeTarget->slip_ratio == 1)
        {
            costFunction(0,P,L);
            std::cout << "PLANNER: the slip ratio is 1, global node is an obstacle" << std::endl;
        }
        else
        {
            costFunction(nodeTarget->terrain,P,L);
            //TODO: make this more general!!!
            P = P/(1-nodeTarget->slip_ratio);
            if(nodeTarget->slip_ratio != 0)
                std::cout << "PLANNER: propagating through node (" << nodeTarget->pose.position[0] << ", " << nodeTarget->pose.position[1] << ") with slip " << nodeTarget->slip_ratio << " and new cost " << P << std::endl;
        }*/
    }

  // Eikonal Equation
    if ((fabs(Tx-Ty)<C)&&(Tx < INF)&&(Ty < INF))
        T = (Tx+Ty+sqrt(2*pow(C,2.0) - pow((Tx-Ty),2.0)))/2;
    else
        T = fmin(Tx,Ty) + C;

    if(T < nodeTarget->total_cost)
    {
        if (nodeTarget->total_cost == INF) //It is not in narrowband
            this->narrowBand.push_back(nodeTarget);
        nodeTarget->total_cost = T;
        nodeTarget->nodeLocMode = costTable[nodeTarget->terrain]->optimalLM;
    }

    if (nodeTarget->total_cost == INF)
    {
        std::cout << "ERROR:Propagation on Node " <<
          nodeTarget->pose.position[0] << "," <<
          nodeTarget->pose.position[1] << ")" << std::endl;
        std::cout << " - Total Cost = " << nodeTarget->total_cost << std::endl;
        std::cout << " - Tx =  " << Tx << std::endl;
        std::cout << " - Ty =  " << Ty << std::endl;
    }

}


//__GRADIENT_DESCENT_METHOD__

bool PathPlanning::getPath(NodeMap * nodes, double tau,
 					     std::vector<base::Waypoint>& trajectory)
{
    double newX, newY, newH, dCostX, dCostY,
           normDCostX, normDCostY, distance = 0, dHeading, risk;
    base::Waypoint sinkPoint;
    base::Waypoint startPoint;

    sinkPoint.position[0] = nodes->getGoal()->pose.position[0];
    sinkPoint.position[1] = nodes->getGoal()->pose.position[1];
    sinkPoint.heading = nodes->getGoal()->pose.orientation;

    std::cout << "PLANNER: working with the path" << std::endl;
    base::Waypoint wNew;
    //if (!trajectory.empty())
    std::cout << "PLANNER: trajectory is empty, starting from actual point" << std::endl;
    trajectory.clear();
    startPoint = nodes->actualPose;
    calculateNextWaypoint(startPoint.position[0], startPoint.position[1], dCostX, dCostY, newH, nodes, risk);
    startPoint.position[2] = newH;
        //trajectory.insert(trajectory.begin(), startPoint);
    trajectory.push_back(startPoint);
    newX = trajectory.back().position[0];
    newY = trajectory.back().position[1];
    std::cout << "PLANNER: trajectory initialized" << std::endl;


    while(sqrt(pow((trajectory.back().position[0] - sinkPoint.position[0]),2) +
             pow((trajectory.back().position[1] - sinkPoint.position[1]),2)) > (2*tau))
    {
        trajectory.back().position[0] = trajectory.back().position[0]*nodes->scale;
        trajectory.back().position[1] = trajectory.back().position[1]*nodes->scale;
        normDCostX = dCostX/sqrt(pow(dCostX,2) + pow(dCostY,2));
        normDCostY = dCostY/sqrt(pow(dCostX,2) + pow(dCostY,2));
        newX = newX - tau*normDCostX;
        newY = newY - tau*normDCostY;

        bool isNearHidden = calculateNextWaypoint(newX, newY, dCostX, dCostY, newH, nodes, risk);
        if (!isNearHidden)
        {
            std::cout<< "PLANNER: node (" << newX << "," << newY <<") is near hidden area"<< std::endl;
            return false;
        }
        else
            wNew.position = Eigen::Vector3d(newX,newY,newH);
        dHeading = acos((normDCostX*dCostX + normDCostY*dCostY)/sqrt(pow(dCostX,2)+pow(dCostY,2)));

        wNew.heading = atan2(-dCostY,-dCostX);

        distance += sqrt(pow((newX - trajectory.back().position[0]),2) +
                 pow((newY - trajectory.back().position[1]),2))*nodes->scale;
        trajectory.push_back(wNew);
        if (trajectory.size()>99)
        {
            std::cout<< "ERROR at node (" << newX << "," << newY <<")"<< std::endl;
            trajectory.back().position[0] = trajectory.back().position[0]*nodes->scale;
            trajectory.back().position[1] = trajectory.back().position[1]*nodes->scale;
            return false;
        }
    }
    trajectory.back().position[0] = trajectory.back().position[0]*nodes->scale;
    trajectory.back().position[1] = trajectory.back().position[1]*nodes->scale;
    calculateNextWaypoint(sinkPoint.position[0], sinkPoint.position[1], dCostX, dCostY, newH, nodes, risk);
    if (sinkPoint.heading == -0)
        sinkPoint.heading = 0;
    sinkPoint.position[0] = sinkPoint.position[0]*nodes->scale;
    sinkPoint.position[1] = sinkPoint.position[1]*nodes->scale;
    sinkPoint.position[2] = newH;
    trajectory.push_back(sinkPoint);
    std::cout<< "PLANNER: Adding final waypoint with heading" << sinkPoint.heading << " "<< trajectory.back().heading<<  std::endl;
    return true;
}


Node* PathPlanning::minCostNode()
{
    Node* nodePointer = this->narrowBand.front();
    uint index = 0;
    uint i;
    double minCost = this->narrowBand.front()->total_cost;
    //std::cout << "Size of Narrow Band is: " << this->narrowBand.size() << std::endl;
    for (i =0; i < this->narrowBand.size(); i++)
    {
        if (this->narrowBand[i]->total_cost < minCost)
        {
            minCost = this->narrowBand[i]->total_cost;
            nodePointer = this->narrowBand[i];
            index = i;
        }
    }
    this->narrowBand.erase(this->narrowBand.begin() + index);
    return nodePointer;
}


void PathPlanning::gradientNode(Node* nodeTarget, double& dnx, double& dny)
{
    double dx, dy;
    /*if((nodeTarget->state == OBSTACLE)||(nodeTarget->state == HIDDEN))
    {
        dnx = 0;
        dny = 0;
    }
    else
    {*/
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
    //}
}

//__INTERPOLATION_ON_WAYPOINT__

bool PathPlanning::calculateNextWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, NodeMap * nodes, double& risk)
{

    uint i = (uint)(x);
    uint j = (uint)(y);
    double a = x - (double)(i);
    double b = y - (double)(j);

    double gx00, gx10, gx01, gx11;
    double gy00, gy10, gy01, gy11;

    Node * node00 = (nodes->getNode(i, j));
    Node * node10 = node00->nb4List[2];
    Node * node01 = node00->nb4List[3];
    Node * node11 = node00->nb4List[2]->nb4List[3];
    if ((isHorizon(node00))||(isHorizon(node01))||
       (isHorizon(node10))||(isHorizon(node11)))
        return false;
    /*
    else
    {*/
    gradientNode( node00, gx00, gy00);
    gradientNode( node10, gx10, gy10);
    gradientNode( node01, gx01, gy01);
    gradientNode( node11, gx11, gy11);

    dCostX = interpolate(a,b,gx00,gx01,gx10,gx11);
    dCostY = interpolate(a,b,gy00,gy01,gy10,gy11);
    height = interpolate(a, b, node00->elevation, node01->elevation,
                             node10->elevation, node11->elevation);
    return true;
}

double PathPlanning::interpolate(double a, double b, double g00, double g01, double g10, double g11)
{
    return g00 + (g10 - g00)*a + (g01 - g00)*b + (g11 + g00 - g10 - g01)*a*b;
}

bool PathPlanning::isHorizon(Node* n)
{
    for (uint k = 0; k < 4; k++)
        if ((n->nb4List[k] != NULL) && ((n->nb4List[k]->state == HIDDEN)))
            return true;
    return false;
}
