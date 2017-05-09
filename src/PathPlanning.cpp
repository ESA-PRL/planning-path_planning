#include "PathPlanning.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning()
{
    printf("Planner created successfully\n");
}

PathPlanning::~PathPlanning()
{
}

void PathPlanning::initNodeMatrix(std::vector< std::vector<double> > elevation, std::vector< std::vector<double> > friction,
                            std::vector< std::vector<double> > slip, std::vector< std::vector<double> > risk){

  std::vector<Node*> nodeRow;

  for (uint j = 0; j < elevation[0].size(); j++){
    for (uint i = 0; i < elevation.size(); i++)
      nodeRow.push_back(new Node(i, j, elevation[i][j], friction[i][j], slip[i][j], risk[i][j]));
    nodeMatrix.push_back(nodeRow);
    nodeRow.clear();
  }

  //Building Neighborhood
  for (uint j = 0; j < elevation[0].size(); j++)
    for (uint i = 0; i < elevation.size(); i++){
      nodeMatrix[j][i]->nb4List.push_back(getNode(i,j-1));
      nodeMatrix[j][i]->nb4List.push_back(getNode(i-1,j));
      nodeMatrix[j][i]->nb4List.push_back(getNode(i,j+1));
      nodeMatrix[j][i]->nb4List.push_back(getNode(i+1,j));
    }
}

void PathPlanning::nodeUpdate(Node* node, Node* nodeParent, double cost, double heading, locomotionMode locMode)
{
  node->cost = cost;
  node->nodeParent = nodeParent;
  node->heading = heading;
  node->nodeLocMode = locMode;
}

Node* PathPlanning::getNode(uint x, uint y)
{
    Node* nodeTarget;
    if ((x >= nodeMatrix.size())||(y >= nodeMatrix.size()))
        return NULL;
    else
        nodeTarget = nodeMatrix[y][x];
    if (nodeTarget->state != OBSTACLE)
        return nodeTarget;
    else
        return NULL;
}


//__CALCULATION_OF_GRADIENT_FIELD

void PathPlanning::calculateFieldGradient(std::vector< std::vector<double> > field,
                                          std::vector< std::vector<double> >& fieldGx,
                                          std::vector< std::vector<double> >& fieldGy)
{
    double Gx,Gy;
    std::vector<double> gradientRowX;
    std::vector<double> gradientRowY;

    for (uint j = 0; j < field.size(); j++)
    {
        for (uint i = 0; i < field[j].size(); i++)
        {
            if (field[j][i] != NULL)
            {
              // Calculation of Gy
                if (j == 0)
                    if (field[1][i] != NULL)
                        Gy = field[1][i] - field[0][i];
                    else
                        Gy = 0;
                else if (j == field.size() - 1)
                    if (field[field.size() - 2][i] != NULL)
                        Gy = field[field.size() - 1][i] - field[field.size() - 2][i];
                    else
                        Gy = 0;
                else if(((field[j-1][i] != NULL))&&((field[j+1][i] != NULL)))
                    Gy = (field[j+1][i] - field[j-1][i])*0.5;
                else if (field[j+1][i] != NULL)
                    Gy = field[j+1][i] - field[j][i];
                else if (field[j-1][i] != NULL)
                    Gy = field[j][i] - field[j-1][i];
                else
                    Gy = 0;
                
              // Calculation of Gx
                if (i == 0)
                    if (field[j][1] != NULL)
                        Gx = field[j][1] - field[j][0];
                    else
                        Gx = 0;
                else if (i == field[j].size() - 1)
                    if (field[j][field[j].size() - 2] != NULL)
                        Gx = field[j][field[j].size() - 1] - field[j][field[j].size() - 2];
                    else
                        Gx = 0;
                else if(((field[j][i-1] != NULL))&&((field[j][i+1] != NULL)))
                    Gx = (field[j][i+1] - field[j][i-1])*0.5;
                else if (field[j][i+1] != NULL)
                    Gx = field[j][i+1] - field[j][i];
                else if (field[j][i-1] != NULL)
                    Gx = field[j][i] - field[j][i-1];
                else
                    Gx = 0;
            }
            gradientRowX.push_back(Gx/sqrt(pow(Gx,2)+pow(Gy,2)));
            gradientRowY.push_back(Gy/sqrt(pow(Gx,2)+pow(Gy,2)));
        }
        fieldGx.push_back(gradientRowX);
        fieldGy.push_back(gradientRowY);
        gradientRowX.clear();
        gradientRowY.clear();
    }
}


//__GRADIENT DESCENT METHOD__

std::vector<base::Waypoint> PathPlanning::gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal, std::vector< std::vector<double> > field, double tau)
{
    double newX, newY, newL, dCostX, dCostY;
    std::vector<base::Waypoint> trajectory;
    trajectory.insert(trajectory.begin(), wGoal);
    base::Waypoint wNew;
    calculateFieldGradient(propagationMatrix, propagationGXMatrix, propagationGYMatrix);
    interpolateWaypoint(wGoal.position[0], wGoal.position[1], dCostX, dCostY);
    while(sqrt(pow((trajectory.front().position[0] - wStart.position[0]),2) +
             pow((trajectory.front().position[1] - wStart.position[1]),2)) > (2*tau))
    {
        newX = trajectory.front().position[0] - tau*dCostX;
        newY = trajectory.front().position[1] - tau*dCostY;
        wNew.heading = atan2(dCostY,dCostX);
        interpolateWaypoint(newX, newY, dCostX, dCostY);
        wNew.position = Eigen::Vector3d(newX,newY,newL);
        trajectory.insert(trajectory.begin(),wNew);
    }
    trajectory.insert(trajectory.begin(), wStart);
    return trajectory;
}


//__PROPAGATION_FUNCTION_USING_EIKONAL_EQUATION__

void PathPlanning::propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand)
{
    double Tx,Ty,P,W;
    locomotionMode L;
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Ty = !(getPropagation(nodeTarget->nb4List[2]) < getPropagation(nodeTarget->nb4List[0]))?getPropagation(nodeTarget->nb4List[0]):getPropagation(nodeTarget->nb4List[2]);
    else if (nodeTarget->nb4List[0] == NULL)
        Ty = getPropagation(nodeTarget->nb4List[2]);
    else
        Ty = getPropagation(nodeTarget->nb4List[0]);

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Tx = !(getPropagation(nodeTarget->nb4List[3]) < getPropagation(nodeTarget->nb4List[1]))?getPropagation(nodeTarget->nb4List[1]):getPropagation(nodeTarget->nb4List[3]);
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = getPropagation(nodeTarget->nb4List[3]);
    else
        Tx = getPropagation(nodeTarget->nb4List[1]);

    L = nodeTarget->nodeLocMode;
    P = costFunction(nodeTarget);

    if ((fabs(Tx-Ty)<P)&&(Tx < INF)&&(Ty < INF)){
        W = (Tx+Ty+sqrt(2*pow(P,2.0) - pow((Tx-Ty),2.0)))/2;
    }else{
        W = (!(Tx<Ty)?Ty:Tx) + P;
    }

    if (nodeTarget->state == NARROW)
        if(W < getPropagation(nodeTarget))
        {
            setPropagation(nodeTarget,W);
            nodeTarget->nodeLocMode = L;
	    nodeTarget->power = P;
        }

    if (nodeTarget->state == OPEN)
    {
        nodeTarget->state = NARROW;
        setPropagation(nodeTarget,W);
        nodeTarget->nodeLocMode = L;
        nodeTarget->power = P;
        narrowBand.push_back(nodeTarget);
    }
}


//__FAST_MARCHING_ALGORITHM__

std::vector<base::Waypoint> PathPlanning::fastMarching(base::Waypoint wStart, base::Waypoint wGoal)
{

  // Setting Start Node
    Node * nodeTarget = getNode(wStart.position[0], wStart.position[1]);

  // Initializing the Narrow Band
    std::vector<Node*> narrowBand;
    narrowBand.push_back(nodeTarget);

  // Initializing the Propagation Matrix
    propagationMatrix.resize(nodeMatrix.size());    
    for (uint j = 0; j < propagationMatrix.size(); j++)
        propagationMatrix[j].resize(nodeMatrix[j].size());

    for (uint j = 0; j < propagationMatrix.size(); j++)
        for (uint i = 0; i < propagationMatrix[j].size(); i++)
            propagationMatrix[j][i] = INF;
    setPropagation(nodeTarget,0);

  // Propagation Loop
    while (narrowBand.size() > 0)
    {
        nodeTarget = minCostNode(narrowBand);
        nodeTarget->state = FROZEN;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state != FROZEN) &&
                (nodeTarget->nb4List[i]->state != CLOSED) &&
                (nodeTarget->nb4List[i]->state != OBSTACLE))
                propagationFunction(nodeTarget->nb4List[i], narrowBand);
    }
    return gradientDescentTrajectory(wStart, wGoal, propagationMatrix, 0.8);
}

double PathPlanning::costFunction(Node* nodeTarget){

    double mu = nodeTarget->soil.friction;
    double s = nodeTarget->soil.slip;
    double ri = nodeTarget->risk.obstacle;
    double m = 20.0;
    double g = 3.711;
    double r = 0.07;
    double R = 19;
    double Kt = 0.0109;

    //std::cout << "Node : (" << nodeTarget->x  << ", " << nodeTarget->y << ") Friction: " << mu << " Slip: " << s << '\n';

    return mu;
}

Node* PathPlanning::minCostNode(std::vector<Node*>& nodeList)
{
    Node* nodePointer = nodeList[0];
    uint index = 0;
    uint i;
    double minCost = getPropagation(nodeList[0]);
    for (i =0; i < nodeList.size(); i++)
    {
        if (getPropagation(nodeList[i]) < minCost)
        {
            minCost = getPropagation(nodeList[i]);
            nodePointer = nodeList[i];
            index = i;
        }
    }
    nodeList.erase(nodeList.begin() + index);
    return nodePointer;
}

double PathPlanning::getPropagation(Node* nodeTarget)
{
    return propagationMatrix[(uint)(nodeTarget->pose.position[1])][(uint)(nodeTarget->pose.position[0])];
}

void PathPlanning::setPropagation(Node* nodeTarget, double value)
{
    propagationMatrix[(uint)(nodeTarget->pose.position[1])][(uint)(nodeTarget->pose.position[0])] = value;
}


//__INTERPOLATION_ON_WAYPOINT__

void PathPlanning::interpolateWaypoint(double x, double y, double& dCostX, double& dCostY)
{
    uint i = (uint)x;
    uint j = (uint)y;
    double a = x - (double)i;
    double b = y - (double)j;

    double gx00 = propagationGXMatrix[j][i];
    double gx10 = propagationGXMatrix[j][i+1];
    double gx01 = propagationGXMatrix[j+1][i];
    double gx11 = propagationGXMatrix[j+1][i+1];

    double gy00 = propagationGYMatrix[j][i];
    double gy10 = propagationGYMatrix[j][i+1];
    double gy01 = propagationGYMatrix[j+1][i];
    double gy11 = propagationGYMatrix[j+1][i+1];

    dCostX = gx00 + (gx10 - gx00)*a + (gx01 - gx00)*b + (gx11 + gx00 - gx10 - gx01)*a*b;
    dCostY = gy00 + (gy10 - gy00)*a + (gy10 - gy00)*b + (gy11 + gy00 - gy10 - gy01)*a*b;
}
