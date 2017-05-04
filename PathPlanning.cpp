#include "PathPlanning.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning()
{
    printf("Planner created successfully\n");
    WD = 0.25;
    WO = 0.25;
    WM = 0.25;
    WR = 0.25;
}

void PathPlanning::setPlanningMode(planningMode mode)
{
    switch (mode) {
        case SHORTEST: WD = 0.5;  WO = 0.3;  WM = 0.1;  WR = 0.1;  break;
        case SAFEST:   WD = 0.1;  WO = 0.1;  WM = 0.3;  WR = 0.5;  break;
        case BALANCED: WD = 0.25; WO = 0.25; WM = 0.25; WR = 0.25; break;
  }
}

PathPlanning::~PathPlanning()
{
}

bool PathPlanning::setStartNode(base::samples::RigidBodyState startPose)
{
    unsigned int x = (uint)startPose.position[0];
    unsigned int y = (uint)startPose.position[1];
    if ((nodeStart = getNode(x,y))==NULL)
        return false;
    nodeStart->roverPose = startPose;
    return true;
}

bool PathPlanning::setStartNode(base::Pose2D startPose)
{
    unsigned int x = (uint)startPose.position[0];
    unsigned int y = (uint)startPose.position[1];
    if ((nodeStart = getNode(x,y))==NULL)
        return false;
    //nodeStart->roverPose = startPose; FIX THIS
    return true;
}

bool PathPlanning::setStartNode(base::Waypoint wStart)
{
    unsigned int x = (uint)wStart.position[0];
    unsigned int y = (uint)wStart.position[1];
    if ((nodeStart = getNode(x,y))==NULL)
        return false;
    //nodeStart->roverPose = startPose; FIX THIS
    return true;
}



bool PathPlanning::setGoal(double gx, double gy){
  nodeGoal = getNode((uint)(gx),(uint)(gy));
  nodeGoal->globalX = gx;
  nodeGoal->globalY = gy;
  if ((nodeGoal == NULL)||(nodeGoal->state != OPEN)){
    printf("Start Node is either an obstacle or out of boundaries\n");
    return false;
  }
  std::cout << "Goal Node: (" << nodeGoal->x << "," << nodeGoal->y << ")" <<'\n';
  std::cout << "Goal Node height: " << nodeGoal->height <<'\n';
  return true;
}

void PathPlanning::showStart(){
  std::cout << "X: " << nodeStart->x << " Y: " << nodeStart->y;
  std::cout << " Height: " << nodeStart->height << '\n';
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


void PathPlanning::showNodeMatrix(){
  std::cout << "NodeMatrix: " << '\n';
  for (uint j = 0; j < 64; j++){
    for (uint i = 0; i < 64; i++)
      std::cout << nodeMatrix[i][j]->work << " " ;
    std::cout << '\n';
  }
}

void PathPlanning::calculatePitchRoll(double slope, double aspect, double yaw, double &roll, double &pitch)
{
  double cs = cos(slope);
  double ss = sin(slope);
  double ca = cos(aspect + yaw);
  double sa = sin(aspect + yaw);
  roll = atan2(ss*sa,cs+ca*ss);
  pitch = atan2(ss*ca,cs);
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
        nodeTarget = nodeMatrix[(uint)y][(uint)x];
    if (nodeTarget->state != OBSTACLE)
        return nodeTarget;
    else
        return NULL;
}


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

std::vector<base::Waypoint> PathPlanning::gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal, std::vector< std::vector<double> > field, double tau)
{
    double newX, newY, newL, dCostX, dCostY;
    std::vector<base::Waypoint> trajectory;
    trajectory.insert(trajectory.begin(), wGoal);
    base::Waypoint wNew;
    calculateFieldGradient(propagationMatrix, propagationGXMatrix, propagationGYMatrix);
    while(sqrt(pow((trajectory.front().position[0] - wStart.position[0]),2) +
             pow((trajectory.front().position[1] - wStart.position[1]),2)) > 1.5)//Fix 1.5 value
    {
        newX = trajectory.front().position[0] - tau*dCostX;
        newY = trajectory.front().position[1] - tau*dCostY;
        interpolateWaypoint(newX, newY, dCostX, dCostY, newL);
        std::cout << "newX: " << newX << " newY: "<< newY << '\n';
        std::cout << "dCostX: " << dCostX << " dCostY: "<< dCostY << '\n';
        wNew.position = Eigen::Vector3d(newX,newY,newL);
        //wNew.heading = 
        trajectory.insert(trajectory.begin(),wNew);
    }
    trajectory.insert(trajectory.begin(), wStart);
}

void PathPlanning::propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand)
{
    double Tx,Ty,Delta,P,W,D=0;
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
    if(W < getPropagation(nodeTarget)){
            setPropagation(nodeTarget,W);
            nodeTarget->nodeLocMode = L;
	    nodeTarget->power = P;
    }
  if (nodeTarget->state == OPEN){
    nodeTarget->state = NARROW;
    setPropagation(nodeTarget,W);
    nodeTarget->nodeLocMode = L;
    nodeTarget->power = P;
    narrowBand.push_back(nodeTarget);
  }
}

std::vector<base::Waypoint> PathPlanning::fastMarching(base::Waypoint wStart, base::Waypoint wGoal)
{
    Node * nodeTarget = getNode(wStart.position[0], wStart.position[1]);
    if (nodeTarget == NULL)
    {
        std::cout << "Bad Start, exiting" << '\n';
        //break; FIX
    }
    //if ((wGoal.position[0] >= nodeMatrix.size())||(wGoal.position[1] >= nodeMatrix.size())) fIX THIS
    std::vector<Node*> narrowBand;
    narrowBand.push_back(nodeTarget);

    propagationMatrix.resize(nodeMatrix.size());
    for (uint j = 0; j < propagationMatrix.size(); j++)
        propagationMatrix[j].resize(nodeMatrix[j].size());

    for (uint j = 0; j < propagationMatrix.size(); j++)
        for (uint i = 0; i < propagationMatrix[j].size(); i++)
            propagationMatrix[j][i] = INF;

    setPropagation(nodeTarget,0);

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
    double ri = nodeTarget->risk;
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

void PathPlanning::interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& L)
{
  uint i = (uint)x;
  uint j = (uint)y;
  double a = x - (double)i;
  double b = y - (double)j;
  Node* node00 = getNode(i,j);
  Node* node10 = getNode(i+1,j);
  Node* node01 = getNode(i,j+1);
  Node* node11 = getNode(i+1,j+1);

  dCostX = (node00->dCostX) +
                       (node10->dCostX - node00->dCostX)*a +
                       (node01->dCostX - node00->dCostX)*b +
                       (node11->dCostX + node00->dCostX - node10->dCostX - node01->dCostX)*a*b;
  dCostY = (node00->dCostY) +
                       (node10->dCostY - node00->dCostY)*a +
                       (node01->dCostY - node00->dCostY)*b +
                       (node11->dCostY + node00->dCostY - node10->dCostY - node01->dCostY)*a*b;
  L =                  (node00->nodeLocMode) +
                       (node10->nodeLocMode - node00->nodeLocMode)*a +
                       (node01->nodeLocMode - node00->nodeLocMode)*b +
                       (node11->nodeLocMode + node00->nodeLocMode - node10->nodeLocMode - node01->nodeLocMode)*a*b;
  if (L<0.5)
    L = 0.0;
  else
    L = 1.0;

  //x(nuevo) = x(anterior) - tau * interpolatePoint(x(anterior), y(anterior), );
  //cost(nuevo) = interpolatePoint(gamma(end,:),cost);
  //height(nuevo) = interpolatePoint(gamma(end,:),heightMap);
}

void PathPlanning::clearPath()
{
  nodePath.clear();
}
