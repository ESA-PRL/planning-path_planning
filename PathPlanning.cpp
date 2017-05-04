#include "PathPlanning.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


namespace PathPlanning_lib
{

// PUBLIC
PathPlanning::PathPlanning(){
  printf("Planner created successfully\n");
  WD = 0.25;
  WO = 0.25;
  WM = 0.25;
  WR = 0.25;
}

void PathPlanning::setPlanningMode(planningMode mode){
  switch (mode) {
    case SHORTEST: WD = 0.5;  WO = 0.3;  WM = 0.1;  WR = 0.1;  break;
    case SAFEST:   WD = 0.1;  WO = 0.1;  WM = 0.3;  WR = 0.5;  break;
    case BALANCED: WD = 0.25; WO = 0.25; WM = 0.25; WR = 0.25; break;
  }
}

PathPlanning::~PathPlanning() {
}

void PathPlanning::setOffset(double dx, double dy, double dtheta)
{
  offsetX = dx;
  offsetY = dy;
  offsetTheta = dtheta;
}

bool PathPlanning::setStart(double gx, double gy, double heading){

  mStartPose.position    = Eigen::Vector3d(gx,gy,0);
  mStartPose.orientation = Eigen::Quaterniond(
                     Eigen::AngleAxisd(heading, Eigen::Vector3d::UnitZ()));
  nodeStart = getNode((uint)gx,(uint)gy);
  std::cout << "Start X: " << nodeStart->x << " Start Y: " << nodeStart->y << '\n';
  nodeStart->globalX = gx;
  nodeStart->globalY = gy;
  if ((nodeStart == NULL)||(nodeStart->state != OPEN))
  {
    printf("Start Node is either an obstacle or out of boundaries\n");
    return false;
  }
  nodeStart->heading = heading;
  nodeStart->work = 0.0;
  std::cout << "gx: " << gx << " gy: " << gy << '\n';
  std::cout << "Start Node: (" << nodeStart->x << "," << nodeStart->y;
  std::cout << "," << nodeStart->heading << ")" <<'\n';
  std::cout << "Start Node height: " << nodeStart->height <<'\n';
  std::cout << "Start Node risk: " << nodeStart->risk <<'\n';
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

Node* PathPlanning::getNode(uint x, uint y){
    if ((x >= nodeMatrix.size())||(y >= nodeMatrix.size()))
      return NULL;
    else
      return nodeMatrix[y][x];
}


void PathPlanning::showNodeMatrix(){
  std::cout << "NodeMatrix: " << '\n';
  for (uint j = 0; j < 64; j++){
    for (uint i = 0; i < 64; i++)
      std::cout << nodeMatrix[i][j]->work << " " ;
    std::cout << '\n';
  }
}

void PathPlanning::costFunction(Node* start, Node* goal, double& heading,
                          double& cost, locomotionMode& locMode){

  // Previous Distance-Orientation calculations
  double dx = (double)goal->x - (double)start->x;
  double dy = (double)goal->y - (double)start->y;
/*  double dz = (double)goal->height - (double)start->height;
  double dH = sqrt(pow(dx,2)+pow(dy,2)+pow(dz,2))/1.5; //1.5=sqrt(1^2 + 1^2 + 2*tan(20º)^2)*/

  //heading = atan2(-dx,dy);
  heading = atan2(dy,dx);

  /*if(goal->aspect < 0){ //slope = 0
    goal->roll = 0;
    goal->pitch = 0;
  }else{
    double cs = cos(goal->slope);
    double ss = sin(goal->slope);
    double ca = cos(goal->aspect + heading);
    double sa = sin(goal->aspect + heading);
    goal->roll = atan2(ss*sa,cs+ca*ss);
    goal->pitch = atan2(ss*ca,cs);
  }

  // LOCOMOTION MODE
  if(goal->slope > 10.0*M_PI/180.0) locMode = WHEEL_WALKING;
  else locMode = DRIVING;

  // COST

  double materialCost = goal->s;
  double headingCost  = 0.5-0.5*(cos(heading)*cos(start->heading)+sin(heading)*sin(start->heading));
  double riskCost     = goal->risk;*/
  /*double localCost;
  if(goal->mu < 0.2){
    cost = 6.465*sqrt(pow(dx,2)+pow(dy,2)) + start->cost;
    locMode = DRIVING;
  }else{
    cost = 18.26*sqrt(pow(dx,2)+pow(dy,2)) + start->cost;
    //W = 61.02;
    locMode = WHEEL_WALKING;
  }*/
  cost = goal->materialCost*sqrt(pow(dx,2)+pow(dy,2)) + start->cost;
  locMode = goal->nodeLocMode;
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

void PathPlanning::dijkstraAlgorithm()
{
  std::vector<Node*> nodeList;
  Node * nodeTarget = nodeStart;
  nodeList.push_back(nodeTarget);
  std::cout << "Starting Dijkstra Path Planning Algorithm" << '\n';
  double cost, heading;
  locomotionMode locMode;
  //Loop
  while (nodeGoal->state != CLOSED)
  {
    for (uint8_t i = 0; i<8; i++)
      if ((nodeTarget->nbList[i] != NULL) && (nodeTarget->nbList[i]->state == OPEN))
      {
        costFunction(nodeTarget,nodeTarget->nbList[i],heading,cost,locMode);
        if (nodeTarget->nbList[i]->nodeParent == NULL) //Not parent == Not in List
        {
          nodeUpdate(nodeTarget->nbList[i], nodeTarget, cost, heading, locMode);
          nodeList.push_back(nodeTarget->nbList[i]);
        }
        else if (nodeTarget->nbList[i]->cost > cost)//Is in List
          nodeUpdate(nodeTarget->nbList[i], nodeTarget, cost, heading, locMode);
      }
    // Target Node is closed
    nodeTarget->state = CLOSED;

    // Select next target node
    cost = INF; //Very high value ~ inf
    for (size_t i =0; i<nodeList.size(); i++)
    {
      /*std::cout << "X: " << nodeList[i]->x<< '\n';
      std::cout << "Y: " << nodeList[i]->y<< '\n';
      std::cout << "cost: " << nodeList[i]->cost << '\n';
      std::cout << "inf: " << INF << '\n';*/
      if ((nodeList[i]->state == OPEN) && (nodeList[i]->cost < cost))
      {
        cost = nodeList[i]->cost;
        nodeTarget = nodeList[i];
      }
    }
  }

  nodeTarget = nodeGoal;
  nodePath.push_back(nodeTarget);
  while (nodeTarget != nodeStart)
  {
    nodePath.push_back(nodeTarget = nodeTarget->nodeParent);
  }
}

void PathPlanning::astarAlgorithm()
{
  std::vector<Node*> nodeList;
  Node * nodeTarget = nodeStart;
  nodeList.push_back(nodeTarget);
  std::cout << "Starting A* Path PLanning Algorithm" << '\n';
  //Loop
  double cost,heading,hcost;
  locomotionMode locMode;
  while (nodeGoal->closed == false)
  {
    // Target Node is closed
    nodeTarget->closed = true;
    for (uint8_t i = 0; i<8; i++)
      if ((nodeTarget->nbList[i] != NULL) && (nodeTarget->nbList[i]->closed == false))
      {
        costFunction(nodeTarget,nodeTarget->nbList[i],heading,cost,locMode);
        if (nodeTarget->nbList[i]->nodeParent == NULL) //Not parent == Not in List
        {
          nodeUpdate(nodeTarget->nbList[i], nodeTarget, cost, heading, locMode);
          nodeList.push_back(nodeTarget->nbList[i]);
          heuristicCostFunction(nodeTarget->nbList[i], nodeGoal);
        }
        else if (nodeTarget->nbList[i]->cost > cost)//Is in List
          nodeUpdate(nodeTarget->nbList[i], nodeTarget, cost, heading, locMode);
      }

    // Select next target node
    hcost = 10000000.0; //Very high value ~ inf
    for (size_t i =0; i<nodeList.size(); i++)
    {
      if ((nodeList[i]->closed == false) && (nodeList[i]->heuristicCost < hcost))
      {
        hcost = nodeList[i]->heuristicCost;
        nodeTarget = nodeList[i];
      }
    }
  }

  nodeTarget = nodeGoal;
  nodePath.push_back(nodeTarget);
  while (nodeTarget != nodeStart)
  {
    nodePath.push_back(nodeTarget = nodeTarget->nodeParent);
  }
}

void PathPlanning::fastMarchingAlgorithm(){
  std::vector<Node*> narrowBand;
  Node * nodeTarget = nodeStart;
  narrowBand.push_back(nodeTarget);
  std::cout << "Starting Fast Marching Path Planning Algorithm" << '\n';
  double D = 0.0;
  locomotionMode L;
  //Loop
  while (narrowBand.size() > 0)
  {
    nodeTarget = minCostNode(narrowBand);
    nodeTarget->state = FROZEN;
    /*std::cout << "W: " << nodeTarget->work << '\n';
    sleep(1);*/
    for (uint i = 0; i<4; i++)
      if ((nodeTarget->nb4List[i] != NULL) &&
          (nodeTarget->nb4List[i]->state != FROZEN) &&
          (nodeTarget->nb4List[i]->state != CLOSED))
        propagationFunction(nodeTarget->nb4List[i], narrowBand);
  }
  std::cout << "Ending Fast Marching Path Planning Algorithm" << '\n';
  calculateCostGradient();

  double newX, newY, newL, dCostX, dCostY, tau = 0.8;

  base::Waypoint wGoal,wNew,wStart;
  wGoal.position = Eigen::Vector3d(nodeGoal->x,nodeGoal->y,nodeGoal->nodeLocMode);
  wGoal.heading      = 15.0/180.0*M_PI;
  wGoal.tol_position = 0.1;
  wGoal.tol_heading  = 5.0 /180*M_PI;
  dCostX = nodeGoal->dCostX; 
  dCostY = nodeGoal->dCostY; 
  trajectory.insert(trajectory.begin(), wGoal);
  std::cout << "Work Start: " << nodeStart->work << '\n';
  std::cout << "dCostX: " << dCostX << " dCostY: "<< dCostY << '\n';
  std::cout << "Estimated work to arrive: " << nodeGoal->work << '\n';
  std::cout << "Trajectory started" << '\n';

  std::cout << "N0: (" << nodeStart->nb4List[0]->x << "," << nodeStart->nb4List[0]->y << ") " << '\n';
  std::cout << "N1: (" << nodeStart->nb4List[1]->x << "," << nodeStart->nb4List[1]->y << ") " << '\n';
  std::cout << "N2: (" << nodeStart->nb4List[2]->x << "," << nodeStart->nb4List[2]->y << ") " << '\n';
  std::cout << "N3: (" << nodeStart->nb4List[3]->x << "," << nodeStart->nb4List[3]->y << ") " << '\n';

  while(sqrt(pow((trajectory.front().position[0] - mStartPose.position[0]),2) +
             pow((trajectory.front().position[1] - mStartPose.position[1]),2)) > 1.5)
  {
    newX = trajectory.front().position[0] - tau*dCostX;
    newY = trajectory.front().position[1] - tau*dCostY;
    interpolateWaypoint(newX, newY, dCostX, dCostY, newL);
    std::cout << "newX: " << newX << " newY: "<< newY << '\n';
    std::cout << "dCostX: " << dCostX << " dCostY: "<< dCostY << '\n';
    wNew.position = Eigen::Vector3d(newX,newY,newL);
    trajectory.insert(trajectory.begin(),wNew);
    //nodePath[nodePath.size()-2]->heading = atan2(nodePath[nodePath.size()-2]->dy-nodePath.back()->dy,nodePath[nodePath.size()-2]->dx-nodePath.back()->dx);
  }


  wStart.position = mStartPose.position;
  wStart.heading      = mStartPose.getYaw();
  wStart.tol_position = 0.1;
  wStart.tol_heading  = 5.0 /180*M_PI;
  trajectory.insert(trajectory.begin(),wStart);
}

void PathPlanning::heuristicCostFunction(Node* start, Node* goal)
{/*
  double dx = (double)goal->x - (double)start->x;
  double dy = (double)goal->y - (double)start->y;
  double dH = sqrt(pow(dx,2)+pow(dy,2))/sqrt(pow((double)grid.numNodes_X,2)+pow((double)grid.numNodes_Y,2));
  start->heuristicCost = start->cost + dH;*/
}

void PathPlanning::propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand)
{
  double Tx,Ty,Delta,P,W,D=0;
  locomotionMode L;
  if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
    Ty = !(nodeTarget->nb4List[2]->work < nodeTarget->nb4List[0]->work)?nodeTarget->nb4List[0]->work:nodeTarget->nb4List[2]->work;
  else if (nodeTarget->nb4List[0] == NULL)
    Ty = nodeTarget->nb4List[2]->work;
  else
    Ty = nodeTarget->nb4List[0]->work;

  if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
    Tx = !(nodeTarget->nb4List[3]->work < nodeTarget->nb4List[1]->work)?nodeTarget->nb4List[1]->work:nodeTarget->nb4List[3]->work;
  else if (nodeTarget->nb4List[1] == NULL)
    Tx = nodeTarget->nb4List[3]->work;
  else
    Tx = nodeTarget->nb4List[1]->work;

  /*  if(nodeTarget->mu < 0.2){
      W = 6.465;
      L = DRIVING;
    }else{
      W = 18.26;
      //W = 61.02;
      L = WHEEL_WALKING;
    }*/
  L = nodeTarget->nodeLocMode;
  P = cFunction(nodeTarget);
  //std::cout << "P: " << P << '\n';
  if ((fabs(Tx-Ty)<P)&&(Tx < INF)&&(Ty < INF)){
    W = (Tx+Ty+sqrt(2*pow(P,2.0) - pow((Tx-Ty),2.0)))/2;
  }else{
    W = (!(Tx<Ty)?Ty:Tx) + P;
  }
  //std::cout << "W: " << W << '\n';
  if (nodeTarget->state == NARROW)
    if(W < nodeTarget->work){
            nodeTarget->work = W;
            nodeTarget->nodeLocMode = L;
	    nodeTarget->power = P;
    }
  if (nodeTarget->state == OPEN){
    nodeTarget->state = NARROW;
    nodeTarget->work = W;
    nodeTarget->nodeLocMode = L;
    nodeTarget->power = P;
    narrowBand.push_back(nodeTarget);
  }
}

double PathPlanning::cFunction(Node* nodeTarget){

double mu = nodeTarget->friction;
double s = nodeTarget->slip;
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
  double minCost = nodeList[0]->work;
  for (i =0; i < nodeList.size(); i++)
  {
    if (nodeList[i]->work < minCost)
    {
      minCost = nodeList[i]->work;
      nodePointer = nodeList[i];
      index = i;
    }
  }
  nodeList.erase(nodeList.begin() + index);
  return nodePointer;
}

void PathPlanning::calculateCostGradient()
{
  double Gx,Gy;
  Node* nodeTarget;
  for (uint j = 0; j < nodeMatrix[0].size(); j++)
    for (uint i = 0; i < nodeMatrix.size(); i++){
      nodeTarget = nodeMatrix[i][j];
      if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Gy = (nodeTarget->nb4List[2]->work - nodeTarget->nb4List[0]->work)*0.5;
      else if (nodeTarget->nb4List[0] == NULL)
        Gy = nodeTarget->nb4List[2]->work - nodeTarget->work;
      else
        Gy = nodeTarget->work - nodeTarget->nb4List[0]->work;
      if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Gx = (nodeTarget->nb4List[3]->work - nodeTarget->nb4List[1]->work)*0.5;
      else if (nodeTarget->nb4List[1] == NULL)
        Gx = nodeTarget->nb4List[3]->work - nodeTarget->work;
      else
        Gx = nodeTarget->work - nodeTarget->nb4List[1]->work;
      nodeTarget-> dCostX = Gx/sqrt(pow(Gx,2)+pow(Gy,2));
      nodeTarget-> dCostY = Gy/sqrt(pow(Gx,2)+pow(Gy,2));
    }
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

} // namespace motion_planning_libraries
