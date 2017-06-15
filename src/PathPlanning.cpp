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

void PathPlanning::initNodeMatrix(std::vector< std::vector<double> > elevation,
				  std::vector< std::vector<double> > cost,
				  std::vector< std::vector<double> > risk)
{
    std::vector<Node*> nodeRow;
    for (uint j = 0; j < elevation[0].size(); j++)
    {
        for (uint i = 0; i < elevation.size(); i++)
            nodeRow.push_back(new Node(i, j, elevation[j][i], cost[j][i], risk[j][i]));
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

void PathPlanning::initTerrainList(std::vector< double > friction, std::vector< double > slip)
{
    for (uint i = 0; i < friction.size(); i++)
    {
	terrainList.push_back(new soilType);
	terrainList[i]->friction = friction[i];
        terrainList[i]->slip = slip[i];
    }
    std::cout<< "Terrain List Created" << std::endl;
    for (uint i = 0; i < friction.size(); i++)
    {
        std::cout<< "Terrain " << i << ":" << std::endl;
        std::cout<< " Friction = " << terrainList[i]->friction << std::endl;
        std::cout<< " Slip = " << terrainList[i]->slip << std::endl;
    }
}

void PathPlanning::nodeUpdate(Node* node, Node* nodeParent, double cost, double heading, locomotionMode locMode)
{
  node->cost = cost;
  node->nodeParent = nodeParent;
  node->heading = heading;
  node->nodeLocMode = locMode;
}

Node* PathPlanning::getNode(double x, double y)
{
    Node* nodeTarget;
    if ((x >= nodeMatrix.size())||(y >= nodeMatrix.size()))
        return NULL;
    else
        nodeTarget = nodeMatrix[(int)(y+0.5)][(int)(x+0.5)];
    if (nodeTarget->state != OBSTACLE)
        return nodeTarget;
    else
        return NULL;
}


//__CALCULATION_OF_GRADIENT_FIELD

void PathPlanning::calculateFieldGradient()
{
    double Gx,Gy;

  //Resizing Gradients in X and Y of Propagation Matrix
    propagationGXMatrix.resize(propagationMatrix.size());    
    for (uint j = 0; j < propagationGXMatrix.size(); j++)
        propagationGXMatrix[j].resize(propagationMatrix[j].size());
    propagationGYMatrix.resize(propagationMatrix.size());    
    for (uint j = 0; j < propagationGYMatrix.size(); j++)
        propagationGYMatrix[j].resize(propagationMatrix[j].size());

  //Normalized Gradients
    for (uint j = 0; j < propagationMatrix.size(); j++)
    {
        for (uint i = 0; i < propagationMatrix[j].size(); i++)
        {
            if (propagationMatrix[j][i] != NULL)
            {
		if (j==0)
		    Gy = propagationMatrix[1][i] - propagationMatrix[0][i];
		else
		{
		    if (j == propagationMatrix.size() - 1)
		    {
			Gy = propagationMatrix[j][i] - propagationMatrix[j-1][i];
		    }
		    else
		    {
			Gy = (propagationMatrix[j+1][i] - propagationMatrix[j-1][i])*0.5;
		    }
	        }
		if (i==0)
		    Gx = propagationMatrix[j][1] - propagationMatrix[j][0];
		else
		{
		    if (i == propagationMatrix.size() - 1)
		    {
			Gx = propagationMatrix[j][i] - propagationMatrix[j][i-1];
		    }
		    else
		    {
			Gx = (propagationMatrix[j][i+1] - propagationMatrix[j][i-1])*0.5;
		    }
	        }
            }
            propagationGXMatrix[j][i] = Gx/sqrt(pow(Gx,2)+pow(Gy,2));
            propagationGYMatrix[j][i] = Gy/sqrt(pow(Gx,2)+pow(Gy,2));
	    if (sqrt(pow(propagationGXMatrix[j][i],2)+pow(propagationGYMatrix[j][i],2)) < 0.99)
	    {
		std::cout<< "ERROR in " << j << "," << i << std::endl;
		sleep(10);
	    }
        }
    }
}


//__GRADIENT DESCENT METHOD__

void PathPlanning::gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal,
					     std::vector< std::vector<double> > field, double tau,
 					     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector)
{
    double newX, newY, newH, dCostX, dCostY, heading, cX, cY;
    short int locMode;

    cX = wGoal.position[0]; //Position to take into account when calculating heading for Wheel-walking waypoints
    cY = wGoal.position[1]; //Position to take into account when calculating heading for Wheel-walking waypoints
    
    base::Waypoint wNew;
    calculateFieldGradient();
    std::cout<< "Fast Marching: field gradient calculated" << std::endl;
    interpolateWaypoint(wGoal.position[0], wGoal.position[1], dCostX, dCostY, newH, locMode);
    wGoal.position[2] = newH;
    trajectory.insert(trajectory.begin(), wGoal);
    locVector.insert(locVector.begin(), locMode);
    //std::cout<< "Loc Mode inicial: "  << locVector.front() << std::endl;
    newX = trajectory.front().position[0];
    newY = trajectory.front().position[1];
    while(sqrt(pow((newX - wStart.position[0]),2) +
             pow((newY - wStart.position[1]),2)) > (2*tau))
    {
        newX = newX - tau*dCostX;
        newY = newY - tau*dCostY;
        
        interpolateWaypoint(newX, newY, dCostX, dCostY, newH, locMode);
        wNew.position = Eigen::Vector3d(newX,newY,newH);


        if ((locMode == 0)&&(locVector.front() == 0)) //Driving Mode
        {
            wNew.heading = atan2(dCostY,dCostX);
        }
 
        if ((locMode == 1)&&(locVector.front() == 1)) //Wheel-Walking Mode
        {
            wNew.heading = atan2(cY-newY, cX-newX);
        }

        if ((locMode == 0)&&(locVector.front() == 1)) //Driving to Wheel-Walking
        {
            wNew.heading = atan2(cY-newY, cX-newX);
        }

        if ((locMode == 1)&&(locVector.front() == 0)) //Wheel-walking to Driving
        {
            cX = newX;
            cY = newY;
            wNew.heading = atan2(dCostY,dCostX);
        }

        trajectory.insert(trajectory.begin(),wNew);
        locVector.insert(locVector.begin(), locMode);
	if (trajectory.size()>500)
	    break;
    }
    interpolateWaypoint(wStart.position[0], wStart.position[1], dCostX, dCostY, newH, locMode);
    wStart.position[2] = newH;
    trajectory.insert(trajectory.begin(), wStart);
    locVector.insert(locVector.begin(), locMode);
}


//__PROPAGATION_FUNCTION_USING_EIKONAL_EQUATION__

void PathPlanning::propagationFunction(Node* nodeTarget, std::vector<Node*>& narrowBand)
{
    double Tx,Ty,P,W;
    locomotionMode L;

  // Neighbor Propagators
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
    costFunction(nodeTarget,P,L);

    P = P + nodeTarget->risk.obstacle;

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

void PathPlanning::fastMarching(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector)
{

  // Setting Start Node
    Node * nodeTarget = getNode(wStart.position[0], wStart.position[1]);
    std::cout<< "Start at (" << nodeTarget->pose.position[0] << "," <<nodeTarget->pose.position[1] << ")" << std::endl;

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
    std::cout<< "Fast Marching: ended propagation loop" << std::endl;
    std::cout<< "Fast Marching: goal cost is " << getPropagation(getNode(wGoal.position[0], wGoal.position[1])) << std::endl;
    gradientDescentTrajectory(wStart, wGoal, propagationMatrix, 0.8, trajectory, locVector);
}

void PathPlanning::costFunction(Node* nodeTarget, double& Power, locomotionMode& lM)
{
    // Here a Look-Up Table should be built

    switch(nodeTarget->terrain)
    {
	case 0: Power = 4.0;   lM = DRIVING; break;
	case 1: Power = 0.088; lM = DRIVING; break;
	//case 2: Power = 0.236; lM = WHEEL_WALKING; break;
	case 2: Power = 1.074; lM = WHEEL_WALKING; break;
    }
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

void PathPlanning::showPropagationMatrix()
{
    
    for (unsigned int j =0; j < propagationMatrix.size(); j++)
    {
        for (unsigned int i =0; i < propagationMatrix.size(); i++)
        {
            std::cout<< propagationGXMatrix[j][i] << " " ;
        }
	std::cout<< std::endl;
    }

}

//__INTERPOLATION_ON_WAYPOINT__

void PathPlanning::interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode)
{
    //uint i = (uint)(x+0.5);
    //uint j = (uint)(y+0.5);
    //double a = x - (double)((uint)x);
    //double b = y - (double)((uint)y);
    
    uint i = (uint)(x);
    uint j = (uint)(y);
    double a = x - (double)(i);
    double b = y - (double)(j);

    double gx00 = propagationGXMatrix[j][i];
    double gx10 = propagationGXMatrix[j][i+1];
    double gx01 = propagationGXMatrix[j+1][i];
    double gx11 = propagationGXMatrix[j+1][i+1];

    double gy00 = propagationGYMatrix[j][i];
    double gy10 = propagationGYMatrix[j][i+1];
    double gy01 = propagationGYMatrix[j+1][i];
    double gy11 = propagationGYMatrix[j+1][i+1];

    double h00 = nodeMatrix[j][i]->elevation;
    double h10 = nodeMatrix[j][i+1]->elevation;
    double h01 = nodeMatrix[j+1][i]->elevation;
    double h11 = nodeMatrix[j+1][i+1]->elevation;

    double L00 = nodeMatrix[j][i]->nodeLocMode;
    double L10 = nodeMatrix[j][i+1]->nodeLocMode;
    double L01 = nodeMatrix[j+1][i]->nodeLocMode;
    double L11 = nodeMatrix[j+1][i+1]->nodeLocMode;

    dCostX = gx00 + (gx10 - gx00)*a + (gx01 - gx00)*b + (gx11 + gx00 - gx10 - gx01)*a*b;
    dCostY = gy00 + (gy10 - gy00)*a + (gy01 - gy00)*b + (gy11 + gy00 - gy10 - gy01)*a*b;
    height = h00 + (h10 - h00)*a + (h10 - h00)*b + (h11 + h00 - h10 - h01)*a*b;

   // if (L00 == WHEEL_WALKING)
    //{
    //    std::cout<< "Suma: " << L00 + L10 + L01 + L11 << std::endl;
    //    std::cout<< "Division: " << (L00 + L10 + L01 + L11)/4.0+0.5 << std::endl;
    //    std::cout<< "Resultado: " << (unsigned int)((L00 + L10 + L01 + L11)/4.0+0.5) << std::endl; sleep(1);
    //}
    locMode = (int)((L00 + L10 + L01 + L11)/4.0); //PROVISIONAL, FIX THIS to properly interpolate locomotion mode
}
