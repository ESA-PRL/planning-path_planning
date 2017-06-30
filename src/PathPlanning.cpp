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

NodeMap::NodeMap(double size, base::Pose2D pos,
                std::vector< std::vector<double> > elevation,
		std::vector< std::vector<double> > cost,
		std::vector< std::vector<double> > risk)
{
    nodeSize = size;
    globalOriginPose = pos;
    std::vector<Node*> nodeRow;
    for (uint j = 0; j < elevation[0].size(); j++)
    {
        for (uint i = 0; i < elevation.size(); i++)
            nodeRow.push_back(new Node(i, j, elevation[j][i], cost[j][i], risk[j][i]));
        nodeMatrix.push_back(nodeRow);
        nodeRow.clear();
    }
    for (uint j = 0; j < elevation[0].size(); j++)
    {
        for (uint i = 0; i < elevation.size(); i++)
        {
            nodeMatrix[j][i]->nb4List.clear();
            nodeMatrix[j][i]->nb4List.push_back(getNode(i,j-1));
            nodeMatrix[j][i]->nb4List.push_back(getNode(i-1,j));
            nodeMatrix[j][i]->nb4List.push_back(getNode(i+1,j));
            nodeMatrix[j][i]->nb4List.push_back(getNode(i,j+1));

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

Node* NodeMap::getNode(uint i, uint j)
        {
            Node* nodeTarget;
            if ((i >= nodeMatrix[0].size())||(j >= nodeMatrix.size()))
            {
                return NULL;
            }
            else
            {
                nodeTarget = nodeMatrix[j][i];
            }
            if (nodeTarget->state != OBSTACLE)
            {
                return nodeTarget;
            }
            else
            {
                return NULL;
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


//__GRADIENT_DESCENT_METHOD__

void PathPlanning::gradientDescentTrajectory(base::Waypoint wStart, base::Waypoint wGoal,
					     NodeMap * nodes, double tau,
 					     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector)
{
    double newX, newY, newH, dCostX, dCostY, heading, cX, cY;
    short int locMode;

    cX = wGoal.position[0]; //Position to take into account when calculating heading for Wheel-walking waypoints
    cY = wGoal.position[1]; //Position to take into account when calculating heading for Wheel-walking waypoints

    base::Waypoint wNew;
    std::cout<< "Fast Marching: field gradient calculated" << std::endl;
    interpolateWaypoint(wGoal.position[0], wGoal.position[1], dCostX, dCostY, newH, locMode, nodes);
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

        interpolateWaypoint(newX, newY, dCostX, dCostY, newH, locMode, nodes);
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
    interpolateWaypoint(wStart.position[0], wStart.position[1], dCostX, dCostY, newH, locMode, nodes);
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
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Ty = !(nodeTarget->nb4List[3]->work < nodeTarget->nb4List[0]->work)?nodeTarget->nb4List[0]->work:nodeTarget->nb4List[3]->work;
    else if (nodeTarget->nb4List[0] == NULL)
        Ty = nodeTarget->nb4List[3]->work;
    else
        Ty = nodeTarget->nb4List[0]->work;

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Tx = !(nodeTarget->nb4List[2]->work < nodeTarget->nb4List[1]->work)?nodeTarget->nb4List[1]->work:nodeTarget->nb4List[2]->work;
    else if (nodeTarget->nb4List[1] == NULL)
        Tx = nodeTarget->nb4List[2]->work;
    else
        Tx = nodeTarget->nb4List[1]->work;

    costFunction(nodeTarget->terrain,P,L);

    P = P + nodeTarget->risk.obstacle;

    if ((fabs(Tx-Ty)<P)&&(Tx < INF)&&(Ty < INF)){
        W = (Tx+Ty+sqrt(2*pow(P,2.0) - pow((Tx-Ty),2.0)))/2;
    }else{
        W = (!(Tx<Ty)?Ty:Tx) + P;
    }

    if (nodeTarget->state == NARROW)
        if(W < nodeTarget->work)
        {
            //setPropagation(nodeTarget,W);
            nodeTarget->work = W;
            nodeTarget->nodeLocMode = L;
	    nodeTarget->power = P;
        }

    if (nodeTarget->state == OPEN)
    {
        nodeTarget->state = NARROW;
        nodeTarget->work = W;
        nodeTarget->nodeLocMode = L;
        nodeTarget->power = P;
        narrowBand.push_back(nodeTarget);
    }
    //std::cout<< "Propagating at node (" << nodeTarget->pose.position[0] << "," <<nodeTarget->pose.position[1] << ")" << std::endl;
    //std::cout<< " - P: " << nodeTarget->power << std::endl;
    //std::cout<< " - W: " << nodeTarget->work << std::endl;
}


//__FIELD_D_STAR_ALGORITHM__

void PathPlanning::fieldDStar(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector, NodeMap * nodes)
{
  // Initialize Start and Target Nodes

    Node * nodeStart  = nodes->getNode((uint)(wStart.position[0] + 0.5), (uint)(wStart.position[1] + 0.5));
    Node * nodeGoal = nodes->getNode((uint)(wGoal.position[0] + 0.5), (uint)(wGoal.position[1] + 0.5));
    Node * nodeTarget = nodeGoal;
    std::cout<< "Goal at (" << nodeTarget->pose.position[0] << "," <<nodeTarget->pose.position[1] << ")" << std::endl;
    std::cout<< "Start at (" << nodeStart->pose.position[0] << "," <<nodeStart->pose.position[1] << ")" << std::endl;

  // Initialize the G and RHS values
    //initializeGRHS();

  // Initialize the List of Open Nodes
    std::list<Node*> openList;


  // Insert nodeTarget (the goal in this case) into the Open List
    openList.push_back(nodeTarget);
    std::cout<< "Open List Initialized" << std::endl;
    setKey(nodeTarget, wStart);
    std::cout<< "Key set on nodeTarget (goal)" << std::endl;
    t1 = base::Time::now();
  // Compute the Shortest Path
    while ((openList.front() == getMinorKey(openList.front(), nodeStart))||(nodeStart->rhs != nodeStart->g))
    {
        //std::cout<< "Inside while loop" << std::endl;
        nodeTarget = openList.front();
        //std::cout<< "Node Target: " << nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
        openList.pop_front();
        //std::cout<< "Open List Size: " << openList.size() << std::endl;
        if (nodeTarget->g <= nodeTarget->rhs)
        {
            nodeTarget->g = std::numeric_limits<double>::max();
        }
        else
        {
            nodeTarget->g = nodeTarget->rhs;
            updateState(nodeTarget, nodeGoal, wStart, openList);
        }
        for (uint i = 0; i<8; i++)
            if ((nodeTarget->nb8List[i] != NULL) &&
                (nodeTarget->nb8List[i]->state != OBSTACLE))
            {
                //std::cout<< "Updating neighbor " << i << " of node "<< nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
                updateState(nodeTarget->nb8List[i], nodeGoal, wStart, openList);
            }
    }
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
    std::cout<< "Field D*: ended computing shortest paths" << std::endl;
    //getPath(nodeStart, wGoal, propagationMatrix, 0.8, trajectory, locVector);
}


//__GET_PATH_FROM_FIELD_D_STAR_COMPUTATION__

void getPath(Node * nodeStart, base::Waypoint wGoal,
	     NodeMap nodes, double tau,
 	     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector)
{
    double newX, newY;

    newX = nodeStart->pose.position[0];
    newY = nodeStart->pose.position[1];

    while(sqrt(pow((newX - wGoal.position[0]),2) +
             pow((newY - wGoal.position[1]),2)) > (2*tau))
    {
        //getWaypoint(newX,newY,nodes);
    }
}

//__GET_WAYPOINT_INTERPOLATING_

void getWaypoint(double& x, double& y, NodeMap nodes)
{
    double dx = x - round(x);
    double dy = y - round(y);

    if ((fabs(dx)<0.05)&&(fabs(dy)<0.05))
    {
        x = round(x);
        y = round(y);

    }
    else
    {

    }
}


//__SET_KEY_VALUE_TO_NODE__

void PathPlanning::setKey(Node * nodeTarget, base::Waypoint wStart)
{
    nodeTarget->key[0] = fmin(nodeTarget->g, nodeTarget->rhs + getHeuristic(nodeTarget, wStart));
    nodeTarget->key[1] = fmin(nodeTarget->g, nodeTarget->rhs);
}


//__GET_HEURISTIC_VALUE__

double PathPlanning::getHeuristic(Node * nodeTarget, base::Waypoint wStart)
{
    double weight = 0.05; // TODO: THIS MUST BE A CONFIGURABLE VALUE
    return weight*sqrt(pow(nodeTarget->pose.position[0] - wStart.position[0],2) + pow(nodeTarget->pose.position[1] - wStart.position[1],2));
}



Node * PathPlanning::getMinorKey( Node * nodeA, Node * nodeB)
{

    if (nodeA->key[0] < nodeB->key[0])
    {
        return nodeA;
    }
    else
    {
        if (nodeA->key[0] > nodeB->key[0])
        {
            return nodeB;
        }
        else
        {
            if (nodeA->key[1] < nodeB->key[1])
            {
                return nodeA;
            }
            else
            {
                return nodeB;
            }
        }
    }
}


void PathPlanning::updateState(Node * nodeTarget, Node * nodeGoal, base::Waypoint wStart, std::list<Node*>& openList)
{
    //std::cout<< " - Updating State of node "<< nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << std::endl;
    if (nodeTarget != nodeGoal)
    {
        double cost, di = 0.0, dj = 0.0;
        computeCost(nodeTarget, di, dj, cost);
        nodeTarget->rhs = cost;
        //std::cout<< " - Computed cost of node "<< nodeTarget->pose.position[0] << " " << nodeTarget->pose.position[1] << " with cost " << nodeTarget->rhs << std::endl;
    }
    openList.remove(nodeTarget);
    //std::cout<< " - openList with size:  "<< openList.size() << std::endl;
    if (nodeTarget->g != nodeTarget->rhs)
    {
        setKey(nodeTarget, wStart);
        //std::cout<< "key set" << std::endl;

        //std::cout<< "key set" << std::endl;
        //std::cout<< "It -> " << (*it)->pose.position[0] << " " << (*it)->pose.position[1] << std::endl;
        if (openList.size() == 0)
        {
            openList.push_back(nodeTarget);
        }
        else
        {
            std::list<Node*>::iterator it = openList.begin();
            //std::cout<< " - It -> " << (*it)->pose.position[0] << " " << (*it)->pose.position[1] << std::endl;
            while(it != openList.end())
            {
	    if (nodeTarget == getMinorKey(nodeTarget, (*it)))
            {
                openList.insert(it, nodeTarget);
                it = openList.end();
                //std::cout<< " - Inserted" << std::endl;
            }
            else
            {
	        std::advance(it,1);
            }
            //std::cout<< " - Iterating" << std::endl;
            }
        //std::cout<< " - After Iterating" << std::endl;
        //std::cout<< " - It -> " << (*it)->pose.position[0] << " " << (*it)->pose.position[1] << std::endl;
            openList.push_back(nodeTarget);
        }
    }
}


//__COMPUTE_COST_OF_NODE__

void PathPlanning::computeCost(Node * nodeTarget, double& di, double& dj, double& cost)
{
  // We have the nodeTarget as noninterpolated node of reference
  // If di > 0 we are computing an horizontal interpolated node
  // If dj > 0 we are computing a vertical interpolated node
  // Otherwise is a noninterpolated node (the reference one)
  // As result we obtain:
  // - The cost of the interpolated resulting node
  // - Maybe a new noninterpolated node as reference, in this case
  //   di and dj are as result the distance to this new reference

    double actualCost, dk = 0, actualdk, g[8];
    unsigned int c[4], optimalEdge = 0;
    nodeInterpType type;
    cost = INF;

    /*if (di < 0.01)
        di = 0;
    if (dj < 0.01)
        dj = 0;
    if (di > 0.99)
        di = 1;
    if (dj > 0.99)
        dj = 1;*/

    if ((di > 0)&&(dj > 0));
        //ERROR

    if (di > 0)
    {
        g[0] = nodeTarget->nb8List[0]->g;
        g[1] = nodeTarget->nb8List[1]->g;
        g[3] = nodeTarget->nb8List[2]->g;
        g[4] = nodeTarget->g;
        g[5] = nodeTarget->nb8List[6]->g;
        g[7] = nodeTarget->nb8List[7]->g;
        g[2] = g[3] + (g[1]-g[3])*di;
        g[6] = g[5] + (g[7]-g[5])*di;
        c[0] = nodeTarget->terrain;
        c[1] = nodeTarget->terrain;
        c[2] = nodeTarget->nb8List[5]->terrain;
        c[3] = nodeTarget->nb8List[5]->terrain;
        type = HORIZONTAL;
    }
    else
    {
        if (dj > 0)
        {
            g[1] = nodeTarget->nb8List[1]->g;
            g[2] = nodeTarget->nb8List[2]->g;
            g[3] = nodeTarget->nb8List[3]->g;
            g[5] = nodeTarget->nb8List[4]->g;
            g[6] = nodeTarget->g;
            g[7] = nodeTarget->nb8List[0]->g;
            g[0] = g[7] + (g[1]-g[7])*dj;
            g[4] = g[5] + (g[3]-g[5])*dj;
            c[0] = nodeTarget->terrain;
            c[1] = nodeTarget->nb8List[4]->terrain;
            c[2] = nodeTarget->nb8List[4]->terrain;
            c[3] = nodeTarget->terrain;
            type = VERTICAL;
        }
        else
        {
            g[0] = nodeTarget->nb8List[0]->g;
            g[1] = nodeTarget->nb8List[1]->g;
            g[2] = nodeTarget->nb8List[2]->g;
            g[3] = nodeTarget->nb8List[3]->g;
            g[4] = nodeTarget->nb8List[4]->g;
            g[5] = nodeTarget->nb8List[5]->g;
            g[6] = nodeTarget->nb8List[6]->g;
            g[7] = nodeTarget->nb8List[7]->g;
            c[0] = nodeTarget->terrain;
            c[1] = nodeTarget->nb8List[4]->terrain;
            c[2] = nodeTarget->nb8List[5]->terrain;
            c[3] = nodeTarget->nb8List[6]->terrain;
            type = NON_INTERPOLATED;
        }
    }

    for (uint edge = 0; edge < 8; edge++)
    {
        switch(edge)
        {
            case 0:
                costInterpolation(g[0], g[1], c[0], c[3],
                                  1-di, 1-dj, actualdk, actualCost);
                break;
            case 1:
                costInterpolation(g[2], g[1], c[0], c[1],
                                  1-dj, 1-di, actualdk, actualCost);
                break;
            case 2:
                costInterpolation(g[2], g[3], c[1], c[0],
                                  1-dj, (type==HORIZONTAL)?di:1, actualdk, actualCost);
                break;
            case 3:
                costInterpolation(g[4], g[3], c[1], c[2],
                                  (type==HORIZONTAL)?di:1, 1-dj, actualdk, actualCost);
                break;
            case 4:
                costInterpolation(g[4], g[5], c[2], c[1],
                                  (type==HORIZONTAL)?di:1, (type==VERTICAL)?dj:1, actualdk, actualCost);
                break;
            case 5:
                costInterpolation(g[6], g[5], c[2], c[3],
                                  (type==VERTICAL)?dj:1, (type==HORIZONTAL)?di:1, actualdk, actualCost);
                break;
            case 6:
                costInterpolation(g[6], g[7], c[3], c[2],
                                  (type==VERTICAL)?dj:1, 1-di, actualdk, actualCost);
                break;
            case 7:
                costInterpolation(g[0], g[7], c[3], c[0],
                                  1-di, (type==VERTICAL)?dj:1, actualdk, actualCost);
                break;
        }

        if (actualCost < cost)
        {
	    cost = actualCost;
            dk = actualdk;
            optimalEdge = edge;
        }
    }

    /*if (di < 0.01)
        di = 0;
    if (dj < 0.01)
        dj = 0;
    if (di > 0.99)
        di = 1;
    if (dj > 0.99)
        dj = 1;*/

    switch(optimalEdge)
    {
        case 0:
            di = 1-di;
            dj = dk; //ACTUALIZAR TAMBIÉN QUIÉN ES EL SIGUIENTE NODE TARGET!!!
            break;
        case 1:
            di = (1-di)*dk;
            dj = 1;
            break;
        case 2:
            di = -((di == 0)?1:di)*dk;
            dj = 1;
            break;
        case 3:
            di = -((di == 0)?1:di);
            dj = dk;
            break;
        case 4:
            di = -((di == 0)?1:di);
            dj = -dk;
            break;
        case 5:
            di = -((di == 0)?1:di)*dk;
            dj = -1;
            break;
        case 6:
            di = (1-di)*dk;
            dj = -1;
            break;
        case 7:
            di = (1-di);
            dj = -dk;
            break;
    }
}

void PathPlanning::costInterpolation(double g1, double g2, uint cTerrain, uint bTerrain, double di, double dj, double& dk, double& vs)
{
    // g2 is the diagonal neighbor
    // di and dj are the horizontal and vertical distance of diagonal neighbor respect to reference
    // dk is the vertical displacement (see paper...)
    locomotionMode cLoc, bLoc;
    double c,b;
    costFunction(cTerrain, c, cLoc);
    costFunction(bTerrain, b, bLoc);

    dk = 0;
    if (fmin(c,b) == INF)
        vs = INF;
    else
    {
        if (g1 <= g2)
            vs = fmin(c,b) + g1;
        else
        {
            double f = (g1 - g2)/dj;
            if(f <= b)
            {
                if(c <= f)
                {
                    vs = c*sqrt(pow(di,2) + pow(dj,2)) + g2;
                    dk = dj;
                }
                else
                {
                    dk = fmin(di*f/sqrt(pow(c,2) - pow(f,2)),dj);
                    vs = c*sqrt(pow(di,2) + pow(dk,2)) + f*(dj-dk) + g2;
                }
            }
            else
            {
                if(c <= b)
                {
                    vs = c*sqrt(pow(di,2) + pow(dj,2)) + g2;
                    dk = dj;
                }
                else
                {
                    dk = dj;
                    double dl = di - fmin(di*b/sqrt(pow(c,2) - pow(b,2)),di);
                    vs = c*sqrt(pow(dj,2) + pow(di-dl,2)) + b*dl + g2;
                }

            }
        }
    }
}


//__FAST_MARCHING_ALGORITHM__

void PathPlanning::fastMarching(base::Waypoint wStart, base::Waypoint wGoal, std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector, NodeMap * nodes)
{

  // Initialize nodeStart, nodeGoal and nodeTarget

    Node * nodeStart  = nodes->getNode((uint)(wStart.position[0] + 0.5), (uint)(wStart.position[1] + 0.5));
    Node * nodeGoal = nodes->nodeMatrix[(uint)(wGoal.position[1] + 0.5)][(uint)(wGoal.position[0] + 0.5)];
    Node * nodeTarget = nodeStart;
    std::cout<< "Goal at (" << nodeGoal->pose.position[0] << "," << nodeGoal->pose.position[1] << ")" << std::endl;
    std::cout<< "Start at (" << nodeStart->pose.position[0] << "," << nodeStart->pose.position[1] << ")" << std::endl;

  // Initializing the Narrow Band
    std::vector<Node*> narrowBand;
    narrowBand.push_back(nodeTarget);

  // Initializing the Propagation Matrix
    /*propagationMatrix.resize(nodeMatrix.size());
    for (uint j = 0; j < propagationMatrix.size(); j++)
        propagationMatrix[j].resize(nodeMatrix[j].size());

    for (uint j = 0; j < propagationMatrix.size(); j++)
        for (uint i = 0; i < propagationMatrix[j].size(); i++)
            propagationMatrix[j][i] = INF;
    setPropagation(nodeTarget,0);*/

    nodeTarget->work = 0;

    Node * node00 = nodes->getNode((uint)(wStart.position[0] + 0.5), (uint)(wStart.position[1] + 0.5));
    Node * node10 = node00->nb4List[2];
    Node * node01 = node00->nb4List[3];
    Node * node11 = node00->nb4List[2]->nb4List[3];
    std::cout<< "Node00 (" << node00->pose.position[0] << "," <<node00->pose.position[1] << ")" << std::endl;
    std::cout<< "Node10 (" << node10->pose.position[0] << "," <<node10->pose.position[1] << ")" << std::endl;
    std::cout<< "Node01 (" << node01->pose.position[0] << "," <<node01->pose.position[1] << ")" << std::endl;
    std::cout<< "Node11 (" << node11->pose.position[0] << "," <<node11->pose.position[1] << ")" << std::endl;
    

  // Propagation Loop
    t1 = base::Time::now();
    //while (narrowBand.size() > 0)
    while (nodeGoal->state != FROZEN)
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
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
    std::cout<< "Fast Marching: goal cost is " << nodeGoal->work << std::endl;
    gradientDescentTrajectory(wStart, wGoal, nodes, 0.8, trajectory, locVector);
    
    
}


//__COST_FUNCTION__

void PathPlanning::costFunction(uint Terrain, double& Power, locomotionMode& lM)
{
    // Here a Look-Up Table should be built

    switch(Terrain)
    {
	case 0: Power = 4.0;   lM = DRIVING; break;
	case 1: Power = 0.088; lM = DRIVING; break;
	case 2: Power = 0.236; lM = WHEEL_WALKING; break;
	//case 2: Power = 1.074; lM = WHEEL_WALKING; break;
    }
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

void PathPlanning::gradientNode(Node* nodeTarget, double& dx, double& dy)
{
    if ((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))
    {
        dx = 0;
    }
    else
    {
        if (nodeTarget->nb4List[1] == NULL)
        {
            dx = nodeTarget->nb4List[2]->work - nodeTarget->work;
        }
        else
        {
            if (nodeTarget->nb4List[2] == NULL)
            {
                dx = nodeTarget->work - nodeTarget->nb4List[1]->work;
            }
            else
            {
                dx = (nodeTarget->nb4List[2]->work - nodeTarget->nb4List[1]->work)*0.5;
            }
        }
    }

    if ((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))
    {
        dy = 0;
    }
    else
    {
        if (nodeTarget->nb4List[0] == NULL)
        {
            dy = nodeTarget->nb4List[3]->work - nodeTarget->work;
        }
        else
        {
            if (nodeTarget->nb4List[3] == NULL)
            {
                dy = nodeTarget->work - nodeTarget->nb4List[0]->work;
            }
            else
            {
                dy = (nodeTarget->nb4List[3]->work - nodeTarget->nb4List[0]->work)*0.5;
            }
        }
    }

    dx = dx/sqrt(pow(dx,2)+pow(dy,2));
    dy = dy/sqrt(pow(dx,2)+pow(dy,2));
}

//__INTERPOLATION_ON_WAYPOINT__

void PathPlanning::interpolateWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode, NodeMap * nodes)
{
    //uint i = (uint)(x+0.5);
    //uint j = (uint)(y+0.5);
    //double a = x - (double)((uint)x);
    //double b = y - (double)((uint)y);

    uint i = (uint)(x);
    uint j = (uint)(y);
    double a = x - (double)(i);
    double b = y - (double)(j);

    double gx00, gx10, gx01, gx11;
    double gy00, gy10, gy01, gy11;

    Node * node00 = (nodes->getNode(i, j));
    std::cout<< "i: " << i << "j: " << j << std::endl;
    Node * node10 = node00->nb4List[2];
    Node * node01 = node00->nb4List[3];
    Node * node11 = node00->nb4List[2]->nb4List[3];

    std::cout<< "Node00 (" << node00->pose.position[0] << "," <<node00->pose.position[1] << ")" << std::endl;
    std::cout<< "Node10 (" << node10->pose.position[0] << "," <<node10->pose.position[1] << ")" << std::endl;
    std::cout<< "Node01 (" << node01->pose.position[0] << "," <<node01->pose.position[1] << ")" << std::endl;
    std::cout<< "Node11 (" << node11->pose.position[0] << "," <<node11->pose.position[1] << ")" << std::endl;

    //sleep(2);

    gradientNode( node00, gx00, gy00);
    gradientNode( node10, gx10, gy10);
    gradientNode( node01, gx01, gy01);
    gradientNode( node11, gx11, gy11);

    double h00 = node00->elevation;
    double h10 = node10->elevation;
    double h01 = node01->elevation;
    double h11 = node11->elevation;

    double L00 = node00->nodeLocMode;
    double L10 = node10->nodeLocMode;
    double L01 = node01->nodeLocMode;
    double L11 = node11->nodeLocMode;

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
