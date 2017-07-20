#include "PathPlanning.hpp"
#include "NodeMap.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning()
{
    printf("Planner created successfully\n");
    this->narrowBand.clear();
}

PathPlanning::~PathPlanning()
{
}


//__LIST_OF_EXISTING_TERRAINS__

void PathPlanning::initTerrainList(std::vector< std::vector<double> > soils)
{
    for (uint i = 0; i < soils.size(); i++)
    {
	      terrainList.push_back(new soilType);
	      terrainList[i]->friction = soils[i][0];
        terrainList[i]->slip = soils[i][1];
    }
    std::cout<< "Terrain List Created" << std::endl;
    for (uint i = 0; i < terrainList.size(); i++)
    {
        std::cout<< "Terrain " << i << ":" << std::endl;
        std::cout<< " Friction = " << terrainList[i]->friction << std::endl;
        std::cout<< " Slip = " << terrainList[i]->slip << std::endl;
    }
}


//__COST_FUNCTION__

void PathPlanning::costFunction(uint Terrain, double& Power, locomotionMode& lM)
{
    // Here a Look-Up Table should be built
    switch(Terrain)
    {
	      case 0: Power = 10.0;   lM = DRIVING; break;
	      case 1: Power = 0.088; lM = DRIVING; break;
        case 2: Power = 0.088; lM = DRIVING; break;
	      //case 2: Power = 0.236; lM = WHEEL_WALKING; break;
	      //case 2: Power = 1.074; lM = DRIVING; break;
    }
}


//__FIELD_D_STAR_ALGORITHM__

void PathPlanning::fieldDStar(base::Waypoint wStart, base::Waypoint wGoal,
                              std::vector<base::Waypoint>& trajectory,
                              std::vector< short int >& locVector,
                              NodeMap * nodes)
{
  // Initialize Start and Target Nodes
    Node * nodeStart  = nodes->getNode((uint)(wStart.position[0] + 0.5),
                                       (uint)(wStart.position[1] + 0.5));
    Node * nodeGoal = nodes->getNode((uint)(wGoal.position[0] + 0.5),
                                     (uint)(wGoal.position[1] + 0.5));
    Node * nodeTarget = nodeGoal;
    std::cout<< "Goal at (" << nodeTarget->pose.position[0] << "," <<nodeTarget->pose.position[1] << ")" << std::endl;
    std::cout<< "Start at (" << nodeStart->pose.position[0] << "," <<nodeStart->pose.position[1] << ")" << std::endl;

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
        nodeTarget = openList.front();
        openList.pop_front();
        if (nodeTarget->g <= nodeTarget->rhs)
            nodeTarget->g = INF;
        else
        {
            nodeTarget->g = nodeTarget->rhs;
            updateState(nodeTarget, nodeGoal, wStart, openList);
        }
        for (uint i = 0; i<8; i++)
            if ((nodeTarget->nb8List[i] != NULL) &&
                (nodeTarget->nb8List[i]->terrain != 0))
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

void PathPlanning::getInterpolatedPath(Node * nodeStart, base::Waypoint wGoal,
	     NodeMap nodes, double tau,
 	     std::vector<base::Waypoint>& trajectory, std::vector< short int >& locVector)
{
    double newX, newY, di, dj, cost, heading;
    short int locMode;
    base::Waypoint wNew;
    Node * nodeTarget = nodeStart;

    wNew.position[0] = nodeTarget->pose.position[0];
    wNew.position[1] = nodeTarget->pose.position[1];

    trajectory.insert(trajectory.begin(), wNew);
    locVector.insert(locVector.begin(), locMode); //Fix this

    while(sqrt(pow((newX - wGoal.position[0]),2) +
             pow((newY - wGoal.position[1]),2)) > (2*tau))
    {
        computeCost(nodeTarget, di, dj, cost);
        wNew.position[0] = nodeTarget->pose.position[0] + di * nodes.scale;
        wNew.position[1] = nodeTarget->pose.position[1] + dj * nodes.scale;
        trajectory.insert(trajectory.begin(),wNew);
        locVector.insert(locVector.begin(), locMode);
        if (trajectory.size()>500)
	          break;
    }
    trajectory.insert(trajectory.begin(), wGoal);
    locVector.insert(locVector.begin(), locMode);
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
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj = dj + dk*(1-dj);
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
        case 1:
            switch(type)
            {
                case VERTICAL:
                    di = dk;
                    dj = 0;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
                case HORIZONTAL:
                    di = (1-di)*dk;
                    dj = 0;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
            }break;
        case 2:
            switch(type)
            {
                case VERTICAL:
                    di = dk;
                    dj = 0;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
                case HORIZONTAL:
                    di = (1-di)*dk;
                    dj = 0;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[2];
                    break;
            }break;
        case 3:
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj += dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
        case 4:
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj += dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
        case 5:
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj += dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
        case 6:
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj += dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
        case 7:
            switch(type)
            {
                case VERTICAL:
                    di = 0;
                    dj += dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case HORIZONTAL:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
                case NON_INTERPOLATED:
                    di = 0;
                    dj = dk;
                    nodeTarget = nodeTarget->nb8List[0];
                    break;
            }break;
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

void PathPlanning::fastMarching(base::Waypoint wStart, base::Waypoint wGoal,
                                NodeMap * nodes)
{

  // Initialize nodeStart, nodeGoal and nodeTarget
    nodes->setActualPos(wStart);
    nodes->setGoal(wGoal);

  // Initializing the Narrow Band
    initNarrowBand(nodes, wGoal);

    Node * nodeTarget;

    /*std::cout << "Fast Marching: propagation will start from node (" <<
        nodeTarget->pose.position[0] << "," <<
        nodeTarget->pose.position[1] << ")" << std::endl;*/

    Node * nodeFinal = nodes->getActualPos();

    if (nodeFinal->terrain == 0)
    {
        std::cout<< "ALERT: entering obstacle area" << std::endl;
        nodeFinal->state = OPEN;
    }

    /*std::cout << "Fast Marching: propagation will finish at node (" <<
        nodeFinal->pose.position[0] << "," <<
        nodeFinal->pose.position[1] << ")" << std::endl;*/

  // Propagation Loop
    std::cout<< "Fast Marching: starting propagation loop" << std::endl;
    t1 = base::Time::now();
    while (nodeFinal->state != CLOSED)
    {
        nodeTarget = minCostNode();
        nodeTarget->state = CLOSED;
        for (uint i = 0; i<4; i++)
            if ((nodeTarget->nb4List[i] != NULL) &&
                (nodeTarget->nb4List[i]->state == OPEN))
                propagationFunction(nodeTarget->nb4List[i], nodes->scale);
    }
    std::cout<< "Fast Marching: ended propagation loop" << std::endl;
    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;
    std::cout<< "Fast Marching: goal cost is " << nodeFinal->work << std::endl;

}


void PathPlanning::initNarrowBand(NodeMap * nodes, base::Waypoint wGoal)
{
    std::cout << "Initializing NarrowBand" << std::endl;
    Node * nodeGoal = nodes->getGoal();

    nodes->resetPropagation();
    narrowBand.clear();

    if(nodeGoal->state != HIDDEN)
    {
        nodeGoal->work = 0;
        narrowBand.push_back(nodeGoal);
    }
    uint sizeI = nodes->nodeMatrix.size();
    uint sizeJ = nodes->nodeMatrix[0].size();

    // HORIZON NODES
    for (uint j = 0; j < sizeJ; j++)
    {
        for (uint i = 0; i < sizeI; i++)
        {
            if(nodes->nodeMatrix[j][i]->state == OPEN)
                for (uint k = 0; k < 4; k++)
                    if((nodes->nodeMatrix[j][i]->nb4List[k] != NULL) &&
                       (nodes->nodeMatrix[j][i]->nb4List[k]->state == HIDDEN))
                    {
                        getHorizonCost(nodes, nodes->nodeMatrix[j][i], wGoal);
                        narrowBand.push_back(nodes->nodeMatrix[j][i]);
                        k = 4;
                    }
        }
    }

    /*if(nodeGoal->state == CLOSED)
    {
        if (nodeGoal->work != 0)
        {
            nodes->resetPropagation();
            nodeGoal->work = 0;
            narrowBand.clear();
            narrowBand.push_back(nodeTarget);
        }
    }
    else
    {
         if (nodeGoal->state == OBSTACLE)
         {
             std::cout << "ERROR: GOAL placed at obstacle" << std::endl;
             nodes->resetPropagation();
             nodeGoal->work = 0;
             narrowBand.clear();
             narrowBand.push_back(nodeGoal); //TODO: FIX THIS
         }
         else
         {
             nodes->resetPropagation();
             nodeGoal->work = 0;
             narrowBand.clear();
             narrowBand.push_back(nodeGoal);
         }
    }*/
}


void PathPlanning::getHorizonCost(NodeMap* nodes, Node* horizonNode, base::Waypoint wGoal)
{
    double x = horizonNode->pose.position[0]*nodes->scale +
               nodes->globalOffset.position[0];
    double y = horizonNode->pose.position[1]*nodes->scale +
               nodes->globalOffset.position[1];
    double d = sqrt(pow(x-wGoal.position[0],2) + pow(y-wGoal.position[1],2));
    double P;
    locomotionMode L;
    costFunction(1,P,L);
    horizonNode->work = 2 * P * d;
}

//__ISOTROPIC_PROPAGATION_FUNCTION_USING_EIKONAL_EQUATION__

void PathPlanning::propagationFunction(Node* nodeTarget, double scale)
{
  // (W-Wx)^2 + (W-Wy)^2 = (P+R)^2
    double Wx,Wy,P,W,R;
    locomotionMode L;
  // Neighbor Propagators Wx and Wy
    if(((nodeTarget->nb4List[0] != NULL))&&((nodeTarget->nb4List[3] != NULL)))
        Wy = fmin(nodeTarget->nb4List[3]->work, nodeTarget->nb4List[0]->work);
    else if (nodeTarget->nb4List[0] == NULL)
        Wy = nodeTarget->nb4List[3]->work;
    else
        Wy = nodeTarget->nb4List[0]->work;

    if(((nodeTarget->nb4List[1] != NULL))&&((nodeTarget->nb4List[2] != NULL)))
        Wx = fmin(nodeTarget->nb4List[1]->work, nodeTarget->nb4List[2]->work);
    else if (nodeTarget->nb4List[1] == NULL)
        Wx = nodeTarget->nb4List[2]->work;
    else
        Wx = nodeTarget->nb4List[1]->work;

  //Cost Function to obtain optimal power and locomotion mode
    costFunction(nodeTarget->terrain,P,L);

    R = nodeTarget->risk.obstacle;

    if ((fabs(Wx-Wy)<(scale*P*(1+R)))&&(Wx < INF)&&(Wy < INF))
        W = (Wx+Wy+sqrt(2*pow(scale*P*(1+R),2.0) - pow((Wx-Wy),2.0)))/2;
    else
        W = fmin(Wx,Wy) + scale*P*(1+R);

    /*if (nodeTarget->state == NARROW)
        if(W < nodeTarget->work)
        {
            nodeTarget->work = W;
            nodeTarget->nodeLocMode = L;
	    nodeTarget->power = P;
        }

    if (nodeTarget->state == FAR)
    {
        nodeTarget->state = NARROW;
        nodeTarget->work = W;
        nodeTarget->nodeLocMode = L;
        nodeTarget->power = P;
        this->narrowBand.push_back(nodeTarget);
    }*/

    if(W < nodeTarget->work)
    {
        if (nodeTarget->work == INF) //It is not in narrowband
            this->narrowBand.push_back(nodeTarget);
        nodeTarget->work = W;
        nodeTarget->nodeLocMode = L;
        nodeTarget->power = P;
    }
}


//__GRADIENT_DESCENT_METHOD__

void PathPlanning::getPath(NodeMap * nodes, double tau,
 					     std::vector<base::Waypoint>& trajectory,
               std::vector< short int >& locVector)
{
    double newX, newY, newH, dCostX, dCostY, heading, cX, cY, normDCostX, normDCostY, distance = 0;
    short int locMode;
    base::Waypoint sinkPoint;
    base::Waypoint startPoint;
    if (nodes->getActualPos()->work == 0)
    {
        sinkPoint.position[0] = nodes->getActualPos()->pose.position[0];
        sinkPoint.position[1] = nodes->getActualPos()->pose.position[1];
        startPoint = nodes->goalPose;
    }
    else
    {
        sinkPoint.position[0] = nodes->getGoal()->pose.position[0];
        sinkPoint.position[1] = nodes->getGoal()->pose.position[1];
        startPoint = nodes->actualPose;
    }

    cX = startPoint.position[0]; //Position to take into account when calculating heading for Wheel-walking waypoints
    cY = startPoint.position[1]; //Position to take into account when calculating heading for Wheel-walking waypoints

    base::Waypoint wNew;
    Node* nextNode;
    calculateNextWaypoint(startPoint.position[0], startPoint.position[1], dCostX, dCostY, newH, locMode, nodes);
    startPoint.position[2] = newH;
    //trajectory.insert(trajectory.begin(), startPoint);
    trajectory.push_back(startPoint);
    //locVector.insert(locVector.begin(), locMode);
    locVector.push_back(locMode);
    //std::cout<< "Loc Mode inicial: "  << locVector.front() << std::endl;
    newX = trajectory.back().position[0];
    newY = trajectory.back().position[1];
    while(sqrt(pow((trajectory.back().position[0] - sinkPoint.position[0]),2) +
             pow((trajectory.back().position[1] - sinkPoint.position[1]),2)) > (2*tau))
    {
        /*if (sqrt(pow(dCostX,2)+pow(dCostY,2)) < 0.9)
             std::cout<< "ERROR: " << sqrt(pow(dCostX,2)+pow(dCostY,2)) << std::endl;*/

        normDCostX = dCostX;///sqrt(pow(dCostX,2) + pow(dCostY,2));
        normDCostY = dCostY;///sqrt(pow(dCostX,2) + pow(dCostY,2));
        newX = newX - tau*normDCostX;
        newY = newY - tau*normDCostY;

        nextNode = calculateNextWaypoint(newX, newY, dCostX, dCostY, newH, locMode, nodes);
        if (nextNode != NULL)
            wNew.position = Eigen::Vector3d(nextNode->pose.position[0],nextNode->pose.position[1],newH);
        else
            wNew.position = Eigen::Vector3d(newX,newY,newH);

        /*if ((locMode == 0)&&(locVector.back() == 0)) //Driving Mode
            wNew.heading = atan2(-dCostY,-dCostX);

        if ((locMode == 1)&&(locVector.back() == 1)) //Wheel-Walking Mode
            wNew.heading = atan2(cY-newY, cX-newX);

        if ((locMode == 0)&&(locVector.back() == 1)) //Driving to Wheel-Walking
            wNew.heading = atan2(cY-newY, cX-newX);

        if ((locMode == 1)&&(locVector.back() == 0)) //Wheel-walking to Driving
        {
            cX = newX;
            cY = newY;
            wNew.heading = atan2(-dCostY,-dCostX);
        }*/

        wNew.heading = atan2(-dCostY,-dCostX);

        distance += sqrt(pow((newX - trajectory.back().position[0]),2) +
                 pow((newY - trajectory.back().position[1]),2))*nodes->scale;
        trajectory.push_back(wNew);
        locVector.push_back(locMode);
	      if (distance > 0.8)
	          break;
        if (trajectory.size()>999)
        {
            std::cout<< "ERROR at node (" << newX << "," << newY <<")"<< std::endl;
            break;
        }
    }
    /*interpolateWaypoint(sinkPoint.position[0], sinkPoint.position[1], dCostX, dCostY, newH, locMode, nodes);
    sinkPoint.position[2] = newH;
    trajectory.push_back(sinkPoint);
    locVector.push_back(locMode);*/

  //Rescaling resulting trajectory
    for (uint i = 0; i<trajectory.size(); i++)
    {
        trajectory[i].position[0] = trajectory[i].position[0]*nodes->scale;
        trajectory[i].position[1] = trajectory[i].position[1]*nodes->scale;
    }
}


Node* PathPlanning::minCostNode()
{
    Node* nodePointer = this->narrowBand.front();
    uint index = 0;
    uint i;
    double minCost = this->narrowBand.front()->work;
    //std::cout << "Size of Narrow Band is: " << this->narrowBand.size() << std::endl;
    for (i =0; i < this->narrowBand.size(); i++)
    {
        if (this->narrowBand[i]->work < minCost)
        {
            minCost = this->narrowBand[i]->work;
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
      if ((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))
          dx = 0;
      else
      {
          if (nodeTarget->nb4List[1] == NULL)
              dx = nodeTarget->nb4List[2]->work - nodeTarget->work;
          else
          {
              if (nodeTarget->nb4List[2] == NULL)
                  dx = nodeTarget->work - nodeTarget->nb4List[1]->work;
              else
                  dx = (nodeTarget->nb4List[2]->work -
                        nodeTarget->nb4List[1]->work)*0.5;
          }
      }
      if ((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))
          dy = 0;
      else
      {
          if (nodeTarget->nb4List[0] == NULL)
              dy = nodeTarget->nb4List[3]->work - nodeTarget->work;
          else
          {
              if (nodeTarget->nb4List[3] == NULL)
                  dy = nodeTarget->work - nodeTarget->nb4List[0]->work;
              else
                  dy = (nodeTarget->nb4List[3]->work -
                        nodeTarget->nb4List[0]->work)*0.5;
          }
      }
      dnx = dx/sqrt(pow(dx,2)+pow(dy,2));
      dny = dy/sqrt(pow(dx,2)+pow(dy,2));
    //}
}

//__INTERPOLATION_ON_WAYPOINT__

Node* PathPlanning::calculateNextWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode, NodeMap * nodes)
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


    /*if ((node00->state == OBSTACLE)||(node01->state == OBSTACLE)||
        (node10->state == OBSTACLE)||(node11->state == OBSTACLE))
    {
        Node* minNode = node00;
        if (node10->work < minNode->work)
            minNode = node10;
        if (node01->work < minNode->work)
            minNode = node01;
        if (node11->work < minNode->work)
            minNode = node11;
        dCostX = x-minNode->pose.position[0];
        dCostY = y-minNode->pose.position[1];
        height = minNode->elevation;
        locMode = minNode->nodeLocMode;
        return minNode;
    }
    else
    {*/
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
        return NULL;
    //}
}
