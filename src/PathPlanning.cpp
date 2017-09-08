#include "PathPlanning.hpp"
#include "NodeMap.hpp"
#include <stdlib.h>
#include <stdio.h>
#include <math.h>


using namespace PathPlanning_lib;

PathPlanning::PathPlanning(plannerType _type):type(_type)
{
    std::cout<< "Created planner type: " << type << std::endl;
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
	      case 0: Power = 1000.0;   lM = DRIVING; break;
	      //case 1: Power = 0.088; lM = DRIVING; break;
              //case 2: Power = 1.074; lM = WHEEL_WALKING; break;
	      //case 2: Power = 0.236; lM = DRIVING; break;
         // SLOWNESS POWER
              case 1: Power = 1/0.07; lM = DRIVING; break;
              case 2: Power = 1/0.05; lM = WHEEL_WALKING; break;
	      //case 2: Power = 1.074; lM = DRIVING; break;
    }
}


//TODO:__FIELD_D_STAR_ALGORITHM__

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
    std::cout<< "Goal at (" << nodeTarget->pose.position[0] << "," << nodeTarget->pose.position[1] << ")" << std::endl;
    std::cout<< "Start at (" << nodeStart->pose.position[0] << "," << nodeStart->pose.position[1] << ")" << std::endl;

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
    double newX, newY, di, dj, cost;
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

void PathPlanning::costInterpolation(double g1, double g2, uint cTerrain,
                                     uint bTerrain, double di, double dj,
                                     double& dk, double& vs)
{
    // g2 is the diagonal neighbor
    // di and dj are the horizontal and vertical distance of diagonal neighbor
    // respect to reference
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
                                NodeMap * nodes, NodeMap * globalNodes)
{



  // Initialize nodeStart, nodeGoal and nodeTarget
    nodes->setActualPos(wStart);
    nodes->setGoal(wGoal);

  // Initializing the Narrow Band
    initNarrowBand(nodes, wGoal, globalNodes);

    Node * nodeTarget;

    /*std::cout << "Fast Marching: propagation will start from node (" <<
        nodeTarget->pose.position[0] << "," <<
        nodeTarget->pose.position[1] << ")" << std::endl;*/



    /*std::cout << "Fast Marching: propagation will finish at node (" <<
        nodeFinal->pose.position[0] << "," <<
        nodeFinal->pose.position[1] << ")" << std::endl;*/

  // Propagation Loop
    t1 = base::Time::now();
    if(this->type == LOCAL_PLANNER)
    {
        std::cout << "PLANNER: starting local propagation loop" << std::endl;

        Node * node00 = nodes->nodeActualPos;
            /*(nodes->getNode((uint)(wStart.position[0]/(nodes->scale)),
            (uint)(wStart.position[1]/(nodes->scale))));*/
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
        while ((node00->state != CLOSED)||
               (nodeN0->state != CLOSED)||
               (nodeN1->state != CLOSED)||
               (nodeN2->state != CLOSED)||
               (nodeN3->state != CLOSED)||
               (nodeN4->state != CLOSED)||
               (nodeN5->state != CLOSED)||
               (nodeN6->state != CLOSED)||
               (nodeN7->state != CLOSED))
        {
            nodeTarget = minCostNode();
            if (first_iteration)
            {
                first_iteration = false;
                nodes->minWork = nodeTarget->work; 
            }
            //std::cout << "PLANNER: next nodetarget is " << nodeTarget->pose.position[0] << "," << nodeTarget->pose.position[1] << std::endl;
            //std::cout << "PLANNER: with work  " << nodeTarget->work << std::endl;
            nodeTarget->state = CLOSED;
            //nodes->closedNodes.push_back(nodeTarget);
            for (uint i = 0; i<4; i++)
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (nodeTarget->nb4List[i]->state == OPEN))
                {
                    propagationFunction(nodeTarget->nb4List[i], nodes->scale);
                    if(nodeTarget->nb4List[i] == node00)
                       std::cout << "PLANNER: work of node00 is " << node00->work << std::endl;
                    nodes->closedNodes.push_back(nodeTarget->nb4List[i]);//To later erase work value of non close nodes as well!!!
                }
        }
        std::cout<< "Fast Marching: ended local propagation loop" << std::endl;
        std::cout<< "PLANNER: value of local work = " << node00->work << std::endl;
    }
    else
    {
        std::cout<< "PLANNER: starting global propagation loop " << std::endl;
        nodes->minWork = 0;
        Node * nodeFinal = nodes->getActualPos();
        if (nodeFinal->terrain == 0)
        {
            std::cout<< "ALERT: entering obstacle area" << std::endl;
            nodeFinal->state = OPEN;
        }
        while (!narrowBand.empty())
        {
            nodeTarget = minCostNode();
            nodeTarget->state = CLOSED;
            for (uint i = 0; i<4; i++)
                if ((nodeTarget->nb4List[i] != NULL) &&
                    (nodeTarget->nb4List[i]->state == OPEN))
                    {
                        propagationFunction(nodeTarget->nb4List[i], nodes->scale);
                        nodes->closedNodes.push_back(nodeTarget->nb4List[i]);
                    }
        }
        std::cout<< "Fast Marching: ended global propagation loop" << std::endl;
    }

    t1 = base::Time::now() - t1;
    std::cout<<"Computation Time: " << t1 <<std::endl;

}


void PathPlanning::initNarrowBand(NodeMap * nodes, base::Waypoint wGoal, NodeMap * globalNodes)
{
    std::cout << "PLANNER: Initializing NarrowBand" << std::endl;
    Node * nodeGoal = nodes->getGoal();

    nodes->resetPropagation();
    narrowBand.clear();
    narrowBand = nodes->horizonNodes;
    if(nodeGoal->state != HIDDEN)
    {
        nodeGoal->work = 0;
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
    /*for (uint i = 0; i < narrowBand.size(); i++)
    {
    std::cout << "PLANNER: horizon node " << i << "with position (" << narrowBand[i]->pose.position[0] << "," << narrowBand[i]->pose.position[1];
    std::cout << ") and work "<< narrowBand[i]->work << std::endl;
    }*/
    //uint sizeI = nodes->nodeMatrix.size();
    //uint sizeJ = nodes->nodeMatrix[0].size();

    // HORIZON NODES
    /*if (globalNodes != NULL)
    {
        for (uint j = 0; j < sizeJ; j++)
        {
            for (uint i = 0; i < sizeI; i++)
            {
                if((nodes->nodeMatrix[j][i]->state == OPEN) &&
                   (nodes->nodeMatrix[j][i]->terrain != 0))
                    for (uint k = 0; k < 4; k++)
                        if((nodes->nodeMatrix[j][i]->nb4List[k] != NULL) &&
                           (nodes->nodeMatrix[j][i]->nb4List[k]->state ==
                            HIDDEN))
                        {
                            getHorizonCost(nodes, nodes->nodeMatrix[j][i], globalNodes);
                            narrowBand.push_back(nodes->nodeMatrix[j][i]);
                            k = 4;
                        }
            }
        }
    }*/
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

    double w00 = node00->work;
    double w10 = node10->work;
    double w01 = node01->work;
    double w11 = node11->work;

    horizonNode->work = w00 + (w01 - w00)*a + (w10 - w00)*b + (w11 + w00 - w10 - w01)*a*b;
    if (horizonNode->work < 0)
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

//__ISOTROPIC_PROPAGATION_FUNCTION_USING_EIKONAL_EQUATION__

void PathPlanning::propagationFunction(Node* nodeTarget, double scale)
{
  // (W-Wx)^2 + (W-Wy)^2 = (P+R)^2
    double Wx,Wy,P,W,R,C;
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
    if(this->type == LOCAL_PLANNER)
    {
        if (nodeTarget->terrain == 0)
            costFunction(nodeTarget->terrain,P,L);
        else
        {
          costFunction(1,P,L);
          //P = P/2;
        }
    }
    else
    {
        if (nodeTarget->slip_ratio == 1)
        {
            costFunction(0,P,L);
            std::cout << "PLANNER: the slip ratio is 1, global node is an obstacle" << std::endl;
        }
        else
        {
            costFunction(nodeTarget->terrain,P,L);
            P = P/(1-nodeTarget->slip_ratio);
            if(nodeTarget->slip_ratio != 0)
                std::cout << "PLANNER: propagating through node (" << nodeTarget->pose.position[0] << ", " << nodeTarget->pose.position[1] << ") with slip " << nodeTarget->slip_ratio << " and new cost " << P << std::endl;
        }
    }

    R = nodeTarget->risk.obstacle;

    double Co = 10;
    //double C = scale * (P + R * (Co - P));
    //double C = scale * P * (1 + R);
    if(this->type == LOCAL_PLANNER)
        C = scale*P*(1+9*R);//scale*P*(1+R);
    else
        C = scale*P;
    if ((fabs(Wx-Wy)<C)&&(Wx < INF)&&(Wy < INF))
        W = (Wx+Wy+sqrt(2*pow(C,2.0) - pow((Wx-Wy),2.0)))/2;
    else
        W = fmin(Wx,Wy) + C;

    if(W < nodeTarget->work)
    {
        if (nodeTarget->work == INF) //It is not in narrowband
            this->narrowBand.push_back(nodeTarget);
        nodeTarget->work = W;
        nodeTarget->nodeLocMode = L;
        nodeTarget->power = P;
    }

    if (nodeTarget->work == INF)
    {
        std::cout << "ERROR:Propagation on Node " <<
          nodeTarget->pose.position[0] << "," <<
          nodeTarget->pose.position[1] << ")" << std::endl;
        std::cout << " - work = " << nodeTarget->work << std::endl;
        std::cout << " - Wx =  " << Wx << std::endl;
        std::cout << " - Wy =  " << Wy << std::endl;
    }

}


//__GRADIENT_DESCENT_METHOD__

bool PathPlanning::getPath(NodeMap * nodes, double tau,
 					     std::vector<base::Waypoint>& trajectory,
               std::vector< short int >& locVector, int currentSegment)
{
    double newX, newY, newH, dCostX, dCostY,
           normDCostX, normDCostY, distance = 0, dHeading, risk;
    short int locMode;
    base::Waypoint sinkPoint;
    base::Waypoint startPoint;

    sinkPoint.position[0] = nodes->getGoal()->pose.position[0];
    sinkPoint.position[1] = nodes->getGoal()->pose.position[1];
    sinkPoint.heading = nodes->getGoal()->pose.orientation;

  //  cX = startPoint.position[0]; //Position to take into account when calculating heading for Wheel-walking waypoints
  //  cY = startPoint.position[1]; //Position to take into account when calculating heading for Wheel-walking waypoints
    std::cout << "PLANNER: working with the path" << std::endl;
    base::Waypoint wNew;
    //if (!trajectory.empty())
    if(false)
    {
          std::cout<< "PLANNER: Repairing a trajectory of " << trajectory.size() << " waypoints"<<  std::endl;
          std::cout<< "PLANNER: Current Segment " << currentSegment <<  std::endl;

          startPoint = nodes->actualPose;
          calculateNextWaypoint(startPoint.position[0], startPoint.position[1],
                                dCostX, dCostY, newH, locMode, nodes, risk);
          dHeading = acos((-cos(trajectory[currentSegment].heading)*dCostX - sin(trajectory[currentSegment].heading)*dCostY)/sqrt(pow(dCostX,2)+pow(dCostY,2)));
          if(dHeading > 0.7854/2)
          {
              //The rover must turn around, the heading is not good
              std::cout<< "PLANNER: Rover must turn around" <<  std::endl;
              trajectory.clear();
              startPoint.position[2] = newH;
              //trajectory.insert(trajectory.begin(), startPoint);
              trajectory.push_back(startPoint);
              //locVector.insert(locVector.begin(), locMode);
              locVector.push_back(locMode);
              //std::cout<< "Loc Mode inicial: "  << locVector.front() << std::endl;
              newX = trajectory.back().position[0];
              newY = trajectory.back().position[1];
          }
          else
          {
              std::cout<< "PLANNER: Rover repairs actual trajectory" <<  std::endl;
              std::cout<< "PLANNER: currentSegment = " << currentSegment <<  std::endl;
              std::cout<< "PLANNER: size of trajectory = " << trajectory.size() << std::endl;
              int size = (int)trajectory.size();
              for(int i = currentSegment; i < size; i++)
              {
                  /*if(i==currentSegment)
                  {
                      startPoint = nodes->actualPose;
                      calculateNextWaypoint(startPoint.position[0], startPoint.position[1], dCostX, dCostY, newH, locMode, nodes, risk);
                      startPoint.position[2] = newH;
                      //trajectory.insert(trajectory.begin(), startPoint);
                      trajectory.push_back(startPoint);
                      //locVector.insert(locVector.begin(), locMode);
                      locVector.push_back(locMode);
                      //std::cout<< "Loc Mode inicial: "  << locVector.front() << std::endl;
                  }*/
                  calculateNextWaypoint(trajectory.back().position[0]/nodes->scale, trajectory.back().position[1]/nodes->scale, dCostX, dCostY, newH, locMode, nodes,risk);
                  /*std::cout<< "PLANNER: dCostX " << dCostX <<  std::endl;
                  std::cout<< "PLANNER: dCostY " << dCostY <<  std::endl;
                  std::cout<< "PLANNER: cos(heading) " << cos(trajectory.back().heading) <<  std::endl;
                  std::cout<< "PLANNER: sin(heading) " << sin(trajectory.back().heading)<<  std::endl;
                  std::cout<< "PLANNER: heading " << trajectory.back().heading <<  std::endl;*/
                  dHeading = acos((-cos(trajectory.back().heading)*dCostX - sin(trajectory.back().heading)*dCostY)/sqrt(pow(dCostX,2)+pow(dCostY,2)));
                  /*std::cout<< "PLANNER: waypoint number " << i <<  std::endl;
                  std::cout<< "PLANNER: dHeading " << dHeading <<  std::endl;*/
                  if(dHeading > 0.7854/2)
                  {
                      std::cout<< "PLANNER: Rover repairs from waypoint " << i <<  std::endl;
                      break;
                  }
              }
              newX = trajectory.back().position[0]/nodes->scale;
              newY = trajectory.back().position[1]/nodes->scale;
              trajectory.back().position[0] = trajectory.back().position[0]/nodes->scale;
              trajectory.back().position[1] = trajectory.back().position[1]/nodes->scale;
          }
    }
    else
    {
        std::cout << "PLANNER: trajectory is empty, starting from actual point" << std::endl;
        trajectory.clear();
        startPoint = nodes->actualPose;
        calculateNextWaypoint(startPoint.position[0], startPoint.position[1], dCostX, dCostY, newH, locMode, nodes, risk);
        startPoint.position[2] = newH;
        //trajectory.insert(trajectory.begin(), startPoint);
        trajectory.push_back(startPoint);
        //locVector.insert(locVector.begin(), locMode);
        locVector.push_back(locMode);
        //std::cout<< "Loc Mode inicial: "  << locVector.front() << std::endl;
        newX = trajectory.back().position[0];
        newY = trajectory.back().position[1];
        std::cout << "PLANNER: trajectory initialized" << std::endl;
    }


    while(sqrt(pow((trajectory.back().position[0] - sinkPoint.position[0]),2) +
             pow((trajectory.back().position[1] - sinkPoint.position[1]),2)) > (2*tau))
    {
        /*if (sqrt(pow(dCostX,2)+pow(dCostY,2)) < 0.9)
             std::cout<< "ERROR: " << sqrt(pow(dCostX,2)+pow(dCostY,2)) << std::endl;*/
        trajectory.back().position[0] = trajectory.back().position[0]*nodes->scale;
        trajectory.back().position[1] = trajectory.back().position[1]*nodes->scale;
        normDCostX = dCostX;//sqrt(pow(dCostX,2) + pow(dCostY,2));
        normDCostY = dCostY;//sqrt(pow(dCostX,2) + pow(dCostY,2));
        newX = newX - tau*normDCostX;
        newY = newY - tau*normDCostY;

        bool isNearHidden = calculateNextWaypoint(newX, newY, dCostX, dCostY, newH, locMode, nodes, risk);
        if (!isNearHidden)
        {
            std::cout<< "PLANNER: node (" << newX << "," << newY <<") is near hidden area"<< std::endl;
            return false;
        }
        else
            wNew.position = Eigen::Vector3d(newX,newY,newH);
        dHeading = acos((normDCostX*dCostX + normDCostY*dCostY)/sqrt(pow(dCostX,2)+pow(dCostY,2)));
        /*if(dHeading > 0.7854)
        {
            std::cout<< "ERROR at node (" << newX << "," << newY <<")"<< std::endl;
            std::cout<< "Heading change larger than (" << (acos((normDCostX*dCostX + normDCostY*dCostY)/sqrt(pow(dCostX,2)+pow(dCostY,2)))) << std::endl;
            return false;
        }*/


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
	      /*if (distance > 0.8)
	          break;*/
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
    calculateNextWaypoint(sinkPoint.position[0], sinkPoint.position[1], dCostX, dCostY, newH, locMode, nodes, risk);
    if (sinkPoint.heading == -0)
        sinkPoint.heading = 0;
    sinkPoint.position[0] = sinkPoint.position[0]*nodes->scale;
    sinkPoint.position[1] = sinkPoint.position[1]*nodes->scale;
    sinkPoint.position[2] = newH;
    trajectory.push_back(sinkPoint);
    locVector.push_back(locMode);
    std::cout<< "PLANNER: Adding final waypoint with heading" << sinkPoint.heading << " "<< trajectory.back().heading<<  std::endl;
    return true;
  //Rescaling resulting trajectory
    /*for (uint i = 0; i<trajectory.size(); i++)
    {
        trajectory[i].position[0] = trajectory[i].position[0]*nodes->scale;
        trajectory[i].position[1] = trajectory[i].position[1]*nodes->scale;
    }*/
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
      if (((nodeTarget->nb4List[1] == NULL)&&(nodeTarget->nb4List[2] == NULL))||
          ((nodeTarget->nb4List[1]->work == INF)&&(nodeTarget->nb4List[2]->work == INF)))
          dx = 0;
      else
      {
          if ((nodeTarget->nb4List[1] == NULL)||(nodeTarget->nb4List[1]->work == INF))
              dx = nodeTarget->nb4List[2]->work - nodeTarget->work;
          else
          {
              if ((nodeTarget->nb4List[2] == NULL)||(nodeTarget->nb4List[2]->work == INF))
                  dx = nodeTarget->work - nodeTarget->nb4List[1]->work;
              else
                  dx = (nodeTarget->nb4List[2]->work -
                        nodeTarget->nb4List[1]->work)*0.5;
          }
      }
      if (((nodeTarget->nb4List[0] == NULL)&&(nodeTarget->nb4List[3] == NULL))||
          ((nodeTarget->nb4List[0]->work == INF)&&(nodeTarget->nb4List[3]->work == INF)))
          dy = 0;
      else
      {
          if ((nodeTarget->nb4List[0] == NULL)||(nodeTarget->nb4List[0]->work == INF))
              dy = nodeTarget->nb4List[3]->work - nodeTarget->work;
          else
          {
              if ((nodeTarget->nb4List[3] == NULL)||(nodeTarget->nb4List[3]->work == INF))
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

bool PathPlanning::calculateNextWaypoint(double x, double y, double& dCostX, double& dCostY, double& height, short int& locMode, NodeMap * nodes, double& risk)
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


/*    if ((node00->state == HIDDEN)||(node01->state == HIDDEN)||
        (node10->state == HIDDEN)||(node11->state == HIDDEN))
    {
        return false;
    }*/
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

        double L00 = node00->nodeLocMode;
        double L10 = node10->nodeLocMode;
        double L01 = node01->nodeLocMode;
        double L11 = node11->nodeLocMode;

        dCostX = interpolate(a,b,gx00,gx01,gx10,gx11);
        dCostY = interpolate(a,b,gy00,gy01,gy10,gy11);
        height = interpolate(a, b, node00->elevation, node01->elevation,
                             node10->elevation, node11->elevation);

        /*dCostX = gx00 + (gx10 - gx00)*a + (gx01 - gx00)*b + (gx11 + gx00 - gx10 - gx01)*a*b;
        dCostY = gy00 + (gy10 - gy00)*a + (gy01 - gy00)*b + (gy11 + gy00 - gy10 - gy01)*a*b;*/
        //height = h00 + (h10 - h00)*a + (h01 - h00)*b + (h11 + h00 - h10 - h01)*a*b;

       // if (L00 == WHEEL_WALKING)
        //{
        //    std::cout<< "Suma: " << L00 + L10 + L01 + L11 << std::endl;
        //    std::cout<< "Division: " << (L00 + L10 + L01 + L11)/4.0+0.5 << std::endl;
        //    std::cout<< "Resultado: " << (unsigned int)((L00 + L10 + L01 + L11)/4.0+0.5) << std::endl; sleep(1);
        //}
        locMode = (int)((L00 + L10 + L01 + L11)/4.0); //PROVISIONAL, FIX THIS to properly interpolate locomotion mode
        return true;
    //}
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
