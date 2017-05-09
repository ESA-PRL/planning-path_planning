
#include <string>
#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include "PathPlanning.hpp"
#include "fstream"
#include <Eigen/Geometry>
#include "geometry_msgs/Pose.h"

using namespace Eigen;
using namespace PathPlanning_lib;

double roverX, roverY, roverYaw,goalX,goalY;
Eigen::Matrix<double,3,3> mRot;
Eigen::Matrix<double,3,1> vEuler;
Eigen::AngleAxisd yawAngle(M_PI, Eigen::Vector3d::UnitZ());
bool syncTest = false, goalPosFlag = false, roverPosFlag = false;
int simState = 0;

std::vector< std::vector<double> > readMatrixFile(std::string map_file, double& Nrow, double& Ncol){

    std::cout<< "Map being loaded from: " << map_file << ", correct?" << std::endl;
    std::vector< std::vector<double> > mapMatrix;
    std::string line;
    std::ifstream eFile(map_file.c_str(), std::ios::in);
    Nrow = 0; Ncol = 0;    
    std::vector <double> row;

    if( eFile.is_open() ){
        while ( std::getline(eFile, line) ){
            std::stringstream ss(line);
            std::string cell;
           while( std::getline(ss, cell, ' ') ){ 
                double val;
                std::stringstream numericValue(cell);
                numericValue >> val;
                row.push_back(val);
                Ncol++;
            }
            mapMatrix.push_back(row);
            row.clear();
            Nrow++;
        }
    eFile.close(); 

    Ncol /= Nrow;
        std::cout << "Cost map of " << Nrow 
                  << " x "          << Ncol << " loaded." << std::endl;        
    } else {
        std::cout << "Problem opening the file" << std::endl;
        return mapMatrix;
    }   
    return mapMatrix;
}

// Main code:
int main(int argc,char* argv[])
{
    printf("ExoTER ARES Path Planning Test started\n");
  // IMPORTING DATA PROCESS
  uint sizeX, sizeY;


  double Nraw, Ncol;

  std::vector< std::vector<double> > elevationMatrix = readMatrixFile("../terrainData/crater/elevation_map.txt", Nraw, Ncol);
  std::vector< std::vector<double> > frictionMatrix = readMatrixFile("../terrainData/crater/crater_costMap.txt", Nraw, Ncol);
  std::vector< std::vector<double> > slipMatrix = readMatrixFile("../terrainData/crater/slip_map.txt", Nraw, Ncol);
  std::vector< std::vector<double> > riskMatrix = readMatrixFile("../terrainData/crater/risk_map.txt", Nraw, Ncol);

  // IMPORTING DATA PROCESS - end

  PathPlanning aresPlanner;

  uint numNodes;

  int option = 0;

  printf("\nChoose Algorithm: \n");
  printf("1-A* 2-Dijkstra 3-Fast Marching 0-Exit \n");
  scanf("%d",&option);
  printf("Waiting for poses \n");

    std::vector<base::Waypoint> trajectory;
    base::Waypoint wRover;
    wRover.position[0] = 60.0;
    wRover.position[1] = 20.0;
    wRover.heading = 90.0*M_PI/180.0;
    base::Waypoint wGoal;
    wGoal.position[0] = 10.0;
    wGoal.position[1] = 10.0;
    wGoal.heading = 90.0*M_PI/180.0;

    aresPlanner.initNodeMatrix(elevationMatrix, frictionMatrix, slipMatrix, riskMatrix);
    trajectory = aresPlanner.fastMarching(wRover,wGoal);
    std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
    for (unsigned int i = 0; i<trajectory.size(); i++)
	std::cout << "Waypoint " << i << " -> Pos: (" << trajectory[i].position[0] << "," << trajectory[i].position[1] << ") Loc: " << trajectory[i].position[2] 
                  << " Heading: " << trajectory[i].heading << std::endl;
    return(0);
}
