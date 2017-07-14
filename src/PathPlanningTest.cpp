
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

std::vector< std::vector<double> > readTerrainFile(std::string terrain_file)
{
    std::cout<< "Map being loaded from: " << terrain_file << ", correct?" << std::endl;
    std::vector< std::vector<double> > terrainList;
    std::string line;
    std::ifstream eFile(terrain_file.c_str(), std::ios::in);
    uint numTerrains = 0, numProperties = 0;
    std::vector <double> row;

    if( eFile.is_open() )
    {
        while ( std::getline(eFile, line) ){
            std::stringstream ss(line);
            std::string cell;
            numProperties = 0;
            while( std::getline(ss, cell, ' ') ){
                double val;
                std::stringstream numericValue(cell);
                numericValue >> val;
                row.push_back(val);
                numProperties++;
            }
            terrainList.push_back(row);
            row.clear();
            numTerrains++;
        }
        eFile.close();
        std::cout << "Terrains detected: " << numTerrains << std::endl;
        std::cout << "Properties detected: " << numProperties << std::endl;
    }
    else
    {
        std::cout << "Problem opening the file" << std::endl;
        return terrainList;
    }
    return terrainList;
}

// Main code:
int main(int argc,char* argv[])
{
    printf("ExoTER ARES Path Planning Test started\n");
  // IMPORTING DATA PROCESS
  uint sizeX, sizeY;


  double Nraw, Ncol;

  std::vector< std::vector<double> > elevationMatrix = readMatrixFile("../../terrainData/prl/prl_elevationMap.txt", Nraw, Ncol);
  std::vector< std::vector<double> > costMatrix = readMatrixFile("../../terrainData/prl/prl_costMap.txt", Nraw, Ncol);
  std::vector< std::vector<double> > riskMatrix = readMatrixFile("../../terrainData/prl/prl_riskMap.txt", Nraw, Ncol);
  std::vector< std::vector<double> > soilList = readTerrainFile("../../terrainData/prl/soilList.txt");

  // IMPORTING DATA PROCESS - end

  PathPlanning aresPlanner;

  uint numNodes;

  int option = 0;

  printf("\nChoose Algorithm: \n");
  printf("1-A* 2-Dijkstra 3-Fast Marching 0-Exit \n");
  scanf("%d",&option);
  printf("Waiting for poses \n");

    std::vector<base::Waypoint> trajectory;
    std::vector<short int> locVector;
    base::Waypoint wRover;
    wRover.position[0] = 1.5;
    wRover.position[1] = 8.0;
    wRover.heading = -90.0*M_PI/180.0;
    base::Waypoint wGoal;
    wGoal.position[0] = 8;
    wGoal.position[1] = 5.5;
    wGoal.heading = 0.0*M_PI/180.0;

    double size = 0.05;
    base::Pose2D pos;
    pos.position[0] = 0.0;
    pos.position[1] = 0.0;
    printf("Creating Map \n");

    PathPlanning_lib::NodeMap* map = new PathPlanning_lib::NodeMap(size, pos, elevationMatrix, costMatrix, riskMatrix);

    printf("Map created \n");

    trajectory.clear();
    aresPlanner.fastMarching(wRover,wGoal,trajectory,locVector, map);

    std::cout<< "Trajectory has " << trajectory.size() << " Waypoints" << std::endl;
    for (unsigned int i = 0; i<trajectory.size(); i++)
	std::cout << "Waypoint " << i << " -> Pos: (" << trajectory[i].position[0] << "," << trajectory[i].position[1] << ") Loc: " << trajectory[i].position[2] 
                  << " Heading: " << trajectory[i].heading << std::endl;
    return(0);
}
