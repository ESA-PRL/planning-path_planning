
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
  // Node Initialization
  printf("ExoTER ARES Path Planning Node started\n");
  // SUBSCRIBERS
// IMPORTING DATA PROCESS
  std::string infoPath, heightPath, costPath, locPath;
  std::ifstream infofile, heightfile, costfile, locfile;

  infoPath = "../terrainData/crater/crater_info.txt";
  heightPath = "../terrainData/crater/crater_heightMap.txt";
  costPath = "../terrainData/crater/crater_costMap.txt";
  locPath = "../terrainData/crater/crater_locMap.txt";

  infofile.open("../terrainData/crater/crater_info.txt");
  std::cout << "infoPath = " << infoPath << std::endl;
  heightfile.open("../terrainData/crater/crater_heightMap.txt");
  costfile.open("../terrainData/crater/crater_costMap.txt");
  locfile.open("../terrainData/crater/crater_locMap.txt");

  if (! (heightfile.is_open()||(infofile.is_open()))){
    printf("No map!!\n");
    return(0);
  }
  uint sizeX, sizeY;
  double minHeight, xd, yd, theta, scale;

  infofile >> sizeX >> sizeY >> minHeight >> xd >> yd >> theta >> scale;

  double Nraw, Ncol;

  std::vector< std::vector<double> > elevationMatrix = readMatrixFile("../terrainData/crater/elevation_map.txt", Nraw, Ncol);
  std::vector< std::vector<double> > frictionMatrix = readMatrixFile("../terrainData/crater/crater_costMap.txt", Nraw, Ncol);
  std::vector< std::vector<double> > slipMatrix = readMatrixFile("../terrainData/crater/slip_map.txt", Nraw, Ncol);
  std::vector< std::vector<double> > riskMatrix = readMatrixFile("../terrainData/crater/risk_map.txt", Nraw, Ncol);

  double heightMap[sizeX*sizeY];
  double costMap[sizeX*sizeY];
  double locMap[sizeX*sizeY];

  for (size_t j = 0; j<sizeY; j++)
    for (size_t i = 0; i<sizeX; i++){
      heightfile    >> heightMap[i+j*sizeX];
      costfile      >> costMap[i+j*sizeX];
      locfile       >> locMap[i+j*sizeX];
    }

  heightfile.close();
  costfile.close();
  locfile.close();
  infofile.close();
  // IMPORTING DATA PROCESS - end

  PathPlanning aresPlanner;

  uint numNodes;

  int option = 0;

  aresPlanner.clearPath();

  printf("\nChoose Algorithm: \n");
  printf("1-A* 2-Dijkstra 3-Fast Marching 0-Exit \n");
  scanf("%d",&option);
  printf("Waiting for poses \n");

  roverX = 60.0;
  roverY = 20.0;
  roverYaw = 90.0*3.14/180.0;
  goalX = 10.0;
  goalY = 10.0;

  aresPlanner.initNodeMatrix(elevationMatrix, frictionMatrix, slipMatrix, riskMatrix);
  aresPlanner.setOffset(xd,yd,theta);
  aresPlanner.setStart(roverX,roverY,roverYaw);
  aresPlanner.setGoal(goalX,goalY);

    switch (option) {
      case 1:
        aresPlanner.setPlanningMode(SAFEST);
        aresPlanner.astarAlgorithm();
        break;
      case 2:
        aresPlanner.setPlanningMode(SAFEST);
        aresPlanner.dijkstraAlgorithm();
        break;
      case 3:
        aresPlanner.fastMarchingAlgorithm();
        break;
      default:
        return(0);
    }

  numNodes = aresPlanner.trajectory.size();

  //aresPlanner.showNodeMatrix();

  for (size_t i = 0; i < numNodes; i++)
  {
    //trajectory.push_back(lpoint);
    std::cout << " Segment: " << i << '\n';
    std::cout << " X: " << aresPlanner.trajectory.at(i).position[0];
    std::cout << " Y: " << aresPlanner.trajectory.at(i).position[1];
    std::cout << " heading: " << aresPlanner.trajectory.at(i).heading;
    std::cout << " LocMode: " << aresPlanner.trajectory.at(i).position[2] << '\n';
  }

  std::cout << " Trajectory: " << aresPlanner.trajectory.back().position <<'\n';
  /*  pathNodes.layout.dim.clear(); //Inizializating pathNodes
    pathNodes.data.clear();
    
    pathNodes.layout.dim.push_back(std_msgs::MultiArrayDimension());
    pathNodes.layout.dim[0].label = "NumNodes";
    pathNodes.layout.dim[0].size = numNodes;
    std::cout << "NumNodes = " << numNodes << '\n';
    for(uint i = 1; i<=numNodes;i++)
    {
      pathNodes.data.push_back(aresPlanner.nodePath[numNodes-i]->globalX);
      pathNodes.data.push_back(aresPlanner.nodePath[numNodes-i]->globalY);
      pathNodes.data.push_back(aresPlanner.nodePath[numNodes-i]->height
                                - minHeight);
      pathNodes.data.push_back(aresPlanner.nodePath[numNodes-i]->heading);
      pathNodes.data.push_back(aresPlanner.nodePath[numNodes-i]->nodeLocMode);

    }*/
  /*for (uint j = 0; j<64 ; j++){
    std::cout << elevationMatrix[63][j] << " ";
  }
  std::cout << '\n';*/
  printf("1-A* 2-Dijkstra 3-Fast Marching 0-Exit \n");
  scanf("%d",&option);
  return(0);
}
