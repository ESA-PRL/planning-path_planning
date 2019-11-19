/*********************************DyMu******************************************
                    -Dynamic Multilayered Path Planner-
                        ARES (ESA-UMA Collaboration)
                University of Malaga - European Space Agency
                                -Author-
                        J. Ricardo Sanchez Ibanez
                             -Contact mail-
                            ricardosan@uma.es
                              -Supervisors-
                        Carlos J. Perez del Pulgar
                             Martin Azkarate
*******************************************************************************/

#include "DyMu.hpp"
#include "iostream"
#include <fstream>

using namespace PathPlanning_lib;
using namespace std;


// Useful functions for reading and writing files
std::vector<std::vector<double>> readMatrixFile(std::string map_file);
void writeGlobalResults(std::vector<std::vector<double>> &total_cost_matrix,
                        std::vector<std::vector<double>> &global_cost_matrix,
                        std::vector<base::Waypoint> &trajectory);


// An example of how to use the planner (only Global Path Planning)
int main()
{
    cout << "\033[1;4;35m DyMu Path Planning Example \033[0m" << endl;


  //--1: TRAVERSABILITY COST IS DEFINED ACCORDING TO LOCOMOTION, SLOPE AND
  //     TERRAIN INDEX

  //----1.1: Available locomotion modes are defined:
    vector<string> locomotion_modes{"DRIVING"};

  //----1.2: The range of traversable slopes is also defined:
    vector<double> slope_values{ 0, 5, 10, 15 };

  //----1.3: The cost look-up table is defined:
  //          Locomotion1        Locomotion2
  //Obstacles slope1-slope2-...  slope1-slope2-... (~= 2xMaxTraversableCost)
  //Terrain1  slope1-slope2-...  slope1-slope2-... 
  //Terrain2  slope1-slope2-...  slope1-slope2-...
    vector<double> cost_data{ 120.0,    120.0,    120.0,  120.0,
                               10.0,     15.0,     25.0,   40.0,
                               10.0,     15.0,     25.0,   40.0,
                               32.0,     37.0,     47.0,   62.0 };
    

  //--2: PLANNER INITIALIZATION    
  // We initialize the planner introducing the relevant variables for local
  // repairings, in case no local planning is expected, just put random values
  // - risk_distance: max distance from obstacles considered risky
  // - reconnect_distance: in CONSERVATIVE approach, it delimits the position
  //   of the waypoint to reconnect, placed further than this distance to the
  //   risky area
  // - risk_ratio: gradient of the expanded risk, the higher the more pronounced
  //   are the repaired paths
  // - input_approach: the approach (see related publication)
    double risk_distance = 1.5;
    double reconnect_distance = 0.0;
    double risk_ratio = 1.1;
    repairingAproach input_approach = SWEEPING;

    DyMuPathPlanner* planner = new PathPlanning_lib::DyMuPathPlanner(
        risk_distance, reconnect_distance, risk_ratio, input_approach);
    

  //--3: GLOBAL LAYER CONSTRUCTION
  // The global layer is here initialized. It is a regular grid formed by global
  // nodes, each of them covering a square portion of the map.
    std::vector<double> global_offset{0,0};
    double global_res = 1.0;
    double local_res = 0.1; // Only relevant for local repairing tasks

  //----3.1: Read the elevation map
    std::string elevation_map_dir = "data/decos_1mDEM.txt";
    vector<vector<double>> elevation_matrix = readMatrixFile(elevation_map_dir);
    uint map_size_X = elevation_matrix[0].size();
    uint map_size_Y = elevation_matrix.size();
    cout << "\033[1;32mPLANNER:\033[0m \033[32mElevation Map of " << map_size_X
         << " x " << map_size_Y << " loaded.\033[0m" << endl;

  //----3.2: Read the terrain map
    std::string terrain_map_dir = "data/decos_1mTerrainMap.txt";
    vector<vector<double>> terrain_matrix = readMatrixFile(terrain_map_dir);
    cout << "\033[1;32mPLANNER:\033[0m \033[32mTerrain Map of " << map_size_X
         << " x " << map_size_Y << " loaded.\033[0m" << endl;

  //----3.3: The Global Layer is initialized
    planner->initGlobalLayer(global_res, local_res, map_size_X, map_size_Y, global_offset);
    

    // Cost values must be assigned to each of the global nodes. In this case, we
    // make use of cost based on locomotion according to slope (computed from the
    // elevation) and type of terrain
    // Btw, do not forget here the terrain 0 is considered as obstacle (non-tra-
    // versable)
    if (!planner->computeCostMap(
                                 cost_data, slope_values, locomotion_modes, 
                                 elevation_matrix, terrain_matrix))
    {
        cout << "\033[1;31mERROR:\033[0m" <<
                " \033[31mSomething went wrong with cost assignation...\033[0m" << endl;
        return 0;
    }


  //--4: PATH PLANNING COMPUTATION
  //----4.1: Defining the goal waypoint
    base::Waypoint goalWaypoint;
    goalWaypoint.position[0] = 45.0;
    goalWaypoint.position[1] = 85.0;

    if (!planner->setGoal(goalWaypoint))
    {
        cout << "\033[1;31mERROR:\033[0m \033[31mnot a valid goal\033[0m" << endl;
        return 0;
    }
    
  //----4.2: Defining the rover position (as a waypoint)
    base::Waypoint roverCurrentPos;
    roverCurrentPos.position[0] = 105.0;
    roverCurrentPos.position[1] = 70.0;

  //----4.3: Total Cost from Goal to Rover position is computed
    planner->computeTotalCostMap(roverCurrentPos);

  //----4.4: The resulting trajectory is obtained
    std::vector<base::Waypoint> trajectory;
    trajectory.clear();
    trajectory = planner->getPath(roverCurrentPos);


  //--5: WRITING RESULTS
    cout << "\033[1;32mPLANNER:\033[0m \033[32mResulting trajectory has " 
         << trajectory.size() << " Waypoints\033[0m" << endl;
    std::vector<std::vector<double>> total_cost_matrix = 
        planner->getTotalCostMatrix();
    std::vector<std::vector<double>> global_cost_matrix = 
        planner->getGlobalCostMatrix();
    writeGlobalResults(total_cost_matrix, global_cost_matrix, trajectory);
    return 0;
}



std::vector<std::vector<double>> readMatrixFile(std::string map_file)
{
    vector<vector<double>> mapMatrix;
    string line;
    ifstream eFile(map_file.c_str(), ios::in);
    double Nrow = 0, Ncol = 0;
    vector<double> row;

    if (eFile.is_open())
    {
        while (getline(eFile, line))
        {
            stringstream ss(line);
            string cell;
            while (getline(ss, cell, ' '))
            {
                double val;
                stringstream numericValue(cell);
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
        //cout << "PLANNER: Map of " << Ncol << " x " << Nrow << " loaded." << endl;
    }
    else
    {
        cout << "PLANNER: Problem opening the file" << endl;
        return mapMatrix;
    }
    return mapMatrix;
}


void writeGlobalResults(std::vector<std::vector<double>> &total_cost_matrix,
                        std::vector<std::vector<double>> &global_cost_matrix,
                        std::vector<base::Waypoint> &trajectory)
{
    std::string total_cost_filename =
        std::string("data/results/TotalCostMap.txt");
    std::string global_cost_filename =
        std::string("data/results/GlobalCostMap.txt");
    std::string path_filename =
        std::string("data/results/Path.txt");
    std::ofstream total_cost_file;
    total_cost_file.open(total_cost_filename);
    std::ofstream global_cost_file;
    global_cost_file.open(global_cost_filename);
    std::ofstream path_file;
    path_file.open(path_filename);

    for (uint j = 0; j < total_cost_matrix.size(); j++)
    {
        for (uint i = 0; i < total_cost_matrix[0].size() - 1; i++)
        {
            total_cost_file << total_cost_matrix[j][i] << " ";
            global_cost_file << global_cost_matrix[j][i] << " ";
        }
        total_cost_file << total_cost_matrix[j][total_cost_matrix[0].size() - 1];
        global_cost_file << global_cost_matrix[j][total_cost_matrix[0].size() - 1];
        total_cost_file << "\n";
        global_cost_file << "\n";
    }
    for (uint k = 0; k < trajectory.size(); k++)
        path_file << trajectory[k].position[0] << " " << trajectory[k].position[1] << "\n";
    total_cost_file.close();
    global_cost_file.close();
    path_file.close();
}
