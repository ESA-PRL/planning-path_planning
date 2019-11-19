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

std::vector<std::vector<double>> readMatrixFile(std::string map_file);

int main()
{
    cout << "\033[1;4;35m Initializing Planner \033[0m" << endl;

    vector<double> cost_data{ 120.0,    120.0,    120.0,  120.0,
                               10.0,     15.0,     25.0,   40.0,
                               10.0,     15.0,     25.0,   40.0,
                               32.0,     37.0,     47.0,   62.0 };

    vector<double> slope_values{ 0, 5, 10, 15 };

    vector<string> locomotion_modes{"DRIVING"};
    
    // We initialize the planner introducing the relevant variables for local
    // repairings, in case no local planning is expected, just put random values
    // - risk_distance: max distance from obstacles considered risky
    // - reconnect_distance: in CONSERVATIVE approach, it delimits the position
    //   of the waypoint to reconnect, placed further than this distance to the
    //   risky area
    // - risk_ratio: gradient of the expanded risk, the higher the more pronounced
    //   are the repaired paths
    // - input_approach: the approach

    double risk_distance = 1.5;
    double reconnect_distance = 0.0;
    double risk_ratio = 1.1;
    repairingAproach input_approach = SWEEPING;

    DyMuPathPlanner* planner = new PathPlanning_lib::DyMuPathPlanner(
        risk_distance, reconnect_distance, risk_ratio, input_approach);
    

    // The global layer is here initialized. It is a regular grid formed by global
    // nodes, each of them covering a square portion of the map.
    std::vector<double> global_offset{0,0};
    double global_res = 1.0;
    double local_res = 0.1;

    std::string elevation_map_dir = "data/decos_1mDEM.txt";
    vector<vector<double>> elevation_matrix = readMatrixFile(elevation_map_dir);
    uint map_size_X = elevation_matrix[0].size();
    uint map_size_Y = elevation_matrix.size();
    cout << "\033[1;32mPLANNER:\033[0m \033[32mElevation Map of " << map_size_X << " x " << map_size_Y <<
            " loaded.\033[0m" << endl;
    std::string terrain_map_dir = "data/decos_1mTerrainMap.txt";
    vector<vector<double>> terrain_matrix = readMatrixFile(terrain_map_dir);
    cout << "\033[1;32mPLANNER:\033[0m \033[32mTerrain Map of " << map_size_X << " x " << map_size_Y << 
            " loaded.\033[0m" << endl;
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
        cout << "\033[1;31mERROR:\033[0m \033[31mSomething went wrong with cost assignation...\033[0m" << endl;
        return 0;
    }

    base::Waypoint goalWaypoint;
    goalWaypoint.position[0] = 45.0;
    goalWaypoint.position[1] = 85.0;
    base::Waypoint roverCurrentPos;
    roverCurrentPos.position[0] = 105.0;
    roverCurrentPos.position[1] = 70.0;

    if (!planner->setGoal(goalWaypoint))
    {
        cout << "\033[1;31mERROR:\033[0m \033[31mnot a valid goal\033[0m" << endl;
        return 0;
    }
    
    planner->computeTotalCostMap(roverCurrentPos);

    std::vector<base::Waypoint> trajectory;
    trajectory.clear();
    trajectory = planner->getPath(roverCurrentPos);

    cout << "\033[1;32mPLANNER:\033[0m \033[32mResulting trajectory has " << trajectory.size() << " Waypoints\033[0m" << endl;
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
