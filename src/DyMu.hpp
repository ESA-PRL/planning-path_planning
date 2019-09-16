#ifndef _PATHPLANNING_LIBRARIES_HPP_
#define _PATHPLANNING_LIBRARIES_HPP_

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

#include <base/samples/RigidBodyState.hpp>
#include <base/samples/Frame.hpp>
#include <base/Waypoint.hpp>
#include <vector>

#include <base-logging/Logging.hpp>

//#define INF 100000000 //Change this value to maximum available for float type

namespace PathPlanning_lib
{
    //double INF = std::numeric_limits<double>::infinity();
    enum node_state
    {
        OPEN,
        CLOSED
    };

    enum repairingAproach
    {
        CONSERVATIVE, // Hazard Avoidance - FM*
        SWEEPING // multiBiFM*
    };

    struct localNode
    {
        base::Pose2D pose; //In Local Units respect to Global Node
        base::Pose2D world_pose; //In physical Units respect to World Frame
        base::Pose2D parent_pose; // Position of Global Node Parent In Global Units
        base::Pose2D global_pose; // Position of this local node in Global Units respect to World Frame
        double deviation;
        double total_cost;
        double cost;
        double risk;
        node_state state;
        std::vector<localNode*> nb4List;
        bool isObstacle;
        localNode(uint x_, uint y_, base::Pose2D _parent_pose)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            parent_pose = _parent_pose;
            state = OPEN;
            deviation = std::numeric_limits<double>::infinity();
            total_cost = std::numeric_limits<double>::infinity();
            isObstacle = false;
            risk = 0.0;
        }
    };

    struct globalNode
    {
        base::Pose2D pose;
        base::Pose2D world_pose;
        double elevation;
        double slope;
        node_state state;
        bool isObstacle;
        bool hasLocalMap; //Has it localmap?
        double raw_cost;
        double cost;
        double hazard_density; //Ratio of obstacle area in the global node area
        double trafficability;
        double total_cost;
        unsigned int terrain;
        std::vector< std::vector<localNode*> > localMap;
        std::vector<globalNode*> nb4List;
        std::vector<globalNode*> nb8List;
        std::string nodeLocMode;
        globalNode(uint x_, uint y_, double res, std::vector<double>  offset)
        {
            pose.position[0] = (double)x_;
            pose.position[1] = (double)y_;
            world_pose.position[0] = (double)x_*res + offset[0];
            world_pose.position[1] = (double)y_*res + offset[1];
            //terrain = (unsigned int) t_;
            //elevation = e_;

            //risk.obstacle = r_;
            isObstacle = false;
            hasLocalMap = false;
            raw_cost = 0.0;
            cost = 0.0;
            total_cost = std::numeric_limits<double>::infinity();
            state = OPEN;
            nodeLocMode = "DONT_CARE";
            hazard_density = 0.0;
            trafficability = 1.0;
        }
    };

    struct costCriteria
    {
		int numSamples;
		double mean;
		double stdDeviation;
		bool bEmpty;
		costCriteria(int numSamples_, double mean_, double stdDeviation_)
		{
			numSamples = numSamples_;
			mean = mean_;
			stdDeviation = stdDeviation_;
			bEmpty = false;
		}
		costCriteria()
		{
			numSamples = 0;
			mean = 0;
			stdDeviation = 0;
			bEmpty = true;
		}

		void addData(std::vector<double> newSamples)
		{
			int n = newSamples.size();
			if(n != 0) 	
			{
				double sum = 0;
				for(int i = 0; i < n; i++)
				{
					sum += newSamples[i];
				}
				double newMean = (mean*numSamples + sum)/(numSamples + n);

				if(numSamples + n - 2 > 0)
				{
					double accDiff = 0;
					for(int i = 0; i < n; i++)
					{
						if(!bEmpty) 	
							accDiff += (newSamples[i] - mean)*(newSamples[i] - newMean);
						else 		
							accDiff += pow(newSamples[i] - newMean,2);
					}
					stdDeviation = sqrt((pow(stdDeviation,2)*(numSamples - 1) + accDiff)/(numSamples + n - 2));
				}
				else std::cout << "ERROR: not enough samples to obtain standard deviation."<<std::endl; 

				numSamples += n;
				mean = newMean;		
				bEmpty = false;
			}
		}

		void addData(int numSamples_, double mean_, double stdDeviation_)
		{
			if(numSamples_ != 0) 	
			{
				double newMean = (mean*numSamples + mean_*numSamples_)/(numSamples + numSamples_);

				stdDeviation = sqrt((pow(stdDeviation,2)*(numSamples - 1) + 
				pow(stdDeviation_,2)*(numSamples_ - 1))/(numSamples + numSamples_ - 2));

				numSamples += numSamples_;
				mean = newMean;		
				bEmpty = false;
			}
		}

		void addData(double newSample)
		{
			double newMean = (mean*numSamples + newSample)/(numSamples + 1);

			double accDiff;
			if(!bEmpty)	accDiff = (newSample - mean)*(newSample - newMean);
			stdDeviation = sqrt((pow(stdDeviation,2)*(numSamples - 1) + accDiff)/(numSamples + 1 - 2));

			numSamples += 1;
			mean = newMean;		
			bEmpty = false;
		}

		void erase()
		{
			numSamples = 0;
			mean = 0;
			stdDeviation = 0;
			bEmpty = true;
		}	
	};

	struct segmentedTerrain
	{
		double cost;
		double slopeRatio; 			 // Ratio of increasing cost per degree (u/°)
		std::vector<costCriteria> criteriaInfo, traverseInfo, rejectedInfo;
		std::vector<std::vector<double>> traverseData;
		bool bTraversed;
		segmentedTerrain()
		{
			cost = 1;
			slopeRatio = 1;
			bTraversed = false;
		}
		segmentedTerrain(double cost_, double slopeRatio_)
		{
			cost = cost_;
			slopeRatio = slopeRatio_;		
			bTraversed = false;
		}
		segmentedTerrain(std::vector<costCriteria> criteriaInfo_)
		{
			criteriaInfo.resize(criteriaInfo_.size());
			traverseInfo.resize(criteriaInfo_.size());
			criteriaInfo = criteriaInfo_;
			bTraversed = true;
		}

		void dataAnalysis()
		{
			if(!bTraversed)
			{
				for(int i = 0; i < criteriaInfo.size(); i++)
				{
					if(traverseData[i].size() > 30)
					{
						traverseInfo[i].addData(traverseData[i]);
						traverseData[i].erase(traverseData[i].begin(),traverseData[i].end());
						criteriaInfo[i].addData(traverseInfo[i].numSamples,
										traverseInfo[i].mean,
										traverseInfo[i].stdDeviation);

						traverseInfo[i].erase();
						bTraversed = true;
					}
				}	
				if(bTraversed) 
				{
					std::cout <<  "\033[1;32mNow we have gathered enough info of the current terrain.\033[0m"<<std::endl;
				}
			}
			else
			{
				for(int i = 0; i < criteriaInfo.size(); i++)
				{
					if(traverseData[i].size() > 9)
					{
						traverseInfo[i].addData(traverseData[i]);
						if(FTest(i)) criteriaInfo[i].addData(traverseInfo[i].numSamples,
													    	 traverseInfo[i].mean,
															 traverseInfo[i].stdDeviation);

						traverseData[i].erase(traverseData[i].begin(),traverseData[i].end());
						traverseInfo[i].erase();
					}
					if(rejectedInfo[i].numSamples > 30)
					{
						if(TTest(i)) criteriaInfo[i].addData(rejectedInfo[i].numSamples,
													    	 rejectedInfo[i].mean,
															 rejectedInfo[i].stdDeviation);
						else if (rejectedInfo[i].numSamples >= criteriaInfo[i].numSamples &&
								 rejectedInfo[i].stdDeviation < criteriaInfo[i].stdDeviation)
						{
							std::cout<<"\033[1;35mWARNING: The amount and quality of the rejected samples is greater than the saved ones, now we are using the rejected info as the correct one. \033[0m"<<std::endl;
							traverseInfo[i].erase();
							traverseInfo[i].addData(criteriaInfo[i].numSamples,
												   	criteriaInfo[i].mean,
												 	criteriaInfo[i].stdDeviation);
							criteriaInfo[i].erase();
							criteriaInfo[i].addData(rejectedInfo[i].numSamples,
													rejectedInfo[i].mean,
													rejectedInfo[i].stdDeviation);
							rejectedInfo[i].erase();
							rejectedInfo[i].addData(traverseInfo[i].numSamples,
													traverseInfo[i].mean,
													traverseInfo[i].stdDeviation);
							traverseInfo[i].erase();

						}
					}
				}
			}	
		}

		bool TTest(int i)
		{
			double n1, n2, s1, s2, x1, x2, t;
			n1 = criteriaInfo[i].numSamples;
			n2 = rejectedInfo[i].numSamples;
			s1 = criteriaInfo[i].stdDeviation;
			s2 = rejectedInfo[i].stdDeviation;
			x1 = criteriaInfo[i].mean;
			x2 = rejectedInfo[i].mean;

			bool result = false;
			t = abs(x1-x2)/sqrt(pow(s1,2)/n1 + pow(s2,2)/n2);
			if(t < 2.00) result = true; 
			return result;			
		}

		bool FTest(int i)
		{
			double s1, s2, F;
			bool result;
			if(criteriaInfo[i].stdDeviation < traverseInfo[i].stdDeviation)
			{
				s1 = traverseInfo[i].stdDeviation;
				s2 = criteriaInfo[i].stdDeviation;
			}
			else
			{
				s1 = criteriaInfo[i].stdDeviation;
				s2 = traverseInfo[i].stdDeviation;
			}
			
			F = pow(s1,2)/pow(s2,2);
			if(F < 2.05) result = studentTTest(i); 
			else result = cochranTTest(i);
			return result;			
		}

		bool studentTTest(int i)
		{
			double n1, n2, s1, s2, x1, x2, sp, t;
			n1 = criteriaInfo[i].numSamples;
			n2 = traverseInfo[i].numSamples;
			s1 = criteriaInfo[i].stdDeviation;
			s2 = traverseInfo[i].stdDeviation;
			x1 = criteriaInfo[i].mean;
			x2 = traverseInfo[i].mean;

			sp = sqrt(((n1-1)*pow(s1,2)+(n2-1)*pow(s2,2))/(n1+n2-2));
			t = sqrt(n1*n2/(n1+n2))*(x1-x2)/sp;
			if(t < 2.02) return true;
			else
			{
				std::cout<<"\033[1;35mWARNING: Sample rejected after Student T test.\033[0m"<<std::endl;
				rejectedInfo[i].addData(traverseInfo[i].numSamples,
										traverseInfo[i].mean,
										traverseInfo[i].stdDeviation);

				return false;
			}
		}

		bool cochranTTest(int i)
		{
			double n1, n2, s1, s2, x1, x2, tcal, ttab;
			n1 = criteriaInfo[i].numSamples;
			n2 = traverseInfo[i].numSamples;
			s1 = criteriaInfo[i].stdDeviation;
			s2 = traverseInfo[i].stdDeviation;
			x1 = criteriaInfo[i].mean;
			x2 = traverseInfo[i].mean;

			tcal = (x1-x2)/sqrt(pow(s1,2)/n1+pow(s2,2)/n2);
			ttab = (2.02*pow(s1,2)/n1 + 2.22*pow(s2,2)/n2)/(pow(s1,2)/n1+pow(s2,2)/n2);
			if(tcal < ttab) return true;
			else
			{
				std::cout<<"\033[1;35mWARNING: Sample rejected after Cochran T test.\033[0m"<<std::endl;
				return false;
			}

		}
    };

//__DYMU_PATH_PLANNER_CLASS__
    class DyMuPathPlanner
    {
        private:
          // Global Layer
            std::vector< std::vector<globalNode*> > global_layer;
          // Dimensions
            uint num_nodes_X;
            uint num_nodes_Y;
          // Global Resolution (same for X and Y axii)
            double global_res;
          // Offset of Global Node (0,0) with respect to world frame
            std::vector<double> global_offset;
          // Local Resolution (same for X and Y axii)
            double local_res;
          // Local Nodes contained within a Global Node edge
            uint res_ratio;

          // Local Repairing Parameters
          // Risk Distance = Risky proximity to obstacles
            double risk_distance;
          // Distance to reconnect after first safe waypoint
            // - Only relevant for Conservative Approach (Hazard Avoidance mode)
            double reconnect_distance;
          // Parameter that controls curvature of resulting repaired paths
            double risk_ratio;
          // Vector of slope values from 0 to the maximum feasible slope
            std::vector<double> slope_range;
          // Vector of available locomotion modes
            std::vector<std::string> locomotion_modes;
          // Approach chosen to do the repairings
            repairingAproach repairing_approach;

		  // Cost ratio updating
		  // Vector of terrain segmentation info
			std::vector<segmentedTerrain> terrainVector;
		  // Vector of criteria weights
			std::vector<double> weights;
		  // Number of segmented terrains
			int numTerrains;
		  // Number of parameters used for cost updating
			int numCriteria;
		  // Base speed to obtain costs
			double baseSpeed;
        public:
          // -- PARAMETERS --

          // The narrow band is formed by those open nodes that have been
          // already visited by the FMM solver
            std::vector<globalNode*> global_narrowband;
          // Compilation of all visited Global Nodes
            std::vector<globalNode*> global_propagated_nodes;
          // Same concept of narrow band as in the global computation
            std::vector<localNode*> local_narrowband;
          // Obstacle Local Nodes whose associated risk must be computed
            std::vector<localNode*> local_expandable_obstacles;
          // Same concept as in global
            std::vector<localNode*> local_propagated_nodes;
          // The last computed path
            std::vector<base::Waypoint> current_path;
          // LookUp Table containing cost values per terrain and slope value
            std::vector<double> cost_lutable;
          // Global Node containing the goal
            globalNode * global_goal;
          // Local Node containing the position of the agent
            localNode * local_agent;
          // Total Cost needed by the agent to reach the goal
            double remaining_total_cost;

          //Index to current path in which the local path meets global
            int reconnecting_index;

          // -- FUNCTIONS --
          // Class Constructor
            DyMuPathPlanner(double risk_distance,
                            double reconnect_distance,
                            double risk_ratio,
                            repairingAproach input_approach);
          // Class Destructor
            ~DyMuPathPlanner();
          // Initialization of Global Layer using Elevation and Terrain Maps
            bool initGlobalLayer(double globalres,  double localres,
                                 uint num_nodes_X, uint num_nodes_Y,
                                 std::vector<double> offset);

            bool setCostMap(std::vector< std::vector<double> > cost_map);

            bool computeCostMap(std::vector<double> costData,
                            std::vector<double> slope_values,
                            std::vector<std::string> locomotionModes,
                            std::vector< std::vector<double> > elevation,
                                std::vector< std::vector<double> > terrainMap);

          // Slope is calculated for nodeTarget
            void calculateSlope(globalNode* nodeTarget);

            void calculateNominalCost(globalNode* nodeTarget, int range,
                                      int numLocs);

            void smoothCost(globalNode* nodeTarget);

          // Returns global node (i,j)
            globalNode* getGlobalNode(uint i, uint j);

            bool setGoal(base::Waypoint wGoal);

            bool computeTotalCostMap(base::Waypoint wPos);
            bool computeEntireTotalCostMap();

            void resetTotalCostMap();
            void resetGlobalNarrowBand();

            void propagateGlobalNode(globalNode* nodeTarget);

            globalNode* minCostGlobalNode();

            globalNode* getNearestGlobalNode(base::Pose2D pos);
            globalNode* getNearestGlobalNode(base::Waypoint wPos);

            std::vector<base::Waypoint> getPath(base::Waypoint wPos);

            bool computeGlobalPath(base::Waypoint wPos);

            base::Waypoint computeNextGlobalWaypoint(base::Waypoint& wPos,
                                                       double tau);

            void gradientNode(globalNode* nodeTarget, double& dnx, double& dny);

            double interpolate(double a, double b, double g00, double g01,
                               double g10, double g11);

            std::string getLocomotionMode(base::Waypoint wPos);

            std::vector< std::vector<double> > getTotalCostMatrix();
            std::vector< std::vector<double> > getGlobalCostMatrix();
            std::vector< std::vector<double> > getHazardDensityMatrix();
            std::vector< std::vector<double> > getTrafficabilityMatrix();

            double getTotalCost(base::Waypoint wInt);

          // LOCAL PATH REPAIRING

            void createLocalMap(globalNode* gNode);

            localNode* getLocalNode(base::Pose2D pos);
            localNode* getLocalNode(base::Waypoint wPos);

            void subdivideGlobalNode(globalNode* gNode);

            bool computeLocalPlanning(base::Waypoint wPos,
                                  base::samples::frame::Frame traversabilityMap,
                                  double res,
                                  std::vector<base::Waypoint>& trajectory,
                                  base::Time &localTime);

            void expandRisk();

            localNode* maxRiskNode();

            void propagateRisk(localNode* nodeTarget);

            void setHorizonCost(localNode* horizonNode);

            double getTotalCost(localNode* lNode);

            localNode * computeLocalPropagation(base::Waypoint wInit, base::Waypoint wOvertake);

            void propagateLocalNode(localNode* nodeTarget);

            localNode* minCostLocalNode(double Tovertake, double minC);

            localNode* minCostLocalNode(localNode* reachNode);

            std::vector<base::Waypoint> getLocalPath(localNode * lSetNode,
                                                     base::Waypoint wInit,
                                                     double tau);




            bool computeLocalWaypointGDM(base::Waypoint& wPos, double tau);
            base::Waypoint computeLocalWaypointDijkstra(localNode * lNode);


            void gradientNode(localNode* nodeTarget, double& dnx, double& dny);





            bool evaluatePath(uint starting_index);

            bool isBlockingObstacle(localNode* obNode, uint& maxIndex, uint& minIndex);

            //void repairPath(std::vector<base::Waypoint>& trajectory, uint minIndex, uint maxIndex);
            int repairPath(base::Waypoint wInit, uint index);

            //TODO: Make function to change repairing approach anytime

            std::vector< std::vector<double> > getRiskMatrix(base::Waypoint rover_pos);
            std::vector< std::vector<double> > getDeviationMatrix(base::Waypoint rover_pos);

			// COST RATIO UPDATING AFTER TRAVERSE (CoRa)
	
			bool initCoRaMethod(int numTerrains_, int numCriteria_, std::vector<double> weights_);
			int getTerrain(base::Waypoint currentPos);
			bool fillTerrainInfo(int terrainId, std::vector<double> data);
			
			std::vector<double> updateCost();
			std::vector<double> computeCostRatio();
    };

} // end namespace

#endif // _PATHPLANNING_LIBRARIES_HPP_
