
#include <ros/ros.h>
#include <string>
#include "std_msgs/String.h"
#include "std_msgs/Header.h"
#include <geometry_msgs/PoseStamped.h>
#include "nav_msgs/OccupancyGrid.h"
#include <math.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>

class MapLayers{
	public:
		MapLayers();
		void init(); // Inicializa el objeto de control de movimiento.
		void setPose( geometry_msgs::PoseStamped pose );
		void updateMap(nav_msgs::OccupancyGrid mapIn);

		//Methods to transform map datas
			bool worldToMap(const double &wx, const double &wy, int &mx, int &my) ;
			void mapToWorld(int &mx, int &my, double &wx, double &wy);
			int getIndex(int mx, int my);
			void indexToCells(int index, int& mx, int& my);
		//Make a plan and trajectory
			bool get_target(std::vector<geometry_msgs::PoseStamped> &plan);
			bool exploration_map(const std::vector<geometry_msgs::PoseStamped>  &goals); // Build a map for exploration
			bool get_trajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan);
		//Obstacle Map Generation
			void inflate_obstacle_map();
			void fill_obstacle_area(int mx, int my, int inflate);
		//Frontier Map Exploration
			bool findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers);
			bool isFrontier(int point);
			bool isFreeFrontiers(int point);
			bool isFrontierReached(int point);
			double getYawToUnknown(int point);
		//States of a simple cell
			bool isValid(int point);
			bool isValidMap(int mx, int my);
			bool isFree(int point);
			bool isOccupied(int point);
			bool isUnknown(int point);
			bool isDangerous(int point);
		//Relation among points, from map to gridCell
			void getStraightPoints(int point, int points[]);
			void getDiagonalPoints(int point, int points[]);
			void getAdjacentPoints(int point, int points[]);
			int left(int point);
			int upleft(int point);
			int up(int point);
			int upright(int point);
			int right(int point);
			int downright(int point);
			int down(int point);
			int downleft(int point);
		//Printer Layers
			void print_explo_map();
			void print_cost_map();
			void print_frontier_map( std::vector<geometry_msgs::PoseStamped> goals );
		//Setting Up the Object Class
			void setupObjects();
			void occupancy_data();
			void deleteMapData();
			void resetMaps();

		//Publishers Map layers
			ros::Publisher  map_explo_pub, map_cost_pub, map_frontier_pub;
			nav_msgs::OccupancyGrid map, map_explo, map_frontier, map_cost;
			bool mapBool, pub_costMap_bool, pub_frontierMap_bool, pub_exploMap_bool;
			double map_ori_X;
			double map_ori_Y;
			double map_ori_W;
			int letal_cost_cte;	//valor de celda "letal"
			int inflated_cost_cte;	//valor de celda dentro de radio de inflado de obstaculos
			int free_cost_cte;	//valor de celda "libre"
			int unk_cost_cte;	//valor de celda "desconocida"
			double laser_range;	//distancia maxima del laser
			double inflated_obstacle_rad;	//radio de inflado de obstaculos
			double p_dist_for_goal_reached;	//distancia minima para considerar una frontera de explorarion

			int map_width;					//width of the map	
			int map_height;				//height of the map
			double map_resol;				//resolution of the map
			int num_map_cells;				//number of cells of a map
			//unsigned int p_dist_for_goal_reached;	//minimum distance beetween to goals
			//unsigned int inflated_obstacle_rad;		//inflated obstacle radius
			int fre_cells;					//free cells
			int unk_cells;					//unknown cells
			int occ_cells;					//occupied cells

		//array that has the costmap information
			boost::shared_array<unsigned int> occupancy_grid_array_;	
		//array that has the obstacle info to avoid collisions between the robot and the world	
			boost::shared_array<unsigned int> obstacle_trans_array_;
		//array that has the exploration information to know the known environment
			boost::shared_array<unsigned int> exploration_;

			boost::shared_array<unsigned int> exploration_trans_array_;	
			geometry_msgs::PoseStamped pose_robot;
};

