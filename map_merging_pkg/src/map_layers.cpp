#include <map_merging_pkg/map_layers.h>
#include <queue>

//Class COnstructor	*******************************************************************************************************
MapLayers::MapLayers(){	/*std::cout << "71921 del 2005 oficio para verificar y leer.  art 206 numeral 4 del estatuto tributario.  Constructor MapLayers _merged"<<std::endl;*/	
	ros::NodeHandle n;	
	map_frontier_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_frontier_merged",1,true);
	map_cost_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_cost_merged",1,true);
	map_explo_pub = n.advertise<nav_msgs::OccupancyGrid>("/map_exploration_merged",1,true);
	pub_costMap_bool = true;
	pub_frontierMap_bool = true;
	pub_exploMap_bool = true;
	inflated_obstacle_rad =0.3;
	p_dist_for_goal_reached = 1.0;
	laser_range = 10.0;
	init();
}

void MapLayers::init( ){
	
	mapBool = false;
	letal_cost_cte = 254;
	inflated_cost_cte = 252;
	free_cost_cte = 0;
	unk_cost_cte = 255;
	occ_cells = 0;
	fre_cells = 0;
	unk_cells = 0;

	std::cout << "****	Initialized Map Layer OBJECT	****" /*<< p_robot_base_frame_ << " - " << p_global_frame_ */<< std::endl;
}

void MapLayers::setPose( geometry_msgs::PoseStamped pose ){
	pose_robot = pose;
}

void MapLayers::updateMap(nav_msgs::OccupancyGrid mapIn){
	map = mapIn; 
	map_width = mapIn.info.width;
	map_height = mapIn.info.height;	
	map_resol = mapIn.info.resolution;		
	num_map_cells = map_width*map_height;
	map_ori_X = mapIn.info.origin.position.x;
	map_ori_Y = mapIn.info.origin.position.y;
	map_ori_W = mapIn.info.origin.orientation.w;

	setupObjects();
	//std::cout << "Iniciando objetos " << std::endl;
	// create obstacle tranform
	if(pub_exploMap_bool){	print_explo_map();	}
	//std::cout << "Mostrando mapa explorado " << std::endl;
	//inflate_obstacle_map();
	//std::cout << "Calculando mapa de Obstaculos " << std::endl;
	//print_cost_map();
	//std::cout << "Mostrando mapa de Obstaculos " << std::endl;
	mapBool = true;
	//std::cout << "Mapa recibido en Map layers " << std::endl;
}

bool MapLayers::worldToMap(const double &wx, const double &wy, int &mx, int &my) {
	if(mapBool){
		mx = (int)( (wx - map_ori_X) / map_resol);
		my = (int)( (wy - map_ori_Y) / map_resol);
		if ( mx < map_width && my < map_height ){   return true;   }
		return false;
	}
}

void MapLayers::mapToWorld(int &mx, int &my, double &wx, double &wy){
	 if(mapBool){
	     wx = map_ori_X + (mx + 0.5) * map_resol;
	     wy = map_ori_Y + (my + 0.5) * map_resol;
	 }
}

int MapLayers::getIndex(int mx, int my){
	return my * map_width + mx;
}
  
void MapLayers::indexToCells(int index, int& mx, int& my){
	my = index / map_width;
	mx = index - (my * map_width);
}


//GETTING A TARGET  ************************************************************************************************
bool MapLayers::get_target(std::vector<geometry_msgs::PoseStamped> &plan){

	std::fill_n(exploration_trans_array_.get(), num_map_cells, UINT_MAX);
 	//int mx, my;
	//worldToMap(pose_robot.pose.position.x, robot_pose.pose.position.y,mx,my);
	//int goal_point = getIndex(mx,my);
	//std::cout<<"Starting POSE Robot "<<goal_point<< " "<<mx<<","<<my<<std::endl;
	plan.clear();
	std::vector<geometry_msgs::PoseStamped> goals;
 	// search for frontiers
	if( !findFrontiers(goals) ){
		ROS_INFO("exploration: no frontiers have been found! starting inner-exploration");
		return false;
	}
	// make a plan
	if( !exploration_map(goals) ){
		return false;
	}
	// get a plan
	if( !get_trajectory(pose_robot, plan) ){
		return false;
	}
	if( plan.empty() || plan.size()<1 ){
		return false;
	}			
	ROS_INFO("Merged: path exploration has been found! ->plansize: %u", (unsigned int)plan.size());
	return true;
}
//************************************************************************************************************************ 
//Build an exploration Map with Goals ************************************************************************************
bool MapLayers::exploration_map(const std::vector<geometry_msgs::PoseStamped>  &goals){
	std::fill_n(exploration_trans_array_.get(), num_map_cells, 14*num_map_cells);//UINT_MAX);    	
	unsigned int init_cost = 0;
	unsigned int straight = 10, diagonal = 14;
 	int mxG, myG;
	std::queue<int> myqueue;

	for( int i=0; i< goals.size(); i++){
		worldToMap(goals[i].pose.position.x, goals[i].pose.position.y, mxG, myG);
		int goal_point = getIndex(mxG,myG);
		if( !isFreeFrontiers(goal_point) ){	
			//std::cout << "LA FRONTERA NO ES LIBRE: "<< (int)cavh.occupancy_grid_array_[goal_point] <<std::endl;
			continue; //Si la frontera ya no es frontera o es una frontera falsa
		}
		myqueue.push(goal_point);
		exploration_trans_array_[goal_point] = init_cost;
	}	
	while(myqueue.size()){
		int point = myqueue.front();
		myqueue.pop();
		unsigned int minimum = exploration_trans_array_[point];
		int straightPoints[4];
		getStraightPoints(point,straightPoints);
		int diagonalPoints[4];
		getDiagonalPoints(point,diagonalPoints);

		for (int i = 0; i < 4; ++i) {
			if(obstacle_trans_array_[straightPoints[i]]  < inflated_cost_cte){
				if (isFree(straightPoints[i]) ) {
					unsigned int neighbor_cost = minimum + straight + obstacle_trans_array_[straightPoints[i]];
					if (exploration_trans_array_[straightPoints[i]] > neighbor_cost) {
						exploration_trans_array_[straightPoints[i]] = neighbor_cost;
						myqueue.push(straightPoints[i]);
					}
				}
			}
			if(obstacle_trans_array_[diagonalPoints[i]]  < inflated_cost_cte){
				if (isFree(diagonalPoints[i]) ) {
					unsigned int neighbor_cost = minimum + diagonal + obstacle_trans_array_[diagonalPoints[i]];
					if (exploration_trans_array_[diagonalPoints[i]] > neighbor_cost) {
						exploration_trans_array_[diagonalPoints[i]] = neighbor_cost;
						myqueue.push(diagonalPoints[i]);
					}
				}
			}
		}			
	}
	return true;
}
//************************************************************************************************************************
//Get Trajectory from Goals **********************************************************************************************
bool MapLayers::get_trajectory(const geometry_msgs::PoseStamped &start, std::vector<geometry_msgs::PoseStamped> &plan){
	 int mxS, myS, mxG, myG;
	if(!worldToMap(start.pose.position.x, start.pose.position.y, mxS, myS)){
		std::cout << "Pose inicial esta por fuera del rango "<<std::endl;
		return false;
	}
	int currentPoint = getIndex(mxS,myS);
	int nextPoint = currentPoint;
	geometry_msgs::PoseStamped trajPoint;
	int cont=0;
	while( exploration_trans_array_[currentPoint] != 0 && (num_map_cells+num_map_cells)>cont ){
		cont++;
		int thisDelta;
		int adjacentPoints[8];
		getAdjacentPoints(currentPoint,adjacentPoints);
		int maxDelta = 0;
		for(int i = 0; i < 8; ++i){
			if(isFree(adjacentPoints[i]) && obstacle_trans_array_[adjacentPoints[i]]  < inflated_cost_cte){
				thisDelta = exploration_trans_array_[currentPoint] - exploration_trans_array_[adjacentPoints[i]];
				if(thisDelta >= maxDelta){
					maxDelta = thisDelta;
					nextPoint = adjacentPoints[i];
				}
			}
		}
		// This happens when there is no valid exploration transform data at the start point for example
		if(maxDelta == 0){
			ROS_WARN("No path to the goal could be found by following gradient!");
			return false;
		}
		// make trajectory point
	 	int sx,sy,gx,gy;
		indexToCells(( int)currentPoint,sx,sy);
		indexToCells(( int)nextPoint,gx,gy);
		double wx,wy;
		mapToWorld(sx,sy,wx,wy);
		trajPoint.pose.position.x = wx;
		trajPoint.pose.position.y = wy;
		trajPoint.pose.position.z = 0.0;
		// assign orientation
		int dx = gx-sx;
		int dy = gy-sy;
		double yaw_path = std::atan2(dy,dx);
		trajPoint.pose.orientation.x = 0.0;
		trajPoint.pose.orientation.y = 0.0;
		trajPoint.pose.orientation.z = sin(yaw_path*0.5f);
		trajPoint.pose.orientation.w = cos(yaw_path*0.5f);
		plan.push_back(trajPoint);
		currentPoint = nextPoint;
		maxDelta = 0;
	}
	if(exploration_trans_array_[currentPoint] != 0){	
		std::cout << "NUNCA se alcanzo pose: "<< currentPoint <<" "<< nextPoint <<" "<< cont << std::endl;
		return false;
	}
	//plan.push_back(goal);
	ROS_DEBUG("END: getTrajectory. Plansize %u", (unsigned int)plan.size());
	return !plan.empty();
}	
//*************************************************************************************************************************
//Inflate the Map with Obstacle Radius ************************************************************************************
void MapLayers::inflate_obstacle_map(){
	int inflate = round(inflated_obstacle_rad/map_resol);
	int mx, my;
	worldToMap(pose_robot.pose.position.x, pose_robot.pose.position.y,mx,my);
	//std::cout << "Inflated Map: "<< inflate  << " ->" << mx << " ->"<< my <<std::endl;
	int laser_cells = round(laser_range/map_resol);
	double dist = sqrt(laser_cells*laser_cells+laser_cells*laser_cells);
	double dTeta = 1; // 1 grad
	double dRad = 0.9; // distance
	double cx = 0.5;
	double cy = 0.5;
	int numTeta = 360/dTeta;
	int numRad = (int)(dist/dRad);
	for(double rad = 0; rad < numRad; rad++){
	  for(double teta = 0; teta < numTeta; teta++){
	  	int x = (int)( mx+cx+(rad*dRad)*cos( ( (teta*dTeta)*M_PI )/180 ) );
	  	int y = (int)( my+cy+(rad*dRad)*sin( ( (teta*dTeta)*M_PI )/180 ) );
		if (isValidMap(x,y)){
			int index = getIndex(x,y);
			if (isOccupied(index)){	
				fill_obstacle_area(x, y, inflate);						
			}
		}
	  }
	}
/*
	int mx,my;
	int o=0;
	for(unsigned int i = 0; i < num_map_cells; ++i){
		if(isOccupied(i)){
			obstacle_trans_array_[i] = letal_cost_cte;
			indexToCells(i, mx, my);
			fill_obstacle_area(mx, my, inflate);
			o++;
		}
	}

*/
	//std::cout << "Numero de celdas de radio: "<< inflate <<"  "<< o <<std::endl;
	if(pub_costMap_bool){	print_cost_map();	}
}

void MapLayers::fill_obstacle_area(int mx, int my, int inflate){
	int index;
	if(true){
		double dist = sqrt(inflate*inflate+inflate*inflate);
		double dTeta = 1.3; // 1 grad
		double dRad = 0.9; // distance
		double cx = 0.5;
		double cy = 0.5;
		int numTeta = 360/dTeta;
		int numRad = (int)(dist/dRad);
		//std::cout << "NUMRAD: "<< numRad << std::endl;
		for(double rad = (int)/*(dist/(2*dRad))*/0; rad < numRad; rad++){
			for(double teta = 0; teta < numTeta; teta++){
				int x = (int)( mx+cx+(rad*dRad)*cos( ( (teta*dTeta)*M_PI )/180 ) );
				int y = (int)( my+cy+(rad*dRad)*sin( ( (teta*dTeta)*M_PI )/180 ) );
				index = getIndex(x,y);
				if (isValid(index) && obstacle_trans_array_[index] < inflated_cost_cte){	
					if( (x>(mx-inflate) && x<(mx+inflate)) && (y>(my-inflate) && y<(my+inflate)) ){
						obstacle_trans_array_[index] = inflated_cost_cte;
					}else{
						obstacle_trans_array_[index] = ((dist-(rad*dRad))/dist)*300;
					}							
				}
			}
		}
	/*
	for(int i=(mx-inflate); i<(mx+inflate); i++){
		for(int j=(my-inflate); j<(my+inflate); j++){
			index = getIndex(i,j);
			if (isValid(index) && obstacle_trans_array_[index] < inflated_cost_cte){	
			obstacle_trans_array_[index] = inflated_cost_cte;
			}
		}
	}
	*/
	}else{
		double dTeta = 1; // 1 grad
		double dRad = 0.7; // distance
		double cx = 0.5;
		double cy = 0.5;
		int numTeta = 360/dTeta;
		int numRad = (int)(inflate/dRad);
		//std::cout << "NUMRAD: "<< numRad << std::endl;
		for(double rad = 0; rad < (numRad); rad++){
			for(double teta = 0; teta < numTeta; teta++){
				int x = (int)( mx+cx+(rad*dRad)*cos( ( (teta*dTeta)*M_PI )/180 ) );
				int y = (int)( my+cy+(rad*dRad)*sin( ( (teta*dTeta)*M_PI )/180 ) );
				index = getIndex(x,y);
				if (isValid(index) && obstacle_trans_array_[index] != letal_cost_cte){	
					obstacle_trans_array_[index] = inflated_cost_cte;
				}
			}
		}
	}
}
//*************************************************************************************************************************
//Find Frontiers **********************************************************************************************************
bool MapLayers::findFrontiers(std::vector<geometry_msgs::PoseStamped> &frontiers){
	frontiers.clear();
	// Frontiers list in the occupancy grid
	std::vector<int> allFrontiers;
	// check for all cells in the occupancy grid whether or not they are frontier cells
	int cnt = 0;
	for( int i = 0; i < num_map_cells; ++i){
		if(isFrontier(i) && obstacle_trans_array_[i] < inflated_cost_cte){
			allFrontiers.push_back(i);  //add point i to the queue
			cnt++;
		}
	}
	for( int i = 0; i < allFrontiers.size(); ++i){
		if(!isFrontierReached(allFrontiers[i])){ // check if the frontier was reached
			geometry_msgs::PoseStamped finalFrontier;
			double wx,wy;
			 int mx,my;
			indexToCells(allFrontiers[i], mx, my);
			mapToWorld(mx,my,wx,wy);
			finalFrontier.pose.position.x = wx;
			finalFrontier.pose.position.y = wy;
			finalFrontier.pose.position.z = 0.0;
			double yaw = getYawToUnknown(getIndex(mx,my));
			finalFrontier.pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
			frontiers.push_back(finalFrontier);
		}
	}
	std::cout << "FRONTERAS ENCNTRADAS: "<< frontiers.size() <<std::endl;
	if(pub_frontierMap_bool){	print_frontier_map( frontiers );	}
	return (frontiers.size() > 0);
}

bool MapLayers::isFrontier(int point){
	if(isFreeFrontiers(point)){
		int adjacentPoints[8];
		getAdjacentPoints(point,adjacentPoints);
		for(int i = 0; i < 8; ++i){
			if(isValid(adjacentPoints[i])){
				if(occupancy_grid_array_[adjacentPoints[i]] == unk_cost_cte && obstacle_trans_array_[i] < inflated_cost_cte){
					int no_inf_count = 0;
					int noInfPoints[8];
					getAdjacentPoints(adjacentPoints[i],noInfPoints);
					for(int j = 0; j < 8; j++){
						if( isValid(noInfPoints[j]) && occupancy_grid_array_[noInfPoints[j]] == unk_cost_cte){
							++no_inf_count;
							if(no_inf_count > 3){
								return true;
							}
						}
					}
				}
			}
		}
	}
	return false;
} 

bool MapLayers::isFreeFrontiers(int point){
	if(isValid(point)){
		if(occupancy_grid_array_[point] == free_cost_cte){
			return true;
		}
	}
	return false;
}

bool MapLayers::isFrontierReached(int point){
	int fx,fy;
	double wfx,wfy;
	indexToCells(point,fx,fy);
	mapToWorld(fx,fy,wfx,wfy);
	double dx = pose_robot.pose.position.x - wfx;
	double dy = pose_robot.pose.position.y - wfy;
	if ( (dx*dx) + (dy*dy) < (p_dist_for_goal_reached*p_dist_for_goal_reached)) {
		return true;
	}
	return false;
}

// Used to generate direction for frontiers ****************************
double MapLayers::getYawToUnknown(int point){
	int adjacentPoints[8];
	getAdjacentPoints(point,adjacentPoints);
	int max_obs_idx = 0;
	for(int i = 0; i < 8; ++i){
		if(isValid(adjacentPoints[i])){
			if(occupancy_grid_array_[adjacentPoints[i]] == unk_cost_cte){
				max_obs_idx = i;
			}
		}
	}
	int orientationPoint = adjacentPoints[max_obs_idx];
	int sx,sy,gx,gy;
	indexToCells(point,sx,sy);
	indexToCells(orientationPoint,gx,gy);
	int x = gx-sx;
	int y = gy-sy;
	double yaw = std::atan2(y,x);
	return yaw;
}
//************************************************************************************************************************
// Funtions to Evaluate the current state of a cell **********************************************************************
bool MapLayers::isValid(int point){
  return (point>=0);
}

bool MapLayers::isValidMap(int mx, int my){
  return (isValid(mx) && isValid(my) && mx < map_width && my < map_height);
}

bool MapLayers::isFree(int point){
  if(isValid(point)){
	  if(occupancy_grid_array_[point] == free_cost_cte){			
	    return true;
	  }
  }			
  return false;
}

bool MapLayers::isOccupied(int point){
  if(isValid(point)){
	  if(occupancy_grid_array_[point] == letal_cost_cte){
	    return true;
	  }
  }
  return false;
}

bool MapLayers::isDangerous(int point){
  if(isValid(point)){
	  if(obstacle_trans_array_[point] >= inflated_cost_cte){			
	    return true;
	  }
  }			
  return false;
}

bool MapLayers::isUnknown(int point){
  if(isValid(point)){
	  if(occupancy_grid_array_[point] == unk_cost_cte){			
	    return true;
	  }
  }			
  return false;
}
//************************************************************************************************************************
/*
 These functions calculate the index of an adjacent point (left,upleft,up,upright,right,downright,down,downleft) to the
 given point. If there is no such point (meaning the point would cause the index to be out of bounds), -1 is returned.
*/
void MapLayers::getStraightPoints(int point, int points[]){
	points[0] = left(point);
	points[1] = up(point);
	points[2] = right(point);
	points[3] = down(point);
}

void MapLayers::getDiagonalPoints(int point, int points[]){
	points[0] = upleft(point);
	points[1] = upright(point);
	points[2] = downright(point);
	points[3] = downleft(point);
}

void MapLayers::getAdjacentPoints(int point, int points[]){
	points[0] = upright(point);//1
	points[1] = downright(point);//3
	points[2] = downleft(point);//5
	points[3] = upleft(point);//7
	points[4] = up(point);//0
	points[5] = right(point);//2
	points[6] = down(point);//4
	points[7] = left(point);//6
}

int MapLayers::left(int point){
	if((point % map_width != 0)){
		return point-1;
	}
	return -1;
}

int MapLayers::upleft(int point){
	if((point % map_width != 0) && (point >= (int)map_width)){
		return point-1-map_width;
	}
	return -1;
}

int MapLayers::up(int point){
	if(point >= (int)map_width){
		return point-map_width;
	}
	return -1;
}

int MapLayers::upright(int point){
	if((point >= (int)map_width) && ((point + 1) % (int)map_width != 0)){
		return point-map_width+1;
	}
	return -1;
}

int MapLayers::right(int point){
	if((point + 1) % map_width != 0){
		return point+1;
	}
	return -1;
}

int MapLayers::downright(int point){
	if(((point + 1) % map_width != 0) && ((point/map_width) < (map_height-1))){
		return point+map_width+1;
	}
	return -1;
}

int MapLayers::down(int point){
	if((point/map_width) < (map_height-1)){
		return point+map_width;
	}
	return -1;
}

int MapLayers::downleft(int point){
	if(((point/map_width) < (map_height-1)) && (point % map_width != 0)){
		return point+map_width-1;
	}
	return -1;
}
//*************************************************************************************************************************
//Print Maps-Layers *******************************************************************************************************
void MapLayers::print_explo_map(){
	//std::cout << "Publishing Exploration Map"<<std::endl;
	std::vector<int8_t> aux; 
	map_explo.header = map.header;
	map_explo.info = map.info;
	for(int i=0; i<num_map_cells;i++){
		aux.push_back((int8_t)exploration_[i] );
	}
	map_explo.data = aux;
	map_explo_pub.publish( map_explo );
}

void MapLayers::print_cost_map(){
	//std::cout << "Publishing Cost Map CAVH"<<std::endl;
	std::vector<int8_t> aux; 
	map_cost.header = map.header;
	map_cost.info = map.info;
	for(int i=0; i<num_map_cells;i++){
		aux.push_back((int8_t)obstacle_trans_array_[i] );
	}
	map_cost.data = aux;
	map_cost_pub.publish( map_cost );
}

void MapLayers::print_frontier_map( std::vector<geometry_msgs::PoseStamped> goals ){
	std::vector<int8_t> aux; 
	map_frontier.header = map.header;
	map_frontier.info = map.info;
	//Calculate Security area
	boost::shared_array<unsigned int> frontier_trans_array_;
	frontier_trans_array_.reset(new unsigned int[num_map_cells]);
	std::fill_n(frontier_trans_array_.get(), num_map_cells, UINT_MAX);
	int mx,my;
	for( int i = 0; i < goals.size(); ++i){
		worldToMap(goals[i].pose.position.x,goals[i].pose.position.y,mx,my);
		int goal_point = getIndex(mx,my);
		frontier_trans_array_[goal_point] =100;
	}
	for(int i=0; i<num_map_cells;i++){
		aux.push_back((int8_t)frontier_trans_array_[i] );
	}
	map_frontier.data = aux;
	map_frontier_pub.publish( map_frontier );
}
//************************************************************************************************************************
//Methods to setup variables and objects *********************************************************************************
void MapLayers::setupObjects(){
	obstacle_trans_array_.reset(new unsigned int[num_map_cells]);
	occupancy_grid_array_.reset(new unsigned int[num_map_cells]);
	exploration_.reset(new unsigned int[num_map_cells]);
	exploration_trans_array_.reset(new unsigned int[num_map_cells]);
	resetMaps();
	occupancy_data();
}

void MapLayers::occupancy_data(){
	fre_cells = 0;
	unk_cells = 0;
	occ_cells = 0;
	for(unsigned int i=0; i<num_map_cells;i++){
		if((int)map.data.at(i) == 100){
			occupancy_grid_array_[i] = letal_cost_cte;
			occ_cells++;
		}else if((int)map.data.at(i) == 0){
			occupancy_grid_array_[i] = free_cost_cte;
			exploration_[i] = letal_cost_cte;
			fre_cells++;
		}else{
			occupancy_grid_array_[i] = unk_cost_cte;
			unk_cells++;
		}
	}
}

void MapLayers::deleteMapData(){
	obstacle_trans_array_.reset();
	occupancy_grid_array_.reset();
	exploration_.reset();
}

void MapLayers::resetMaps(){
  std::fill_n(obstacle_trans_array_.get(), num_map_cells, 0);
  std::fill_n(occupancy_grid_array_.get(), num_map_cells, UINT_MAX);
  std::fill_n(exploration_.get(), num_map_cells, 0);
}
//*************************************************************************************************************************



