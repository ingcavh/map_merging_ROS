#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <kbk_service/robot.h>
#include <kbk_service/mapaRed.h>
#include <sensor_msgs/image_encodings.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <cmath>
#include "tf/tf.h"
#include <tf/transform_broadcaster.h>
#include <iostream>
#include <geometry_msgs/PoseStamped.h>


#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include "opencv/cv.h"
#include "opencv2/core/core.hpp"
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include "opencv2/xfeatures2d.hpp"
#include <opencv2/xfeatures2d/nonfree.hpp>

#include "matching_class.cpp"
#include "robotEntity_class.cpp"
#include "map_layers.cpp"



//unir imagenes en una sola - retornar matriz tranformación
class map_merging{

public:
	//Constructor
	map_merging();
	//Init Objects
	void init_objects(std::vector <std::string> names_);
	//Get RobotEntity
	robotEntity *get_robotEntity(int idx);
	//Verifying names
	bool has_only_spaces(std::string& str);
	//Método de emparejamiento de caracteristicas
	void matching_feature( const ros::TimerEvent& e );

	void tf_merged_timer( const ros::TimerEvent& e );
	void tf_merged();
	bool pose_robot(robotEntity *r, geometry_msgs::PoseStamped &pose);
	int8_t decision_making(robotEntity *rT, robotEntity *rQ);
	void order_generator(robotEntity *rT, robotEntity *rQ);

	void update_merge( const ros::TimerEvent& e );

	tf::TransformListener listener;
  tf::StampedTransform trans_;
	tf::Quaternion q_;

protected:
	//Timer for matching_feature execution
	ros::Timer timer_merging; 
	//Timer for tf_tree execution
	ros::Timer timer_tf_merging;
	//Timer for update_merge execution
	ros::Timer timer_update_merge;
	MapLayers layerObj;
	
	matchingClass matchingObj;
	bool matching;
	tf_merge tf_aux;
	std::vector< tf_merge >tf_tree_merge_vect;
	std::vector<int>idx_robots;
	robotEntity *robot1, *robot2, *robot3, *robot4, *robot5, *robot6, *robot7, *robot8;
	int tam_r;

	ros::Publisher merge_pub;
};

map_merging::map_merging(){
	std::string names[] ={"jade", "luna", "mars", " ", "    "};
	std::vector<std::string> na;
	for(int i=0; i<(int)sizeof(names)/sizeof(names[0]); i++){
		na.push_back(names[i]);
	}
	init_objects( na );
	matching = false;
	std::cout << "Map merging creado" << std::endl;
}

void map_merging::init_objects(std::vector <std::string> names_){
	int tam = names_.size();
	if(tam==0){	
		std::cout << "ERROR_MERGING: Size of robotNames must be greater than 0, but less than 8." << std::endl;
		return;
	}
	if(tam>8){	
		std::cout << "ERROR_MERGING: Size of robotNames must be smaller than 8." << std::endl;
		return;
	}
	idx_robots.clear();
	robot1 = new robotEntity;
	robot2 = new robotEntity;
	robot3 = new robotEntity;
	robot4 = new robotEntity;
	robot5 = new robotEntity;
	robot6 = new robotEntity;
	robot7 = new robotEntity;
	robot8 = new robotEntity;
	for(int i=0; i<tam; i++){
		std::string s = names_.at(i);
		switch( i+1 ){
			case 1  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 1 );
						robot1->begin(s, 0b00000001);	
					}
				}
				break;
			case 2  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 2 );
						robot2->begin(s, 0b00000010);
					}
				}
				break;
			case 3  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 3 );
						robot3->begin(s, 0b00000100);
					}
				}
				break;	
			case 4  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 4 );
						robot4->begin(s, 0b00001000);
					}
				}
				break;	
			case 5  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 5 );
						robot5->begin(s, 0b00010000);
					}
				}
				break;	
			case 6  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 6 );
						robot6->begin(s, 0b00100000);
					}
				}
				break;	
			case 7  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 7 );
						robot7->begin(s, 0b01000000);	
					}
				}
				break;
			case 8  :
				if(!s.empty() && s!=""){
					if(has_only_spaces(s)){
						idx_robots.push_back( 8 );
						robot8->begin(s, 0b10000000);
					}
				}
				break;		
			default :
				break;
		}
	}
	tam_r = (int)idx_robots.size();
	std::cout << "LISTA DE ROBOtS " << idx_robots.size() << std::endl;
	//Timer matching, tf_tree y update_merge
	ros::NodeHandle n;
	timer_merging = n.createTimer(ros::Duration( 1.0 ), &map_merging::matching_feature, this, false );
	timer_tf_merging = n.createTimer(ros::Duration( 0.01 ), &map_merging::tf_merged_timer, this, false );
	timer_update_merge = n.createTimer(ros::Duration( 15.0 ), &map_merging::update_merge, this, false );

	merge_pub = n.advertise<nav_msgs::OccupancyGrid>("map_merged", 2,true);
}

robotEntity *map_merging::get_robotEntity(int idx){
	robotEntity *rta;
	switch( idx ){
		case 1  :
			rta = robot1;
			break;
		case 2  :
			rta = robot2;
			break;
		case 3  :
			rta = robot3;
			break;	
		case 4  :
			rta = robot4;
			break;
		case 5  :
			rta = robot5;
			break;	
		case 6  :
			rta = robot6;
			break;
		case 7  :
			rta = robot7;
			break;
		case 8  :
			rta = robot8;
			break;		
		default :
			break;
	}
	return rta;
}

bool map_merging::has_only_spaces(std::string& str){
	char* token = strtok(const_cast<char*>(str.c_str()), " ");
	while (token != NULL){   
	  if (*token != ' '){   
      return true;
	  }   
	}   
	return false;
}
	
void map_merging::matching_feature( const ros::TimerEvent& e ){
	//Matching de Mapas de robots disponibles.
	std::vector< std::vector< double > > distances;
	std::vector< double > rta;
	std::stringstream ss;
	//if(matching) return;
	

	for(int i=0; i<tam_r; i++){
		robotEntity *rT = get_robotEntity( idx_robots.at(i) );
		if(rT-> tf_mapToWorld.child_bool) continue;
		if(rT->robots==tam_r) return; //matching se completó
		/*if(rT->world_tf_bool){
			rT->world_tf_bool = false;
			tf_tree_merge_vect.push_back( rT->tf_mapToWorld );
		}	*/
		for(int j=i+1; j<tam_r; j++){
			robotEntity *rQ = get_robotEntity( idx_robots.at(j) );
			if(rQ->tf_mapToWorld.child_bool) continue;	
			/*if(rQ->world_tf_bool){
				rQ->world_tf_bool = false;
				tf_tree_merge_vect.push_back( rQ->tf_mapToWorld );
			}*/	
			if(!rT->descriptors.empty() && !rQ->descriptors.empty() && rT->feature_bool && rQ->feature_bool){
				std::cout << "Analizando " << rT->name_robot << " y " << rQ->name_robot << std::endl;
				matchingObj.compute_distances(rQ->descriptors, rT->descriptors, distances);
				matching = matchingObj.compute_matching( distances, rQ->features, rT->features, rQ->img_ori, rT->img_ori, rta);
				if( matching ){
					std::cout << "MATCHING ENCONTRADO **************" << std::endl;
					std::cout << "	->tx: " << rta.at(0) << " ->ty: " << rta.at(1) << " ->ang: " << rta.at(2) << std::endl;

					ss << rT->name_merged << "_" << rQ->name_merged;
					rT->name_merged = ss.str();	
					rQ->name_merged = ss.str();
					ss.str("");
					ss << "/map_" << rT->name_merged;
			
					nav_msgs::OccupancyGrid map_;
					map_.header.seq = rT->map.header.seq;
					map_.header.stamp = rT->map.header.stamp;
					map_.header.frame_id = ss.str();
					map_.info.map_load_time = rT->map.info.map_load_time;
					map_.info.resolution = rT->map.info.resolution;	
					std::vector<int> wi_he_;
					matchingObj.merging_images(rQ->img_ori, rT->img_ori, rta, map_, wi_he_);
					//map_.info.origin.position.x=0; 
					//map_.info.origin.position.y=0; 
					merge_pub.publish( map_ );
					layerObj.updateMap(map_);
					
					std::cout << " ->LAYERS*** " << std::endl;

					//Creando trama entre mapas mezclados
					tf_merge tf_aux;
					tf_aux.x = rta.at(0)*rT->map.info.resolution;
					tf_aux.y = rta.at(1)*rT->map.info.resolution; 
					tf_aux.z = 0; 
					geometry_msgs::Quaternion q = tf::createQuaternionMsgFromYaw( rta.at(2) );
					tf_aux.qx = q.x; 
					tf_aux.qy = q.y; 
					tf_aux.qz = q.z; 
					tf_aux.qw = q.w; 	 
					tf_aux.father = rT->world_frame_id;
					tf_aux.child =  rQ->world_frame_id;	

					//Include tf transforms to tf_tree
					//tf_tree_merge_vect.push_back( rT->tf_mapToWorld );
					//tf_tree_merge_vect.push_back( rQ->tf_mapToWorld );
					tf_tree_merge_vect.push_back( tf_aux );
					
					tf_merge tf_;
					tf_.x = -map_.info.origin.position.x - wi_he_.at(0)*rT->map.info.resolution;
					tf_.y = -map_.info.origin.position.y - wi_he_.at(1)*rT->map.info.resolution;
					tf_.father = rT->tf_mapToWorld.father;
					tf_.child = map_.header.frame_id;
					tf_tree_merge_vect.push_back( tf_);
					//Update tf_tree					
					tf_merged();

					//Actualizando info en cada robot
					rT->idx_maps.push_back( idx_robots.at(j) );
					for(int k=0; k<rQ->idx_maps.size(); k++){
						robotEntity *rAux = get_robotEntity( rQ->idx_maps.at(k) );
						rAux->setUpdateMergedMap(map_, wi_he_);			
						rAux->global_frame_id = rT->world_frame_id;
						rT->idx_maps.push_back( rQ->idx_maps.at(k) );
					}
					rT->using_merged_map = true;
					rT->setUpdateMergedMap(map_, wi_he_);
					rT->using_merged_map = false;
					rT->tf_mapToWorld.merge = true;
					rT->robots+=rQ->robots; 
					rQ->using_merged_map = true;
					rQ->setUpdateMergedMap(map_, wi_he_);
					rQ->using_merged_map = false;		
					rQ->tf_mapToWorld.merge = true;
					rQ->tf_mapToWorld.child_bool = true;						
					rQ->global_frame_id = rT->world_frame_id;
					std::cout << std::endl;
					std::cout << std::endl;
					
					//Este el algoritmo de tomar decisiones, listo para mandar objetivo
					order_generator(rT, rQ);
					
							//geometry_msgs::PoseStamped pose_rQ;
							//pose_robot(rQ, pose_rQ);
					
				}
			}
			//delete rQ;
		}
		//delete rT;
	}
}

void map_merging::tf_merged_timer( const ros::TimerEvent& e ){
	tf_merged();
}
void map_merging::tf_merged(){
	
	for(int i=0; i< tf_tree_merge_vect.size(); i++){
		tf_aux = tf_tree_merge_vect.at(i);
		static tf::TransformBroadcaster br;
		tf::Transform transform;
		transform.setOrigin( tf::Vector3(tf_aux.x, tf_aux.y, tf_aux.z) );
		tf::Quaternion q(tf_aux.qx, tf_aux.qy, tf_aux.qz, tf_aux.qw);
		transform.setRotation(q);
		br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), tf_aux.father, tf_aux.child));
	}
	//std::cout << "Publiqué " << tf_tree_merge_vect.size() << std::endl;
}

void map_merging::update_merge( const ros::TimerEvent& e ){

	geometry_msgs::PoseStamped pose_;
	std::vector<geometry_msgs::PoseStamped> target;
std::cout << " TIMER UPDATING MERGER ********************** " << std::endl;
	for(int i=0; i<tam_r; i++){
		robotEntity *rT = get_robotEntity( idx_robots.at(i) );
		if(rT-> tf_mapToWorld.child_bool || rT->idx_maps.size()==0) continue;
		if(rT->robots==tam_r) return; //matching se completó

		for(int k=0; k<rT->idx_maps.size(); k++){
			robotEntity *rAux = get_robotEntity( rT->idx_maps.at(k) );
			if(rT->merged_map.data.size() != rAux->merged_map.data.size() || rT->using_merged_map || rAux->using_merged_map) continue;
			std::vector<int8_t> m1 = rT->merged_map.data;
			std::vector<int8_t> m2 = rAux->merged_map.data;
			std::vector<int8_t> m3;
			int tam = (int)m1.size();
			for(int j=0; j<tam; j++){
				int8_t d1 = m1.at(j);
				int8_t d2 = m2.at(j);
				if(d1==100 || d2==100){
						m3.push_back( 100 );
				}else if(d1==0 || d2==0){
						m3.push_back( 0 );
				}else{
						m3.push_back( -1 );
				}
			}
			if(!rT->using_merged_map){
				rT->merged_map.data = m3;
			}
			if(!rAux->using_merged_map){
				rAux->merged_map.data = m3;
			}
			//Con los mapas mezclados de cada robot, llamar procedimientos para generar nuevos puntos de trayectoria.

/*			
			int map_height = rT->merged_map.info.height;
			int map_width  = rT->merged_map.info.width;
			int map_size   = m3.size();
			cv::Mat imgu(map_height, map_width, CV_8UC1, double(127));
			for(int i=0; i<map_size; i++){
				int y = (int) i/map_width;
				int x = i - y*map_width;
				//Binarizando la imagen
				int data_ = (int) m3.at(i); 
				switch( data_ ){
					case 100  :	
						imgu.at<uchar>( cv::Point( x, y ) ) = 0;	
						break;
					case 0  :
						imgu.at<uchar>( cv::Point( x, y ) ) = 255;
						break;
					default :
						break;
				}
			}
			std::stringstream ss;
			ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/imgu.jpg";
			cv::imwrite( ss.str(), imgu );
*/


			layerObj.updateMap(rAux->merged_map);
			pose_robot(rAux, pose_);
			layerObj.setPose( pose_ );
			layerObj.inflate_obstacle_map();
			if(!layerObj.get_target( target )){
				rAux->orderPub( 13, target/*pose_rQ/*target.at( target.size()-1)*/ );
			}
		}
		pose_robot(rT, pose_);
		layerObj.setPose( pose_ );
		layerObj.inflate_obstacle_map();
		if(!layerObj.get_target( target )){
			rT->orderPub( 13, target/*pose_rQ/*target.at( target.size()-1)*/ );
		}

	}
	std::cout << " ******************************************** " << std::endl;
}

bool map_merging::pose_robot(robotEntity *r, geometry_msgs::PoseStamped &pose){
	//Update tf_tree					
	tf_merged();
  try{
		std::stringstream ss;
		ss << "/map_" << r->name_merged;
		std::cout << " ->FRAMES :" << ss.str() << " - " << r->world_frame_id << " - " << r->map_frame_id << " - " << r->odom_frame_id << std::endl;
		if( listener.waitForTransform(ss.str(), r->world_frame_id, ros::Time(0), ros::Duration(0.025)) ){
			tf::StampedTransform tf_1;
			listener.lookupTransform(ss.str(), r->world_frame_id, ros::Time(0), tf_1); 

			tf::Quaternion q_1 = tf_1.getRotation(); 
			tf::Matrix3x3 m_1( q_1 );	
			tf::Quaternion q_2(r->pose.pose.orientation.x, r->pose.pose.orientation.y, r->pose.pose.orientation.z, r->pose.pose.orientation.w); 
			tf::Matrix3x3 m_2( q_2 );
			tf::Quaternion q_3(r->tf_mapToWorld.qx, r->tf_mapToWorld.qy, r->tf_mapToWorld.qz, r->tf_mapToWorld.qw);
			tf::Matrix3x3 m_3( q_3 );
			double yaw_ = 0;
			double roll, pitch, yaw;
			m_1.getRPY(roll, pitch, yaw);
			yaw_+=yaw;

			double x_ = r->pose.pose.position.x + r->tf_mapToWorld.x;
			double y_	= r->pose.pose.position.y + r->tf_mapToWorld.y;		
			pose.pose.position.x = x_*cos(yaw) - y_*sin(yaw) + tf_1.getOrigin().x();
			pose.pose.position.y = x_*sin(yaw) + y_*cos(yaw) + tf_1.getOrigin().y(); 
			pose.pose.position.z = 0; //r->pose.pose.position.z + r->tf_mapToWorld.z + tf_1.getOrigin().z(); 

			std::cout << " ->Transform glo->wor x:" << tf_1.getOrigin().x() << " y: " << tf_1.getOrigin().y() << " yaw: " << yaw << std::endl;
			m_2.getRPY(roll, pitch, yaw);
			yaw_+=yaw;
			std::cout << " ->Transform map->odo x:" << r->pose.pose.position.x << " y: " << r->pose.pose.position.y << " yaw: " << yaw << std::endl;
			m_3.getRPY(roll, pitch, yaw);
			yaw_+=yaw;
			std::cout << " ->Transform wor>-map x:" <<r->tf_mapToWorld.x << " y: " << r->tf_mapToWorld.y << " yaw: " << yaw << std::endl;
			geometry_msgs::Quaternion q_ = tf::createQuaternionMsgFromYaw(yaw_);
			pose.pose.orientation = q_;
			std::cout << " ->Transform FOUND x:" << pose.pose.position.x << " y: " << pose.pose.position.y << " yaw: " << yaw_ << std::endl;
			return true;
		}else{
			std::cout << " ERROR 1" << std::endl;
			return false;
		}
	}catch( tf::LookupException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "No Transform available. Error looking up robot pose: %s\n", ex.what());
			return false;
	}catch( tf::ConnectivityException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "Connectivity. Error looking up robot pose: %s\n", ex.what());
			return false;
	}catch( tf::ExtrapolationException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "Extrapolation. Error looking up robot pose: %s\n", ex.what());
			return false;
	}
}

int8_t map_merging::decision_making(robotEntity *rT, robotEntity *rQ){
	int rt_status = rT->estatus.status;	//0: apagado. 1:operando. -1:fallo. 2: pausado. -2: detenido.
	int rq_status = rQ->estatus.status;
	double rt_drest = (double)rT->estatus.distanceTraveled/rT->estatus.pathDistance;
	double rq_drest = (double)rQ->estatus.distanceTraveled/rQ->estatus.pathDistance;
	double rt_dtrav = (double)rT->allDistTravel;
	double rq_dtrav = (double)rQ->allDistTravel;
	double rt_explo = (double)rT->estatus.kno_cells/(rT->estatus.kno_cells+rT->estatus.unk_cells);
	double rq_explo = (double)rQ->estatus.kno_cells/(rQ->estatus.kno_cells+rQ->estatus.unk_cells);
	//std::cout << " ->Decision maker " << std::endl;
	int8_t rta = 0; //0: no desviar ningun robot; 1: desviar rt; 2: pausar rt; -1: desviar rq; -2: pausar rq
	if(rt_status ==1){
		if(rt_drest<rq_drest){
			//ENviar objetivo a rt
			rta = 1;
		}else if(rq_status ==1){
			if(rt_dtrav<rq_dtrav){
				if(rt_explo<rq_explo){
					//ENviar objetivo a rt
					rta = 1;
				}else{
					//ENviar objetivo a rq
					rta = -1;
				}
			}else{
				//ENviar objetivo a rq
				rta = -1;
			}
		} 
	}else if(rq_status ==1){
		if(rq_drest<rt_drest){
			//ENviar objetivo a rq
			rta = -1;
		}else if(rt_status ==1){
			if(rq_dtrav<rt_dtrav){
				if(rq_explo<rt_explo){
					//ENviar objetivo a rq
					rta = -1;
				}else{
					//ENviar objetivo a rt
					rta = 1;
				}
			}else{
				//ENviar objetivo a rt
				rta = 1;
			}
		} 
	}else if(rt_status <=0 && rq_status == 2){
		//Sacar de pausa a rq
		//ENviar objetivo a rq
		rta = -2;
	}else if(rq_status <=0 && rt_status == 2){
		//Sacar de pausa a rt
		//ENviar objetivo a rt
		rta = 2;
	}
	return rta;
}
void map_merging::order_generator(robotEntity *rT, robotEntity *rQ){
	std::cout << " ->INIT Desicion: " << std::endl;
	int8_t decision = decision_making(rT, rQ);
	std::cout << " ->Desicion: " << (int)decision << std::endl;
	std::vector<geometry_msgs::PoseStamped> target;
	if(decision<0){
		if(decision==-1){
			geometry_msgs::PoseStamped pose_rQ;
			pose_robot(rQ, pose_rQ);
			layerObj.setPose( pose_rQ );
			layerObj.inflate_obstacle_map();
			target.clear();
			if(layerObj.get_target( target )){
				rQ->orderPub( 1, target/*pose_rQ/*target.at( target.size()-1)*/ );
				std::cout << " ->RQ Target x:" << target.at( target.size()-1).pose.position.x << " y: " << target.at( target.size()-1).pose.position.y << std::endl;
			}else{
				rQ->orderPub( 13, target/*pose_rQ/*target.at( target.size()-1)*/ );
			}
		}else if(decision==-2){
				rQ->orderPub( 15, target/*pose_rQ/*target.at( target.size()-1)*/ );
		}
	}else if(decision>0){
		if(decision==1){
			geometry_msgs::PoseStamped pose_rT;
			pose_robot(rT, pose_rT);
			layerObj.setPose( pose_rT );
			layerObj.inflate_obstacle_map();
			target.clear();
			if(layerObj.get_target( target )){
				rT->orderPub( 1, target /*pose_rT/*target.at( target.size()-1)*/ );
				std::cout << " ->RT Target x:" << target.at( target.size()-1).pose.position.x << " y: " << target.at( target.size()-1).pose.position.y << std::endl;
			}else{
				rT->orderPub( 13, target /*pose_rT/*target.at( target.size()-1)*/ );
			}
		}else if(decision==2){
				rT->orderPub( 15, target /*pose_rT/*target.at( target.size()-1)*/ );
		}
	}


}

int main(int argc, char **argv) {
	/*std::cout << "Cargando Imagen" << std::endl;
	cv::Mat in1 = cv::imread("/home/ingcavh/Workspace/workspace_qt/prueba_carlos_qt/imagenes/lena.png");

	std::cout << "Imagen Cargada" << std::endl;
	cv::namedWindow("Test1 Image");
	if(!in1.empty()){ 
		cv::Mat src_gray_car;
    	cv::cvtColor(in1,src_gray_car, CV_BGR2GRAY );
		cv::imshow("Test1 Image", src_gray_car);
		cv::waitKey(10);
		std::cout << "Imagen Contiene Algo, No es Vacia" << std::endl;
	}else{
		std::cout << "Imagen Vacia" << std::endl;
	}
	
	std::cout << "Imagen Mostrada" << std::endl;
*/
	ros::init(argc, argv, "map_merging_pkg");

	map_merging ec;

	ros::spin();


  return 0;
}
