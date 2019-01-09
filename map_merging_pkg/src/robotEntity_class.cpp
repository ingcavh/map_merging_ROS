#include "ros/ros.h"
#include "std_msgs/String.h"
#include "nav_msgs/OccupancyGrid.h"
#include <kbk_service/robotEstatus.h>
#include <sensor_msgs/image_encodings.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <cmath>
#include <tf/transform_datatypes.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <kbk_service/ordenAgv.h>

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

#include "feature_gridMap.cpp"

class tf_merge{
	public:
		tf_merge();
		bool merge; //control if the map was merged into another one
		bool child_bool; //control if the map is merged into another one (child) or it is merged with another one (father)
		double x, y, z, qx, qy, qz, qw;
		std::string father;
		std::string child;
};
tf_merge::tf_merge(){
	merge = false;
	child_bool = false;
	x = 0;	y = 0;	z = 0;
	qx = 0;	qy = 0;	qz = 0;	qw = 1;
}

class robotEntity {
	public:
		robotEntity();
		void begin(std::string name_robot, unsigned char b );
		void mapCallback(const nav_msgs::OccupancyGridConstPtr& msg);
		void estatusCallback(const kbk_service::robotEstatusConstPtr& msg);
		void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
		void orderPub(int cmd, std::vector<geometry_msgs::PoseStamped> target_);
		bool getPoseMap(geometry_msgs::PoseStamped pIn, geometry_msgs::PoseStamped &pOut);
		bool world_to_map(double wx, double wy, int &mx, int &my, double oX, double oY, double r, int w, int h);
		void map_to_world(int mx, int my, double &wx, double &wy, double oX, double oY, double r);
		void updateMergedMap(nav_msgs::OccupancyGrid &map_);
		void setUpdateMergedMap(nav_msgs::OccupancyGrid msg, std::vector<int> wi_he);

		void set_merge(bool a);
		bool get_merge();

		//Mapas
		nav_msgs::OccupancyGrid map, map_ori, merged_map;
		//Puntos Caracteristicos de Mapa
		std::vector<cv::KeyPoint> features;
		//Descriptores asociados a Puntos Caracteristicos de Mapa
		std::vector<cv::Mat> descriptors;
		//Imagenes del robot
		cv::Mat img_, img_ori;
		//Nombres de Frames
		std::string name_robot, name_merged, global_frame_id, world_frame_id, map_frame_id, odom_frame_id;
		//Robot Estatus
		kbk_service::robotEstatus estatus;  

		geometry_msgs::PoseStamped pose;

		int numTask;

		//8 bits for 8 maps
		uint8_t robots;
		//Booleana de Mapa Recibido
		bool map_bool;
		//Booleana de Publicacion trama World->Map
		bool world_tf_bool;
		//Booleana de disponibilidad de datos
		bool feature_bool;
		//Lista de mapas mezclados con este robot
		std::vector<int> idx_maps; 
		//Transformada tf world->map
		tf_merge tf_mapToWorld;
		//Variable de uso de mapa mezclado con mapa recibido a traves de mapCallback
		bool using_merged_map;

		double allDistTravel, allDistTravel_last;

	private:
		//Suscriptores a Mapas publicados por Robots
		ros::Subscriber map_sub, estatus_sub, pose_sub;
		ros::Publisher  orden_pub;//, merged_pub;
		featureGridMap fea_proc;
		bool init_tf;
		tf::TransformListener tf_;
	  tf::StampedTransform trans_;
		tf::Quaternion q;
		int hei_min, wid_min;

		
		unsigned char bitOr, bitAnd;
};
robotEntity::robotEntity(){
	world_tf_bool = false; 
	map_bool = false;
	robots = 1;
	feature_bool = false;
	init_tf = false;
	using_merged_map = false;
	hei_min = 0;
	wid_min = 0;
	allDistTravel = 0;
	allDistTravel_last = 0;
	numTask = 0;
}

void robotEntity::set_merge(bool a) { tf_mapToWorld.merge = a; }
bool robotEntity::get_merge(){ return tf_mapToWorld.merge; }

void robotEntity::begin(std::string name_robot1, unsigned char b ){
	ros::NodeHandle n;
	name_robot = name_robot1;
	name_merged = name_robot1;
	std::stringstream ss;
	ss << "/map_" << name_robot;
	std::cout << " Map: " << ss.str();
	map_frame_id = ss.str();
	ss.str("");
	ss << "world_" << name_robot;
	world_frame_id = ss.str();
	global_frame_id = ss.str();
	std::cout << " World: " << ss.str();
	ss.str("");
	ss << "kobuki_" << name_robot << "_tf/base_link";
	odom_frame_id = ss.str();
	ss.str("");
	ss << "robot_status_" << name_robot;
	std::string rob_top = ss.str();
	ss.str("");
	ss << "slam_out_pose_" << name_robot;
	std::string pose_top = ss.str();
	ss.str("");
	ss << "mod_opera_app_" << name_robot;
	std::string orden_top = ss.str();
	ss.str("");
	map_sub 		= n.subscribe<nav_msgs::OccupancyGrid>( map_frame_id,1,&robotEntity::mapCallback,this );
	//estatus_sub = n.subscribe<kbk_service::robotEstatus>( rob_top,1,&robotEntity::estatusCallback,this );
	pose_sub 		= n.subscribe<geometry_msgs::PoseStamped>( pose_top,1,&robotEntity::poseCallback,this );
	orden_pub   = n.advertise<kbk_service::ordenAgv>(orden_top,2,true);

	//ss.str("");
	//ss << "map_merged_" << name_robot;
	//merged_pub   = n.advertise<nav_msgs::OccupancyGrid>(ss.str(),2,true);
	//ss.str("");
  
	bitOr = b;
	bitAnd = ~b;
	std::cout << " or: " << (int)bitOr << " and: " << (int)bitAnd << std::endl;
}
void robotEntity::mapCallback(const nav_msgs::OccupancyGridConstPtr& msg){	
	map.header = msg->header;	
	map.info   = msg->info;	
	map.data   = msg->data;
	map_ori.header =  msg->header;
	map_ori.info   =  msg->info;
	map_ori.data   =  msg->data;
	features.clear();

	std::cout << " Size map: " << (int)map.data.size() << std::endl;
	if(tf_mapToWorld.merge && !using_merged_map){
		using_merged_map = true;
		updateMergedMap(map);
		using_merged_map = false;
		std::cout << " UPDATE Size map: " << (int)map.data.size() << std::endl;
	}

	
	int i, x, y, data_;
	int map_height = map.info.height;
	int map_width  = map.info.width;
	int map_size   = map.data.size();
	cv::Mat img(map_height, map_width, CV_8UC1);
	img_ori = cv::Mat(map_height, map_width, CV_8UC1, double(127));
	for(i=0; i<map_size; i++){
		y = (int) i/map_width;
		x = i - y*map_width;
		//Binarizando la imagen
		data_ = (int) map.data.at(i); 
		switch( data_ ){
			case 100  :
				img.at<uchar>( cv::Point( x, y ) ) = 255;	
				img_ori.at<uchar>( cv::Point( x, y ) ) = 0;	
				break;
			case 0  :
				img.at<uchar>( cv::Point( x, y ) ) = 0;
				img_ori.at<uchar>( cv::Point( x, y ) ) = 255;
				break;
			default :
				img.at<uchar>( cv::Point( x, y ) ) = 0;
				break;
		}
	}
	//std::cout << "->L Grid(W,H): (" << map_luna.info.width << " , " << map_luna.info.height << ") ^ Mat(W,H):( " << img.cols << " , " << img.rows << ")" << std::endl;

	//std::stringstream ss;
	//ss << "/home/ingcavh/map_image_bin.jpg";
	//cv::imwrite( ss.str(), img );
	//ss .str("");
	//ss << "/home/ingcavh/map_image_ori.jpg";
	//cv::imwrite( ss.str(), img_ori_jade );

	if(!init_tf){
		init_tf = true;	
		world_tf_bool = true;
		map_frame_id = map.header.frame_id;
		tf_mapToWorld.x = -map.info.origin.position.x;
		tf_mapToWorld.y = -map.info.origin.position.y; 
		tf_mapToWorld.z = -map.info.origin.position.z; 
		tf_mapToWorld.qx = -map.info.origin.orientation.x; 
		tf_mapToWorld.qy = -map.info.origin.orientation.y; 
		tf_mapToWorld.qz = -map.info.origin.orientation.z; 
		tf_mapToWorld.qw = -map.info.origin.orientation.w; 	 
		tf_mapToWorld.father = world_frame_id;
		tf_mapToWorld.child = map_frame_id;
	}
	static tf::TransformBroadcaster br;
	tf::Transform transform;
	transform.setOrigin( tf::Vector3(tf_mapToWorld.x, tf_mapToWorld.y, tf_mapToWorld.z) );
	tf::Quaternion q(tf_mapToWorld.qx, tf_mapToWorld.qy, tf_mapToWorld.qz, tf_mapToWorld.qw);
	transform.setRotation(q);
	br.sendTransform(tf::StampedTransform(transform, map.header.stamp/*ros::Time::now()*/, tf_mapToWorld.father, tf_mapToWorld.child));


	//available = available & bitAnd;
	feature_bool = false;
	//Detector
	features.clear();
	features = fea_proc.feature_detector( img, img_, name_robot );
	//Descriptor
	int tam_fea = features.size();
	if(tam_fea!=0){
		descriptors.clear();
		fea_proc.feature_descriptor( img_, features, descriptors );
		for( i=0; i<tam_fea; i++ ){
				features.at( i ).pt.x*=2;
				features.at( i ).pt.y*=2;
		}
		std::cout << " ->" << name_robot << " *Fea: " << tam_fea << " *Descp: " << descriptors.size() << std::endl;
		if(descriptors.size()>5 && !descriptors.empty()){
			//available = available | bitOr;
			feature_bool = true;
		}
	}
	map_bool = true;
}
void robotEntity::estatusCallback(const kbk_service::robotEstatusConstPtr& msg){
	estatus.status = msg->status;
	estatus.distanceTraveled = msg->distanceTraveled;
	estatus.pathDistance = msg->pathDistance;
	estatus.kno_cells = msg->kno_cells;
	estatus.unk_cells = msg->unk_cells;
	if(allDistTravel_last != estatus.pathDistance){
		allDistTravel_last = estatus.pathDistance;
		allDistTravel += estatus.pathDistance;
	}
}

void robotEntity::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg){
	pose.header = msg->header;
	pose.pose = msg->pose;
}

void robotEntity::orderPub(int cmd, std::vector<geometry_msgs::PoseStamped> target_){
		geometry_msgs::PoseStamped target, aux;
		bool pubOK = false;
		int mx, my;
		double oX = map_ori.info.origin.position.x;
		double oY = map_ori.info.origin.position.y;
		double r  = map_ori.info.resolution;
		double w  =	map_ori.info.width;
		double h 	= map_ori.info.height;  
 
		for( int i=0; i<target_.size(); i++){
			if(getPoseMap( target_.at(i), aux ) ){	
				if( world_to_map(aux.pose.position.x, aux.pose.position.y, mx, my, oX, oY, r, w, h) ){
					int idx = mx+my*w;
					if(map_ori.data.at(idx)!=0){
						break;
					}
					pubOK = true;
					target = aux;
					
					std::cout << "   " << i << " ->PoseO: " << target_.at(i).pose.position.x << " - " << target_.at(i).pose.position.y;
					std::cout << " ->PoseN: " << aux.pose.position.x << " - " << aux.pose.position.y  << std::endl;
				}
			}
		}	
		if(cmd!=1 && cmd!=0){
			pubOK = true;
		}
		if(pubOK){
			kbk_service::ordenAgv ord;
			//agv_service::robot rob;
			std::vector<geometry_msgs::PoseStamped> poses;
			poses.push_back(target);
			ord.poses = poses; 
			ord.robotOp.orden = cmd;
			numTask++;
			ord.robotOp.numOp = numTask; 
			ord.usuID.data = "mergerMaster";
			orden_pub.publish( ord );
		}

}
bool robotEntity::getPoseMap(geometry_msgs::PoseStamped pIn, geometry_msgs::PoseStamped &pOut){
		std::stringstream ss;
		ss << "/map_" << name_merged;
		if( tf_.waitForTransform(world_frame_id, ss.str(), ros::Time(0), ros::Duration(0.025)) ){
			tf::StampedTransform tf_1;
			tf_.lookupTransform(world_frame_id, ss.str(), ros::Time(0), tf_1); 

			tf::Quaternion q_1 = tf_1.getRotation(); 
			tf::Matrix3x3 m_1( q_1 );	
			tf::Quaternion q_2(pIn.pose.orientation.x, pIn.pose.orientation.y, pIn.pose.orientation.z, pIn.pose.orientation.w); 
			tf::Matrix3x3 m_2( q_2 );
			tf::Quaternion q_3(-tf_mapToWorld.qx, -tf_mapToWorld.qy, -tf_mapToWorld.qz, -tf_mapToWorld.qw);
			tf::Matrix3x3 m_3( q_3 );
			double yaw_ = 0;
			double roll, pitch, yaw;
			m_1.getRPY(roll, pitch, yaw);
			yaw_+=yaw;

			pOut.pose.position.x = pIn.pose.position.x*cos(yaw) - pIn.pose.position.y*sin(yaw) + tf_1.getOrigin().x() - tf_mapToWorld.x;
			pOut.pose.position.y = pIn.pose.position.x*sin(yaw) + pIn.pose.position.y*cos(yaw) + tf_1.getOrigin().y() - tf_mapToWorld.y; 
			pOut.pose.position.z = 0; 

			//std::cout << " ->Transform wor->glo x:" << tf_1.getOrigin().x() << " y: " << tf_1.getOrigin().y() << " yaw: " << yaw << std::endl;
			m_2.getRPY(roll, pitch, yaw);
			yaw_+=yaw;
			//std::cout << " ->Transform glo->odo x:" << pIn.pose.position.x << " y: " << pIn.pose.position.y << " yaw: " << yaw << std::endl;
			m_3.getRPY(roll, pitch, yaw);
			yaw_+=yaw;
			//std::cout << " ->Transform map->wor x:" <<-tf_mapToWorld.x << " y: " << -tf_mapToWorld.y << " yaw: " << yaw << std::endl;
			geometry_msgs::Quaternion q_ = tf::createQuaternionMsgFromYaw(yaw_);
			pOut.pose.orientation = q_;
			//std::cout << " ->Transform FOUND x:" << pOut.pose.position.x << " y: " << pOut.pose.position.y << " yaw: " << yaw_ << std::endl;
			
			return true;
		}
		return false;
}

bool robotEntity::world_to_map(double wx, double wy, int &mx, int &my, double oX, double oY, double r, int w, int h){
	mx = (int)( (wx - oX) / r);
	my = (int)( (wy - oY) / r);
	if ( mx < w && my < h ){
		  return true;
	}
	return false;
}
void robotEntity::map_to_world(int mx, int my, double &wx, double &wy, double oX, double oY, double r){
	wx = oX + (mx + 0.5) * r;
	wy = oY + (my + 0.5) * r;
}


void robotEntity::updateMergedMap(nav_msgs::OccupancyGrid &map_){
  try{    
		tf_.waitForTransform(global_frame_id, world_frame_id, ros::Time(0), ros::Duration(0.05) );
	  tf_.lookupTransform(global_frame_id, world_frame_id, ros::Time(0), trans_);
	}catch( tf::LookupException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
			return;
	}catch( tf::ConnectivityException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
			return;
	}catch( tf::ExtrapolationException& ex ){
  		ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
			return;
	}
	q = trans_.getRotation(); 
  tf::Matrix3x3 m_( q );
  double roll, pitch, yaw;
  m_.getRPY(roll, pitch, yaw);	
	double tx = trans_.getOrigin().x()/map_.info.resolution;
	double ty = trans_.getOrigin().y()/map_.info.resolution;
	//std::cout << " ->Transform x:" << tx << " y: " << ty << " yaw: " << yaw << std::endl;
	double sang = sin(yaw);
	double cang = cos(yaw); 
	int i_, j_, x_, y_;
	int id1, id2, d1, d2;
	int w1 = map_.info.width;
	int h1 = map_.info.height;
	int w2 = merged_map.info.width;
	int h2 = merged_map.info.height;
	for(i_=0; i_<h1; i_++ ){
		for(j_=0; j_<w1; j_++ ){
			x_ = j_*cang - i_*sang + tx + wid_min;
			y_ = j_*sang + i_*cang + ty + hei_min;
			id1 = j_+(i_*w1);
			id2 = x_+(y_*w2);	
			if(id1>map_.data.size()){
				std::cout << "index " << id1 << " map_" << std::endl;
			}
			if(id2>merged_map.data.size()){
				std::cout << "index " << id2  << " (" << w2 << "-" << h2 << ")" << " (" << x_ << "-" << y_ << ") fuera de merged_map" << std::endl;
			}
			d1 = map_.data.at(id1);
			d2 = merged_map.data.at(id2);
			if(d1 == 100 || d1==0){
				merged_map.data.at(id2) = d1;
			}
		}
	}
	map_.info = merged_map.info;
	map_.data = merged_map.data;
	std::cout << "Updating merged_map to " << name_robot << std::endl;
}
void robotEntity::setUpdateMergedMap(nav_msgs::OccupancyGrid msg, std::vector<int> wi_he){
		merged_map = msg;
		wid_min = wi_he.at(0);
		hei_min = wi_he.at(1);
		std::cout << "Setting merged_map to " << name_robot << " tam: " <<	merged_map.data.size() << " w: " <<	wid_min << " h: " <<	hei_min << std::endl;
}

