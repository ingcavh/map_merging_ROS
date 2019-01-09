#include "ros/ros.h"
#include <geometry_msgs/Point32.h>
#include <sensor_msgs/PointCloud.h>
#include "boost/date_time/posix_time/posix_time.hpp"
#include <cmath>

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


class featureGridMap {
	public:
		featureGridMap();
		std::vector<cv::KeyPoint> feature_detector( cv::Mat img,  cv::Mat& img_out, std::string name );
		void feature_descriptor( cv::Mat img, std::vector<cv::KeyPoint>& feature, std::vector<cv::Mat>& descriptor_ );

	private:
		//Variables de Configuracion para Descriptor
		double dTeta, dRad;
		int radMax;
		double cx, cy;
		int numTeta, numRad, fea_cnt;
		double deg_to_rad;    // pi/180
		double rad_to_deg;    // 180/pi

		bool high_resol;
};

featureGridMap::featureGridMap(){
	//Descriptor
		dTeta = 1; 
    dRad = 0.7; 
    radMax = 25; 
    cx = 0.5;
    cy = 0.5;
    numTeta = 360/dTeta;
    numRad = (int)(radMax/dRad);
    if((360%(int)dTeta) != 0){
        dTeta = 1;
    }
		deg_to_rad = 0.017453293;    // pi/180
		rad_to_deg = 57.29577951;    // 180/pi

	high_resol = false;
	fea_cnt = 0;
}


std::vector<cv::KeyPoint> featureGridMap::feature_detector( cv::Mat img,  cv::Mat& img_out, std::string name ){
	int umbral = 41;//85
	//Mapa	
	cv::Mat img_;
	//Puntos Caracteristicos de Mapa
	std::vector<cv::KeyPoint> feature;
	if(high_resol){
		//original resolution
		img_ = img;
	}else{
		//Low resolution: 0.5*original
		cv::resize(img, img_, cv::Size(), 0.5, 0.5, cv::INTER_AREA);
		cv::threshold(img_,img_,umbral,255,0);
	}
	img_out = img_;
	boost::posix_time::ptime time_start(boost::posix_time::microsec_clock::local_time());
	//Preparacion mapa********************************************
  int col_dst = img_.cols;
  int row_dst = img_.rows;
	int8_t map_binarized[row_dst][col_dst];
  int8_t mdist [row_dst][col_dst];
	std::fill(mdist[0], mdist[0] + row_dst * col_dst, 0);
	int i,j;
	for( i=0; i<row_dst; i++ ){
	   for( j=0; j<col_dst; j++ ){
		   map_binarized[i][j] =  ( (int)img_.at<uchar>(i,j) > 1 ? 1:0);
	   }
	}

	int16_t numCor=60;
	int16_t maxDist = (img_.rows < img_.cols) ?  ((img_.rows/numCor)>1 ? (img_.rows/numCor):1) : ((img_.cols/numCor)>1 ? (img_.cols/numCor):1);           
	int x,y;
	int8_t A, A1, B, B1, P, P1, Q, Q1, r1, r2, r3, r4, rD, rS, r, sup, inf, esqP, der, izq;
	//r5-r6
	for ( y=1; y<(row_dst-1); y++) {
	   for ( x=1; x<(col_dst-1); x++) {
		   if(map_binarized[y][x]==0){
           A = map_binarized[y+1][x  ];		A1 = map_binarized[y-1][x  ];
           B = map_binarized[y  ][x+1];   B1 = map_binarized[y  ][x-1];
           P = map_binarized[y+1][x+1];   P1 = map_binarized[y-1][x-1];
           Q = map_binarized[y-1][x+1];  	Q1 = map_binarized[y+1][x-1];
		       r1 = ( A + A1 ); //left + right
		       r2 = ( P + P1 ); //downLeft + upRight
		       r3 = ( B + B1 ); //up + down
		       r4 = ( Q + Q1 ); //upLeft + downRight
		       rD = (r2+r4);
		       rS = (r1+r3);
		       r =  (rS+rD);
		       sup = (P + B + Q);
		       inf = (P1 + B1 + Q1);
		       esqP =(A + P + B);
		       der = (P + A + Q1);
		       izq = (Q + A1 + P1);
           if( r==5 && ( ( ( (sup+der)%2==0 && (inf+der)%2==0 ) && rS==2 ) || ( rS ==3 && (sup+der==2 ||sup+der==5) ) ) ){
               if(rS==3){
                   if(sup==0 && (map_binarized[y+2][x+1]==1 || map_binarized[y-2][x+1]==1 ) ){
                       cv::KeyPoint b;
                       b.pt.x = x;
                       b.pt.y = y;
                       b.angle = 90*deg_to_rad;
                       feature.push_back(b);
                   }else if(inf==0 && (map_binarized[y+2][x-1]==1 || map_binarized[y-2][x-1]==1 ) ){
                       cv::KeyPoint b;
                       b.pt.x = x;
                       b.pt.y = y;
                       b.angle = 270*deg_to_rad;
                       b.octave = r;
                       feature.push_back(b);
                   }else if(der==0 && (map_binarized[y+1][x+2]==1 || map_binarized[y+1][x-2]==1 ) ){
                       cv::KeyPoint b;
                       b.pt.x = x;
                       b.pt.y = y;
                       b.angle = 0*deg_to_rad;
                       b.octave = r;
                       feature.push_back(b);
                   }else if(izq==0 && (map_binarized[y-1][x+2]==1 || map_binarized[y-1][x-2]==1) ){
                       cv::KeyPoint b;
                       b.pt.x = x;
                       b.pt.y = y;
                       b.angle = 270*deg_to_rad;
                       b.octave = r;
                       feature.push_back(b);
                   }
               }else{
                   cv::KeyPoint b;
                   b.pt.x = x;
                   b.pt.y = y;
                   b.octave = r;
                   if(esqP==0){
                       b.angle = 45*deg_to_rad;
                   }else if(esqP==3){
                       b.angle = 225*deg_to_rad;
                   }else if(esqP==2 && sup==3){
                       b.angle = 315*deg_to_rad;
                   }else{
                       b.angle = 135*deg_to_rad;
                   }
                   feature.push_back(b);
               }
               for( int i=(y-maxDist)<0?0:(y-maxDist); i<(y+maxDist); i++)
                   for( int j=( (x-maxDist)<0?0:(x-maxDist) ); j<(x+maxDist); j++)
                       mdist[ i>row_dst-1 ? row_dst-1 : i ][ j>col_dst-1 ? col_dst-1:j ] = 1;
           }
           if( r==6 && (rS==3 && rD==3) ){
               cv::KeyPoint b;
               b.pt.x = x;
               b.pt.y = y;
               b.octave = r;
               if(A+P==0){
                   b.angle = 22.5*deg_to_rad;
               }else if (P+B==0){
                   b.angle = 67.5*deg_to_rad;
               }else if (B+Q==0){
                   b.angle = 112.5*deg_to_rad;
               }else if (Q+A1==0){
                   b.angle = 157.5*deg_to_rad;
               }else if (A1+P1==0){
                   b.angle = 202.5*deg_to_rad;
               }else if (P1+B1==0){
                   b.angle = 247.5*deg_to_rad;
               }else if (B1+Q1==0){
                   b.angle = 292.5*deg_to_rad;
               }else{
                   b.angle = 337.5*deg_to_rad;
               }
               feature.push_back(b);
               for( int i=(y-maxDist)<0?0:(y-maxDist); i<(y+maxDist); i++)
                   for( int j=( (x-maxDist)<0?0:(x-maxDist) ); j<(x+maxDist); j++)
                       mdist[ i>row_dst-1 ? row_dst-1 : i ][ j>col_dst-1 ? col_dst-1:j ] = 1;
           }
		   }
	   }
	}
	//r4-r7
  for (y=1; y<(row_dst-1); y++) {
     for (x=1; x < (col_dst-1); x++) {
         if(mdist[y][x] !=0){
             continue;
         }
         if(map_binarized[y][x]==0){	
             A = map_binarized[y+1][x  ];		A1 = map_binarized[y-1][x  ];
             B = map_binarized[y  ][x+1];   B1 = map_binarized[y  ][x-1];
             P = map_binarized[y+1][x+1];   P1 = map_binarized[y-1][x-1];
             Q = map_binarized[y-1][x+1];  	Q1 = map_binarized[y+1][x-1];
             r1 = ( A + A1 ); //left + right
             r2 = ( P + P1 ); //downLeft + upRight
             r3 = ( B + B1 ); //up + down
             r4 = ( Q + Q1 ); //upLeft + downRight
             rD = (r2+r4);
             rS = (r1+r3);
             r =  (rS+rD);
             sup = (P + B + Q);
             inf = (P1 + B1 + Q1);
             esqP =(A + P + B);
             der = (P + A + Q1);

             if( r==7){
	             cv::KeyPoint b;
	             b.pt.x = x;
	             b.pt.y = y;
	             b.octave = r;
	             if(A==0){
	                 b.angle =0*deg_to_rad;
	             }else if (A1==0){
	                 b.angle = 180*deg_to_rad;
	             }else if(B==0){
	                 b.angle = 90*deg_to_rad;
	             }else if (B1==0){
	                 b.angle = 270*deg_to_rad;
	             }else if(P==0){
	                 b.angle = 45*deg_to_rad;
	             }else if (P1==0){
	                 b.angle = 225*deg_to_rad;
	             }else if(Q==0){
	                 b.angle = 135*deg_to_rad;
	             }else{
	                 b.angle = 315*deg_to_rad;
	             }
	             feature.push_back(b);
	             for( int i=(y-maxDist)<0?0:(y-maxDist); i<(y+maxDist); i++)
	               	for( int j=( (x-maxDist)<0?0:(x-maxDist) ); j<(x+maxDist); j++)
	                 	mdist[ i>row_dst-1 ? row_dst-1 : i ][ j>col_dst-1 ? col_dst-1:j ] = 1;
             }
	           if( r==4 && esqP%2==0 && r2!=r4 && (rS==2 )  && (sup+inf==3) ){
	             cv::KeyPoint b;
	             b.pt.x = x;
	             b.pt.y = y;
	             b.octave = r;
	             if(r2==0){
	                 if(sup==2){
	                     b.angle =225*deg_to_rad;
	                 }else{
	                     b.angle = 45*deg_to_rad;
	                 }
	             }else{
	                 if(sup==2){
	                     b.angle = 315*deg_to_rad;
	                 }else{
	                     b.angle = 135*deg_to_rad;
	                 }
	             }
	             feature.push_back(b);
	             for( int i=(y-maxDist)<0?0:(y-maxDist); i<(y+maxDist); i++)
	                 for( int j=((x-maxDist)<0?0:(x-maxDist)); j<(x+maxDist); j++)
	             				mdist[ i>row_dst-1 ? row_dst-1 : i ][ j>col_dst-1 ? col_dst-1:j ] = 1;
	           }
         }
     }
  }

	boost::posix_time::ptime time_end(boost::posix_time::microsec_clock::local_time());
	boost::posix_time::time_duration duration(time_end - time_start);
	//std::cout << "Time: " << duration.total_microseconds() << std::endl;

	//std::cout << "Features: " << feature.size() << " " << img_out.cols << std::endl;

	cv::drawKeypoints(img_, feature, img_);
	//cv::imwrite( "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/Gray_Image.jpg", img );

	std::stringstream ss;
	ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/fea_image_" << name << "_" << fea_cnt << ".jpg";
	cv::imwrite( ss.str(), img_ );
	fea_cnt++;

	return feature;
}


void featureGridMap::feature_descriptor( cv::Mat img, std::vector<cv::KeyPoint>& feature, std::vector<cv::Mat>& descriptor_ ){

    std::vector<cv::KeyPoint> keys;
    int tam_keys = (int)feature.size();
		int i, x, y;
		double rad, teta, px, py, shiftTeta;
		int rows_ = img.rows;
		int cols_ = img.cols;
    for(i=0; i<tam_keys; i++){
        px = feature.at(i).pt.x;
        py = feature.at(i).pt.y;
        shiftTeta = -round( feature.at(i).angle*rad_to_deg );

        //cv::Mat descriptor(numRad, numTeta, CV_8UC1);
        cv::Mat descriptor = cv::Mat(numRad, numTeta, CV_8UC1,255);
        for(rad = 0; rad < (numRad); rad++){
            for(teta = 0; teta < numTeta; teta++){
                x = (int)( px + cx + (rad*dRad+3)*cos( ((teta*dTeta)+shiftTeta)*deg_to_rad ) );
                y = (int)( py + cy + (rad*dRad+3)*sin( ((teta*dTeta)+shiftTeta)*deg_to_rad ) );

                if( x <0 || y <0 || x > cols_ || y > rows_){
                    continue;
                }
                descriptor.at<uchar>( (int)(rad), (int)(teta) ) = (int)img.at<uchar>( cv::Point(x,y) );
            }
        }
        descriptor_.push_back(descriptor);
        keys.push_back(feature.at(i));

				//std::stringstream ss;
				//	ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/descriptor_"<< i <<".jpg";
				//cv::imwrite( ss.str(), descriptor );
    }
    //std::cout<< "Keypoints Out: "<< keys.size() << std::endl;
    feature.clear();
    feature = keys;
}

