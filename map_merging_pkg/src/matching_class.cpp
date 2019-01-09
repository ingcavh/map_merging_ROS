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


class matchingClass {
	public:
		matchingClass();
		//Método de distancia entre caracteristicas
		void compute_distances(std::vector<cv::Mat>& descriptor1, std::vector<cv::Mat>& descriptor2, std::vector< std::vector< double > >& dist);
		//Método de selección de parejas entre caracteristicas
		bool compute_matching(std::vector< std::vector< double > > distances, std::vector<cv::KeyPoint> k1, std::vector<cv::KeyPoint> k2, cv::Mat img1, cv::Mat img2, std::vector< double >& rta);
		//Método RANSAC
		bool ransac( std::vector< std::vector<int> >  indexs, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, cv::Mat img1, cv::Mat img2, std::vector< double > &trans );
		//Método de Refinamiento de matriz de transformación
		void alignment_images( std::vector< std::vector<cv::KeyPoint> > keypoints, double error, std::vector< double > &trans );
		//Método de verificación (aceptación/rechazo) de matching
		bool verify_matching(std::vector< double > &trans, cv::Mat img1, cv::Mat img2, std::vector< std::vector<cv::KeyPoint> > keypoints);
		//Método de mezcla de caracteristicas
		void merging_images(cv::Mat img1, cv::Mat img2, std::vector< double > trans, nav_msgs::OccupancyGrid &map, std::vector< int > &wi_he);



	private:
		//Booleana de Publicacion Imagen Mezclada
		bool pub_merged_images;
		double deg_to_rad;    // pi/180
		double rad_to_deg;    // 180/pi
};

matchingClass::matchingClass(){
	deg_to_rad = 0.017453293;    // pi/180
	rad_to_deg = 57.29577951;    // 180/pi
	pub_merged_images = true;
}
void matchingClass::compute_distances(std::vector<cv::Mat>& descriptor1, std::vector<cv::Mat>& descriptor2, std::vector< std::vector< double > >& distances){
	distances.clear();
	int tam1 = (int)descriptor1.size();
	int tam2 = (int)descriptor2.size();
	int i, j, x, y, rows_, cols_;
	double distance, dif;
	for(i=0; i<tam1; i++){
		//i: selector d descriptor dl conjunto d descriptores d la imagen d consulta  para empajerar
		for(j=0; j<tam2; j++){
			//if(descriptor1.at(i).cols != descriptor2.at(j).cols || descriptor1.at(i).rows != descriptor2.at(j).rows) { continue; }
			distance = 0;
      cv::Mat query = descriptor1.at(i);
      cv::Mat train = descriptor2.at(j);
			rows_ = (int)query.rows;
			cols_ = (int)query.cols;
			for(x=0; x< rows_; x++){
			    for(y=0; y< cols_; y++){
			        dif = ( fabs( (255-(int)query.at<uchar>(x, y)) - (255-(int)train.at<uchar>(x, y)) ) )/255.0; //para normalizar
			        distance +=  dif;
			    }
			}
			std::vector< double > dist_aux;
			dist_aux.push_back( distance );
			dist_aux.push_back( i );
			dist_aux.push_back( j );
			distances.push_back( dist_aux );
		}
	}
	std::sort(distances.begin(), distances.end());
}

bool matchingClass::compute_matching(std::vector< std::vector< double > > distances, std::vector<cv::KeyPoint> k1, std::vector<cv::KeyPoint> k2, cv::Mat img1, cv::Mat img2, std::vector< double >& rta){
 ///Matching con Descriptor circular
 ///
    //std::cout << ">> Keypoints1: " << keypoints1.size() << " - Descriptors1: " << descriptors1.size();
    //std::cout << " y >> Keypoints2: " << keypoints2.size() << " - Descriptors2: " << descriptors2.size() << std::endl;

    std::vector< std::vector< int > > indexs1;
    std::vector<cv::DMatch> matches;
		double max = distances.at( (int)(distances.size()-1) ).at(0);
		double min = distances.at( 0 ).at(0);
    double porcen_sel = 0.25;//.25
    double distance_selection = (max-min)*porcen_sel+min;
		int i, queryIdx, trainIdx;
		int tam_ = (int)distances.size();
    for (i=0; i<tam_ ; i++){
        if ( distances.at(i).at(0) <= distance_selection ){
            cv::DMatch match;
            queryIdx = distances.at(i).at(1);
            trainIdx = distances.at(i).at(2);
            match.distance = distances.at(i).at(0);
            match.queryIdx = queryIdx;
            match.trainIdx = trainIdx;
            matches.push_back(match);

            std::vector<int> cur_indexs;
            cur_indexs.push_back(queryIdx);
            cur_indexs.push_back(trainIdx);
            indexs1.push_back( cur_indexs );
        }
    }
    //std::cout << "  Good Matches: " << matches.size() << std::endl;
    //std::cout << "   matches: " << matches.size() << std::endl;

    //cv::Mat img_matches;
    //cv::drawMatches(src_gray1, keypoints1, src_gray2, keypoints2, matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

    //-- Show detected matches
    //cv::imshow( "Good Matches", img_matches );

	return ransac(indexs1, k1, k2, img1, img2, rta);
}


bool matchingClass::ransac( std::vector< std::vector<int> >  indexs, std::vector<cv::KeyPoint> &keypoints1, std::vector<cv::KeyPoint> &keypoints2, cv::Mat img1, cv::Mat img2, std::vector< double > &trans  ){

//Method configuration variables
    int minGoodMatchingLine = 2;
    double dist_accep = 0.92;

    std::cout << "+******************************** LLAMADO Modelo Por Ransac " << indexs.size() << " k1: " << keypoints1.size() << " k2: " << keypoints2.size() << std::endl;


    std::vector<cv::DMatch> matchesLine, goodMatchesLine;
    std::vector< std::vector<cv::KeyPoint> > kpts_ransac, kpts_aux; //almacena conjunto de parejas de keypoints
    std::vector<cv::KeyPoint> pair_ransac; //almacena pareja de keypoints
		std::vector<double> por_vec, err_vec;
    cv::DMatch matchPair;

    cv::KeyPoint queryK1, trainK1, queryK2, trainK2;
    int queryIndex1, trainIndex1, queryIndex2, trainIndex2;
    double angT_Q, tx, ty; //variables de transformada

    double query2X, query2Y, train2X, train2Y, Xt, Yt, difY, difX, dx, dy;
    int goodLine;
    bool new_model = false; //Variable de retorno
    double err_ = img1.rows*img2.rows; //error de la transformación de Q a T
    double err_ind = err_; //error individual o promedio de la transformación de Q a T
    double err_cur; //error actual de iteración
    double err_ind_cur;// error actual promedio de iteración


    //std::cout << " ssssssssss-> err_ind: " << err_ind << " -> err_: " << err_ << std::endl;
		std::stringstream ss;
		int cnt_m = 0;
				cv::Mat result;

    int i,j, tam = (int) indexs.size();
    for( i=0; i < tam; i++ ){
        matchesLine.clear();
        pair_ransac.clear();
        kpts_aux.clear();
				por_vec.clear(); 
				err_vec.clear();

        queryIndex1 = (int) indexs.at(i).at(0);
        trainIndex1 = (int) indexs.at(i).at(1);
        queryK1 = keypoints1.at( queryIndex1 );
        trainK1 = keypoints2.at( trainIndex1 );
        pair_ransac.push_back( queryK1);
        pair_ransac.push_back( trainK1);
        kpts_aux.push_back(pair_ransac);

        matchPair.queryIdx =queryIndex1;
        matchPair.trainIdx =trainIndex1;
				//matchesLine.clear();
        matchesLine.push_back( matchPair);

				por_vec.push_back(1.0); 
				err_vec.push_back(0.0);

        //std::cout << " -> Qx: " << queryK1.pt.x << " Qy: " << queryK1.pt.y << " Qang: " << (queryK1.angle*180)/M_PI;
        //std::cout << "    Tx: " << trainK1.pt.x << " Ty: " << trainK1.pt.y << " Qang: " << (trainK1.angle*180)/M_PI << std::endl;
        //angT_Q = trainK1.angle - queryK1.angle;
				angT_Q = queryK1.angle - trainK1.angle;
        tx = trainK1.pt.x - queryK1.pt.x*cos(angT_Q) + queryK1.pt.y*sin(angT_Q);
        ty = trainK1.pt.y - queryK1.pt.x*sin(angT_Q) - queryK1.pt.y*cos(angT_Q);
        //std::cout << "    dx: " << tx << " dy: " << ty << " da: " << angT_Q << std::endl;


        err_cur = 0;
        for( j = 0; j < tam; j++ ){
            if(i==j){ continue; }
            //std::cout << " Index j: " << j << std::endl;
            queryIndex2 = (int) indexs.at(j).at(0);
            trainIndex2 = (int) indexs.at(j).at(1);
            queryK2 = keypoints1.at( queryIndex2 );
            trainK2 = keypoints2.at( trainIndex2 );
            query2X = queryK2.pt.x;
            query2Y = queryK2.pt.y;
            train2X = trainK2.pt.x;
            train2Y = trainK2.pt.y;
            Xt = query2X*cos(angT_Q) - query2Y*sin(angT_Q) + tx;
            Yt = query2X*sin(angT_Q) + query2Y*cos(angT_Q) + ty;

            difY = 0;
            difX = 0;
            if(Xt > train2X){
                difX = train2X/Xt;
            }else{
                difX = Xt/train2X;
            }
            if(Yt > train2Y){
                difY = train2Y/Yt;
            }else{
                difY = Yt/train2Y;
            }

						dx = Xt-train2X;
						dy = Yt-train2Y;	
						//err_cur = err_cur + sqrt(dx*dx+dy*dy);
						double metrica = sqrt(dx*dx+dy*dy);

            //if((difX+difY)/2 > dist_accep){
						if(metrica < 20.2){
                //std::cout << "   Q2: (" << query2X << "," << query2Y << ")";
                //std::cout << "   trans: (" << Xt << "," << Yt << ")";
                //std::cout << "  a  T2: (" << train2X << "," << train2Y << ") " << (difX+difY)/2 << std::endl;
								
								bool accep = true;
	
								for(int h=0; h<kpts_aux.size(); h++){
									cv::KeyPoint queryK3 = kpts_aux.at(h).at(0);
									cv::KeyPoint trainK3 = kpts_aux.at(h).at(1);
	                //std::cout << "     Q3: (" << queryK3.pt.x << "," << queryK3.pt.y << ")";
                	//std::cout << "   T3: (" << trainK3.pt.x << "," << trainK3.pt.y << ") " << por_vec.at(h) << std::endl;
									if( (queryK2.pt.x==queryK3.pt.x && queryK2.pt.y==queryK3.pt.y) || (trainK2.pt.x==trainK3.pt.x && trainK2.pt.y==trainK3.pt.y) ){
										if((difX+difY)/2>por_vec.at(h)){
											matchesLine.erase(matchesLine.begin() + h);
											pair_ransac.erase(pair_ransac.begin() + h);
											kpts_aux.erase(kpts_aux.begin() + h);
											por_vec.erase(por_vec.begin() + h); 
											err_vec.erase(err_vec.begin() + h);

            					//std::cout << " Borrando: " << h << std::endl;
										}else{
											accep = false;
										}															

									}

								}

								if(accep){
		              matchPair.queryIdx =queryIndex2;
		              matchPair.trainIdx =trainIndex2;
		              matchesLine.push_back( matchPair);

									por_vec.push_back((difX+difY)/2); 

            			//std::cout << "  -> " << (difX+difY)/2 << " - " << metrica << std::endl;
		              dx = Xt-train2X;
		              dy = Yt-train2Y;	
		              //err_cur = err_cur + sqrt(dx*dx+dy*dy);
									err_vec.push_back(sqrt(dx*dx+dy*dy));

		              pair_ransac.clear();
		              pair_ransac.push_back(queryK2);
		              pair_ransac.push_back(trainK2);
		              kpts_aux.push_back(pair_ransac);

                  //std::cout << "   Q2: (" << query2X << "," << query2Y << ")";
                  //std::cout << "   trans: (" << Xt << "," << Yt << ")";
                  //std::cout << "  a  T2: (" << train2X << "," << train2Y << ") " << (difX+difY)/2 << std::endl;

								}
            }
        }
				goodLine = kpts_aux.size();
				for(int h=0; h<goodLine; h++){
					err_cur = err_cur + err_vec.at(h);
				}
        err_ind_cur = err_cur/goodLine;
        //std::cout << "    -> err_cur: " << err_cur << " -> err_ind_cur: " << err_ind_cur;
        //std::cout << "  -> goodLine: " << goodLine << std::endl;
				//std::cout << std::endl;	
        if( goodLine > minGoodMatchingLine && err_ind_cur < err_ind){
            err_ind = err_ind_cur;
            err_ = err_cur;

            minGoodMatchingLine = goodLine;
            goodMatchesLine.clear();
            //cv::DMatch matchPair;

            goodMatchesLine = matchesLine;
            trans.clear();
            trans.push_back(tx);
            trans.push_back(ty);
            trans.push_back(angT_Q);
            new_model = true;

            kpts_ransac.clear();
            kpts_ransac = kpts_aux;

//bag jade 220 - bag luna 307
/*
				    alignment_images(kpts_ransac, err_, trans);
				    verify_matching(trans, img1, img2,kpts_ransac);
						merging_images(img1, img2, trans);
        		cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatchesLine, result, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
						ss.str("");
						ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/matching_" << cnt_m <<".jpg";
						cv::imwrite( ss.str(), result );
						cnt_m++;
*/

            //std::cout << "       Nuevo Modelo encontrado: " << matchesLine.size() << " last: " << goodMatchesLine.size() << std::endl;

        }

    }
    if( new_model ){
        std::cout << "       Modelo de RANSAC Encontrado " << goodMatchesLine.size() << std::endl;
        //std::cout << "       ***Procedimiento de Alineacion*** " << std::endl;
        alignment_images(kpts_ransac, err_, trans);
        //std::cout << "       *** FIN Procedimiento de Alineacion*** " << std::endl;

        cv::drawMatches(img1, keypoints1, img2, keypoints2, goodMatchesLine, result, cv::Scalar::all(-1), cv::Scalar::all(-1), std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);
        //-- Show detected matches
        //cv::imshow( "Matches With Ransac Model", result);
				ss.str("");
				ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/matching_images.jpg";
				cv::imwrite( ss.str(), result );

        return verify_matching(trans, img1, img2,kpts_ransac);
    }
    std::cout << " NO se encontró modelo entre los mapas " << std::endl;
    return false;

}
void matchingClass::alignment_images( std::vector< std::vector<cv::KeyPoint> > keypoints, double error, std::vector< double > &trans ){
    //Method configuration variables
    double ang = 7*deg_to_rad;
    double d_ang = 0.5*deg_to_rad;

    double ang_min = trans.at(2)-ang;
    double ang_max = trans.at(2)+ang;

    cv::KeyPoint queryK1 = keypoints.at(0).at(0);
    cv::KeyPoint trainK1 = keypoints.at(0).at(1);
    cv::KeyPoint queryK2, trainK2;
    double queryX = queryK1.pt.x;
    double queryY = queryK1.pt.y;
    double trainX = trainK1.pt.x;
    double trainY = trainK1.pt.y;
    double query2X, query2Y, train2X, train2Y;

    //std::cout << " -Iniciando ErrorIn: " << error << " -ang_min: " << ang_min<< " -ang_max: " << ang_max << " -ang_in: " << ang_in << std::endl;
    //std::cout << " -(" << queryX << "," << queryY<< ") -(" << trainX << "," << trainY << ")" << std::endl;

    //std::cout << "       Transformada In -tx: " << trans.at(0) << " -ty: " << trans.at(1) << " -a: " << trans.at(2) << std::endl;
    double angT_Q;
    int j, tam = (int) keypoints.size();
    double ca, sa, tx, ty, err_cur, Xt, Yt, dx, dy;
    for(angT_Q = ang_min; angT_Q < ang_max; angT_Q = angT_Q+d_ang){
        ca = cos(angT_Q);
        sa = sin(angT_Q);
        tx = trainX - queryX*ca + queryY*sa;
        ty = trainY - queryX*sa - queryY*ca;
        //std::cout << "  -dx: " << tx << " dy: " << ty << " da: " << angT_Q << "  tam: " << indexs.size() << std::endl;
        err_cur = 0;
        for(j = 1; j < tam; j++){
            queryK2 = keypoints.at(j).at(0);
            trainK2 = keypoints.at(j).at(1);
            query2X = queryK2.pt.x;
            query2Y = queryK2.pt.y;
            train2X = trainK2.pt.x;
            train2Y = trainK2.pt.y;
            Xt = query2X*ca - query2Y*sa + tx;
            Yt = query2X*sa + query2Y*ca + ty;

            dx = Xt-train2X;
            dy = Yt-train2Y;
            err_cur = err_cur + sqrt(dx*dx+dy*dy);
            //std::cout << "   Q2: (" << query2X << "," << query2Y << ")";
            //std::cout << "   trans: (" << Xt << "," << Yt << ")";
            //std::cout << "   a T2: (" << train2X << "," << train2Y << ")";
            //std::cout << "  difX: " << dx << "  difY: " << dy << std::endl;
        }
        //std::cout << "  - ErrorCur: " << err_cur << " - ang: " << angT_Q*deg_to_rad << std::endl;
        if( err_cur < error){
            error = err_cur;
            trans.clear();
            trans.push_back(tx);
            trans.push_back(ty);
            trans.push_back(angT_Q);
            //std::cout << " -> Error: " << error << std::endl;
        }
    }
    //std::cout << "       Transformada Out -tx: " << trans.at(0) << " -ty: " << trans.at(1) << " -a: " << trans.at(2) << std::endl;

}
bool matchingClass::verify_matching(std::vector< double > &trans, cv::Mat img1, cv::Mat img2, std::vector< std::vector<cv::KeyPoint> > keypoints){
    //Method configuration variables
    int rad=60, angle = 360;
    double acceptance = 0.9015; //porcentaje de aceptación [0,1]

    double tx = trans.at(0);
    double ty = trans.at(1);
    double ang = trans.at(2);
    double cang = cos(ang);
    double sang = sin(ang);
    int tam = keypoints.size(), col = img2.cols, row = img2.rows;
    double cnt = 0, sim = 0;
    int i,r,t;
    int x,y,Xt,Yt,data1,data2;
    double qX, qY;
    cv::KeyPoint queryK1;
    for(i=0; i<tam; i++){
        queryK1 = keypoints.at(i).at(0);
        qX = queryK1.pt.x;
        qY = queryK1.pt.y;
        for(r=1; r<rad; r++){
            for(t=0; t<angle; t=t+1){
                x = (int)qX + (r*cos(t*deg_to_rad));
                y = (int)qY + (r*sin(t*deg_to_rad));
                if( x <0 || y <0 || x > col || y > row){
                    continue;
                }
                cnt++;
                Xt = (int)x*cang - y*sang + tx;
                Yt = (int)x*sang + y*cang + ty;
                data1  = (int)img1.at<uchar>( cv::Point(x,y) );
                data2  = (int)img2.at<uchar>( cv::Point(Xt,Yt) );

                //std::cout << "   r: " << r << " t: "<< t << " (" << x << "," << y << ") -> (" << Xt << "-" << Yt << ") " << "    data1 " << data1 << " -> data2 " << data2 << std::endl;

                if(data1==data2){
                    //std::cout << "    equal points " << data1 << " - " << data2 << std::endl;
                    sim++;
                }else if(data1==127 || data2==127){ //en algoritmo o es asi
                    sim+=0.5;
                }
            }
        }
    }

    std::cout << "       Se contaron " << cnt << " y de simil " << sim  << " % " << sim/cnt << std::endl;
    if(sim/cnt>acceptance){
        return true;
    }else{
        return false;
    }
}

int merg = 0;
void matchingClass::merging_images(cv::Mat img1, cv::Mat img2, std::vector< double > trans, nav_msgs::OccupancyGrid &map, std::vector< int > &wi_he){
	double tx = trans.at(0);
	double ty = trans.at(1);
	double angT_Q = trans.at(2);// + (3.5*M_PI)/180;
	double sang = sin(angT_Q);
	double cang = cos(angT_Q);
	std::cout << "       -> Transformada: (" << tx << "," << ty << "," << angT_Q << ") " << std::endl;

	int row_min=0, row_max=0, col_min=0, col_max=0;
	int row1=(int)img1.rows, row2=(int)img2.rows, col1=(int)img1.cols, col2=(int)img2.cols;
	int height = row2;
	int width = col2;
	int height_min = 0;
	int width_min  = 0;
	int height_max = row2;
	int width_max  = col2;

	int i,j;
	int x,y;
	for(i=0; i<row1; i++ ){
		  for(j=0; j<col1; j++ ){
		      x = j*cang - i*sang + tx;
		      y = j*sang + i*cang + ty;
		      //std::cout << "i " << i << " y j " << j << " x " << x << " y y " << y << std::endl;
		      if(x < col_min){
		          col_min = x;
		      }else if(x > col_max){
		          col_max = x;
		      }
		      if(y < row_min){
		          row_min = y;
		      }else if(y > row_max){
		          row_max = y;
		      }
		  }
	}

	//int x = 106*cos(angT_Q) - 205*sin(angT_Q) + tx;
	//int y = 106*sin(angT_Q) + 205*cos(angT_Q) + ty;
	//std::cout << "EXTREMES " << x << " y " << y << std::endl;
	std::cout << "       -> Lims: " << row_min << "," << row_max << " y " << col_min << "," << col_max << std::endl;


	if(row_min<0){	height_min = fabs(row_min);	height+= height_min;	}
	if(row_max>row2){	height_max = (row_max-row2)+1;	height+= height_max;	};
	if(col_min<0){	width_min  = fabs(col_min);	width+=  width_min;	};
	if(col_max>col2){	width_max  = (col_max-col2)+1;	width+=  width_max;	};

	std::cout << "       -> Height: " << height_min << " y " << height_max <<  " -> Width: " << width_min << " y " << width_max << std::endl;
	std::cout << "       -> Height: " << height << " H_min " << height_min <<  " -> Width: " << width << " W_min " << width_min << std::endl;

	cv::Mat img;
	if(pub_merged_images){	img = cv::Mat( height, width, CV_8UC1, double(127) );	};
	int8_t map_[height+1][width+1];
	std::fill(map_[0], map_[0] + (height+1)*(1+width), -1);
  int img_dat = 127;
  int img12_dat = 127;

	wi_he.push_back(width_min);
	wi_he.push_back(height_min);
//Poniendo Imagen 1 en img_merged
	
	std::cout << "       -> Img1: " << img1.cols << "," << img1.rows << std::endl;
	std::cout << "       -> Img2: " << img2.cols << "," << img2.rows << std::endl;
	std::cout << "       -> Img: " <<  img.cols <<  "," << img.rows << std::endl;

  int rs =  sizeof map_ / sizeof map_[0]; // 2 rows  
  int cs = sizeof map_[0] / sizeof(int8_t); // 5 cols

	std::vector<int8_t> data(height*width);
	std::fill (data.begin(),data.begin()+height*width,-1);
	std::cout << "       -> Poniendo Imagen 1 en img_merged " << rs << " " << cs << std::endl;
  for(i=0; i<row1; i++ ){
      for(j=0; j<col1; j++ ){
          x = j*cang - i*sang + tx + width_min;
          y = j*sang + i*cang + ty + height_min;
					//std::cout << "       -> img1 (" << j << "," << i << ")" << " (" << x << "," << y << "): " << img1.at<uchar>(i,j)  << " " << x+y*width << std::endl;
					if(y>=height || x>=width) continue;
          
					img_dat = img1.at<uchar>(i,j);
					data.at(x+y*width) = (img_dat<10) ? 100: (img_dat>250) ? 0:-1;
					//map_[y][x] =  (img_dat<10) ? 100: (img_dat>250) ? 0:-1;
					if(pub_merged_images){	img.at<uchar>(y,x) = img_dat;	};
      }
  }

//Poniendo Imagen 2 en img_merged

	std::cout << "       -> Poniendo Imagen 2 en img_merged "<< std::endl;
	for(i=0; i<row2; i++ ){
		  for(j=0; j<col2; j++ ){
		      x = j + width_min;
		      y = i + height_min;
					img12_dat = img2.at<uchar>(i,j);
					img_dat = (img12_dat<10) ? 100: (img12_dat>250) ? 0:-1 ;
			
					switch ( img_dat ) {
						case 0:
							if( /*map_[y][x]*/data.at(x+y*width) ==-1 ){
								//map_[y][x] = img_dat;
								data.at(x+y*width) = img_dat;
								if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
							}
							break;
						case 100:
							if( /*map_[y][x]*/data.at(x+y*width) ==-1 ){
								//map_[y][x] = img_dat;
								data.at(x+y*width) = img_dat;
								if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
							}else if( /*map_[y][x]*/data.at(x+y*width) ==0 ){
								//map_[y][x] = img_dat;
								data.at(x+y*width) = img_dat;	
								if(pub_merged_images){	img.at<uchar>(y,x) = img12_dat;	};
							}
							break;
						default:
							break;
					}
		  }
	}

	std::cout << "       -> Construyendo objeto OccupancyGrid "<< std::endl;

	/*for(y=0; y<height; y++){
		for(x=0; x<width; x++){
			data.push_back(map_[y][x]);
		}
	}*/
	map.data  = data;
	map.info.width  = width;
	map.info.height  = height;
	map.info.origin.position.x  = -((map.info.resolution/2.f)+(width/2)*map.info.resolution);
	map.info.origin.position.y  = -((map.info.resolution/2.f)+(height/2)*map.info.resolution);


	if(pub_merged_images){
		std::stringstream ss;
		ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_images1.jpg";
		cv::imwrite( ss.str(), img1 );
		ss.str("");
		ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_images2.jpg";
		cv::imwrite( ss.str(), img2 );
		ss.str("");
		ss << "/home/ingcavh/Workspace/catkin_ws/src/tesis_pkg/map_merging_pkg/images/merged_img" << merg << ".jpg";
		cv::imwrite( ss.str(), img );
		merg++;
		std::cout << "       -> Save Merged images: " << merg << std::endl;

	}
}
