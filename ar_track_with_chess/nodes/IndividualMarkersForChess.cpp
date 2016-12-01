#include <ros/ros.h>
#include <iostream>
#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include "ar_track_alvar/CvTestbed.h"
#include "ar_track_alvar/MarkerDetector.h"
#include "ar_track_alvar/Shared.h"
#include <cv_bridge/cv_bridge.h>
#include <ar_track_alvar_msgs/AlvarMarker.h>
#include <ar_track_alvar_msgs/AlvarMarkers.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>
#include <sensor_msgs/image_encodings.h>
#include <dynamic_reconfigure/server.h>
#include <ar_track_alvar/ParamsConfig.h>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> //remove those 2 if you are in paionaios..

using namespace alvar;
using namespace std;

bool init=true;
Camera *cam;
cv_bridge::CvImagePtr cv_ptr_;
image_transport::Subscriber cam_sub_;
ros::Publisher arMarkerPub_;
ros::Publisher rvizMarkerPub_;
ar_track_alvar_msgs::AlvarMarkers arPoseMarkers_;
visualization_msgs::Marker chessSquares_,chessPoints_;
tf::TransformListener *tf_listener;
tf::TransformBroadcaster *tf_broadcaster;
MarkerDetector<MarkerData> marker_detector;

bool enableSwitched = false;
bool enabled = true;
double max_frequency;
double marker_size;
double max_new_marker_error;
double max_track_error;
/*tf::Vector3 id_0;
tf::Vector3 id_1;
tf::Vector3 id_2;
tf::Vector3 id_3;*/
std::string cam_image_topic; 
std::string cam_info_topic; 
std::string output_frame;

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg);

const char* getTfName(int tf_category, int data_name)
{			
	string name;
	ostringstream convert;  

 	//chess_knobs
 	if(tf_category==1){
		convert << data_name;
		name = "ar_knob_" + convert.str();   		
 	}
	const char *nameInChar = name.c_str();
 	return nameInChar;
}

void getCapCallback (const sensor_msgs::ImageConstPtr & image_msg)
{
    //If we've already gotten the cam info, then go ahead
	if(cam->getCamInfo_){
		try{
			tf::StampedTransform CamToOutput;
			//ROS_INFO("%s" , "mpe");
    			try{
					tf_listener->waitForTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, ros::Duration(1.0));
					tf_listener->lookupTransform(output_frame, image_msg->header.frame_id, image_msg->header.stamp, CamToOutput);
   				}
    			catch (tf::TransformException ex){
      				ROS_ERROR("%s",ex.what());
    			}

    		//ROS_INFO("CamToOutput x=%f ,y=%f , z=%f\n",CamToOutput.getOrigin()[0],CamToOutput.getOrigin()[1] ,CamToOutput.getOrigin()[2]);
            //ROS_INFO("CamToOutput rotattion x=%f ,y=%f , z=%f , w=%f\n",CamToOutput.getRotation ()[0],CamToOutput.getRotation ()[1] ,CamToOutput.getRotation ()[2],CamToOutput.getRotation ()[3]);
            
            //Convert the image
            cv_ptr_ = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::BGR8);

            //Get the estimated pose of the main markers by using all the markers in each bundle

            // GetMultiMarkersPoses expects an IplImage*, but as of ros groovy, cv_bridge gives
            // us a cv::Mat. I'm too lazy to change to cv::Mat throughout right now, so I
            // do this conversion here -jbinney
            IplImage ipl_image = cv_ptr_->image;

			if(cam_image_topic.compare("/naoqi_driver_node/camera/front/image_raw") != 0){
				marker_detector.Detect(&ipl_image, cam, true, true, max_new_marker_error, max_track_error, CVSEQ, true);
				cv::imshow("OPENCV_WINDOW", cv_ptr_->image);
				cv::waitKey(3);
			}else{
				marker_detector.Detect(&ipl_image, cam, true, false, max_new_marker_error, max_track_error, CVSEQ, true);
			}

      	    arPoseMarkers_.markers.clear ();
			for (size_t i=0; i<marker_detector.markers->size(); i++) 
			{
				//Get the pose relative to the camera
        		int id = (*(marker_detector.markers))[i].GetId(); 
				Pose p = (*(marker_detector.markers))[i].pose;  // ! ayto prepei na deis
				double px = (p.translation[0])/100.0;
				double py = (p.translation[1])/100.0;
				double pz = (p.translation[2])/100.0;
				double qx = p.quaternion[1];
				double qy = p.quaternion[2];
				double qz = p.quaternion[3];
				double qw = p.quaternion[0];

				//ROS_INFO("oi quat einai %f %f %f %f ",qx,qy,qz,qw);

				//quat to euler
				tf::Quaternion rotation (qx,qy,qz,qw);
/*				tf::Matrix3x3 matrix(temp_rotation);
				double roll, pitch, yaw;
				matrix.getRPY(roll, pitch, yaw); //stable on pitch (on the same surface ~ roll and yaw some littl differences )
				ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
				//pitch=0;

				tf::Quaternion qua_matrix(roll, pitch, yaw);
				ROS_INFO("oi nees quat einai %f %f %f %f ",qua_matrix.y() ,qua_matrix.x(),qua_matrix.z(),qua_matrix.w());

                tf::Quaternion rotation (qua_matrix.y() ,qua_matrix.x(),qua_matrix.z(),qua_matrix.w());*/
                tf::Vector3 origin (px,py,pz);
                tf::Transform t (rotation, origin);

                // marker pose in the camera frame - visualize and place the marker in a relation to the camera pose (initial marker pose in 
                //relation with the camera pose ::transational->(0,0,0) orientation is the same)..
                
                tf::Vector3 markerOrigin (0, 0, 0);
                tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin); //creating a transform (0,0,0,1),(0,0,0)
                tf::Transform markerPose = t * m; 



                //Chess' Squares and Knobs Spatial arrangment in 3D space in relation with the Camera Pose.  

				std::vector<tf::Transform> chess_SQvector; // vector of chess squares's transforms
				std::vector<tf::Transform> chess_KNvector; // vector of chess knob's transforms
				tf::Transform chessPose = t * m; //just for initialization
				double x_ax,y_ax,z_ax;

				for (double i = -3.5; i < 4; i++)
			    {
			    	for (int j = 1; j < 9; j++)
			    	{
						double marker_area=0.001+marker_size/100.0; // 0.001 is for the chess line bordering
						
						x_ax=marker_area*i;
						y_ax=marker_area*j;
						z_ax=0;	
						//ROS_INFO("ftiaxnei to %f %f %f",x_ax,y_ax,z_ax);	
						markerOrigin.setValue(x_ax,y_ax,z_ax); //the vector
						chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
						chess_SQvector.push_back (chessPose);
    					//ROS_INFO("o %f %f %f",chess_SQvector[0].getOrigin().m_floats[0],chess_SQvector[0].getOrigin().m_floats[1],chess_SQvector[0].getOrigin().m_floats[2]);
               			

						//and for the 4 knobs of the current square..
						
//NEEDS OPTIMISATION LIKE 2D !!

						if(i==-3.5 && j==1){								
								//visualize all the knobs (4 total)
								markerOrigin.setValue(x_ax-marker_area*0.5,y_ax-marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
								tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

								markerOrigin.setValue(x_ax-marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
								tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
								tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax-marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);

							}else if(i==-3.5){
								//visualize the 2 right knobs
								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
								tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

								markerOrigin.setValue(x_ax-marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
							}else if(j==1){
								//visualize the 2 top knobs				
								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);
								tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax-marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);

							}else{
								//visualize only the top right knob
								markerOrigin.setValue(x_ax+marker_area*0.5,y_ax+marker_area*0.5,z_ax+0.001); 
								chessPose = t * tf::Transform(tf::Quaternion::getIdentity(), markerOrigin);
								chess_KNvector.push_back (chessPose);

						} 

						tf_broadcaster->sendTransform(tf::StampedTransform(chess_KNvector[chess_KNvector.size()-1], image_msg->header.stamp, image_msg->header.frame_id, getTfName(1,chess_KNvector.size()) ) );

					}
			    }
				

				//ROS_INFO("ekane synolika %d",chess_KNvector.size());
    			 //tf::Vector3 markerOrigin (0.001+marker_size/100.0*1, 0, 0);
                //tf::Transform m (tf::Quaternion::getIdentity (), markerOrigin); //creating a transform (0,0,0,1),(0,0,0)
                //tf::Transform markerPose = t * m; 
                
                //ROS_INFO("%f",origin.m_floats[0]); //yes
                //ROS_INFO("mpe %f",origin.x());

                //ROS_INFO("t %f %f %f",t.getOrigin().m_floats[0],t.getOrigin().m_floats[1],t.getOrigin().m_floats[2]);
				//ROS_INFO("t+ %f %f %f",t.getOrigin().m_floats[0]+0.01,t.getOrigin().m_floats[1],t.getOrigin().m_floats[2]);
				
				//ROS_INFO("m %f %f %f",markerPose.getOrigin().m_floats[0],markerPose.getOrigin().m_floats[1],markerPose.getOrigin().m_floats[2]);
				

                //ROS_INFO("id:%d quat %f %f %f %f\n",id,p.quaternion[1],p.quaternion[2],p.quaternion[3],p.quaternion[0]);
                //ROS_INFO("BRIKE %d\n",marker_detector.markers->size());
                //ROS_INFO("id:%d raw translation %f %f %f\n",id,p.translation[0],p.translation[1],p.translation[2]);

				//ROS_INFO("id:%d %f %f %f",id, px, py, pz);

               // ROS_INFO("transf %f %f %f",  origin.x(),origin.y(),origin.z());
                ////ROS_INFO("%f",marker_size);

                //ROS_INFO("?%f %f %f",tf::Quaternion::getIdentity().y(),tf::Quaternion::getIdentity().z() ,tf::Quaternion::getIdentity().w()  );

                tf::Vector3 z_axis_cam = tf::Transform(rotation, tf::Vector3(0,0,0)) * tf::Vector3(0, 0, 1);
                //ROS_INFO("%02i Z in cam frame: %f %f %f",id, z_axis_cam.x(), z_axis_cam.y(), z_axis_cam.z());
                /// as we can't see through markers, this one is false positive detection
                if (z_axis_cam.z() > 0)
                {
                    continue;
                }

				//Publish the transform from the camera to the marker		
				std::string markerFrame = "ar_marker_";
				std::stringstream out;
				out << id;
				std::string id_string = out.str();
				markerFrame += id_string;

				tf::StampedTransform camToMarker (t, image_msg->header.stamp, image_msg->header.frame_id, markerFrame.c_str());
    			tf_broadcaster->sendTransform(camToMarker);
    			
    			
    			// For RVIZ
    			
				tf::poseTFToMsg (markerPose, chessSquares_.pose);
				tf::poseTFToMsg (markerPose, chessPoints_.pose);
				//ROS_INFO("%f",chessSquares_.pose.position.x);
				chessSquares_.header.frame_id = chessPoints_.header.frame_id = image_msg->header.frame_id;
				chessSquares_.header.stamp = chessPoints_.header.stamp = image_msg->header.stamp;
				chessSquares_.ns = chessPoints_.ns =  "basic_shapes";
				chessSquares_.action = chessPoints_.action = visualization_msgs::Marker::ADD;
				chessSquares_.id = id;
				chessPoints_.id = 101;
				chessSquares_.type = visualization_msgs::Marker::CUBE_LIST;
				chessPoints_.type = visualization_msgs::Marker::POINTS;	

				chessSquares_.scale.x = 1.0 * marker_size/100.0;
				chessSquares_.scale.y = 1.0 * marker_size/100.0;
				chessSquares_.scale.z = 0.2 * marker_size/100.0;

				chessPoints_.scale.x = 0.2 * marker_size/100.0;
				chessPoints_.scale.y = 0.2 * marker_size/100.0;

				int tag_index=0; //mallon perito.. xrhsh gia parapanw apo 1 tag

				chessSquares_.color.a = 0.5; // alpha color-color densities                          --!!--
				chessPoints_.color.a = 1;
				bool enable_chess=false;

				geometry_msgs::Point pi;
				pi.x = pi.y = pi.z = 0;				
				chessSquares_.points.push_back(pi);

				//green points
				chessPoints_.color.g = 1.0f;

				if(id==1){
					chessSquares_.color.r = 0.5f;
					chessSquares_.color.g = 0.0f;
					chessSquares_.color.b = 0.5f;
					tag_index=1;
				}else if(id==2){
					chessSquares_.color.r = 0.5f;
					chessSquares_.color.g = 0.5f;
					chessSquares_.color.b = 0.0f;
					tag_index=1;
				}else if(id==3){
					chessSquares_.color.r = 0.0f;
					chessSquares_.color.g = 0.5f;
					chessSquares_.color.b = 0.5f;
					tag_index=1;
				}else if(id==4){
					chessSquares_.color.r = 0.5f;
					chessSquares_.color.g = 0.5f;
					chessSquares_.color.b = 0.2f;
					tag_index=1;
				}else if(id==5){
					chessSquares_.color.r = 0.0f;
					chessSquares_.color.g = 0.5f;
					chessSquares_.color.b = 0.0f;
					tag_index=1;
				}else if(id==6){
					chessSquares_.color.r = 0.0f;
					chessSquares_.color.g = 0.0f;
					chessSquares_.color.b = 0.5f;
					tag_index=1;
				}else if(id==7){
					chessSquares_.color.r = 0.5f;
					chessSquares_.color.g = 0.0f;
					chessSquares_.color.b = 0.0f;
					tag_index=1;
				}
				else{
					chessSquares_.color.r = 1.0f;
					chessSquares_.color.g = 1.0f;
					chessSquares_.color.b = 1.0f;
					tag_index=-1;
				}

				chessSquares_.frame_locked=false; // ? 
				
				if(enable_chess){
					for (double i = -3.5; i < 4; i++)
				    {
				    	for (int j = 1; j < 9; j++)
				    	{
							geometry_msgs::Point pi;
							double marker_area=0.001+marker_size/100.0; // 0.001 is for the chess line bordering
							
							pi.x = tag_index*marker_area*i;
							pi.y = marker_area*j;
							pi.z = 0; // in a relation with the started pose.. init tag pose
							chessSquares_.points.push_back(pi);
		
							//and for the 4 knobs of the current square..
							pi.z=pi.z+0.001;
							double init_x = pi.x;
							double init_y = pi.y;	

							// inverse x-y axis system - (x is on y_axis and y on x_axis)
							// so for points (x,y) -> 
							//(-1,-1) bottom_left corner 
							//(-1, 1) bottom right				
							//(1,-1) top left 
							//(1, 1) top right

//NEEDS OPTIMISATION LIKE 2D !!

							if(i==-3.5 && j==1){								
								//visualize all the knobs (4 total)
								pi.x=init_x-marker_area*0.5;
								pi.y=init_y-marker_area*0.5;						
								chessPoints_.points.push_back(pi);
								pi.y=init_y+marker_area*0.5;
								chessPoints_.points.push_back(pi);
								pi.x=init_x+marker_area*0.5;	
								chessPoints_.points.push_back(pi);
								pi.y=init_y-marker_area*0.5;
								chessPoints_.points.push_back(pi);
							}else if(i==-3.5){
								//visualize the 2 right knobs
								pi.x=init_x+marker_area*0.5;
								pi.y=init_y+marker_area*0.5;						
								chessPoints_.points.push_back(pi);
								pi.x=init_x-marker_area*0.5;
								chessPoints_.points.push_back(pi);
							}else if(j==1){
								//visualize the 2 top knobs				
								pi.x=init_x+marker_area*0.5;
								pi.y=init_y+marker_area*0.5;						
								chessPoints_.points.push_back(pi);
								pi.y=init_y-marker_area*0.5;
								chessPoints_.points.push_back(pi);
							}else{
								//visualize only the top right knob
								pi.x=init_x+marker_area*0.5;
								pi.y=init_y+marker_area*0.5;						
								chessPoints_.points.push_back(pi);
							} 
							
						}
				    }
				}

				chessSquares_.lifetime = chessPoints_.lifetime = ros::Duration (1.0);	
				rvizMarkerPub_.publish (chessSquares_);
				//rvizMarkerPub_.publish (chessPoints_);

				//Get the pose of the tag in the camera frame, then the output frame (usually torso)				
				tf::Transform tagPoseOutput = CamToOutput * markerPose;

				//Create the pose marker messages
				ar_track_alvar_msgs::AlvarMarker ar_pose_marker;
				tf::poseTFToMsg (tagPoseOutput, ar_pose_marker.pose.pose);
				
      			ar_pose_marker.header.frame_id = output_frame;
			    ar_pose_marker.header.stamp = image_msg->header.stamp;
			    ar_pose_marker.id = id;
			    //ROS_INFO("%f",tagPoseOutput.getOrigin().m_floats[0]);
			    //ROS_INFO("%02i Z in cam frame: %f ",id, tagPoseOutput.position.x);
                ar_pose_marker.pose.pose.position.y=ar_pose_marker.pose.pose.position.y+10;

			    arPoseMarkers_.markers.push_back (ar_pose_marker);	
			}
			arMarkerPub_.publish (arPoseMarkers_);
		}
        catch (cv_bridge::Exception& e){
      		ROS_ERROR ("Could not convert from '%s' to 'rgb8'.", image_msg->encoding.c_str ());
    	}
	}
}

void configCallback(ar_track_alvar::ParamsConfig &config, uint32_t level)
{
  ROS_INFO("AR tracker reconfigured: %s %.2f %.2f %.2f %.2f", config.enabled ? "ENABLED" : "DISABLED",
           config.max_frequency, config.marker_size, config.max_new_marker_error, config.max_track_error);

  enableSwitched = enabled != config.enabled;

  enabled = config.enabled;
  max_frequency = config.max_frequency;
  marker_size = config.marker_size;
  max_new_marker_error = config.max_new_marker_error;
  max_track_error = config.max_track_error;
}

void enableCallback(const std_msgs::BoolConstPtr& msg)
{
    enableSwitched = enabled != msg->data;
    enabled = msg->data;
}

int main(int argc, char *argv[])
{
	ros::init (argc, argv, "marker_detect");
	ros::NodeHandle n, pn("~");
	
	//for the plane equation estimation
/*	id_0.setValue(0,0,0);
	id_1.setValue(0,0,0);
	id_2.setValue(0,0,0);
	id_3.setValue(0,0,0);
*/
	if(argc < 7){
		std::cout << std::endl;
		cout << "Not enough arguments provided." << endl;
		cout << "Usage: ./individualMarkersForChess <chess block ? size in cm> <max new marker error> "
		     << "<max track error> <cam image topic> <cam info topic> <output frame> [ <max frequency> ]";
		std::cout << std::endl;
		return 0;
	}

	// Get params from command line
	marker_size = atof(argv[1]);
	max_new_marker_error = atof(argv[2]);
	max_track_error = atof(argv[3]);
	cam_image_topic = argv[4];
	cam_info_topic = argv[5];
    output_frame = argv[6];
	marker_detector.SetMarkerSize(marker_size);

  if (argc > 7)
    max_frequency = atof(argv[7]);

  // Set dynamically configurable parameters so they don't get replaced by default values
  pn.setParam("marker_size", marker_size);
  pn.setParam("max_new_marker_error", max_new_marker_error);
  pn.setParam("max_track_error", max_track_error);

  if (argc > 7)
    pn.setParam("max_frequency", max_frequency);

	cam = new Camera(n, cam_info_topic);
	tf_listener = new tf::TransformListener(n);
	tf_broadcaster = new tf::TransformBroadcaster();
	arMarkerPub_ = n.advertise < ar_track_alvar_msgs::AlvarMarkers > ("ar_pose_marker", 0);
	rvizMarkerPub_ = n.advertise < visualization_msgs::Marker > ("visualization_marker", 0);
	
  // Prepare dynamic reconfiguration
  dynamic_reconfigure::Server < ar_track_alvar::ParamsConfig > server;
  dynamic_reconfigure::Server<ar_track_alvar::ParamsConfig>::CallbackType f;

  f = boost::bind(&configCallback, _1, _2);
  server.setCallback(f);

	//Give tf a chance to catch up before the camera callback starts asking for transforms
  // It will also reconfigure parameters for the first time, setting the default values
	ros::Duration(1.0).sleep();
	ros::spinOnce();	
	 
	image_transport::ImageTransport it_(n);

  // Run at the configured rate, discarding pointcloud msgs if necessary
  ros::Rate rate(max_frequency);

  /// Subscriber for enable-topic so that a user can turn off the detection if it is not used without
  /// having to use the reconfigure where he has to know all parameters
  ros::Subscriber enable_sub_ = pn.subscribe("enable_detection", 1, &enableCallback);

  enableSwitched = true;
  while (ros::ok())
  {
    ros::spinOnce();
    rate.sleep();

    if (std::abs((rate.expectedCycleTime() - ros::Duration(1.0 / max_frequency)).toSec()) > 0.001)
    {
      // Change rate dynamically; if must be above 0, as 0 will provoke a segfault on next spinOnce
      ROS_DEBUG("Changing frequency from %.2f to %.2f", 1.0 / rate.expectedCycleTime().toSec(), max_frequency);
      rate = ros::Rate(max_frequency);
    }

    if (enableSwitched)
    {
      // Enable/disable switch: subscribe/unsubscribe to make use of pointcloud processing nodelet
      // lazy publishing policy; in CPU-scarce computer as TurtleBot's laptop this is a huge saving
        if (enabled)
            cam_sub_ = it_.subscribe(cam_image_topic, 1, &getCapCallback);
        else
            cam_sub_.shutdown();
        enableSwitched = false;
    }
  }

    return 0;
}
