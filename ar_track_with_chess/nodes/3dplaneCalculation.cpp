
#include <fstream>

namespace a=ar_track_alvar;
namespace gm=geometry_msgs;

// Random float between a and b
float randFloat (float a, float b)
{
  const float u = static_cast<float>(rand())/RAND_MAX;
  return a + u*(b-a);
}

// Generate points in a square in space of form p+av+bw where 
// a and b range from 0 to 1
a::ARCloud::Ptr generateCloud(const double px, const double py, const double pz,
                              const double vx, const double vy, const double vz,
                              const double wx, const double wy, const double wz)
{
  const double INC=0.1;
  const double NOISE=0.01;

  a::ARCloud::Ptr cloud(boost::make_shared<a::ARCloud>());
  for (double u=0; u<1+INC/2; u+=INC)
  {
    for (double v=0; v<1+INC/2; v+=INC)
    {
      a::ARPoint p;
      p.x = px+u*vx+v*wx+randFloat(-NOISE, NOISE);
      p.y = py+u*vy+v*wy+randFloat(-NOISE, NOISE);
      p.z = pz+u*vz+v*wz+randFloat(-NOISE, NOISE);
      cloud->points.push_back(p);
    }
  }
  return cloud;
}

//in getCapCallback

// 3D Plane Estimation

        /* //fakes translations
        pt.x=0.035301; pt.y=0.138741;pt.z=0.862000;
        cloud->points.push_back(pt);  
        pt.x=0.012314; pt.y=0.130531;pt.z=0.862000;
        cloud->points.push_back(pt);  
        pt.x=0.048605; pt.y=0.119452;pt.z=0.865000;
        cloud->points.push_back(pt);  
        pt.x=0.025656; pt.y=0.101797;pt.z=0.869000;
        cloud->points.push_back(pt);*/  

          /*
        a::ARCloud::Ptr cloud(new a::ARCloud());
        a::ARPoint pt;

        // more than 3 detected markers needed for better plane equation estimation .. 
        if(id==0){
          id_0.setValue(px,py,pz);
        }else if(id==1){
          id_1.setValue(px,py,pz);
        }else if(id==2){
          id_2.setValue(px,py,pz);
        }else if(id==3){
          id_3.setValue(px,py,pz);
        }

        //the real one
        pt.x=id_0.x(); pt.y=id_0.y();pt.z=id_0.z();
        cloud->points.push_back(pt);  
        pt.x=id_1.x(); pt.y=id_1.y();pt.z=id_1.z();
        cloud->points.push_back(pt);  
        pt.x=id_2.x(); pt.y=id_2.y();pt.z=id_2.z();
        cloud->points.push_back(pt);  
        pt.x=id_3.x(); pt.y=id_3.y();pt.z=id_3.z();
        cloud->points.push_back(pt);

        //ROS_INFO("Cloud has %zu points such as (%.2f, %.2f, %.2f)",cloud->points.size(), cloud->points[0].x, cloud->points[0].y,cloud->points[0].z);

        a::PlaneFitResult res = a::fitPlane(cloud); //plane equation calculation
        ROS_INFO("Plane equation is %.3fx + %.3fy + %.3fz + %.3f = 0",
        res.coeffs.values[0], res.coeffs.values[1], res.coeffs.values[2],
        res.coeffs.values[3]);

        //gm::Quaternion q = a::extractOrientation(res.coeffs, p1, p2, p3, p1);
        //ROS_INFO_STREAM("Orientation is " << q);
          
          if(id==3){
            std::string markerFrame2 = "somethingpanwapototria2";
            tf::Quaternion rotation2 (qx,qy,qz,qw);
            pz=pz+0.03;
            px=px;
            py=-(res.coeffs.values[0]*px+ res.coeffs.values[2]*pz + res.coeffs.values[3])/res.coeffs.values[1];
            //py=py-0.014;
                  tf::Vector3 origin2 (px,py,pz);
                  ROS_INFO("mine %f %f %f",px,py,pz);
                  tf::Transform t2 (rotation2, origin2);
                  //ROS_INFO("t %f %f %f", t2.getOrigin().m_floats[0],t2.getOrigin().m_floats[1],t2.getOrigin().m_floats[2]);
          tf::StampedTransform camToMarker2 (t2, image_msg->header.stamp, image_msg->header.frame_id, markerFrame2.c_str());
            tf_broadcaster->sendTransform(camToMarker2);
          std::string markerFrame3 = "somethingpanwapototria3";
          tf::Quaternion rotation3 (qx,qy,qz,qw);
          pz=pz+0.06;
          px=px;
          py=-(res.coeffs.values[0]*px+ res.coeffs.values[2]*pz + res.coeffs.values[3])/res.coeffs.values[1];
          //py=py-0.014;
          tf::Vector3 origin3 (px,py,pz);
          ROS_INFO("mine %f %f %f",px,py,pz);
          tf::Transform t3 (rotation3, origin3);
          //ROS_INFO("t %f %f %f", t3.getOrigin().m_floats[0],t3.getOrigin().m_floats[1],t3.getOrigin().m_floats[3]);
          tf::StampedTransform camToMarker3 (t3, image_msg->header.stamp, image_msg->header.frame_id, markerFrame3.c_str());
          tf_broadcaster->sendTransform(camToMarker3);
          std::string markerFrame4 = "somethingpanwapototria4";
            tf::Quaternion rotation4 (qx,qy,qz,qw);
            pz=pz+0.09;
            px=px;
            py=-(res.coeffs.values[0]*px+ res.coeffs.values[2]*pz + res.coeffs.values[3])/res.coeffs.values[1];
            //py=py-0.014;
                  tf::Vector3 origin4 (px,py,pz);
                  ROS_INFO("mine %f %f %f",px,py,pz);
                  tf::Transform t4 (rotation4, origin4);
                  //ROS_INFO("t %f %f %f", t4.getOrigin().m_floats[0],t4.getOrigin().m_floats[1],t4.getOrigin().m_floats[4]);
          tf::StampedTransform camToMarker4 (t4, image_msg->header.stamp, image_msg->header.frame_id, markerFrame4.c_str());
            tf_broadcaster->sendTransform(camToMarker4);

        }
            
        */