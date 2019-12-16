#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>
#include <std_msgs/UInt16MultiArray.h>

enum attributes { x=0, y=1, s=2 };
enum angles { yaw=0, pitch=1, roll=2 };


// TODO: track-point to model-point identification
std::vector<cv::Point2f> track_points;
bool new_trackpoints_available = false;
void updateTrackpoints(const std_msgs::UInt16MultiArray &blobs){
  track_points.clear();
  for (int pt = 0; pt < blobs.layout.dim[0].size; pt++){
    track_points.push_back( cv::Point2f( blobs.data[pt*3+x],blobs.data[pt*3+y]) );
  }
  new_trackpoints_available = true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "opencv_posit_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  double publish_rate;
  nhLocal.param("publish_rate", publish_rate, 10.0);

  ros::Subscriber tracker = nh.subscribe("/spacecam/trackpoints", 50, updateTrackpoints);

  std::vector<cv::Point3f> model_points;
  model_points.push_back(cv::Point3d(0.0,   0.0,  0.0));
  model_points.push_back(cv::Point3d(0.025, 0.0,  0.0));
  model_points.push_back(cv::Point3d(0.0,   0.0,  0.075));
  model_points.push_back(cv::Point3d(0.0,   0.5,  0.0));

  double focal_length = 1024; // horizontal resolution of pixart chip as approx.
  cv::Point2d center = cv::Point2d(512,384);
  cv::Mat camera_matrix = (cv::Mat_<double>(3,3) << focal_length, 0, center.x, 0 , focal_length, center.y, 0, 0, 1);
  cv::Mat dist_coeffs = cv::Mat::zeros(4,1,cv::DataType<double>::type); // distortion is compensated in chip and in plugin as well

  // posit results
  cv::Mat rotation_vector;
  cv::Mat translation_vector(3, 1, cv::DataType<float>::type);

  // model coord. --> camera coord.
  cv::Mat rotation_matrix   = cv::Mat(3, 3, CV_64FC1); // 3 x 3  R
  cv::Mat projection_matrix = cv::Mat(3, 4, CV_64FC1); // 3 x 4  P = R | T

  // decomposition results
  cv::Mat out_intrinsics  = cv::Mat(3, 3, CV_64FC1);
  cv::Mat out_rotation    = cv::Mat(3, 3, CV_64FC1);
  cv::Mat out_translation = cv::Mat(4, 1, CV_64FC1); // decomposeProjectionMatrix output is cv::Mat(4, 1, CV_64FC1);
  cv::Mat euler_angles    = cv::Mat(3, 1, CV_64FC1);

  bool debug;
  nhLocal.param("debug", debug, true);
  if (debug) cv::namedWindow("debug",1);
  cv::Mat image = cv::Mat::zeros( 768, 1024, CV_8UC3 ); // debug image
  std::ostringstream debugtext;
  double projection_error;
  double error_threshold = 35.0;
  std::vector<cv::Point2f> projectedPoints; // reprojection

  ros::Rate r(publish_rate);
  while (ros::ok()) {

    ros::spinOnce();
    if (new_trackpoints_available){

    /* SOLVEPNP_ITERATIVE

      Iterative method is based on Levenberg-Marquardt optimization.
      In this case the function finds such a pose that minimizes reprojection error,
      that is the sum of squared distances between the observed projections imagePoints
      and the projected ( using projectPoints ) objectPoints.     */

      cv::solvePnP(model_points, track_points, camera_matrix, dist_coeffs, rotation_vector, translation_vector, cv::SOLVEPNP_ITERATIVE);

    /* REPROJECT MODEL POINTS   */
      // rotation vector to rotation matrix ( 1 x 3 --> 3 x 3 )
      cv::Rodrigues(rotation_vector, rotation_matrix);
      cv::projectPoints(model_points, rotation_matrix, translation_vector, camera_matrix, dist_coeffs, projectedPoints);
      projection_error = cv::norm(projectedPoints, track_points)/projectedPoints.size();
      //ROS_INFO_STREAM("Error: " << projection_error);

    /* CALCULATE EULER ANGLES    */

      if (projection_error < error_threshold){
        // horizontal matrix concatenation ( projection_matrix 3 x 4   P = R | T )
        cv::hconcat(rotation_matrix, translation_vector, projection_matrix);
        // decompose projection matrix into rotation and translation in the camera coordinate system, also produce Euler angles
        cv::decomposeProjectionMatrix(projection_matrix, out_intrinsics, out_rotation, out_translation, cv::noArray(), cv::noArray(), cv::noArray(), euler_angles);
        //ROS_INFO_STREAM("Yaw: " << euler_angles.at<double>(yaw) << "\tPitch: " << euler_angles.at<double>(pitch) << "\tRoll: " << euler_angles.at<double>(roll));
      }
      new_trackpoints_available = false;
    }

    if (debug){
      image = cv::Mat::zeros( 768, 1024, CV_8UC3 );
      debugtext.str(""); debugtext.clear();
      debugtext << "Error: " << projection_error;
      cv::putText(image, debugtext.str(), cv::Point2f(10, 15), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
      debugtext.str(""); debugtext.clear();
      debugtext << "Pitch: " << euler_angles.at<double>(pitch);
      cv::putText(image, debugtext.str(), cv::Point2f(10, 35), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));

      // visualize track_points with order labels
      for (unsigned int i = 0; i < track_points.size(); i++) {
        debugtext.str(""); debugtext.clear(); debugtext << i;
        cv::circle(image, cv::Point2f(track_points[i].x, track_points[i].y), 5, cv::Scalar(255, 255, 255), -1);
        cv::putText(image, debugtext.str(), cv::Point2f(track_points[i].x+10, track_points[i].y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
      }
      // visualize reprojection error
      if (projection_error < error_threshold){
        for (unsigned int i = 0; i < projectedPoints.size(); i++) {
          debugtext.str(""); debugtext.clear(); debugtext << i;
          cv::circle(image, cv::Point2f(projectedPoints[i].x, projectedPoints[i].y), 5, cv::Scalar(0, 0, 255), -1);
          cv::putText(image, debugtext.str(), cv::Point2f(projectedPoints[i].x+10, projectedPoints[i].y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(0, 0, 255));
        }
      }

      cv::imshow("debug", image);
      unsigned char key = cv::waitKey(1);
      if (key == 27) {
        break;
      }
    }
  }
  return 0;
}
