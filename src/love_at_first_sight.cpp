#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/types_c.h>
#include <std_msgs/UInt16MultiArray.h>
#include <iostream>

enum attributes { x=0, y=1, s=2 };
enum colors {red=0, blue=1, green=2, yellow=3};

std::vector<cv::Point2f> followed_points(4);
std::vector<cv::Point2f> track_points(4);
std::vector<cv::Point2f> track_points_buffer(16);
std::vector<cv::Point2f> red_points(4);
std::vector<cv::Point2f> green_points(4);
std::vector<cv::Point2f> blue_points(4);
std::vector<cv::Point2f> yellow_points(4);

std_msgs::UInt16MultiArray blobs;
bool got_points = false;

void updateTrackpoints(const std_msgs::UInt16MultiArray &blobs){
  for (int pt = 0; pt < blobs.layout.dim[0].size; pt++){
    track_points_buffer.push_back( cv::Point2f( blobs.data[pt*3+x],blobs.data[pt*3+y]) );
    track_points_buffer.erase(track_points_buffer.begin());
  }
  track_points.clear();
  for (int pt = 0; pt < blobs.layout.dim[0].size; pt++){
    track_points.push_back( cv::Point2f( blobs.data[pt*3+x],blobs.data[pt*3+y]) );
  }
  got_points = true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "love_at_first_sight_ordering");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  double publish_rate;
  nhLocal.param("publish_rate", publish_rate, 100.0);
  ros::Rate r(publish_rate);

  bool debug;
  nhLocal.param("debug", debug, true);
  if (debug) cv::namedWindow("debug",1);

  double motion_threshold = 50;
  nhLocal.param("motion_threshold", motion_threshold, 50.0);

  blobs.layout.dim.push_back(std_msgs::MultiArrayDimension());
  blobs.layout.dim[0].label  = "points";
  blobs.layout.dim[0].size   = 4;
  blobs.layout.dim[0].stride = 3*4;
  blobs.layout.dim.push_back(std_msgs::MultiArrayDimension());
  blobs.layout.dim[1].label  = "attributes";
  blobs.layout.dim[1].size   = 3;
  blobs.layout.dim[1].stride = 3;
  blobs.data.resize(3*4);

  ros::Subscriber tracker = nh.subscribe("/spacecam/trackpoints", 50, updateTrackpoints);
  ros::Publisher  blob_pub = nh.advertise<std_msgs::UInt16MultiArray>("/spacecam/trackpoints_hopefully_ordered", 100); // :)

  bool initialized = false;
  while (ros::ok() && !initialized){
    ros::spinOnce();
    if (got_points) {
      followed_points = track_points;
      initialized = true;
    }
  }

  while (ros::ok()) {

    ros::spinOnce();

    // point to color group
    for (unsigned int y = 0; y < track_points_buffer.size(); y++){
      for (unsigned int x = 0; x < followed_points.size(); x++){
        double norm = cv::norm(cv::Mat(followed_points[x]), cv::Mat(track_points_buffer[y]));
        if (x==red && norm < motion_threshold) {
          red_points.push_back(track_points_buffer[y]);
          red_points.erase(red_points.begin());
        }
        if (x==blue && norm < motion_threshold) {
          blue_points.push_back(track_points_buffer[y]);
          blue_points.erase(blue_points.begin());
        }
        if (x==green && norm < motion_threshold) {
          green_points.push_back(track_points_buffer[y]);
          green_points.erase(green_points.begin());
        }
        if (x==yellow && norm < motion_threshold) {
          yellow_points.push_back(track_points_buffer[y]);
          yellow_points.erase(yellow_points.begin());
        }
      }
    }

    // trackpoint to color
    for (unsigned int y = 0; y < track_points.size(); y++){
      for (unsigned int x = 0; x < green_points.size(); x++){
        double norm = cv::norm(cv::Mat(track_points[y]), cv::Mat(green_points[x]));
        if (norm < motion_threshold){ followed_points[green] = track_points[y]; }
      }
      for (unsigned int x = 0; x < red_points.size(); x++){
        double norm = cv::norm(cv::Mat(track_points[y]), cv::Mat(red_points[x]));
        if (norm < motion_threshold){ followed_points[red] = track_points[y]; }
      }
      for (unsigned int x = 0; x < blue_points.size(); x++){
        double norm = cv::norm(cv::Mat(track_points[y]), cv::Mat(blue_points[x]));
        if (norm < motion_threshold){ followed_points[blue] = track_points[y]; }
      }
      for (unsigned int x = 0; x < yellow_points.size(); x++){
        double norm = cv::norm(cv::Mat(track_points[y]), cv::Mat(yellow_points[x]));
        if (norm < motion_threshold){ followed_points[yellow] = track_points[y]; }
      }
    }

    blobs.data[x]   = followed_points[red].x;
    blobs.data[y]   = followed_points[red].y;
    blobs.data[3+x] = followed_points[blue].x;
    blobs.data[3+y] = followed_points[blue].y;
    blobs.data[6+x] = followed_points[green].x;
    blobs.data[6+y] = followed_points[green].y;
    blobs.data[9+x] = followed_points[yellow].x;
    blobs.data[9+y] = followed_points[yellow].y;
    blob_pub.publish(blobs);

    if (debug){
      cv::Mat image = cv::Mat::zeros( 768, 1024, CV_8UC3 );

      // draw labeled history
      for (unsigned int i = 0; i < track_points.size(); i++) {
        cv::circle(image, red_points[i], 5, cv::Scalar(0, 0, 150), -1);
        cv::arrowedLine (image, red_points[i], followed_points[red], cv::Scalar(0, 0, 150), 1, 8, 0, 0.01);
        cv::circle(image, blue_points[i], 5, cv::Scalar(150, 0, 0), -1);
        cv::arrowedLine (image, blue_points[i], followed_points[blue], cv::Scalar(150, 0, 0), 1, 8, 0, 0.01);
        cv::circle(image, green_points[i], 5, cv::Scalar(0, 150, 0), -1);
        cv::arrowedLine (image, green_points[i], followed_points[green], cv::Scalar(0, 150, 0), 1, 8, 0, 0.01);
        cv::circle(image, yellow_points[i], 5, cv::Scalar(0, 150, 150), -1);
        cv::arrowedLine (image, yellow_points[i], followed_points[yellow], cv::Scalar(0, 150, 150), 1, 8, 0, 0.01);
      }

      // draw labels
      cv::circle(image, followed_points[red], 5, cv::Scalar(0,0,255),   -1);
      cv::circle(image, followed_points[blue], 5, cv::Scalar(255,0,0),   -1);
      cv::circle(image, followed_points[green], 5, cv::Scalar(0,255,0),   -1);
      cv::circle(image, followed_points[yellow], 5, cv::Scalar(0,255,255), -1);

      cv::imshow("debug", image);
      unsigned char key = cv::waitKey(1);
      if (key == 27) {
        break;
      }
      if (key == 's'){
        followed_points = track_points;
      }
    }
  }
  return 0;
}
