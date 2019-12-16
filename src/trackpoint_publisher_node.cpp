#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d.hpp>
#include <std_msgs/UInt16MultiArray.h>

using namespace cv;

Ptr<SimpleBlobDetector> detector;
ros::Publisher blob_pub;
std_msgs::UInt16MultiArray blobs;
enum attributes { x=0, y=1, s=2 };

double map(double x, double in_min, double in_max, double out_min, double out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
};

bool use_reference;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{

  cv_bridge::CvImagePtr cv_ptr;
  try {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e) {
    ROS_ERROR("SpaceCam exception: %s", e.what());
    return;
  }

  if (use_reference){
    cv::Mat segmented_image;
    std::vector<cv::KeyPoint> keypoints;

    cv::inRange(cv_ptr->image, cv::Scalar(0, 0, 0), cv::Scalar(0, 0, 255), segmented_image); // red filter
    detector->detect(segmented_image, keypoints);
    if (keypoints.size()>0) {
      blobs.data[x] = keypoints[0].pt.x;
      blobs.data[y] = keypoints[0].pt.y;
      blobs.data[s] = keypoints[0].size;
    } else {
      blobs.data[x] = 1023;
      blobs.data[y] = 1023;
      blobs.data[s] = 0;
    }

    cv::inRange(cv_ptr->image, cv::Scalar(0, 0, 0), cv::Scalar(255, 0, 0), segmented_image); // blue filter
    detector->detect(segmented_image, keypoints);
    if (keypoints.size()>0) {
      blobs.data[3+x] = keypoints[0].pt.x;
      blobs.data[3+y] = keypoints[0].pt.y;
      blobs.data[3+s] = keypoints[0].size;
    } else {
      blobs.data[3+x] = 1023;
      blobs.data[3+y] = 1023;
      blobs.data[3+s] = 0;
    }

    cv::inRange(cv_ptr->image, cv::Scalar(0, 0, 0), cv::Scalar(0, 255, 0), segmented_image); // green filter
    detector->detect(segmented_image, keypoints);
    if (keypoints.size()>0) {
      blobs.data[6+x] = keypoints[0].pt.x;
      blobs.data[6+y] = keypoints[0].pt.y;
      blobs.data[6+s] = keypoints[0].size;
    } else {
      blobs.data[6+x] = 1023;
      blobs.data[6+y] = 1023;
      blobs.data[6+s] = 0;
    }

    cv::inRange(cv_ptr->image, cv::Scalar(0, 128, 128), cv::Scalar(0, 255, 255), segmented_image); // yellow filter
    detector->detect(segmented_image, keypoints);
    if (keypoints.size()>0) {
      blobs.data[9+x] = keypoints[0].pt.x;
      blobs.data[9+y] = keypoints[0].pt.y;
      blobs.data[9+s] = keypoints[0].size;
    } else {
      blobs.data[9+x] = 1023;
      blobs.data[9+y] = 1023;
      blobs.data[9+s] = 0;
    }
  }

  else {
    cv::Mat filtered_image;
    cv::cvtColor(cv_ptr->image,filtered_image,CV_BGR2GRAY);
    std::vector<cv::KeyPoint> keypoints;
    detector->detect(filtered_image, keypoints);


    if (keypoints.size()>0) {
      for (unsigned int pt=0; pt<keypoints.size(); pt++){
        blobs.data[pt*3+x] = keypoints[pt].pt.x;
        blobs.data[pt*3+y] = keypoints[pt].pt.y;
        blobs.data[pt*3+s] = keypoints[pt].size; // TODO: map and constrain [0-15]
      }
      for (unsigned int pt=keypoints.size(); pt<blobs.data.size(); pt++) {
        blobs.data[pt*3+x] = 1023;
        blobs.data[pt*3+y] = 1023;
        blobs.data[pt*3+s] = 0;
      }
    }
    else {
      for (unsigned int pt=keypoints.size(); pt<blobs.data.size(); pt++) {
        blobs.data[pt*3+x] = 1023;
        blobs.data[pt*3+y] = 1023;
        blobs.data[pt*3+s] = 0;
      }
    }
  }
  blob_pub.publish(blobs);

}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "trackpoint_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  nhLocal.param("use_reference", use_reference, false);

  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("spacecam/image_raw", 1, imageCallback);
  blob_pub = nh.advertise<std_msgs::UInt16MultiArray>("/spacecam/trackpoints", 100);

  ROS_INFO_STREAM("Using OpenCV" << CV_MAJOR_VERSION);

  // Configure Blob detector
  SimpleBlobDetector::Params params;
  params.filterByArea = false;
  params.filterByColor = true;
  params.blobColor = 255;
  detector = SimpleBlobDetector::create(params);

  blobs.layout.dim.push_back(std_msgs::MultiArrayDimension());
  blobs.layout.dim[0].label  = "points";
  blobs.layout.dim[0].size   = 4;
  blobs.layout.dim[0].stride = 3*4;
  blobs.layout.dim.push_back(std_msgs::MultiArrayDimension());
  blobs.layout.dim[1].label  = "attributes";
  blobs.layout.dim[1].size   = 3;
  blobs.layout.dim[1].stride = 3;
  blobs.data.resize(3*4);

  ros::spin();
  return 0;
}
