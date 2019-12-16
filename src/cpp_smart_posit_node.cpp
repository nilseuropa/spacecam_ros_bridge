#include <ros/ros.h>
#include <std_msgs/UInt16MultiArray.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <MatrixMath.h>
#include <pos.h>

TImage  image;
TObject model;
TCamera pose;
TEuler  euler;
POS     posit;

struct Blob {
   	int X;
   	int Y;
   	int Size;
};
Blob blobArray[4];

void updateImage() {
  // use reordered marker index and create image point vectors
  for (uint8_t i = 0; i < model.nbPts; i++){
    image.imagePts[i][0] = blobArray[i].X;
    image.imagePts[i][1] = blobArray[i].Y;
    for(uint8_t j=0; j < 2; j++){
      image.imageVects[i][j]=(double)(image.imagePts[i][j]-image.imagePts[0][j]);
    }
  }
}

enum attributes { x=0, y=1, s=2 };
bool new_trackpoints_available = false;

void updateTrackpoints(const std_msgs::UInt16MultiArray &blobs){
  for (int pt = 0; pt < blobs.layout.dim[0].size; pt++) { // NOTE: Runtime error if blobs.layout.dim[0].size > 4 !
    blobArray[pt].X = blobs.data[pt*3+x];
    blobArray[pt].Y = blobs.data[pt*3+y];
  }
  if (blobs.layout.dim[0].size == 4) {
    // Only use posit if exactly 4 points arrive
    updateImage();
    new_trackpoints_available = true;
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "cpp_smart_posit_node");
  ros::NodeHandle nh;
  ros::NodeHandle nhLocal("~");

  double publish_rate;
  nhLocal.param("publish_rate", publish_rate, 10.0);
  bool debug;
  nhLocal.param("debug", debug, true);

  if (debug) cv::namedWindow("cpp_smart_posit_debug", 1);
  cv::Mat debugframe = cv::Mat::zeros( 768, 1024, CV_8UC3 ); // debug image
  std::ostringstream debugtext;

  ros::Subscriber tracker = nh.subscribe("/spacecam/trackpoints", 50, updateTrackpoints);

  /* Object data (Constant) */
  model.nbPts = 4;

  model.objectPts[0][0] = 0.0f;  // Marker 1 (origin)
  model.objectPts[0][1] = 0.0f;
  model.objectPts[0][2] = 0.0f;
  model.objectPts[1][0] = 0.25f; // Marker 2 (hx)
  model.objectPts[1][1] = 0.0f;
  model.objectPts[1][2] = 0.0f;
  model.objectPts[2][0] = 0.0f;  // Marker 3
  model.objectPts[2][1] = 0.5f;  // (hy)
  model.objectPts[2][2] = 0.0f;
  model.objectPts[3][0] = 0.0f;  // Marker 4 
  model.objectPts[3][1] = 0.0f;
  model.objectPts[3][2] = 0.75f; // (hz)

  model.objectMatrix[0][0] = 0.0f; // Pseudoinverse of objectPts matrix
  model.objectMatrix[0][1] = 1.0f/0.25f; // 1/hx
  model.objectMatrix[0][2] = 0.0f;
  model.objectMatrix[0][3] = 0.0f;
  model.objectMatrix[1][0] = 0.0f;
  model.objectMatrix[1][1] = 0.0f;
  model.objectMatrix[1][2] = 1.0f/0.5f;  // 1/hy
  model.objectMatrix[1][3] = 0.0f;
  model.objectMatrix[2][0] = 0.0f;
  model.objectMatrix[2][1] = 0.0f;
  model.objectMatrix[2][2] = 0.0f;
  model.objectMatrix[2][3] = 1.0f/0.75f; // 1/hz

  for (uint8_t i = 0; i < model.nbPts; i++){
  	for(uint8_t j = 0; j < 3; j++){
  		model.objectVects[i][j] = model.objectCopy[i][j] = model.objectPts[i][j] - model.objectPts[0][j];
  	}
  }

  /* Image data */
  image.imageCenter[0] = 512;
  image.imageCenter[1] = 384;
  image.nbPts = 4;

  /* Camera data */
  pose.focalLength = 1024; // TODO: What's the dimension of this?
  pose.rotation[0][0]=1.0;
  pose.rotation[0][1]=0.0;
  pose.rotation[0][2]=0.0;
  pose.rotation[1][0]=0.0;
  pose.rotation[1][1]=1.0;
  pose.rotation[1][2]=0.0;
  pose.rotation[2][0]=0.0;
  pose.rotation[2][1]=0.0;
  pose.rotation[2][2]=1.0;

  ros::Rate r(publish_rate);
  while (ros::ok()) {
    ros::spinOnce();

    if (new_trackpoints_available) {
      TCorrespondence corr;
      posit.findBestCorrespondence(model, image, &pose, &corr);
      posit.estimate(model, image, &pose, &corr);
      euler = posit.eulerOf(pose.rotation);
      new_trackpoints_available = false;
    }

    if (debug) {
      debugframe = cv::Mat::zeros( 768, 1024, CV_8UC3 );
      debugtext.str(""); debugtext.clear();
      debugtext << "Yaw   : " << euler.yaw;
      cv::putText(debugframe, debugtext.str(), cv::Point2f(10, 15), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
      debugtext.str(""); debugtext.clear();
      debugtext << "Pitch : " << euler.roll;
      cv::putText(debugframe, debugtext.str(), cv::Point2f(10, 35), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));

      // visualize track_points with order labels
      for (unsigned int i = 0; i < sizeof(blobArray); i++) {
        cv::circle(debugframe, cv::Point2f(blobArray[i].X, blobArray[i].Y), 5, cv::Scalar(255, 255, 255), -1);
        debugtext.str(""); debugtext.clear(); debugtext << i;
        cv::putText(debugframe, debugtext.str(), cv::Point2f(blobArray[i].X+10, blobArray[i].Y), cv::FONT_HERSHEY_PLAIN, 1.0, cv::Scalar(255, 255, 255));
      }

      cv::imshow("cpp_smart_posit_debug", debugframe);
      unsigned char key = cv::waitKey(1);
      if (key == 27) {
        break;
      }
    }
  }
  return 0;
}
