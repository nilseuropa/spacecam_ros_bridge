#ifndef _POS_H_
#define _POS_H_

#include <MatrixMath.h>
#include <vector>

typedef struct TObject{
	int nbPts;
	double objectPts[4][3];/* Array of coordinates of points */
	double objectVects[4][3];/* Array of coordinates of vectors from reference pt to all pts */
	double objectCopy[4][3];/* Copy of objectVects, used because SVD code destroys input data */
	double objectMatrix[3][4];/* Pseudoinverse of objectVects */
} TObject;

typedef struct TImage{
	int nbPts;
	int imageCenter[2];
	int imagePts[4][2];
	double imageVects[4][2];    /* scaled orthographic projections */
	double oldImageVects[4][2]; /* projections at previous iteration step */
	double epsilon[4];          /* Corrections to scaled orthographic projections at each iteration */
} TImage;

typedef struct TCamera{
	int focalLength;
	double rotation[3][3];/* Rotation of SCENE in camera reference, NOT other way around */
	double translation[3];/* Translation of SCENE in camera reference */

} TCamera;

typedef struct TEuler{
	/*Euler Angle representation of rotation matrix*/
	double yaw;
	double pitch;
	double roll;
} TEuler;

typedef struct Quaternion{
	double w;
	double x;
	double y;
	double z;
} Quaternion;

typedef struct TCorrespondence {
  std::vector<std::size_t> indices;
} TCorrespondence;

class POS{
public:
  void estimate(TObject object, TImage image, TCamera *camera);
  void estimate(TObject object, TImage image, TCamera *camera, TCorrespondence *corr);
  void findBestCorrespondence(TObject object, TImage image, TCamera *camera, TCorrespondence* out_best_corr);
  TEuler eulerOf(double rotation[3][3]);
  Quaternion quaternionOf(double m[3][3]);
};

#endif
