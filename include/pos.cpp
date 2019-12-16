#include <math.h>
#include <pos.h>

#include <iostream>
#include <algorithm>

double scalar_product(const double* a, const double* b) {
	return a[0]*b[0]+a[1]*b[1]+a[2]*b[2];
}

void normalize_vector(double* v) {
	double invlen = 1.0/sqrt(v[0]*v[0]+v[1]*v[1]+v[2]*v[2]);
	v[0] *= invlen;
	v[1] *= invlen;
	v[2] *= invlen;
}

void scale_vector(const double s, const double *v, double *result) {
	result[0] = s*v[0];
	result[1] = s*v[1];
	result[2] = s*v[2];
}

void subtract_vector(const double *a, const double* b, double *result) {
	result[0] = a[0]-b[0];
	result[1] = a[1]-b[1];
	result[2] = a[2]-b[2];
}

/*
POS function
(Pose from Orthography and Scaling, a scaled orthographic proj. approximation).
Returns one translation and one rotation.
*/


const int nbObjectCoords = 3; /* x, y, z */
const int nbImageCoords = 2; /* x, y */

double u2[3];
double u3[3];
double temp[3];

void POS::estimate(TObject object, TImage image, TCamera *camera) {

  double    I0[3], J0[3], row1[3], row2[3], row3[3];
  double    I0I0, J0J0;
  int       i, j;
  double    scale, scale1, scale2;

	/*Computing I0 and J0, the vectors I and J in TRs listed above */
	for (i=0;i<nbObjectCoords;i++){
    	I0[i]=0;
    	J0[i]=0;
    	for (j=0;j<object.nbPts;j++){
			I0[i]+=object.objectMatrix[i][j]*image.imageVects[j][0];
			J0[i]+=object.objectMatrix[i][j]*image.imageVects[j][1];
    	}
	}

	I0I0=I0[0]*I0[0] + I0[1]*I0[1] + I0[2]*I0[2];
	J0J0=J0[0]*J0[0] + J0[1]*J0[1] + J0[2]*J0[2];

	scale1 = sqrt(I0I0);
	scale2 = sqrt(J0J0);
	scale = (scale1 + scale2) / 2.0;

	/*Computing TRANSLATION */
	camera->translation[0] = image.imagePts[0][0]/scale;
	camera->translation[1] = image.imagePts[0][1]/scale;
	camera->translation[2] = camera->focalLength/scale;

	/* Computing ROTATION */
	for (i=0;i<3;i++){
 	   row1[i]=I0[i]/scale1;
 	   row2[i]=J0[i]/scale2;
	}
	row3[0]=row1[1]*row2[2]-row1[2]*row2[1];/* Cross-product to obtain third row */
	row3[1]=row1[2]*row2[0]-row1[0]*row2[2];
	row3[2]=row1[0]*row2[1]-row1[1]*row2[0];

    /* Gyebro: Orthogonalize the rotation matrix */
	/* u2 = normalize(row2 - <row2,row1>*row1) */
	scale_vector(scalar_product(row2, row1), row1, temp);
	subtract_vector(row2, temp, u2);					  // row2 - <row2,row1>*row1
	normalize_vector(u2);
	/* u3 = normalize(row3 - <row3,row1>*row1 - <row3,row2>*row2) */
	scale_vector(scalar_product(row3, row1), row1, temp);
	subtract_vector(row3, temp, u3);					  // u3 = row3 - <row3,row1>*row1
	scale_vector(scalar_product(row3, row2), row2, temp);
	subtract_vector(u3, temp, u3);						  // u3 -= <row3,row2>*row2
	normalize_vector(u3);

	for (i=0;i<3;i++){
 	   camera->rotation[0][i]=row1[i];
 	   camera->rotation[1][i]=row2[i];
 	   camera->rotation[2][i]=row3[i];
	}
}

Quaternion POS::quaternionOf(double m[3][3]) {
  Quaternion q;
  double trace = m[0][0] + m[1][1] + m[2][2];
  if (trace > 0) {
    double S = sqrt(trace+1.0) * 2; // S=4*qw
    q.w = S/4.0f;
    q.x = (m[2][1] - m[1][2]) / S;
    q.y = (m[0][2] - m[2][0]) / S;
    q.z = (m[1][0] - m[0][1]) / S;
  } else if ((m[0][0] > m[1][1])&(m[0][0] > m[2][2])) {
    double S = sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2; // S=4*qx
    q.w = (m[2][1] - m[1][2]) / S;
    q.x = S/4.0f;
    q.y = (m[0][1] + m[1][0]) / S;
    q.z = (m[0][2] + m[2][0]) / S;
  } else if (m[1][1] > m[2][2]) {
    double S = sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2; // S=4*qy
    q.w = (m[0][2] - m[2][0]) / S;
    q.x = (m[0][1] + m[1][0]) / S;
    q.y = S/4.0f;
    q.z = (m[1][2] + m[2][1]) / S;
  } else {
    double S = sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2; // S=4*qz
    q.w = (m[1][0] - m[0][1]) / S;
    q.x = (m[0][2] + m[2][0]) / S;
    q.y = (m[1][2] + m[2][1]) / S;
    q.z = S/4.0f;
  }
  return q;
}

TEuler POS::eulerOf(double R[3][3]){
  TEuler euler;
    euler.yaw   = atan2f(R[0][1],R[0][0]); // Yaw
    euler.pitch = asinf(R[0][2]);          // Pitch
    euler.roll  = atan2f(R[1][2],R[2][2]); // Roll
  return euler;
}

void POS::estimate(TObject object, TImage image, TCamera *camera, TCorrespondence *corr) {
	// Reorder the image points based on correspondence
	TImage reordered = image;
	for (uint8_t i = 0; i < 4; i++) {
		reordered.imagePts[corr->indices[i]][0] = image.imagePts[i][0];
		reordered.imagePts[corr->indices[i]][1] = image.imagePts[i][1];
	}
	// Update imageVectors
	for (uint8_t i = 0; i < 4; i++) {
		for(uint8_t j = 0; j < 2; j++){
			reordered.imageVects[i][j]=(double)(reordered.imagePts[i][j]-reordered.imagePts[0][j]);
		}
	}
	estimate(object, reordered, camera);
}

void POS::findBestCorrespondence(TObject object, TImage image, TCamera *camera, TCorrespondence *out_best_corr) {
	/// Check all possible trackpoint combinations with posit to find the one which is closest to the current camera->rotation matrix
	std::vector<size_t> idx = {0,1,2,3};
	TCorrespondence corr;
	TEuler target = eulerOf(camera->rotation);
	TEuler euler;
	double error;
	double min_error = 10000.0;
	do {
		TCamera cam = *camera;
		corr.indices = idx;
		estimate(object, image, &cam, &corr);
		euler = eulerOf(cam.rotation);
		error = fabs(target.pitch-euler.pitch)+fabs(target.roll-euler.roll)+fabs(target.yaw-euler.yaw);
		if (fabs(target.yaw-euler.yaw) > 1.5*M_PI) {
			error -= 2*M_PI; // For yaw's wrap around at 2 Pi
		}
		if (error < min_error) {
			min_error = error;
			out_best_corr->indices = idx;
		}
	} while(std::next_permutation(idx.begin(), idx.end()));
	/*std::cout << "Best point correspondence: ";
	for (const size_t& v : out_best_corr->indices) std::cout << v << " ";
	std::cout << std::endl;*/
}
