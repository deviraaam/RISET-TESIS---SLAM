#ifndef __icp_h__
#define __icp_h__

#include <opencv/cxcore.h>
#include <opencv/cv.h>
#include <opencv/cv.hpp>
#include <opencv/ml.h>
#include <opencv2/imgcodecs/imgcodecs_c.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/core/core_c.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui_c.h>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/ml/ml.hpp>

#include <kdtree.h>


#ifdef __cplusplus
extern "C" {
#endif

using namespace cv;
    
// Compute icp with 2 set points (between ref points and new points)
// inputs :
//   - new_points   : set of new input points (source point clouds)
//   - nb_point_new : number of input new points
//   - ref_points   : set of reference input points.
//   - nb_point_ref : number of input old points
//   - criteria     : default is cvTermCriteria( CV_TERMCRIT_ITER, nb_max_iteration, stop_epsilon )
//   - image        : optional (for draw nearest points )
// output 2 matrix :
//   - r_out : rotation matrix, 2x2 with type = CV_32F
//   - t_out : translation maxtrix 2x1 with type = CV_32F
// return :
//   - err : cumulative sum of distance points
float icp(const CvPoint2D32f* new_points,int nb_point_new,
          const CvPoint2D32f* ref_points,int nb_point_ref,
          Mat *r_out,Mat *t_out,
		  float* R, float* T,
		  CvTermCriteria criteria) ;

float icp_ori(const CvPoint2D32f* new_points,int nb_point_new,
          const CvPoint2D32f* ref_points,int nb_point_ref,
          CvMat * r_final,CvMat *t_final,
	      CvTermCriteria criteria,IplImage * image);

float icp_vrep(const CvPoint2D32f* new_points,int nb_point_new,
          const CvPoint2D32f* ref_points,int nb_point_ref,
          Mat *r_final1,Mat *t_final1,
		  float *R, float *T,
	      CvTermCriteria criteria,Mat image);

float icp_pl(const CvPoint2D32f* new_points,int nb_point_new,
          const CvPoint2D32f* ref_points,int nb_point_ref,
          Mat * r_final,Mat *t_final,
          float *R, float *T,
	      CvTermCriteria criteria);


float test(float* R,float* T);
void cetak_icp();
void cetak_kdtree_icp();
float err_odom(const CvPoint2D32f* new_points,int nb_point_new,
  	const CvPoint2D32f* ref_points,int nb_point_ref,
	CvTermCriteria criteria) ;

#ifdef __cplusplus
}
#endif
#endif //__icp_h__
