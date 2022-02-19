#include "icp.h"
#include "iostream"

//#include <conio.h>
#include <fstream>
using namespace std;
using namespace cv;
float R_temp[4]={0,0,0,0};float T_temp[2]={0,0};
fstream erroricp1;
fstream erroricp2;
Mat r_temp=Mat(2,2,CV_32F);
Mat t_temp=Mat(2,1,CV_32F);
float mean_ax, mean_ay, mean_bx, mean_by;
// return the rigid transformation (rotation and tranlation matrix), for more inforation see http://nghiaho.com/?page_id=671
static void getRTMatrixSVD( const CvPoint2D32f* a, const CvPoint2D32f* b, int count ,Mat *r,Mat *t) {
	int i;
	float H[4] = {0.f,0.f,0.f,0.f}; Mat h = Mat(2,2,CV_32F,H);
	float U[4] = {0.f,0.f,0.f,0.f}; Mat u = Mat(2,2,CV_32F,U);
	float W[4] = {0.f,0.f,0.f,0.f}; Mat w = Mat(2,2,CV_32F,W);
	float V[4] = {0.f,0.f,0.f,0.f}; Mat v = Mat(2,2,CV_32F,V);
	CvPoint2D32f mean_a = cvPoint2D32f(0.f,0.f);
	CvPoint2D32f mean_b = cvPoint2D32f(0.f,0.f);
	
	for( i = 0 ; i < count ; i ++) {
		mean_a.x += a[i].x;
		mean_a.y += a[i].y;
		mean_b.x += b[i].x;
		mean_b.y += b[i].y;
	}
	mean_a.x /= (float)count;
	mean_a.y /= (float)count;
	mean_b.x /= (float)count;
	mean_b.y /= (float)count;
	for( i = 0 ; i < count ; i ++) {
		float AX = (a[i].x-mean_a.x);
		float AY = (a[i].y-mean_a.y);
		float BX = (b[i].x-mean_b.x);
		float BY = (b[i].y-mean_b.y);
		H[0] += AX*BX;
		H[1] += AX*BY;
		H[2] += AY*BX;
		H[3] += AY*BY;
	}
	//printf("H : %f %f %f %f\n",H[0],H[1],H[2],H[3]);
	//printf("before V: %f\n",V[3]);

	SVD::compute(h,w,u,v,SVD::MODIFY_A);
	//cvSVD(&h,&w,&u,&v,CV_SVD_MODIFY_A);

	//printf("after V: %f\n",V[3]);
	float mat_temp;
//	mat_temp = H[1];
//	H[1] = H[2];
//	H[2] = mat_temp;
//	mat_temp = U[1];
//	U[1] = U[2];
//	U[2] = mat_temp;
//	mat_temp = W[1];
//	W[1] = W[2];
//	W[2] = mat_temp;
	mat_temp = V[1];
	V[1] = V[2];
	V[2] = mat_temp;
	//printf("H : %f %f %f %f\n",H[0],H[1],H[2],H[3]);
	//printf("U : %f %f %f %f\n",U[0],U[1],U[2],U[3]);
	//printf("W : %f %f %f %f\n",W[0],W[1],W[2],W[3]);
	//printf("V : %f %f %f %f\n",V[0],V[1],V[2],V[3]);

	{// get result :
		Mat j(2,2,CV_32F);
		j = *r;
		//float *R1 = (float)&r->data;

		float R[4];
		float T[2];
		//cout << *r << endl;
		for(int a=0; a<4; a++){
			R[a] = R_temp[a];}
		//R[0] = R_temp[0];R[1] = R_temp[2];R[2] = R_temp[1];R[3] = R_temp[3];
		for(int a=0; a<2; a++){
			T[a] = T_temp[a];}
		//cout << R[0] << endl;
		//for(int i=0; i<4; i++){R[i]=100;T[i]=100;}
		// R = V * transpose(U)
		R[0] = V[0]*U[0] + V[1]*U[1];
		R[1] = V[0]*U[2] + V[1]*U[3];
		R[2] = V[2]*U[0] + V[3]*U[1];
		R[3] = V[2]*U[2] + V[3]*U[3];
		
		// special reflection case
		//printf("R : %f %f %f %f\n",R[0],R[1],R[2],R[3]);
		//cout << determinant(j) << endl;
		if ( determinant(j) < 0 ) {
			R[0] = V[0]*U[0] - V[1]*U[1];
			R[1] = V[0]*U[2] - V[1]*U[3];
			R[2] = V[2]*U[0] - V[3]*U[1];
			R[3] = V[2]*U[2] - V[3]*U[3];
		}
		for(int a=0; a<4; a++){
			R_temp[a] = R[a];
		}

		// T = -R*meanA+meanB
		T[0] = -R[0]*mean_a.x - R[1]*mean_a.y + mean_b.x;
		T[1] = -R[2]*mean_a.x - R[3]*mean_a.y + mean_b.y;
		for(int a=0; a<2; a++){
			T_temp[a] = T[a];
		}
	}
	mean_ax = mean_a.x; mean_ay = mean_a.y; mean_bx = mean_b.x; mean_by = mean_b.y;
	//printf("mean_ax = %f %f %f %F\n",mean_ax, mean_ay, mean_bx, mean_by);
}

/* returns the distance squared between two dims-dimensional double arrays */
static float dist_sq( float *a1, float*a2, int dims ) {
	float dist_sq = 0, diff;
	while( --dims >= 0 ) {
		diff = (a1[dims] - a2[dims]);
		dist_sq += diff*diff;
	}
	return dist_sq;
}

float test(float* R,float* T){
	for(int i=0;i<4;i++){
	R[i]=R_temp[i];
	T[i]=T_temp[i];
	}

	return 0;
}

void cetak_icp(){
	cout<<"ICP connect"<<endl;
	//coba_kdtree();
}

void cetak_kdtree_icp(){
	cetak_kdtree();
	printf("icp kdtree tersambung");
}

float icp(const CvPoint2D32f* new_points,int nb_point_new,
          const CvPoint2D32f* ref_points,int nb_point_ref,
          Mat *r_final1,Mat *t_final1,
		  float *R, float *T,
	      CvTermCriteria criteria) {
	//cout<<*r_final1<<endl;
	int k,i;
	float prev_err = FLT_MAX;
	int scan_size = nb_point_new;
	float err;
	float err_cnt;
	//struct kdtree *ptree = kd_create( 2 );
	CvPoint2D32f * input_correlation_old = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f)*nb_point_new );
	CvPoint2D32f * input_correlation_new = (CvPoint2D32f *)malloc(sizeof(CvPoint2D32f)*nb_point_new );
	//cout<<nb_point_new;

	float R1[4] = {1.f,0.f,0.f,1.f},T1[2] = {0.,0.};
	Mat r_final=Mat(2,2,CV_32F,R1);
	Mat t_final=Mat(2,1,CV_32F,T1);
	//r_final = *r_final1;
	//t_final = *t_final1;

	/*r_final.data[0] = 1.f; r_final.data[1] = 0.f;
	r_final.data[2] = 0.f; r_final.data[3] = 1.f;
	t_final.data[0] = 0.f;
 	t_final.data[1] = 0.f;*/
 	//cout<<r_final<<endl;
	/*for( i = 0; i < nb_point_ref; i++ ) 
		kd_insertf((struct kdtree*) ptree, (float*)&ref_points[i], 0);*/

	for( i = 0; i < nb_point_new; i++ )
		input_correlation_new[i] = new_points[i];
	//cout << input_correlation_new[0].x << endl;
	//cout << input_correlation_old[0].x << endl;
	int lop = 0;
	for ( k = 0 ; k < criteria.max_iter; k++ ) {
		float R[4];
		for(int a=0; a<4; a++){
			R[a] = R_temp[a];}
		float T[2];
		for(int a=0; a<2; a++){
			T[a] = T_temp[a];}
		Mat r = Mat(2,2,CV_32F,R);
		Mat t = Mat(2,1,CV_32F,T);

		err = 0.;
/*
		for( i = 0 ; i < nb_point_new ; i++) {
			struct kdres * presults = kd_nearestf( (struct kdtree *)ptree, (float*)&input_correlation_new[i]);
			//if(i==309)cout << input_correlation_new[i].x << endl;
			kd_res_end( presults );
			kd_res_itemf( presults, (float*)&input_correlation_old[i] );
			//if(i==309)cout << input_correlation_old[i].x << endl;
			err += sqrtf( dist_sq( (float*)&input_correlation_old[i], (float*)&input_correlation_new[i], 2 ) );
			//if(i==309)cout << input_correlation_old[i].x <<" "<< input_correlation_new[i].x << endl;
			if(err<1.5){++err_cnt;}
			kd_res_free( presults );
		}//for i < nb_point_new
		//printf("T : %f \n",T[0]);
		//printf("R : %f %f %f %f\n",R[0],R[1],R[2],R[3]);
		//printf("input1 : %f \n",input_correlation_new[0].x);
	*/

		/////////////////////////////////////////////////////////////////////////////////////////////////////////////
		//alamat ref terdekat dr setiap new
		int jj=0,data=5;
		float centroidx[1085],centroidy[1085];
		printf("scan_size = %d\n",scan_size);
		for(int i=0; i<scan_size/data; i++)
		{
			centroidx[i]=0; centroidy[i]=0;
		}
		int dataa;
		for(int i=0; i<scan_size/data; i++)
		{
			for(int j=jj; j<jj+data; j++)
			{
				centroidx[i] = centroidx[i] + ref_points[j].x;
				centroidy[i] = centroidy[i] + ref_points[j].y;
			}
			centroidx[i]=centroidx[i]/data;
			centroidy[i]=centroidy[i]/data;
			//printf("centroidx[%d] = %f %f\n",i,centroidx[i],centroidy[i]);
			jj=jj+data;
			dataa = jj;
		}
		//printf("dataa = %d\n",dataa);
		//return 0;
		if(scan_size%data!=0)
		{
			for(int i=dataa; i<scan_size; i++)
			{
				centroidx[(scan_size/data)] = centroidx[(scan_size/data)] + ref_points[i].x;
				centroidy[(scan_size/data)] = centroidy[(scan_size/data)] + ref_points[i].y;
			}
			centroidx[(scan_size/data)] = centroidx[(scan_size/data)]/(scan_size%data);
			centroidy[(scan_size/data)] = centroidy[(scan_size/data)]/(scan_size%data);
			//printf("centroid[%d] = %f %f\n",(scan_size/data)/+1, centroidx[(scan_size/data)/+1],centroidy[(scan_size/data)/+1]);
		}
		else
		{
			centroidx[scan_size/data]=99999;
			centroidy[scan_size/data]=99999;
		}
		//TESTING
		//jarak testing-learning
		float jarak[1085];
		for(int l=0; l<scan_size; l++)
		{
			for(int j=0; j<(scan_size/data)+1; j++)
			{
				jarak[j] = sqrt(pow((input_correlation_new[l].x-centroidx[j]),2)+pow((input_correlation_new[l].y-centroidy[j]),2));
				//printf("jarak[%d] = %f\n",j,jarak[j]);
			}
			//return 0;

		//ranking
		float rank[1085];
		for(int i=0; i<(scan_size/data)+1; i++)
		{
			rank[i]=1;
			for(int b=0; b<(scan_size/data)+1; b++)
			{
				if(jarak[i]>jarak[b])
				{
					rank[i]++;
				}
			}
		}

		int datku;
		//printf("Nilai Ujian\t Rangking\n");
		for(int i=0; i<(scan_size/data)+1; i++)
		{
			//printf("%d %f\t\t %f\n",i,jarak[i], rank[i]);
		}
		//printf("\n");
		for(int i=0; i<(scan_size/data)+1; i++)
		{
			if(rank[i]==1)
			{
				//printf("%d %f\t\t %f\n",i,jarak[i], rank[i]);
				datku = i;
				break;
			}
		}

		//jarak data new-ref yang sebenarnya
		float distt[1085];int nearest;
		if(scan_size%data!=0)
		{
			if(datku!=scan_size/data)
			{
				for(int i=datku*data; i<(datku*data)+data; i++)
				{
					distt[i] = sqrt(pow((input_correlation_new[i].x-ref_points[i].x),2)+ pow((input_correlation_new[i].y-ref_points[i].y),2));
				}

				//ranking
				float ranking[1085];
				for(int i=datku*data; i<(datku*data)+data; i++)
				{
					ranking[i]=1;
					for(int b=datku*data; b<(datku*data)+data; b++)
					{
						if(distt[i]>distt[b])
						{
							ranking[i]++;
						}
					}
				}

				//printf("\n");
				for(int i=datku*data; i<(datku*data)+data; i++)
				{
					if(ranking[i]==1)
					{
						//printf("distt[%d][%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
						//printf("distt[%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
						//printf("pq_centered = %f %f %f %f\n",p_centered[1][l],p_centered[2][l],q_centered[1][i],q_centered[2][i]);
						nearest = i;
						break;
					}
				}
			}
			else
			{
				for(int i=scan_size-(scan_size%data); i<scan_size; i++)
				{
					distt[i] = sqrt(pow((input_correlation_new[i].x-ref_points[i].x),2)+ pow((input_correlation_new[i].y-ref_points[i].y),2));						
				}

				//ranking
				float ranking[1085];
				for(int i=datku*data; i<(datku*data)+data; i++)
				{
					ranking[i]=1;
					for(int b=datku*data; b<(datku*data)+data; b++)
					{
						if(distt[i]>distt[b])
						{
							ranking[i]++;
						}
					}
				}

				//printf("\n");
				for(int i=datku*data; i<(datku*data)+data; i++)
				{
					if(ranking[i]==1)
					{
					//printf("distt[%d][%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
					//printf("distt[%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
					//printf("pq_centered = %f %f %f %f\n",p_centered[1][l],p_centered[2][l],q_centered[1][i],q_centered[2][i]);
						nearest = i;
						break;
					}
				}
			}
		}
		else
		{
			for(int i=datku*data; i<(datku*data)+data; i++)
			{
				distt[i] = sqrt(pow((input_correlation_new[i].x-ref_points[i].x),2)+ pow((input_correlation_new[i].y-ref_points[i].y),2));					
			}
			
			//ranking
			float ranking[1085];
			for(int i=datku*data; i<(datku*data)+data; i++)
			{
				ranking[i]=1;
				for(int b=datku*data; b<(datku*data)+data; b++)
				{
					if(distt[i]>distt[b])
					{
						ranking[i]++;
					}
				}
			}

			//printf("\n");
			for(int i=datku*data; i<(datku*data)+data; i++)
			{
				if(ranking[i]==1)
				{
					//printf("distt[%d][%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
					//printf("distt[%d] = %f\t\t %f\n",i,distt[i], ranking[i]);
					//printf("pq_centered = %f %f %f %f\n",p_centered[1][l],p_centered[2][l],q_centered[1][i],q_centered[2][i]);
					nearest = i;
					break;
				}
			}	
		}



		//printf("%d nearest = %d\n",l,nearest);
		//xx_old[1][l]= xx[1][nearest];	yy_old[1][l]=yy[1][nearest];
		input_correlation_old[l].x = ref_points[nearest].x; input_correlation_old[l].y = ref_points[nearest].y;
		//printf("%d %f %f\n",l,xx_old[1][l],yy_old[1][l]);
		}	getRTMatrixSVD(&input_correlation_new[0],&input_correlation_old[0],nb_point_new,&r,&t);
		//printf("input2 : %f \n",input_correlation_new[0].x);
		printf("nb_point_new = %d\n",nb_point_new);
		for(i = 0; i < nb_point_new ; i++ ) {
			float x = input_correlation_new[i].x;
			float y = input_correlation_new[i].y;
			float X = (R_temp[0]*x + R_temp[1]*y + T_temp[0]);
			float Y = (R_temp[2]*x + R_temp[3]*y + T_temp[1]);
			input_correlation_new[i].x = X;
			input_correlation_new[i].y = Y;
		}
		//printf("input2 : %f \n",input_correlation_new[0].x);
		//printf("T : %f \n",T[0]);
		lop = k;
		//if ( fabs(err - prev_err) < criteria.epsilon )  break;
		//else prev_err = err;
		if(err<1000)
		{
			erroricp2.open("/home/devira/catkin_ws5/src/tes_vrep/tes/footlidar_1/erroricp2.txt", ios::in | ios::out |ios::app);
			erroricp2 << err << endl;
			erroricp2.close();		
			break;
		}
		//else if(err>3000)k--;
    }//for k < criteria.max_iter
    getRTMatrixSVD(&new_points[0],&input_correlation_new[0],nb_point_new,&r_final,&t_final);
    for(int i=0;i<4;i++){
		R[i]=R_temp[i];
		T[i]=T_temp[i];
	}
	//kd_free( ptree );
	//free(input_correlation_old);
	//free(input_correlation_new);
	//err_cnt = err/nb_point_new;
	return err;
}

