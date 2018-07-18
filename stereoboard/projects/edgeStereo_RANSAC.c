/**
 * Color.c
 * @author: Kirk Scheper
 * @details
 *
 * - Sends color images over serial
 * - Can optionally filter image before sending
 *
 */

#include "edgeFS.h"
#include "edgeflow.h"
#include "main_parameters.h"
#include "main.h"
#include "dcmi.h"
#include "image.h"
#include "raw_digital_video_stream.h"
#include "ransac.h"

struct edgeflow_t edgeflow;
/* compute the geometric residuals corresponding to pose as the (squared) distance between actual and predicted point */
static void poseRTResidualsGeom(double rt[NUM_RTPARAMS], int numres, void *adata, double *resid)
{
register int i;
double P[NUM_PPARAMS], X, Y, Z, s, ppt[2];
struct RTdata *dat=(struct RTdata *)adata;
double (*pts2D)[2]=dat->pts2D, (*pts3D)[3]=dat->pts3D;

  /* P=K[R t] */
  posest_PfromKRt(P, dat->K, rt);

  for(i=0; i<numres; ++i){
    /* project 3D point */
    X=pts3D[i][0]; Y=pts3D[i][1]; Z=pts3D[i][2];
     s=1.0/(P[8]*X + P[9]*Y + P[10]*Z + P[11]);
    ppt[0]=(P[0]*X + P[1]*Y + P[2]*Z  + P[3])*s;
    ppt[1]=(P[4]*X + P[5]*Y + P[6]*Z  + P[7])*s;

    ppt[0]-=pts2D[i][0]; ppt[1]-=pts2D[i][1];
    resid[i]=(SQR(ppt[0]) + SQR(ppt[1]));
  }
}


void init_project(void)
{
	//edgeflow_init(IMAGE_WIDTH, IMAGE_HEIGHT, USE_MONOCAM, &cam_state);
}

void run_project(void)
{
    //edgeflow_total((uint8_t *)current_image_pair.buf, current_image_pair.pprz_ts);
    //send_edgeflow();
	float X[128];
	float Y[128];

	int i;
	for(i=0;i++;i<128)
	{
		X = (float)i;
		Y = (float)2*i;

	}

	  int nbData = 128;       /* I: the number of Data, which is ordered from 0 */
	  int sizeSet=4;      /* I: size of each randomly selected subset of data */
	  int **sets;       /* I: randomly selected subsets, can be set to NULL forcing a default routine to be used for generating subsets */
	  int nbSets = 0;       /* I: the number of subsets, set to 0 if you have no idea */
	  int (*estimator)( /* I: function which estimate the parameters returning the number of solutions */
	    double *x,      /* O: estimated parameter vector */
	    int nb,         /* I: number of data to be used */
	    int *indexes,   /* I: indexes of data to be used */
	    void *adata),   /* I: additional data */
	  int isResidualSqr,/* I: set 1 if `residual' computes the squared residuals */
	  int verbose,      /* I: verbose mode */
	  int maxNbSol,     /* I: maximum number of solutions given by "estimator" */
	  double consensusThresh, /* I: threshold for outlier detection. squared internally if 'isResidualSqr' is set */
	  int dim,          /* I: dimension of the parameter vector */
	  double percentageOfGoodData,  /* I: the percentage (between epsilon and 1.0) of good data. often 0.5 */
	  int blocksz,      /* I: block size for preemption. a non-positive value indicates the default; lower implies heavier preemption */
	  void *adata,      /* I: pointer to additional data, passed uninterpreted to residual() and estimator() */
	  double *estimate, /* O: the corresponding estimate of parameters */
	  int *outliersMap, /* O: contains 1 in indexes of detected outliers, 0 in indexes of inliners */
	  int *nbOutliers   /* O: the number of detected outliers */

    ransacfit(nbData,sizeSet,  **sets, nbSets,
    		poseRTResidualsGeom,
                      int (*estimator)(double *x, int nb, int *indexes, void *adata),
                      int isResidualSqr, int verbose, int maxNbSol, double consensusThresh,
                      int prematureConsensus, int dim, double percentageOfGoodData, void *adata,
                      double *estimate, int *bestSet, int *outliersMap, int *nbOutliers);
}
