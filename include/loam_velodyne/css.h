// util.h
#ifndef LOAM_CSS_H
#define LOAM_CSS_H

#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

#include <pcl/common/common_headers.h>
#include <pcl/common/distances.h>
#include <pcl/common/centroid.h>
#include <pcl/io/pcd_io.h>

//#include "util.h"


//#define SIGMA_SIZE 7
//#define WIDTH_SIZE 7
//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;

static int counter = 0;

const float FLOAT_MIN = std::numeric_limits<float>::min();
const float FLOAT_MAX = std::numeric_limits<float>::max();
const int NUM_SCALES = 16;
const int NUM_GAUSSIANS = 160;
//const int NUM_SCALES = 15;
//const int NUM_SCALES = 9;

typedef struct curvatureTriplet
{
	int index;
	float sIndex;
	float curvature;
}CurvatureTriplet;

typedef struct keypoint
{
	pcl::PointXYZI keypoint;
	int index;
	float curvature;
	int cluster;
}Keypoint;

bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j);

typedef struct curvatureCounter
{
	float index;
	int counter;
	float curvature;
}CurvatureCounter;

struct CompareSet {
  bool operator() (const float& lhs, const float& rhs) const
  {
  	return lhs<rhs;
  }
};

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j);
bool compareKeypoints (Keypoint i, Keypoint j);

void printKeypoints(std::vector<CurvatureTriplet> keypoints);
int findByThresholdAtMinScale(CurvatureTriplet a, std::vector<CurvatureTriplet> vec, float threshold);
int findByThreshold(CurvatureTriplet a, std::vector<CurvatureTriplet> vec, float threshold);
int findCurvatureTriplet (std::vector<CurvatureTriplet> vec, CurvatureTriplet c);

void removeConstantCurvature(std::vector<CurvatureTriplet>& keypoints);//revisar

void parametrizeCurve(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> &s);
void getCurvatureExtrema(std::vector<float> curvature, std::vector<float> s, std::vector<CurvatureTriplet>& keypoints, 
						 float min, float max, bool isMaxScale);

void computeCurvature(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> gaussian1[NUM_GAUSSIANS], std::vector<float> gaussian2[NUM_GAUSSIANS],
					  float kernelFactor, std::vector<float>& curvature, std::vector<float>& s, std::vector<CurvatureTriplet>& keypoints, std::vector<float>& gauss,
					  std::vector<float>& kernel0, bool isMaxScale);

void computeScaleSpace(pcl::PointCloud<pcl::PointXYZI> in, std::vector<float> gaussian1[NUM_GAUSSIANS], std::vector<float> gaussian2[NUM_GAUSSIANS], 
					  std::vector<CurvatureTriplet> keypoints[NUM_SCALES], std::vector<float>& s, std::vector<float>& curvatureMaxScale);


void getFinalKeypointsAtMinScale(pcl::PointCloud<pcl::PointXYZI> in, std::vector<CurvatureTriplet> keypoints[NUM_SCALES], std::vector<float> s,
								 std::vector<float> curvatureMaxScale, int cluster, std::vector<Keypoint>& cornerKeypoints, std::vector<Keypoint>& flatKeypoints,
								 pcl::PointCloud<pcl::PointXYZI>& downsampledKeypoints);



#endif
