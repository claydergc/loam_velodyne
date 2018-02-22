// css.cpp
#include "loam_velodyne/css.h"

//#include "matplotlibcpp.h"
//namespace plt = matplotlibcpp;
using namespace std;
using namespace pcl;
using namespace Eigen;

/*bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j)
{
	return (i.sIndex<j.sIndex);
}*/

bool compareCurvatureTriplet (CurvatureTriplet i, CurvatureTriplet j)
{
	return (i.curvature>j.curvature);
}

bool compareKeypoints (Keypoint i, Keypoint j)
{
	return (i.curvature>j.curvature);
}

bool compareCurvatureCounter (CurvatureCounter i, CurvatureCounter j)
{ 
	return (i.counter>j.counter);
}

int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;
	float minDiff = FLOAT_MAX;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<=threshold && diff<minDiff)
		//if(diff<threshold && diff<minDiff)
		{
			idx1 = vec[i].index;
			minDiff = diff;
		}
	}

	return idx1;
}

int findByThresholdAtMinScale(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold)
{
	int idx1 = -1;
	float diff;

	int sumIdx = 0;
	int counter = 0;

	for(int i=0; i<vec.size(); ++i)
	{
		diff = abs(a.sIndex-vec[i].sIndex);
		
		if(diff<0.0000001)
		{
			return vec[i].index;
		}
		//else if(diff<threshold)
		else if(diff<=threshold)
		{
			sumIdx += vec[i].index;
			counter++;
		}
	}

	idx1 = (counter==0)?-1:(sumIdx/counter);
	
	return idx1;
}

/*int findByThreshold(CurvatureTriplet a, vector<CurvatureTriplet> vec, float threshold, float& idx2)
{
	for(int i=0; i<vec.size(); ++i)
		if(abs(a.sIndex-vec[i].sIndex)<threshold)
		{
			idx2 = ;
			return vec[i].index;
		}
	return -1;
}*/

int findCurvatureTriplet (vector<CurvatureTriplet> vec, CurvatureTriplet c)
{
  for(int i=0; i<vec.size(); ++i)
  	if(vec[i].sIndex==c.sIndex)
  		return i;
  return -1;
}

void parametrizeCurve(PointCloud<PointXYZI> in, vector<float> &s)
{
	s = vector<float>(in.size());
	vector<float> d(in.size());

	float Dp = 0.0;
	float sumDistances = 0.0;

	d[0] = 0.0;
	s[0] = 0.0;	

	PointXYZI a, b;

	for(int i=1; i<in.size(); ++i)
	{
		a.x = in[i].x; b.x = in[i-1].x;
		a.y = 0; b.y = 0;
		a.z = in[i].z; b.z = in[i-1].z;
		//d[i] = pcl::euclideanDistance(in[i], in[i-1]);
		d[i] = pcl::euclideanDistance(a, b);
		Dp += d[i];
	}

	for(int i=1; i<in.size(); ++i)
	{
		for(int j=1; j<=i; ++j)
			sumDistances += d[j];

		s[i] = sumDistances/Dp;
		sumDistances = 0.0;
	}
}

void printKeypoints(vector<CurvatureTriplet> keypoints)
{
	for(int k=0; k<keypoints.size(); ++k)
		cout<<keypoints[k].index<<"->"<<keypoints[k].sIndex<<":"<<keypoints[k].curvature<<", ";

	cout<<endl<<endl;
}

void printVector(vector<float> v)
{
	for(int i=0; i<v.size(); ++i)
		cout<<v[i]<<endl;

}

void printArray(float v[], int size)
{
	for(int i=0; i<size; ++i)
		cout<<v[i]<<endl;
}

void getCurvatureExtrema(vector<float> curvature, vector<float> s, vector<CurvatureTriplet>& keypoints, float min, float max, bool isMaxScale)
{
	for(int i=2; i<curvature.size()-2; ++i)
	{
		if( abs(curvature[i])>abs(curvature[i-1]) && abs(curvature[i])>abs(curvature[i+1]) &&
			abs(curvature[i])>abs(curvature[i-2]) && abs(curvature[i])>abs(curvature[i+2])
		  )
		{
			//if(curvature[i]>0 && curvature[i]>(max/4.2) ||
			//   curvature[i]<0 && curvature[i]<(min/4.0)
			//  )
			/*if(curvature[i]>0 && curvature[i]>(max/4.3) ||
			   curvature[i]<0 && curvature[i]<(min/4.5)
			  )*/

			CurvatureTriplet c;
			
			
			if(isMaxScale)
			{
				/*if(curvature[i]>0 && curvature[i]>(max/2.3) ||
				   curvature[i]<0 && curvature[i]<(min/2.3)
				  )*/
				if( (curvature[i]>0 && curvature[i]>(max/2.3) && curvature[i]>abs(min/4.0)) ||
				    (curvature[i]<0 && curvature[i]<(min/2.3) && abs(curvature[i])>(max/4.0))
				  )
				/*if( (curvature[i]>0 && curvature[i]>(max/2.3) && curvature[i]>abs(min/6.0)) ||
				    (curvature[i]<0 && curvature[i]<(min/2.3) && abs(curvature[i])>(max/6.0))
				  )*/
				{
					c.index = i;
					c.sIndex = s[i];
					c.curvature = curvature[i];
					keypoints.push_back(c);
				}
			}
			else
			{
				c.index = i;
				c.sIndex = s[i];
				c.curvature = curvature[i];
				keypoints.push_back(c);
			}

		}
	}
}

int sign(float num)
{
	return (int)((float)(num)/(float)abs(num));
}

void removeConstantCurvature(vector<CurvatureTriplet>& keypoints)
{
	CurvatureTriplet keypointsArray[keypoints.size()];
	CurvatureTriplet keypointTmp;

	//int counter = 0;
	int counter = -1;
	//int i=1;

	//cout<<"hola1"<<endl;

	for(int i=1; i<keypoints.size(); ++i)
	//while(i<keypoints.size())
	{
		if(sign(keypoints[i].curvature)==sign(keypoints[i-1].curvature))
		{
			if( (keypoints[i].index-keypoints[i-1].index)<3)
			{
				keypointTmp=(abs(keypoints[i].curvature)>abs(keypoints[i-1].curvature))?keypoints[i]:keypoints[i-1];
				//keypointsArray[counter] = keypointTmp;
				if(counter==-1)
					keypointsArray[++counter] = keypointTmp;
				else
					keypointsArray[counter] = keypointTmp;
				//counter++;
				//cout<<"keypointTmp: "<<keypointTmp.index<<endl;
			}
			else
			{
				//cout<<"hola1"<<endl;
				if(counter==-1)
				{
					keypointsArray[++counter] = keypoints[i-1];
					keypointsArray[++counter] = keypoints[i];
				}
				else
				{
					keypointsArray[++counter] = keypoints[i];	
				}
				//cout<<"hola2"<<endl;
			}

			
		}
		else if(abs(keypoints[i].curvature-keypoints[i-1].curvature)>35000)
		{
			if(counter==-1)
			{
				keypointsArray[++counter] = keypoints[i-1];
				keypointsArray[++counter] = keypoints[i];
			}
			else
			{
				keypointsArray[++counter] = keypoints[i];
			}
			//i++;
		}
		/*else if( (abs(keypoints[i].curvature-keypoints[i-1].curvature)>35000) && counter!=-1)
		{
			keypointsArray[++counter] = keypoints[i];
		}*/
		//cout<<"counter:"<<counter<<endl;

	}

	//cout<<"hola2"<<endl;
	//cout<<"hola3"<<endl;

	if(keypoints.size()==1)
		keypointsArray[++counter] = keypoints[0];

	if(counter>-1)
		keypoints = vector<CurvatureTriplet>(keypointsArray, keypointsArray + counter + 1 );
}

//void computeCurvature(PointCloud<PointXYZI> in, float sigma, int kernelWidth, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
//void computeCurvature(PointCloud<PointXYZI> in, float sigma, int width, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints)
//void computeCurvature(PointCloud<PointXYZI> in, float sigma, const int width, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0)
void computeCurvature(PointCloud<PointXYZI> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS], float kernelFactor, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0, bool isMaxScale)
//void computeCurvature(PointCloud<PointXYZI> in, float kernelFactor, vector<float>& curvature, vector<float>& s, vector<CurvatureTriplet>& keypoints, vector<float>& gauss, vector<float>& kernel0, bool isMaxScale)
{
	//kernel factor = percentage of the points to define the width and std deviation of gaussian kernel
	int m = in.size();
	int kernelWidth = ((int)((float)(m)*kernelFactor)%2==0)?(m*kernelFactor+1):(m*kernelFactor);
	float sigma;
	
	float sum = 0.0;

	int extension = kernelWidth/2;
	PointCloud<PointXYZI> tmp;
	tmp.width = in.size() + extension*2;
	tmp.height = 1;
	tmp.resize(tmp.width * tmp.height);

	float x0=0;
	float y0=0;
	float x1=0;
	float y1=0;
	float x2=0;
	float y2=0;
	float a,b;

	int i, j, k;
	curvature = vector<float>(in.points.size());

	float sAux = s[0];
	vector<float> sTmp(m+2*extension);

	for(int i=0; i<extension; ++i)
	{
		tmp[i] = PointXYZI(in.points[0]);
		//sAux = sAux - (s[(extension-1)-(i+1)]-s[(extension-1)-i]);
		sAux = sAux - (s[i+1]-s[i]);
		sTmp[extension-1-i] = sAux;

		//yTmp[i] = in.points[0].y;
	}

	for(int i=0; i<in.points.size(); ++i)
	{
		tmp[i+extension] = PointXYZI(in.points[i]);
		sTmp[i+extension] = s[i];

		//yTmp[i+extension] = in.points[i].y;
	}

	sAux = s[m-1];

	//for(int i=in.points.size(); i<in.points.size()+extension; ++i)
	for(int i=0; i<extension; ++i)
	{
		//tmp[i+extension] = PointXYZI(in.points[in.points.size()-1]);
		tmp[i+m+extension] = PointXYZI(in.points[in.points.size()-1]);
		sAux = sAux + (s[m-i-1]-s[m-i-2]);
		//sTmp[m+i] = sAux;
		sTmp[i+m+extension] = sAux;

		//yTmp[i+m+extension] = in.points[in.points.size()-1].y;
	}


	float min = FLOAT_MAX;
	float max = FLOAT_MIN;

	bool containsZeroCurvature = false;
  	
	for(i = 0; i < in.size(); ++i)
	{
		//Convolution
		for(j = 0; j<kernelWidth; ++j)
		{
			//x0 += tmp.points[i+j].x * gaussian0[j];
			//y0 += tmp.points[i+j].y * gaussian0[j];
		    
		    /*x1 += tmp.points[i+j].x * gaussian1[j];
			y1 += tmp.points[i+j].y * gaussian1[j];
			x2 += tmp.points[i+j].x * gaussian2[j];
			y2 += tmp.points[i+j].y * gaussian2[j];*/

			x1 += tmp.points[i+j].x * gaussian1[kernelWidth/2-1][j];
			y1 += tmp.points[i+j].z * gaussian1[kernelWidth/2-1][j];
			x2 += tmp.points[i+j].x * gaussian2[kernelWidth/2-1][j];
			y2 += tmp.points[i+j].z * gaussian2[kernelWidth/2-1][j];
		}

		curvature[i] = (x1*y2-y1*x2);

		x0 = 0.0;
		y0 = 0.0;
		x1 = 0.0;
		y1 = 0.0;
		x2 = 0.0;
		y2 = 0.0;

		if(curvature[i]<min)
			min = curvature[i];
		if(curvature[i]>max)
			max = curvature[i];

		if(isMaxScale && abs(curvature[i])<1e6)
			containsZeroCurvature=true;

	}
	
	if(isMaxScale)
	{
		//if(containsZeroCurvature && (max-min)>7.5e5)
		if(containsZeroCurvature && (max-min)>1.5e5)
			getCurvatureExtrema(curvature, s, keypoints, min, max, true);
	}
	else
		getCurvatureExtrema(curvature, s, keypoints, min, max, false);
}

//void computeScaleSpace(vector<CurvatureTriplet> keypoints[SIGMA_SIZE][WIDTH_SIZE]) //, vector<CurvatureTriplet>& keypointsFinestScale)
void computeScaleSpace(PointCloud<PointXYZI> in, vector<float> gaussian1[NUM_GAUSSIANS], vector<float> gaussian2[NUM_GAUSSIANS], vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s, vector<float>& curvatureMaxScale)
//void computeScaleSpace(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float>& s)
{
	float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8};
	//float scales[NUM_SCALES] = {0.05, 0.15, 0.25, 0.35, 0.45, 0.55, 0.65, 0.75, 0.85};
	//float scales[NUM_SCALES] = {0.05, 0.1, 0.15, 0.2, 0.25, 0.3, 0.35, 0.40, 0.45, 0.50, 0.55, 0.6, 0.65, 0.7, 0.75};

	vector<float> curvature;
	//vector<float> s;
 	
 	parametrizeCurve(in, s);

 	vector<float> gauss;
	vector<float> kernel0;

 	//recorrido normal
 	for(int i=0; i<NUM_SCALES; ++i)
	{
		curvature.clear();

		if(i==0 && in.size()<40)
			continue;

		if(i==NUM_SCALES-1)
		{
			//computeCurvature(in, scales[i], curvature, s, keypoints[i], gauss, kernel0, false); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], gauss, kernel0, true); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
			curvatureMaxScale = curvature;
			removeConstantCurvature(keypoints[i]);
		}
		else
			//computeCurvature(in, scales[i], curvature, s, keypoints[i], gauss, kernel0, true); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
			computeCurvature(in, gaussian1, gaussian2, scales[i], curvature, s, keypoints[i], gauss, kernel0, false); //10% puntos = 7 ptos = distancia de 0.1 en x para la gaussiana y dev standar de 0.015
		
		//printKeypoints(keypoints[i]);

		//if(i==NUM_SCALES-1)
			//removeConstantCurvature(keypoints[i]);

		//if(i==15)
		//{
			//plt::plot(s, curvature);
			//plt::plot(kernel0, gauss);
			//plt::show();
		//}

		//cout<<"*******************************************"<<endl;
	}
}

//void getFinalKeypointsAtMinScale(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float> s, PointCloud<PointXYZI>& keypointsCloud)
//void getFinalKeypointsAtMinScale(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float> s, vector<CurvatureTriplet>& finalKeypoints)
void getFinalKeypointsAtMinScale(PointCloud<PointXYZI> in, vector<CurvatureTriplet> keypoints[NUM_SCALES], vector<float> s, vector<float> curvatureMaxScale, int cluster, vector<Keypoint>& cornerKeypoints, vector<Keypoint>& flatKeypoints, PointCloud<PointXYZI>& downsampledKeypoints)
{
	vector<CurvatureTriplet> keypointsScaleIterator;
	keypointsScaleIterator = keypoints[NUM_SCALES-1];
	
	vector<CurvatureCounter> keypointsFinalCounter(keypointsScaleIterator.size());


	for(int k=0; k<keypointsFinalCounter.size(); ++k)
	{
		CurvatureCounter cc;
		cc.index = keypointsScaleIterator[k].index;
		cc.curvature = keypointsScaleIterator[k].curvature;
		cc.counter = 0;

		keypointsFinalCounter[k] = cc;
	}

	//recorrido para llegar a la posicion del keypoint del finest scale
	int idx;
	//int idxAux = -1;
	float threshold = 0.0;
	
	for(int i=NUM_SCALES-1; i>0; --i)
	{
		for(int j=0; j<keypointsScaleIterator.size(); ++j)
		{
			//idx1 = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], 0.0298, idx2);
			if(keypointsScaleIterator[j].index>1 && keypointsScaleIterator[j].index<in.size()-2)
				threshold = abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2])>abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2])?
							abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index-2]):abs(s[keypointsScaleIterator[j].index]-s[keypointsScaleIterator[j].index+2]);

			if(i==1)
				idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], threshold);
				//idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], 0.0298);
				//idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], 0.02);
				//idx = findByThresholdAtMinScale(keypointsScaleIterator[j], keypoints[i-1], 0.05587);
			else
				//idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], 0.0298);
				idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], threshold);
				//idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], 0.02);
				//idx = findByThreshold(keypointsScaleIterator[j], keypoints[i-1], 0.03);
	
			if(idx!=-1)
			{
				/*keypointsScaleIterator[j].index = keypoints[i-1][idx2].index;
				keypointsScaleIterator[j].sIndex = keypoints[i-1][idx2].sIndex;

				keypointsFinalCounter[j].index = keypoints[i-1][idx2].index;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;*/

				keypointsScaleIterator[j].index = idx;
				keypointsScaleIterator[j].sIndex = s[idx];

				keypointsFinalCounter[j].index = idx;
				keypointsFinalCounter[j].counter = keypointsFinalCounter[j].counter + 1;
				
			}
			
			//cout<<keypointsFinalCounter[j].index<<"->"<<keypointsFinalCounter[j].counter<<", ";	
			//cout<<keypointsScaleIterator[j].index<<"->"<<keypointsScaleIterator[j].sIndex<<", ";
		}

		//cout<<endl;
	}

	cornerKeypoints.clear();
	vector<int> keypointsIndices;

	//int diffSeparation = 500;

	for(int i=0; i<keypointsFinalCounter.size(); ++i)
	{
		//if(keypointsFinalCounter[i].counter>10)
		//if(keypointsFinalCounter[i].counter>13)
		//if(keypointsFinalCounter[i].counter>10)
		//if(keypointsFinalCounter[i].counter>7)
		if(keypointsFinalCounter[i].counter>13)
		//if(keypointsFinalCounter[i].counter>10)
		{
			if(i==0)
			{
				Keypoint kp;
				kp.keypoint = in[keypointsFinalCounter[i].index];
				kp.index = keypointsFinalCounter[i].index;
				kp.curvature = abs(keypointsScaleIterator[i].curvature);
				kp.cluster = cluster;
				cornerKeypoints.push_back(kp);
				keypointsIndices.push_back(keypointsFinalCounter[i].index);
			}
			else if(i>0 && keypointsFinalCounter[i].index!=keypointsFinalCounter[i-1].index)
			{

				//if(keypointsFinalCounter[i].index-keypointsFinalCounter[i-1].index>5)
				//if(keypointsFinalCounter[i].index-keypointsFinalCounter[i-1].index>7)
				if(keypointsFinalCounter[i].index-keypointsFinalCounter[i-1].index>3)
				{	
					Keypoint kp;
					kp.keypoint = in[keypointsFinalCounter[i].index];
					kp.index = keypointsFinalCounter[i].index;
					kp.curvature = abs(keypointsScaleIterator[i].curvature);
					kp.cluster = cluster;
					cornerKeypoints.push_back(kp);
					keypointsIndices.push_back(keypointsFinalCounter[i].index);
				}
			}

			/*if(i>0)
			{
				if(keypointsFinalCounter[i].index-keypointsFinalCounter[i-1].index<3)
				{
					cout<<"ALERTAAAAAAAAAAAAAAAAAAAA!!!"<<endl;
					cout<<keypointsFinalCounter[i].index<<","<<keypointsFinalCounter[i-1].index<<endl;
					
					std::stringstream ss2;
          			ss2 << "/home/claydergc/catkin_ws/cluster_errors/object_"<<counter<<".pcd";
          			pcl::io::savePCDFile< pcl::PointXYZI >(ss2.str(), in);
          			counter++;
				}
			}*/
		}
			//keypointsCloud.push_back(in[keypointsFinalCounter[j].index]);


	}

	/*for(int i=0; i<cornerKeypoints.size(); ++i)
	{
		if(i>0)
		{
			if(cornerKeypoints[i].index-cornerKeypoints[i-1].index<3)
			{
				cout<<"ALERTAAAAAAAAAAAAAAAAAAAA!!!"<<endl;
				cout<<cornerKeypoints[i].index<<","<<cornerKeypoints[i-1].index<<endl;
			}
		}
	}*/


	
	//cout<<"Hey1"<<endl;


	/*flatKeypoints.clear();
	downsampledKeypoints.clear();
	//Keypoints that are not corners
	PointCloud<PointXYZI> downsampledCloud;
	PointCloud<PointXYZI> downsampledCloudTmp;
	int j=0;

	//cout<<"kp idx size:"<<keypointsIndices.size()<<endl;
	//cout<<"in size:"<<in.size()<<endl;

	//for(int i=0; i<keypointsIndices.size(); ++i)
		//cout<<"idx: "<<keypointsIndices[i]<<endl;

	if(keypointsIndices.size()==0)
	{
		downsampledCloudTmp = in;
		getDownSampledPoints(downsampledCloudTmp, downsampledCloud, 7, 0.85);
		downsampledKeypoints += downsampledCloud;
		downsampledCloud.clear();
		downsampledCloudTmp.clear();
	}
	else
	{
		for(int i=0; i<in.size(); ++i)
		{
			if(i!=keypointsIndices[j])
			{
				downsampledCloudTmp.push_back(in[i]);
				Keypoint kp;
				kp.keypoint = in[i];
				kp.index = i;
				kp.curvature = abs(curvatureMaxScale[i]);
				kp.cluster = cluster;
				flatKeypoints.push_back(kp);
				//cout<<"Hey1.1"<<endl;
			}
			else if(i==keypointsIndices[j] || i==in.size()-1)
			{
				//cout<<"Hey1.2"<<endl;
				getDownSampledPoints(downsampledCloudTmp, downsampledCloud, 7, 0.85);
				downsampledKeypoints += downsampledCloud;
				j++;
				downsampledCloud.clear();
				downsampledCloudTmp.clear();
				//cout<<"Hey1.3"<<endl;
			}
		}
	}*/


	//cout<<"Hey2"<<endl;

	/*for(int j=0; j<keypointsFinalCounter.size(); ++j)
	{
		if(keypointsFinalCounter[j].counter>10)
			finalKeypoints.push_back(keypointsScaleIterator[j]);
			//keypointsCloud.push_back(in[keypointsFinalCounter[j].index]);
	}*/

	/*keypointsCloud.clear();

	for(int j=0; j<keypointsFinalCounter.size(); ++j)
	{
		if(keypointsFinalCounter[j].counter>10)
			keypointsCloud.push_back(in[keypointsFinalCounter[j].index]);
	}*/
}