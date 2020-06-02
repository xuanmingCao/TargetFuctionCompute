#define _CRT_SECURE_NO_WARNINGS
#define _SCL_SECURE_NO_WARNINGS
#include<iostream>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/registration/icp.h> 
#include <pcl/registration/transformation_estimation_point_to_plane_lls_weighted.h>
#include <pcl/registration/transformation_estimation_point_to_plane_weighted.h>
#include <math.h>
#include <algorithm>

using namespace std;

//目标函数第一项correspondence，返回对应的坐标序号
bool CorrespondenceFunc1(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &src, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &tgt, int K, vector<int>&pair_src, vector<int>&pair_tgt)
{
	pcl::KdTreeFLANN<pcl::PointNormal> tree;
	tree.setInputCloud(tgt);
	pcl::PointNormal searchPoint;
	vector<int> pointIdxNKNSearch;
	vector<float> pointNKNSquaredDistance;
	for (size_t i = 0; i < src->points.size(); i++)
	{
		searchPoint = src->points[i];
		if (tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			//获取对应点的indice
			pair_src.push_back(i);
			pair_tgt.push_back(pointIdxNKNSearch[0]);
		}
	}
	return true;
}

//迭代最近点找 correspondence，返回对应的坐标点
bool CorrespondenceFunc1_returnP(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &src, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &tgt, int K, pcl::PointCloud<pcl::PointNormal>::Ptr &corres_src, pcl::PointCloud<pcl::PointNormal>::Ptr &corres_tgt,int &NUM)
{
	pcl::KdTreeFLANN<pcl::PointNormal> tree;
	tree.setInputCloud(tgt);
	pcl::PointNormal searchPoint;
	vector<int> pointIdxNKNSearch;
	vector<float> pointNKNSquaredDistance;
	for (size_t i = 0; i < src->points.size(); i++)
	{
		searchPoint = src->points[i];
		if (tree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			//获取对应点的indice
			corres_src->points.push_back(src->points[i]);
			corres_tgt->points.push_back(tgt->points[pointIdxNKNSearch[0]]);
			NUM++;
		}
	}
	return true;
}

double getRMS(const pcl::PointCloud<pcl::PointNormal>::ConstPtr &src, const pcl::PointCloud<pcl::PointNormal>::ConstPtr &tgt)
{
	pcl::KdTreeFLANN<pcl::PointNormal> tree;
	tree.setInputCloud(tgt);
	pcl::PointNormal searchPoint;
	vector<int> pointIdxNKNSearch;
	vector<float> pointNKNSquaredDistance;
	double RMS=0,rms=0;
	for (size_t i = 0; i < src->points.size(); i++)
	{
		searchPoint = src->points[i];
		if (tree.nearestKSearch(searchPoint, 1, pointIdxNKNSearch, pointNKNSquaredDistance) > 0)
		{
			rms = rms + pointNKNSquaredDistance[0];
		}
	}
	RMS = sqrt(rms);
	return RMS;
}


//目标函数第二项correspondence

//目标函数第二项correspondence


//将三个对应关系合成一个向量
//构建与对应关系向量相同维数的权重向量

//利用pcl中weight icp求解变换矩阵T（循环改变对应关系，更新变换矩阵T）

void main()
{
	double RMS, rms1, rms2, rms3;
	int iter_num = 0, k=0, m=0, n=0;
	double w1 = sqrt(0.5), w2 = sqrt(0.5), w3 = 0.0;
	//double w1 = 0.2, w2 = 0.8, w3 = 0.0;
	vector<double>R;

	pcl::PointCloud<pcl::PointNormal>::Ptr source(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr source1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target1(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr source2(new pcl::PointCloud<pcl::PointNormal>);
	pcl::PointCloud<pcl::PointNormal>::Ptr target2(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::PointCloud<pcl::PointNormal>::Ptr source3(new pcl::PointCloud<pcl::PointNormal>);
	//pcl::PointCloud<pcl::PointNormal>::Ptr target3(new pcl::PointCloud<pcl::PointNormal>);

	//pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test3\501_circle.ply)", *source);
	//pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test3\501_circle_1.ply)", *target);
	pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test4\sub_circle.ply)", *source1);
	pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test4\sub_circle_1.ply)", *target1);
	pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test4\sub_remain.ply)", *source2);
	pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\ICP_weight\data\test4\sub_remain_1.ply)", *target2);
	//pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\Coarse_registration\data\102-sample.ply)", *source3);
	//pcl::io::loadPLYFile<pcl::PointNormal>(R"(F:\CXM\Coarse_registration\data\103-sample.ply)", *target3);

	pcl::registration::TransformationEstimation<pcl::PointNormal, pcl::PointNormal, float>::Matrix4 matrix_temp;//每次迭代变换矩阵Ti
	pcl::registration::TransformationEstimation<pcl::PointNormal, pcl::PointNormal, float>::Matrix4 matrix_T;//最终变换矩阵T
	
	matrix_T.setIdentity();
	matrix_temp.setZero();

	//==============================只有一个误差项LM优化结果===========================//
	//while (1)//迭代停止条件还需考虑？
	//{
	//	iter_num++;
	//	RMS = 0;
	//	vector<int>pair_src_1, pair_tgt_1, pair_src_2, pair_tgt_2, pair_src_3, pair_tgt_3;

	//	//不断更新correspondence
	//	CorrespondenceFunc1(source, target, 1, pair_src_1, pair_tgt_1);

	//	//利用pcl中weight icp求解变换矩阵T
	//	pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointNormal, pcl::PointNormal> PointToPlane;
	//	PointToPlane.setWeights(weights);
	//	PointToPlane.setUseCorrespondenceWeights(true);
	//	PointToPlane.estimateRigidTransformation(*source, pair_src_1, *target, pair_tgt_1, matrix_temp);
	//	pcl::transformPointCloud(*source, *source, matrix_temp);//source的点数在增加？已解决，pair_src_1设为了全局变量，导致一直在累加
	//	//计算RMS
	//	for (size_t i = 0; i < pair_src_1.size(); i++)
	//	{
	//		double rms_x, rms_y, rms_z, rms;
	//		rms_x = pow((source->points[pair_src_1[i]].x - target->points[pair_tgt_1[i]].x), 2);
	//		rms_y = pow((source->points[pair_src_1[i]].y - target->points[pair_tgt_1[i]].y), 2);
	//		rms_z = pow((source->points[pair_src_1[i]].z - target->points[pair_tgt_1[i]].z), 2);
	//		rms = rms_x + rms_y + rms_z;
	//		RMS = RMS + rms;
	//	}
	//	RMS = sqrt(RMS / pair_src_1.size());
	//	R.push_back(RMS);

	//	matrix_T = matrix_T*matrix_temp;
	//	cout << "iter_num：" << iter_num << "   RMS:" << RMS << endl;

	//	//收敛条件
	//	//if (iter_num > 15) break;
	//	//if (RMS < 0.001)break;
	//	if (iter_num > 1 && (R[iter_num - 2]==R[iter_num - 1])) break;
	//}
	//==============================只有一个误差项LM优化结果===========================//


	//==============================多个误差项LM优化结果============================//
	while (1)//迭代停止条件还需考虑？
	{
		iter_num++;//迭代次数
		RMS = 0;
		int k = 0, m = 0, n= 0;
		pcl::PointCloud<pcl::PointNormal>::Ptr corres_src(new pcl::PointCloud<pcl::PointNormal>);
		pcl::PointCloud<pcl::PointNormal>::Ptr corres_tgt(new pcl::PointCloud<pcl::PointNormal>);

		//不断更新correspondence
		CorrespondenceFunc1_returnP(source1, target1, 1, corres_src, corres_tgt, k);
		CorrespondenceFunc1_returnP(source2, target2, 1, corres_src, corres_tgt, m);
		//CorrespondenceFunc1_returnP(source3, target3, 1, corres_src, corres_tgt,n);

		vector<double>weights;
		//weight赋值
		for (int i = 0; i < k; i++) { weights.push_back(w1); }
		for (int i = 0; i < m; i++) { weights.push_back(w2); }
		//for (int i = 0; i < n; i++) { weights.push_back(w3); }

		//利用pcl中weight icp求解变换矩阵T
		pcl::registration::TransformationEstimationPointToPlaneWeighted<pcl::PointNormal, pcl::PointNormal> PointToPlane;
		PointToPlane.setWeights(weights);
		PointToPlane.setUseCorrespondenceWeights(true);
		PointToPlane.estimateRigidTransformation(*corres_src, *corres_tgt, matrix_temp);
		pcl::transformPointCloud(*source1, *source1, matrix_temp);//source的点数在增加？已解决，pair_src_1设为了全局变量，导致一直在累加
		pcl::transformPointCloud(*source2, *source2, matrix_temp);
		//pcl::transformPointCloud(*source3, *source3, matrix_temp);
		//pcl::transformPointCloud(*source, *source, matrix_temp);

		//变换矩阵T
		matrix_T = matrix_temp*matrix_T;
		
	    // 计算原始点云RMS
	 	/*for (size_t i = 0; i < min(source1->points.size(),target1->points.size()); i++)
		{
			double rms_x, rms_y, rms_z, rms;
			rms_x = pow((source1->points[i].x - target1->points[i].x), 2);
			rms_y = pow((source1->points[i].y - target1->points[i].y), 2);
			rms_z = pow((source1->points[i].z - target1->points[i].z), 2);
			rms = rms_x + rms_y + rms_z;
			RMS = RMS + rms;
		}
		RMS = sqrt(RMS / min(source1->points.size(), target1->points.size()));
		R.push_back(RMS);*/
		/*RMS = getRMS(source1, target1);
		R.push_back(RMS);*/

		//计算对应点RMS
		for (size_t i = 0; i < corres_src->points.size(); i++)
		{
			double rms_x, rms_y, rms_z, rms;
			rms_x = pow((corres_src->points[i].x - corres_tgt->points[i].x), 2);
			rms_y = pow((corres_src->points[i].y - corres_tgt->points[i].y), 2);
			rms_z = pow((corres_src->points[i].z - corres_tgt->points[i].z), 2);
			rms = rms_x + rms_y + rms_z;
			RMS = RMS + rms;
		}
		RMS = sqrt(RMS / corres_src->points.size());
		R.push_back(RMS);

		//输出迭代次数与RMS
		cout << "iter_num：" << iter_num << "   RMS:" << RMS << endl;

		//收敛条件
		//if (iter_num > 3) break;
		//if (RMS < 0.001)break;
		if (iter_num > 1 && (R[iter_num - 2] == R[iter_num - 1])) break;
	}
	//==============================多个误差项LM优化结果============================//

	cout << "输出变换矩阵： " << endl;
	cout << matrix_T << endl;
	system("pause");
}