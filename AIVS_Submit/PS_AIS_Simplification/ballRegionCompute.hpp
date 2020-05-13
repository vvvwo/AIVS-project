#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/kdtree/kdtree_flann.h>

class Point_Index {
public:
	int parentIndex;
	int index;

	Point_Index(int d, int b) {
		parentIndex = d;
		index = b;
	}
};

class BallRegion {

public:

	std::vector<std::vector<double>> pointCloudData;
	std::vector<int> pointCloudData_boxIndex;//the box index of point cloud
	std::vector<std::vector<double>> pointNormal;//normal vector	
	std::vector<int> pointBorder;//minx,y,x; maxx,y,z
	std::vector<std::vector<int>> squareBoxes; //the point index in different boxes， if no，then to 0。
	std::vector<int> XYZNumber;//achieve the box number in different axes
	std::vector<double> minXYZ;//achieve the min cordinate of XYZ
	double unitSize;//the box scale	
	int pointNumEsti;//points define for radius estimation,8 points should be find at least
	int boxNum;//Voxel biggest number(Totally)	
	pcl::KdTreeFLANN<pcl::PointXYZ> kdtree;//kdtree

public:

	void BallRegion_init(std::vector<std::vector<double>> pointCloudData_input,
		std::vector<std::vector<double>> pointNormal_input, std::vector<int> pointBorder_input) {

		boxNum = BallRegion_EstimateBoxScale(pointCloudData_input.size());
		pointNumEsti = 12;
		pointCloudData = pointCloudData_input;
		pointNormal = pointNormal_input;
		pointBorder = pointBorder_input;

		std::cout << "Init BallRegion" << std::endl;
		BallRegion_AchieveXYZ();//build boxes region
		//std::cout << "BallRegion_BoxInput()" << std::endl;
		BallRegion_BoxInput();//put point cloud into boxRegions
		std::cout << "Init kdtree" << std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
		cloud->width = pointCloudData.size();
		cloud->height = 1;
		cloud->points.resize(cloud->width * cloud->height);
		// fills a PointCloud with random data
		for (int i = 0; i < pointCloudData.size(); i++)
		{
			pcl::PointXYZ pxyz;
			cloud->points[i].x = pointCloudData[i][0];
			cloud->points[i].y = pointCloudData[i][1];
			cloud->points[i].z = pointCloudData[i][2];

		}
		kdtree.setInputCloud(cloud);		
	}
	

private:		

	void BallRegion_BoxInput() {//Input points into the boxes

		//achieve the x, y, z
		for (int i = 0; i < pointCloudData.size(); i++) {
			double x_i = pointCloudData[i][0];
			double y_i = pointCloudData[i][1];
			double z_i = pointCloudData[i][2];
			double xNum = (x_i - minXYZ[0]) / unitSize;
			double yNum = (y_i - minXYZ[1]) / unitSize;
			double zNum = (z_i - minXYZ[2]) / unitSize;
			int xNumIndex = xNum;
			int yNumIndex = yNum;
			int zNumIndex = zNum;
			if (xNumIndex < xNum || xNumIndex == 0) {
				xNumIndex++;
			}
			if (yNumIndex < yNum || yNumIndex == 0) {
				yNumIndex++;
			}
			if (zNumIndex < zNum || zNumIndex == 0) {
				zNumIndex++;
			}

			int index_Num = xNumIndex + XYZNumber[0] * (yNumIndex - 1) + (XYZNumber[0] * XYZNumber[1] * (zNumIndex - 1));

			squareBoxes[index_Num].push_back(i);
			pointCloudData_boxIndex.push_back(index_Num);
		}


		vector<double> p11;
		vector<double> p12;
		int index1;
		for (int i = 0; i < squareBoxes.size(); i++) {
			if (squareBoxes[i].size() >= 2) {
				p11 = pointCloudData[squareBoxes[i][0]];
				p12 = pointCloudData[squareBoxes[i][1]];
				index1 = i;
				break;
			}
		}
		

	}

	void BallRegion_AchieveXYZ() {//build box index regions

		int minX = pointBorder[0];
		int minY = pointBorder[1];
		int minZ = pointBorder[2];
		int maxX = pointBorder[3];
		int maxY = pointBorder[4];
		int maxZ = pointBorder[5];
		double min_X = pointCloudData[minX][0];
		double min_Y = pointCloudData[minY][1];
		double min_Z = pointCloudData[minZ][2];
		double max_X = pointCloudData[maxX][0];
		double max_Y = pointCloudData[maxY][1];
		double max_Z = pointCloudData[maxZ][2];

		if (minXYZ.size() > 0) {
			minXYZ.clear();
		}

		minXYZ.push_back(min_X);
		minXYZ.push_back(min_Y);
		minXYZ.push_back(min_Z);

		double x_dis = abs(max_X - min_X);
		double y_dis = abs(max_Y - min_Y);
		double z_dis = abs(max_Z - min_Z);
		double large_dis = x_dis;
		if (large_dis < y_dis) {
			large_dis = y_dis;
		}
		if (large_dis < z_dis) {
			large_dis = z_dis;
		}
		unitSize = large_dis / double(boxNum);
		double numX = x_dis / unitSize;
		double numY = y_dis / unitSize;
		double numZ = z_dis / unitSize;
		int numXI = numX;
		int numYI = numY;
		int numZI = numZ;
		if (numX > (double)numXI) {
			numXI++;
		}
		if (numY > (double)numYI) {
			numYI++;
		}
		if (numZ > (double)numZI) {
			numZI++;
		}

		if (XYZNumber.size() > 0) {
			XYZNumber.clear();
		}

		XYZNumber.push_back(numXI);
		XYZNumber.push_back(numYI);
		XYZNumber.push_back(numZI);

		//init squareBoxes
		if (squareBoxes.size() == 0) {
			squareBoxes.clear();
		}

		int squareBoxesSize = numXI * numYI * numZI;
		for (int i = 0; i <= squareBoxesSize; i++) {
			std::vector<int> squareBoxesSize_i;
			squareBoxes.push_back(squareBoxesSize_i);
		}
	}

	void BallRegion_AchieveXYZ2(BallRegion br_input) {

		minXYZ = br_input.minXYZ;
		unitSize = br_input.unitSize;
		XYZNumber = br_input.XYZNumber;

		int squareBoxesSize = XYZNumber[0] * XYZNumber[1] * XYZNumber[2];
		for (int i = 0; i <= squareBoxesSize; i++) {
			std::vector<int> squareBoxesSize_i;
			squareBoxes.push_back(squareBoxesSize_i);
		}

	}
	
	int BallRegion_EstimateBoxScale(int pointcloudNum) {

		if (pointcloudNum < 10000) {
			return 10;		
		}
		else if (pointcloudNum < 50000) {
			return 20;		
		}
		else if (pointcloudNum < 100000) {
			return 30;
		}
		else if (pointcloudNum < 500000) {
			return 40;
		}
		else if (pointcloudNum < 1000000) {
			return 50;
		}
		else {
			return (int)pow((double)pointcloudNum / 8.0, 1.0 / 3.0);		
		}	
	}

public:	

	vector<double> BallRegion_ReturnBoxCenter_Radius(int boxIndex) {

		int z_num = boxIndex / (XYZNumber[0] * XYZNumber[1]) + 1;
		int leveZ = boxIndex % (XYZNumber[0] * XYZNumber[1]);
		int y_num = leveZ / XYZNumber[0] + 1;
		int x_num = leveZ % XYZNumber[0];

		if (x_num == 0) {
			x_num = XYZNumber[0];
			y_num = y_num - 1;
		}

		double xcenter = (minXYZ[0] + (x_num - 1) * unitSize + minXYZ[0] + x_num * unitSize) / 2;
		double ycenter = (minXYZ[1] + (y_num - 1) * unitSize + minXYZ[1] + y_num * unitSize) / 2;
		double zcenter = (minXYZ[2] + (z_num - 1) * unitSize + minXYZ[2] + z_num * unitSize) / 2;

		double x1 = minXYZ[0] + (x_num - 1) * unitSize;
		double y1 = minXYZ[1] + y_num * unitSize;
		double z1 = minXYZ[2] + (z_num - 1) * unitSize;

		double radius = sqrt((xcenter - x1) * (xcenter - x1) + (ycenter - y1) * (ycenter - y1) + (zcenter - z1) * (zcenter - z1));
		vector<double> result;

		double xU = minXYZ[0] + x_num * unitSize;
		double xD = minXYZ[0] + (x_num - 1) * unitSize;
		double yU = minXYZ[1] + y_num * unitSize;
		double yD = minXYZ[1] + (y_num - 1) * unitSize;
		double zU = minXYZ[2] + z_num * unitSize;
		double zD = minXYZ[2] + (z_num - 1) * unitSize;

		result.push_back(xU);
		result.push_back(xD);
		result.push_back(yU);
		result.push_back(yD);
		result.push_back(zU);
		result.push_back(zD);

		result.push_back(xcenter);
		result.push_back(ycenter);
		result.push_back(zcenter);
		result.push_back(radius);

		return result;

	}
	
	double disPointPair(std::vector<double> p1, std::vector<double> p2) {

		double dis = (p1[0] - p2[0]) * (p1[0] - p2[0]) + (p1[1] - p2[1]) * (p1[1] - p2[1]) + (p1[2] - p2[2]) * (p1[2] - p2[2]);
		dis = sqrt(dis);
		return dis;

	}

	int indexReturn(double data, vector<double> v) {

		for (int i = 0; i < v.size(); i++) {
			if (data == v[i]) {
				return i;
			}
		}
		return -1;
	}

};


