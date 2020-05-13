#pragma once
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h> 
#include "GLM/glm.h"
#include "GLM/glmVector.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <math.h> 
#include <time.h>
#include "ballRegionCompute.hpp"
#include "normalCompute.hpp"
#include "fileProcess\writePointSimpli.hpp"

class simplification_Method_AIS_new {//Laplaic graphic

public:

	vector<bool> labelG;
	BallRegion br;	
	vector<vector<double>> pointsNew;

private:

	int iter = 10;
	

public:

	void simplification_Method_AIS_init(BallRegion br_i) {
		br = br_i;
		
	}
	
	vector<vector<double>> simplification_Method_AIS_Sim(int pointNum) {		

		vector<bool> labelG_init(br.pointCloudData.size(), true);
		labelG = labelG_init;

		//vector<int> centerIndex;
		//achieve the simplification number sum
		int numberSum = 0;
		//achieve the number of block with max number
		int numberMax = 0;
		//achieve the number of block with min number
		int numberMin = 99999;

		double rate = (double)pointNum / (double)br.pointCloudData.size();
		//achieve the points set of block
		vector<vector<int>> blockIndex(br.squareBoxes.size());
		for (int i = 0; i < br.squareBoxes.size(); i++) {

			vector<int> block_i;
			vector<int> squareBoxes_i = br.squareBoxes[i];
			double numberT = (double)squareBoxes_i.size() * rate;
			int number = numberT;
			if (numberT - (double)number > 0.2) {
				number = number + 1;
			}
			// if block is empty, then put the empty vector into the structure. 
			if (squareBoxes_i.size() == 0 || number == 0 || numberT < 1) {
				blockIndex[i] = block_i;
			}
			// if block is not empty, then FPS searching 
			else {
				if ((br.squareBoxes.size() - i) % 100 == 0) {
					cout << "block:" << br.squareBoxes.size() - i << endl;
				}

				//compute centre
				double sumX = 0;
				double sumY = 0;
				double sumZ = 0;
				for (int j = 0; j < squareBoxes_i.size(); j++) {
					sumX = sumX + br.pointCloudData[squareBoxes_i[j]][0];
					sumY = sumY + br.pointCloudData[squareBoxes_i[j]][1];
					sumZ = sumZ + br.pointCloudData[squareBoxes_i[j]][2];
				}
				sumX = sumX / squareBoxes_i.size();
				sumY = sumY / squareBoxes_i.size();
				sumZ = sumZ / squareBoxes_i.size();				

				//achieve box infor
				vector<double> boxInfor = br.BallRegion_ReturnBoxCenter_Radius(i);
				double xu = boxInfor[0]; double xd = boxInfor[1];
				double yu = boxInfor[2]; double yd = boxInfor[3];
				double zu = boxInfor[4]; double zd = boxInfor[5];
				double xc = boxInfor[6]; double yc = boxInfor[7]; double zc = boxInfor[8];
				double ra = boxInfor[9]; double ral = boxInfor[9] * 1.2;
				double rb = (xu - xd) / 8;

				int K = squareBoxes_i.size() * 8;
				std::vector<int> pointIdxNKNSearch(K);
				std::vector<float> pointNKNSquaredDistance(K);
				vector<int> label;
				vector<int> plabel;
				pcl::PointXYZ searchPoint;
				searchPoint.x = sumX;
				searchPoint.y = sumY;
				searchPoint.z = sumZ;
				br.kdtree.nearestKSearch(searchPoint, K, pointIdxNKNSearch, pointNKNSquaredDistance);

				vector<int> block_store;
				//centerIndex.push_back(pointIdxNKNSearch[0]);

				bool judge = false;
				for (int j = 0; j < squareBoxes_i.size(); j++) {
					int indexj = squareBoxes_i[j];
					if (labelG[indexj]) {						
						label.push_back(1);
						plabel.push_back(indexj);						
					}
					else {
						label.push_back(0);
						plabel.push_back(indexj);
						block_store.push_back(indexj);
						number--;
					}
				}
				
				//bool judge = false;//judge if there have fake point
				for (int j = 0; j < pointIdxNKNSearch.size(); j++) {

					if (sqrt(pointNKNSquaredDistance[j]) <= ral) {
						int indexj = pointIdxNKNSearch[j];
						double xj = br.pointCloudData[indexj][0];
						double yj = br.pointCloudData[indexj][1];
						double zj = br.pointCloudData[indexj][2];
						if (xj <= xu && xj >= xd && yj <= yu && yj >= yd && zj <= zu && zj >= zd) {
							continue;
						}
						else if (xj <= xu + rb && xj >= xd - rb && yj <= yu + rb && yj >= yd - rb && zj <= zu + rb && zj >= zd - rb) {
							if (!labelG[indexj]) {
								label.push_back(2);
								plabel.push_back(indexj);
								judge = true;
							}
						}
					}
					else {
						break;
					}
				}
				
				if (!judge) {
					if (label.size() > 0) {
						label[0] = 0;
					}
					else {
						cout << "Error! label is empty!";
					}
				}

				if (number > 0) {
					block_i = AIS_Voronoi_new(number, plabel, label);
				}
				
				block_i.insert(block_i.end(), block_store.begin(), block_store.end());

				if (block_i.size() > numberMax) {
					numberMax = block_i.size();
				}
				if (block_i.size() < numberMin && block_i.size() > 0) {
					numberMin = block_i.size();
				}

				blockIndex[i] = block_i;
				numberSum = numberSum + block_i.size();
			}
		}		
		int numberMiddle = (numberMax - numberMin) / 3;
		int dTiff = numberSum - pointNum;
		
		vector<vector<double>> finalResult;

		if (dTiff <= 0) {
			for (int i = 0; i < blockIndex.size(); i++) {
				for (int j = 0; j < blockIndex[i].size(); j++) {
					vector<double> p = br.pointCloudData[blockIndex[i][j]];
					finalResult.push_back(p);
				}			
			}
			labelG.clear();
			return finalResult;
		}

		else {
			int iii = 0;
			vector<int> blockIndexNum(blockIndex.size());
			for (int i = 0; i < blockIndex.size(); i++) {
				blockIndexNum[i] = blockIndex[i].size();
			}
			bool singleJudge = false;
			while (1) {
				if (dTiff == 0) {
					break;
				}
				if (iii == blockIndex.size()) {
					iii = 0;
					singleJudge = !singleJudge;
				}
				if (!singleJudge) {
					if (blockIndexNum[iii] > numberMiddle&& iii % 2 == 0) {
						blockIndexNum[iii] --;
						dTiff--;
					}
				}
				else {
					if (blockIndexNum[iii] > numberMiddle&& iii % 2 != 0) {
						blockIndexNum[iii] --;
						dTiff--;
					}
				}
				iii++;
			}
			for (int i = 0; i < blockIndex.size(); i++) {
				for (int j = 0; j < blockIndexNum[i]; j++) {
					vector<double> p = br.pointCloudData[blockIndex[i][j]];
					finalResult.push_back(p);
				}
			}
			labelG.clear();
			return finalResult;		
		}				
	}
	
	vector<int> AIS_Voronoi_new(int number, vector<int> pointSet, vector<int> label) {//使用全局index来索引point

		//number: sampling points number		
		//label[i] == 0 selected  1 unselected 2 unselected and outside the box
		vector<int> pointSampling;
		vector<double> minDex(pointSet.size());

		if (number <= 0) {
			return pointSampling;
		}

		//init distance matrix;
		for (int i = 0; i < minDex.size(); i++) {
			if (label[i] == 0) {
				minDex[i] = 0;//selected point
				pointSampling.push_back(pointSet[i]);
				if (labelG[pointSet[i]]) {
					labelG[pointSet[i]] = false;
				}
			}
			else if (label[i] == 2) {
				label[i] = 0;
				minDex[i] = 0;//selected point
				if (labelG[pointSet[i]]) {
					labelG[pointSet[i]] = false;
				}
			}
			else {
				vector<double> p = br.pointCloudData[pointSet[i]];
				double disMin = 9999;
				int indexij = -1;
				for (int j = 0; j < label.size(); j++) {
					if (label[j] == 0 || label[j] == 2) {
						vector<double> pi = br.pointCloudData[pointSet[j]];
						double dis_pij = sqrt((p[0] - pi[0]) * (p[0] - pi[0]) +
							(p[1] - pi[1]) * (p[1] - pi[1]) +
							(p[2] - pi[2]) * (p[2] - pi[2]));
						if (dis_pij < disMin) {
							disMin = dis_pij;
						}
					}
				}
				minDex[i] = disMin;
			}
		}

		//Voronoi Updata
		while (1) {

			double max = -1;
			int index_select = -1;
			//1. Search the furthest point from the pointSet
			for (int i = 0; i < pointSet.size(); i++) {
				if (label[i] == 1 && minDex[i] > max) {
					index_select = i;
					max = minDex[i];
				}
			}
			if (index_select == -1) {
				break;
			}
			//if the point is inside the box, then add the point into the pointSampling
			//if the point out side the box, then dont add the point into the pointSampling
			if (label[index_select] == 1) {
				pointSampling.push_back(pointSet[index_select]);
				label[index_select] = 0;
				labelG[pointSet[index_select]] = false;
				minDex[index_select] = 0;
			}

			vector<double> p_source = br.pointCloudData[pointSet[index_select]];

			//2. Update the pointSet
			for (int i = 0; i < pointSet.size(); i++) {
				if (label[i] == 1) {
					vector<double> p = br.pointCloudData[pointSet[i]];
					double dis = sqrt((p[0] - p_source[0]) * (p[0] - p_source[0]) +
						(p[1] - p_source[1]) * (p[1] - p_source[1]) +
						(p[2] - p_source[2]) * (p[2] - p_source[2]));
					if (dis < minDex[i]) {
						minDex[i] = dis;
					}
				}
			}
			if (pointSampling.size() >= number) {
				break;
			}
		}

		return pointSampling;
	}

private:

	bool AIS_searching(int index, vector<int> indexV) {

		for (int i = 0; i < indexV.size(); i++) {
			if (index == indexV[i]) {
				return true;
			}
		}
		return false;
	}
		
};