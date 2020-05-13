#pragma once
#include "View.h"
#include <stdio.h>
#include <stdlib.h>
#include <GL/freeglut.h>   // OpenGL GLUT Library Header
#include "FileProcess/LoadFileDlg.h"
#include "trackball.h"
#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include "pointProcessPipeline.hpp"
#include "Method_AIS_new.hpp"


using namespace std;

int main()
{
	cout << "start!" << endl;

	//input point cloud data file path, support:txt, xyz, ply, obj, off
	
	string pathFile = "E:/database/standford/bunny.obj"; 
	//input simplification number
	int simplificationNum = 10000;
	
	pointProcessPipeline ppp;	
	
	//Pre-process for point cloud process: 1. point load; 2. normal rebuild; 
	//3. kd-tree building; 4; Voxelization for AIS 
	ppp.pointProcessPipeline_init(pathFile, true);
	vector<vector<double>> pdata = ppp.lpc.pointSet_uniform;
	cout << "Orignal Points:" << pdata.size() << endl;
	vector<vector<double>> ndata = ppp.ne.normalVector;
	vector<int> borderdata = ppp.lpc.indexBorder;		

	//AIVS simplification
	simplification_Method_AIS_new smAIS;	
	smAIS.simplification_Method_AIS_init(ppp.br);
	vector<vector<double>> point_sim = smAIS.simplification_Method_AIS_Sim(simplificationNum);

	cout << "Finish!" << endl;
	//OutPut Simplification Method: point_sim
	cout << "simplification: " << pdata.size() << " to " << point_sim.size();
	

}
