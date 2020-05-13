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
#include "Method_AIVS_OM.hpp"

using namespace std;

int main(int argc, char* argv[])
{
	cout << "start!" << endl;
	//1. Input file path
	//data: Angel; Armadillo; Asian Dragon;Buddha;Bunny;Dragon;Horse;Lucy;Thai Statue
	string pathobj = "Bunny";	
	string pathFile = "E:/database/standford/" + pathobj + ".obj";
	int simplificationNum = 10000;

	//Pre-process point cloud
	pointProcessPipeline ppp;
	vector<vector<double>> point_sim;	
	ppp.pointProcessPipeline_init(pathFile, true);		

	if (1) {
		simplification_Method_AIVS_OM smAISCuda;
		smAISCuda.simplification_Method_AIVS_init(ppp.br);		
		point_sim = smAISCuda.AIVS_simplification(simplificationNum);
	}	

	cout << "end!" << endl;
}

