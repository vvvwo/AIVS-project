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

	string pathOutFileLogFile = "errorLog.txt";

	if (argc < 4) {

		ofstream fout(pathOutFileLogFile, ios::app);		
		fout << "Parameter Error! The input parameters are not enrough!" << endl;		
		fout << endl;
		fout.close();

		cout << "Parameter Error! The input parameters are not enrough!" << endl;
		cout << "Press any Key to exit..." << endl;
		getchar();
		return 0;
	}	

	string pathFile(argv[1]);
	int simplificationNum = (int)atoi(argv[2]);
	string pathOutFile(argv[3]);
	
	//string pathobj = "Bunny";	
	//string pathFile = "E:/chen_database/RemshOriginal/stand/" + pathobj + ".obj";
	//int simplificationNum = 10000;
	//Pre-process point cloud
	pointProcessPipeline ppp;
	vector<vector<double>> point_sim;	
	ppp.pointProcessPipeline_init(pathFile, true);	

	if (ppp.br.pointCloudData.size() <= simplificationNum) {

		ofstream fout(pathOutFileLogFile, ios::app);
		fout << "The input simplification point number is larger than original point cloud!" << endl;		
		fout << endl;
		fout.close();
		cout << "The input simplification point number is larger than original point cloud!" << endl;
		cout << "Press any Key to exit..." << endl;
		getchar();
		return 0;
	}

	if (1) {

		simplification_Method_AIVS_OM smAIVS;
		smAIVS.simplification_Method_AIVS_init(ppp.br);
		point_sim = smAIVS.AIVS_simplification(simplificationNum);

	}	

	//save the simplification result.

	ofstream fout(pathOutFile, ios::app);	
	for (int i = 0; i < point_sim.size(); i++) {
		fout <<"v "<< point_sim[i][0] << " " << point_sim[i][1] << " " << point_sim[i][2] << endl;
	}
	fout << endl;
	fout.close();	
	cout << "End!"<< endl;

}

