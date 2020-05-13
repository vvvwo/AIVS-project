#pragma once
/*
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
#include "ShowPointCloud.hpp"
#include "pointProcessPipeline.hpp"
#include "simplificationCompute.hpp"
#include "Method4.hpp"
#include "Method4_old.hpp"
#include "Method4_global.hpp"
#include "Method_AIS.hpp"
#include "Method_AIS_old.hpp"
#include "Method_AIS_new.hpp"
#include "Method_AIS_global.hpp"

using namespace std;

// The size of the GLUT window
int window_width = 800;
int window_height = 600;
// The OBJ model
GLMmodel* pModel = NULL;
GLMmodel* pModelM = NULL;
// The current modelview matrix
double pModelViewMatrix[16];
// If mouse left button is pressed
bool bLeftBntDown = false;
// Old position of the mouse
int OldX = 0;
int OldY = 0;
vector<vector <int>> neiborPoint;
string filename;
string filenamePoint;
double scale = 100;
int showPointStyle = 0;
ShowPointCloud spc;//point cloud shown and uniform
pointProcessPipeline ppp;
//simplification result
vector<vector<double>> point_sim;
vector<vector<double>> point_sim_n;
vector<vector<double>> point_sim_rn;
vector<vector<double>> point_sim2;
vector<vector<double>> point_resample;
vector<vector<double>> resultMin;

vector<double> disValue;

clock_t t1;
clock_t t2;
clock_t t3;
clock_t t4;


int main(int argc, char* argv[])
{
	cout << "start!" << endl;
	//1. Input file path
	//1_MLSValue/; 2_DHD; 3_LaplasGraphOld/; 4_LaplasGraph/; CGAL_Grid/; CGAL_Hierarchy/; CGAL_Wlot/; AIVS/;
	string pathMethod = "AIVS/";
	//data: Angel; Armadillo; Asian Dragon;Buddha;Bunny;Dragon;Horse;Lucy;Thai Statue
	string pathobj = "Asian Dragon";
	string pathTail = ".obj";
	string path10000Tail = "10000.txt";
	string path01Tail = "01.txt";
	string pathObjBacis = "E:/database/standford/";
	string pathFile = pathObjBacis + pathobj + pathTail; //"3DModel/bunny.obj";//T164.obj	
	string pathBasic = "E:/database/standford/simplification/";
	string pathFile10000 = pathBasic + pathMethod + pathobj + path10000Tail;
	string pathFile01 = pathBasic + pathMethod + pathobj + path01Tail;


	char* p = new char[strlen(pathFile.c_str()) + 1];
	strcpy(p, pathFile.c_str());
	pModelM = glmReadOBJ(p);

	glmFacetNormals(pModelM);
	//glmVertexNormals(pModelM, 0.25);
	glmTurnNorm2Trangular(pModelM);
	float modelCenter2[] = { 0.0f, 0.0f, 0.0f };
	glmUnitize(pModelM, modelCenter2);
	filename = pathFile;

	//data: Angel; Armadillo; Asian Dragon;Buddha;Bunny;Dragon;Horse;Lucy;Thai Statue
	ppp.pointProcessPipeline_init(pathFile, true);
	vector<vector<double>> pdata = ppp.lpc.pointSet_uniform;
	cout << "Orignal Points:" << pdata.size() << endl;
	vector<vector<double>> ndata = ppp.ne.normalVector;
	vector<int> borderdata = ppp.lpc.indexBorder;
	
	//rebuild
	ppp.pointProcessPipeline_fixBallRegion();

	if (1) {

		simplification_Method_AIS_new smAIS;
		//simplification_Method4_old sm4;
		smAIS.simplification_Method_AIS_init(ppp.br_fix);
		t1 = clock();
		
		point_sim = smAIS.simplification_Method_AIS_Sim(10000, pathFile10000);
		//point_sim = smAIS.simplification_Method_AIS_Sim(10000, pathFile10000);
		t2 = clock();
		t3 = clock();
		point_sim2 = smAIS.simplification_Method_AIS_Sim(pdata.size() * 0.1, pathFile01);
		//point_sim2 = smAIS.simplification_Method_AIS_Sim(pdata.size() * 0.1, pathFile01);
		t4 = clock();
		//std::cout << "10000 points running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		//std::cout << "0.1 rate running time:" << (t4 - t3) / 1000.0 << "s" << endl;	

	}


	//simplification_Method4_global sm4;
	/*
	if (1) {
		//simplification_Method_AIS sm4;
		simplification_Method_AIS_new sm4;
		sm4.simplification_Method_AIS_init(ppp.br);
		t1 = clock();
		point_sim = sm4.simplification_Method_AIS_SimValue(10000,pathFile10000);
		t2 = clock();

		t3 = clock();
		point_sim2 = sm4.simplification_Method_AIS_SimValue(pdata.size() * 0.1,pathFile01);
		t4 = clock();

		point_sim_n = sm4.simplification_Method_AIS_Simplification_Normal(point_sim);
		BallRegion br_i;
		br_i.BallRegion_init2(point_sim, point_sim_n, ppp.br);
		point_sim_rn = br_i.pointNormal;
	}*/

	if (1) {

		simMeasurement smt;
		smt.simMeasurement_init(point_sim, ppp.br);
		vector<double> avg_max = smt.simMeasurement_ave_max_fast();
		disValue = smt.pointSet_ShapeValue;
		spc.ShowPointCloud_Draw_Face2_init(disValue);
		smt.simMeasurement_minDis();
		//smt.simMeasurement_GuassCurvature(pathFile);
		point_resample = smt.pointSet_Orignal_New;

		simMeasurement smt2;
		smt2.simMeasurement_init(point_sim2, ppp.br);
		vector<double> avg_max2 = smt2.simMeasurement_ave_max_fast();
		smt2.simMeasurement_minDis();
		//smt2.simMeasurement_GuassCurvature(pathFile);		
		std::cout << "10000 points running time:" << (t2 - t1) / 1000.0 << "s" << endl;
		std::cout << "0.1 rate running time:" << (t4 - t3) / 1000.0 << "s" << endl;
		cout << "Result:" << endl;
		//cout << "h:" << h << endl;
		cout << "Detail:" << smt.pointSet_Simplification.size() << "/" << smt.pointSet_Orignal_New.size() << endl;
		cout << "maxError:" << avg_max[1] << endl;
		cout << "avgError:" << avg_max[0] << endl;
		//cout << "GCDifferMax:" << smt.diffMax << endl;
		//cout << "GCDifferavg:" << smt.diffAvg << endl;
		cout << "DDiffDifferavg:" << smt.abs_dis << endl;

		cout << "Result:" << endl;
		//cout << "h:" << h << endl;
		cout << "Detail:" << smt2.pointSet_Simplification.size() << "/" << smt2.pointSet_Orignal_New.size() << endl;
		cout << "maxError:" << avg_max2[1] << endl;
		cout << "avgError:" << avg_max2[0] << endl;
		//cout << "GCDifferMax:" << smt2.diffMax << endl;
		//cout << "GCDifferavg:" << smt2.diffAvg << endl;
		cout << "DDiffDifferavg:" << smt2.abs_dis << endl;
	}

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH | GLUT_STENCIL);
	glutInitWindowSize(window_width, window_height);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("OpenGL 3D Object");
	init();
	// Set the callback function
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
	glutIdleFunc(idle);
	glutMainLoop();
	return 0;

}

/// Initialize the OpenGL
void init()
{

	glClearColor(0.0, 0.0, 0.0, 0.0);
	glShadeModel(GL_FLAT);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (float)window_width / (float)window_height, 0.01f, 200.0f);
	glClearColor(1, 1, 1, 1);//backGround color
	glClear(GL_DEPTH_BUFFER_BIT | GL_COLOR_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
	glEnable(GL_DEPTH_TEST);
	glEnable(GL_NORMALIZE);
	glEnable(GL_CULL_FACE);
	//glEnable(GL_COLOR_MATERIAL);//Ôö¼ÓÑÕÉ«
	// Setup other misc features.
	glEnable(GL_LIGHTING);
	glEnable(GL_NORMALIZE);
	//glShadeModel( GL_SMOOTH );
	glEnable(GL_COLOR_MATERIAL);
	// Setup lighting model.
	GLfloat light_model_ambient[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light0_diffuse[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	GLfloat light0_direction[] = { 0.0f, 0.0f, 10.0f, 0.0f };
	GLfloat light_specular[] = { 1.0f, 1.0f, 1.0f, 1.0f };
	glLightfv(GL_LIGHT0, GL_POSITION, light0_direction);
	glLightfv(GL_LIGHT0, GL_AMBIENT, light_model_ambient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, light0_diffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
	glEnable(GL_LIGHT0);
	//glLightModeli(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	// Init the dlg of the open file
	PopFileInitialize(NULL);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glGetDoublev(GL_MODELVIEW_MATRIX, pModelViewMatrix);

}

/// Display the Object
void display()
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	glTranslated(0.0, 0.0, -5);
	glMultMatrixd(pModelViewMatrix);
	glColorMaterial(GL_FRONT, GL_AMBIENT_AND_DIFFUSE);
	if (pModel) {
		glColor3f(0.3, 0.3, 0.3);
		glmDraw(pModel, GLM_FLAT);
	}
	if (showPointStyle == 0) {
		spc.ShowPointCloud_Draw_Origanal(ppp.lpc.pointSet_uniform);
	}
	else if (showPointStyle == 1) {
		spc.ShowPointCloud_Draw_Origanal(ppp.lpc.pointSet_uniform);
		spc.ShowPointCloud_Draw_Resample(point_resample);
	}
	else if (showPointStyle == 2) {
		spc.ShowPointCloud_Draw_Simpling(point_sim);
		spc.ShowPointCloud_Draw_Simpling_Key(resultMin);
		//spc.ShowPointCloud_Draw_VolumePixel(ppp.br);
	}
	else if (showPointStyle == 3) {
		spc.ShowPointCloud_Draw_Face2(pModelM);
		//spc.ShowPointCloud_Draw_VolumePixel(ppp.br);
	}
	else if (showPointStyle == 4) {
		if (pModelM) {
			spc.ShowPointCloud_Draw_Face(point_sim, pModelM);
		}
		//else {			
			//glmTurnNorm2Trangular(pModelM);			
			//glmFacetNormals(pModelM);
			//glmVertexNormals(pModelM, 0.25);
			//float modelCenter[] = { 0.0f, 0.0f, 0.0f };
			//glmUnitize(pModelM, modelCenter);
		//}		
		//spc.ShowPointCloud_Draw_Simpling_Normal(point_sim, point_sim_rn, 1);
		//spc.ShowPointCloud_Draw_Simpling_Key(resultMin);
		//spc.ShowPointCloud_Draw_VolumePixel(ppp.br);
	}
	else {
		spc.ShowPointCloud_Draw_Simpling(point_sim);
		spc.ShowPointCloud_Draw_Resample(point_resample);
		//spc.ShowPointCloud_Draw_Normal(ppp.lpc.pointSet_uniform, ppp.ne.normalVector, 0.02);
	}
	glutSwapBuffers();
}

/// Reshape the Window
void reshape(int w, int h)
{
	window_width = w;
	window_height = h;
	glViewport(0, 0, window_width, window_height);
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(45.0f, (float)window_width / (float)window_height, 0.01f, 200.0f);
	glutPostRedisplay();
}

/// Keyboard Messenge
void keyboard(unsigned char key, int x, int y)
{
	// The obj file will be loaded
	char FileName[128] = "";
	char TitleName[128] = "";
	float modelCenter[] = { 0.0f, 0.0f, 0.0f };
	switch (key) {
	case 'c':
	case 'C':
		showPointStyle++;
		if (showPointStyle >= 5) {
			showPointStyle = 0;
		}
		break;
	case 'r':
	case 'R':
		glmTurnNorm2Trangular(pModelM);
		break;
	case 'o':
	case 'O':
		PopFileOpenDlg(NULL, FileName, TitleName);
		// If there is a obj model has been loaded, destroy it
		if (pModel)
		{
			glmDelete(pModel);
			pModel = NULL;
		}
		// Load the new obj model
		pModel = glmReadOBJ(FileName);
		filename = FileName;
		glmFacetNormals(pModel);
		glmVertexNormals(pModel, 0.25);
		glmUnitize(pModel, modelCenter);
		// Init the modelview matrix as an identity matrix
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glGetDoublev(GL_MODELVIEW_MATRIX, pModelViewMatrix);
		break;
	case '+':
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glLoadMatrixd(pModelViewMatrix);
		glScaled(1.05, 1.05, 1.05);
		glGetDoublev(GL_MODELVIEW_MATRIX, pModelViewMatrix);
		break;
	case '-':
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glLoadMatrixd(pModelViewMatrix);
		glScaled(0.95, 0.95, 0.95);
		glGetDoublev(GL_MODELVIEW_MATRIX, pModelViewMatrix);
		break;
	default:
		break;
	}
	glutPostRedisplay();
}

/// Mouse Messenge
void mouse(int button, int state, int x, int y)
{

	if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON)
	{
		OldX = x;
		OldY = y;
		bLeftBntDown = true;
	}
	else if (state == GLUT_DOWN && button == GLUT_LEFT_BUTTON)
	{
		bLeftBntDown = false;
	}

}

/// Motion Function
void motion(int x, int y)
{
	if (bLeftBntDown)
	{
		float fOldX = 2.0f * OldX / (float)window_width - 1.0f;
		float fOldY = -2.0f * OldY / (float)window_height + 1.0f;
		float fNewX = 2.0f * x / (float)window_width - 1.0f;
		float fNewY = -2.0f * y / (float)window_height + 1.0f;
		double pMatrix[16];
		trackball_opengl_matrix(pMatrix, fOldX, fOldY, fNewX, fNewY);
		glMatrixMode(GL_MODELVIEW);
		glLoadIdentity();
		glLoadMatrixd(pMatrix);
		glMultMatrixd(pModelViewMatrix);
		glGetDoublev(GL_MODELVIEW_MATRIX, pModelViewMatrix);
		OldX = x;
		OldY = y;
		glutPostRedisplay();
	}
}

/// Idle function
void idle(void)
{
	glutPostRedisplay();
}

