#pragma once

#include "LoadPointCloud.hpp"
#include "normalCompute.hpp"
#include "ballRegionCompute.hpp"

class pointProcessPipeline {

public:
	LoadPointCloud lpc;//load different kinds of files to achieve point cloud	
	NormalEstimation ne;//normal estimation for point cloud
	BallRegion br;//neibor structure construct
	BallRegion br_fix;//add points to fix neibor structure construct
	vector<int> fixBox;//
	int theorld;

public:

	void pointProcessPipeline_init(string filepath, bool brj) {
		//ne.test();
		cout << "pointProcessPipeline_init run!"<< endl;
		lpc.PointCloud_Load(filepath);//support obj, off, ply, xyz, txt
		//lpc.PointCloud_Load_Fast(filepath);
		
		ne.estimateNormal_init(lpc.FileNormal);
		if (ne.normalLoad()) {
			cout << "pointProcessPipeline_init: normal file exist and the data are loaded." << endl;
		}
		else {
			//ne.estimateNormal(lpc.pointSet_uniform, 'j');	
			ne.estimateNormal_PCL_MP(lpc.pointSet_uniform);
		}	
		

		if (brj) {
			vector<vector<double>> pdata = lpc.pointSet_uniform;
			vector<vector<double>> ndata = ne.normalVector;
			vector<int> borderdata = lpc.indexBorder;
			br.BallRegion_init(pdata, ndata, borderdata);		
		}	

	}


};
