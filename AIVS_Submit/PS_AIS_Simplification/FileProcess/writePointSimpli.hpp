#pragma once

#include <iostream>
#include <fstream>
#include <string>
#include <vector>

using namespace std;

void writePointSimpli(vector<vector<double>> pointSim, string filename) {

	ofstream f1(filename, ios::app);

	f1 << pointSim.size() << endl;
	for (int i = 0; i < pointSim.size(); i++) {
		f1 << pointSim[i][0] << " " << pointSim[i][1] << " " << pointSim[i][2] << " " << endl;
	}
	//f1 << endl;
	f1.close();

}

vector<vector<double>> readPointSimpli(string filename) {

	vector<vector<double>> result;

	ifstream fin(filename);
	if (fin)
	{
		fin.close();
		fstream out2;
		out2.open(filename, ios::in);
		int number;
		out2 >> number;
		for (int i = 0; i < number; i++) {
			double xi, yi, zi;
			out2 >> xi >> yi >> zi;
			vector<double> pi;
			pi.push_back(xi);
			pi.push_back(yi);
			pi.push_back(zi);
			result.push_back(pi);
		}
		out2.close();
		return result;
	}
	else {
		fin.close();
		return result;
	}

}
