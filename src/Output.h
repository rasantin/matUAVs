/*
 * Output.h
 *
 *  Created on: 17 de jun de 2017
 *      Author: rsantin
 */

#ifndef OUTPUT_H_
#define OUTPUT_H_


#include "gurobi_c++.h"
#include <iostream>
#include <fstream>
#include <vector>
#include <sys/stat.h>
#include "Node.h"
#include "Input.h"
#include "Solution.h"
using namespace std;

// alias 'uint' for 'unsigned int'
using uint = unsigned int;



class Output {


public:
	Output();
	Output(Input input);
	virtual ~Output();

	Input input;

	void writeNodes();
	void writeSolution(Solution sol, int iter, string op, Solution::alog vars);
	void writeOutput(double maxCost, int iter, string op );
	void writeSolutions(Solution s, string op);
	void writeParetoSet(Solution sol,string pname, string iName, string date,
			double elapsed_time, int targetsNum, vector<bool> validation, int nexec,int total_exec, int m, int n, int cvl_subset_num);

	void writePredictions(Solution sol);
	void gurobiCallInfo(Solution sol);


	void createDir(int it);
	string createDirSol(int sol);
	void createDirInst(string iName);
	void createDirOutput();
	void createDataDir(string dates);
	string createPathDir(string sol, int path);
	void createOutput(string date);


	bool fileExists(const std::string& file);

private:
	string execPath;
	string nodesPath;
	string solutionPath;
	string instPath;

};


#endif /* OUTPUT_H_ */
