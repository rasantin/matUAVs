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
#include <filesystem>  // C++17 cross-platform filesystem support
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
	void writeSolution(Solution sol, int iter, std::string op, Solution::alog vars);
	void writeOutput(double maxCost, int iter, std::string op );
	void writeSolutions(Solution s, std::string op);
	void writeParetoSet(const Solution& sol,std::string pname, std::string iName, std::string date,
			double elapsed_time, int targetsNum, std::vector<bool> validation, int nexec,int total_exec, int m, int n, int cvl_subset_num);

	void writePredictions(const Solution& sol);
	void gurobiCallInfo(const Solution& sol);

	void createDir(int it);
	std::string createDirSol(int sol);
	void createDirInst(std::string iName);
	void createDirOutput();
	void createDataDir(std::string dates);
	std::string createPathDir(std::string sol, int path);
	void createOutput(std::string date);
	bool fileExists(const std::string& file);

private:
	std::string execPath;
	std::string nodesPath;
	std::string solutionPath;
	std::string instPath;

};

#endif /* OUTPUT_H_ */
