/*
 * Input.h
 *
 *  Created on: 3 de jun de 2017
 *      Author: rsantin
 */

#ifndef INPUT_H_
#define INPUT_H_

#include <iostream>
#include <fstream>
#include <string>
#include "Node.h"
#include "Robot.h"
#include <vector>
#include <stdexcept>
#include <algorithm>
#include <map>

using namespace std;


class Input {

private:
	void sortNodes(bool changeID);
	std::vector<Robot> robots;
	double euclidianDistance(Node &a, Node &b);
	std::vector<std::vector < std::vector<double> > > F;
	vector<vector<int>> g;

	typedef pair<double, bool> cell ;

	int depotNum = 0;
	int baseNum = 0;
	int targetNum = 0;
	int nodeNum = 0;
	int depotsInserted=0;
	int nexec=1;
	int n=1;
	int m=1;
	int max_cvl_subset = 5;

	std::vector<int> nodesDepotsIndexes;
	std::vector<int> nodesTargetsIndexes;

	std::map<int,int> mapTargetDepot;

	void nodesIndexes();

	string file_name;


public:

	std::string GetFileName(){
		return file_name;
	}
	std::vector<Node> nodes;
	Input(std::string fileName);
	Input();
	void readFile(std::string fileName);
	void readFileNames(std::string fileName);
	void calculeFuelCosts();

	void printNodes();
	void printRobots();

	std::vector<double> maxFuelCost;
	std::vector<double> constM;

	int getTargetNum();
	int getDepotNum();
	int getBaseNum();
	int getRobotNum();
	int getNodesNum();

	void initMCost(std::vector < std::vector<cell>>& G);
	int getRobotBaseId(int k);
	double getRobotFuel(int robot);
	double getRobotVel(int robot);
	double getRobotProp(int robot);
	double getF(int i,int j, int k);
	double getDistance(int i,int j);
	double getDistance(Node i,Node j);

	int insertNodes(double x, double y, std::string type);
	Node getNode(int node);

	int getDepotIdOnTarget(int id);
	bool isTarget(int id);
	string getRobotConfigName(int k);

	int getDepotsInserted();

	int getNExec();
	int getN();
	int getM();

	int getMaxCVLSubSet();

	vector<int> getDepotsIndexes();
	vector<int> getTargetsIndexes();

	void insertDepotsOnTargets();


	bool nodesEmpty();

	virtual ~Input();
};

#endif /* INPUT_H_ */
