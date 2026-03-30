/*
 * Graph.h
 *
 *  Created on: 18 de mai de 2018
 *      Author: rsantin
 */

#ifndef SRC_GRAPH_H_
#define SRC_GRAPH_H_

#include <cstdlib>
#include <iostream>
#include <functional>
#include <vector>
#include "Input.h"
#include <set>
#include <cmath>
#include <sstream>
#include <memory>
#include <map>
#include "Utils.h"
#include <queue>
#include <limits>
#include <algorithm>
#include <unordered_set>
#include <random>


class Graph {

public:

	Input &input;

	std::map<int,int> map_nodes_on_cl;

	// ==============================
	// DATA STRUCTURES
	// ==============================

	struct sub_set_data {

		std::vector<int> cvLines;
		std::vector<int> depots;
		double length;
	};

	struct Set {

		std::vector<int> cvLines;
		std::vector<int> depots;
		double length;

		int robotID;
		int set_id;

		std::vector<sub_set_data> sub_set;
	};

	//nodesSets
	std::vector<Set> nodesSets;
	std::vector<Set> nodesSets_Bckup;

	std::map<int,Node> link_nid_to_ninfo;

protected:

	struct graphInfo{

		int T = 0;
		int D = 0;
		int baseID = 0;
		int robotID =0;
	};

	using uint = unsigned int;

	//initial complete graph
	std::vector<std::vector<double>> graph;

	std::map<int, int> nodeIdToIndex;
	std::map<int, int> indexToNodeId;

	std::vector<int> graphDepotsIndexes;
	std::vector<int> graphTargetsIndexes;

	std::map<int,int> mapNodesTypes;

	std::map<int,std::set<int>> mapRobotGroup;
	std::map<int,std::set<int>> mapRobotGroup_Bckup;

	std::map<int,int> mapGroupRobot;
	std::map<int,int> mapGroupRobot_Bckup;

	std::map<int,int> map_cvset_id_to_node_id;
	std::map<int,double> min_fuel;

	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> nGraphs;

	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> coverageSets;

	std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo> coverage_set;

	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> coverageSets_Bckup;

	float cLines = 0;
	float nRobots = 0;
	float nLines = 0.0;

	typedef std::function<bool(std::pair<int, double>, std::pair<int, double>)> Comparator;

	Comparator compFunctor =
		[](std::pair<int, double> elem1 ,std::pair<int, double> elem2)
		{
			return elem1.second < elem2.second;
		};

	Comparator compFunctor2 =
		[](std::pair<int,double> p1, std::pair<int,double> p2)
		{
			return p1.second > p2.second;
		};

	// ==============================
	// INTERNAL METHODS
	// ==============================

	void buildGraph();

	void splitGraph();

	std::vector<Set> splitHGraph();

	void SplitSubGraph();

	void insertDepotsOnTargets();

	void setAllNodesCosts();

	double getFlightTime(double distance, double vel);

	void printGraph();

	void setGraphInfo();

	void copyNSets();

	void sortNodesX();

	std::vector<Node> nodesX;

	void insertDepotsOnNodesSets();

	void mapRobotTypeGroups();

	void UpdateSubSet(int nodeset);

	void set_min_fuel_2_depot();

	int cvl_subset_num = 5;

public:

	Graph(const Graph &g):input(g.input){}

	Graph(Input &input_, int nsubset):input(input_) {

		cvl_subset_num = nsubset;

		reset();

		buildGraph();

		nodesSets.clear();

	


		std::vector<Set> sets = splitHGraph();

		input.maxFuelCost.resize(input.getRobotNum());
		input.constM.resize(input.getRobotNum());

		for(auto &s : sets){
			Set ns;
			ns.robotID = s.robotID;
			ns.set_id = s.set_id;
			ns.cvLines = s.cvLines;
			ns.length = s.length;

			nodesSets.push_back(ns);
		}

		sortNodesX();

	
		for (Node n : nodesX)
			link_nid_to_ninfo.emplace(n.nodeId, n);

		mapRobotTypeGroups();

		setMapGroupOfRobot();

		insertDepotsOnNodesSets();

		SplitSubGraph();

		map_nodes_on_cl = GetALLCLines();
	}

	Graph();

	double getCost(unsigned int k, unsigned int i, unsigned int j);

	double getCost(unsigned int i, unsigned int j);

	double getCostOnGraph(unsigned int robotID, unsigned int i, unsigned int j);

	int getNOfGraphs();

	int getNNodes(int k);

	int getIndex(int k, int i);

	int getIndex(int i);

	int getCVLIndex(int k, int i);

	int getTargetNum(int k);

	int getTargetNum();

	int getDepotNum(int k);

	int getDepotNum();

	int getNumberOfGraphs();

	int getNumberOfSets();

	int getNumberOfLines(int k);

	void swapCLine(uint k1, uint k2, int l1, int l2);

	void shiftCLine(uint k1, uint k2, uint l);

	void removeCLine(uint k1, uint l);

	void printGroupOfLines();

	void restoreNGraphs();

	void restoreNSets();

	std::vector<std::pair<int,double>> getSetsArea();

	std::vector<int> getDepotsBetweenNodes(int n1, int n2);

	int getBaseID(int k);

	int getBaseID();

	void removeDepots(int n1, std::vector<int>& depots);

	void getDistanceBetweenCVLines(int cvLine1,int cvLine2);

	bool insertNewCVLine(int k1,int k2, int targetID,
			std::vector<int>&depotsK1,
			std::vector<int>&depotsK2);

	void updateNodesSets(std::vector<Set> ns);

	void updateCoverageSets(std::vector<Set> ns);

	void Convert_NS_to_CS(Set ns);

	void updateAllSets(std::vector<Set> ns);

	void updateCoverageSet(int k);

	void insertNSDepots(int id, std::vector<int> depots);

	void swapRobotsNodesSets(int g1,int g2);

	void UpdateDepotsOnSubSet(int nodeset_id,
			int subset_id,
			std::vector<int> depot);

	void UpdateDepots(int id, std::vector<int> depots);

	int getMapRobotGroupSize();

	std::set<int> getRobotGroups(int k);

	void swapRobotsGroups(int k1, int g1, int k2, int g2);

	int getTargetsGraphIndexNum();

	std::map<int,int> GetALLCLines();

	bool IsCLine(int node1, int node2);

	double get_min_fuel_2_depot(int i);

	int GetGroupOfRobot(int path_id);

	void setMapGroupOfRobot();

	double getFeasibleDistance(int robotID, int src, int dst);

	double getDistanceBetweenLines(int robotID, int a, int b);

	virtual ~Graph();

	virtual void reset();
};

#endif