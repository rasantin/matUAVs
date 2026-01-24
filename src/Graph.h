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
#include <iostream>
#include "Input.h"
#include <memory>
#include <map>
#include "Utils.h"


class Graph {
protected:
	struct graphInfo{
		int T = 0;
		int D = 0;
		int baseID = 0;
		int robotID =0;
	};

	//Alias 'uint' for  'unsigned int'
	using uint = unsigned int;  

	//initial complete graph ;
	std::vector < std::vector<double>> graph;

	std::map<int, int> nodeIdToIndex;   // nodeId -> índice da matriz
	std::map<int, int> indexToNodeId;   // índice da matriz -> nodeId

	//vector of depots indexes
	std::vector<int> graphDepotsIndexes;

	//vector of targets indexes
    std::vector<int> graphTargetsIndexes;

    std::map<int,int> mapNodesTypes;

    std::map<int,std::set<int>> mapRobotGroup;
    std::map<int,std::set<int>> mapRobotGroup_Bckup;

    //mapear cada robô ao seu grupo
    std::map<int,int> mapGroupRobot;
    std::map<int,int> mapGroupRobot_Bckup;

    //map coverage_set_id to node id on graph;
    std::map<int,int> map_cvset_id_to_node_id;

    //fuel to closest depot
    std::map<int,double> min_fuel;

	// NOTE: Estrutura complexa (pair<vector<pair<vector<double>,int>>, graphInfo>)
	// Representa um subgrafo com:
	//  - first  -> lista de arestas (custos + nó)
	//  - second -> informações do grafo (robot, base, T, D)
	// TODO(refactor): substituir por struct GraphBlock quando o VNS estiver estável

	//matriz com os ngrafos
	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> nGraphs;

	//matriz com os nConjuntos de cobertura
	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> coverageSets;

	std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo> coverage_set;

	//matriz de backup dos ngraphs.
	std::vector<std::pair<std::vector<std::pair<std::vector<double>,int>>,graphInfo>> coverageSets_Bckup;

	float cLines = 0;
	float nRobots = 0;
	float nLines = 0.0;

	// Declaring the type of Predicate that accepts 2 pairs and return a bool
	typedef std::function<bool(std::pair<int, double>, std::pair<int, double>)> Comparator;

	// Defining a lambda function to compare two pairs. It will compare two pairs using second field
	Comparator compFunctor =
			[](std::pair<int, double> elem1 ,std::pair<int, double> elem2)
			{
				return elem1.second < elem2.second;
			};

	// Defining a lambda function to compare two nodes. It will compare two nodes using node's position (X position).
	Comparator compFunctor2 =
			[](std::pair<int,double> p1, std::pair<int,double> p2)
			{
		return p1.second > p2.second;
			};


	//atrubui para a matriz graph o tempo de voo entre todos os nós lidos da entrada (input)
	void buildGraph();

	//split graph in N subgraphs
	void splitGraph();

	//split graph in N subgraphs considering robots capacity
	void splitHGraph();

	void SplitSubGraph();

	//insert depots on targets position
	void insertDepotsOnTargets();

	// atribui  para nGraph o tempo de voo de cada aresta para cada um dos n grafos, obtidos da função splitGraph.
	void setAllNodesCosts();

	//calcular o tempo de voo
	double getFlightTime(double distance, double vel);

	//imprime os valores do grafo completo
	void printGraph();

	//armazenar informações referente a número de depots e targets de cada grafo
	void setGraphInfo();

	void copyNSets();

	//ordenar vetor de nós relativos a disposição  dos nós em relação ao eixo X
	void sortNodesX();

	//vetor ordenado de nós em relação ao eixo X
	std::vector<Node> nodesX;

	void insertDepotsOnNodesSets();
	void mapRobotTypeGroups();
	void UpdateSubSet(int nodeset);

	void set_min_fuel_2_depot();


	int cvl_subset_num =5;



public:
	Input &input;

	std::map<int,int >map_nodes_on_cl;


	//data set
	struct sub_set_data {
		//vector of coverage line, only first node id.
		std::vector<int> cvLines;

		//set of depots including robot base
		std::vector<int> depots;

		//sum of all coverage line;
		double length;
	};

	//data set
	struct Set {
		//vector of coverage line, only first node id.
		std::vector<int> cvLines;

		//set of depots including robot base
		std::vector<int> depots;

		//sum of all coverage line;
		double length;

		//robot's id of the set
		int robotID;

		int set_id;

		std::vector<sub_set_data> sub_set;
	};


	//nodesSets conjunto com um nós representande da linha de cobertura e as informações dos depósitos, tamanho e robô
	std::vector<Set> nodesSets;
	std::vector<Set> nodesSets_Bckup;

	std::map<int,Node> link_nid_to_ninfo;
	Graph(const Graph &g):input(g.input){}


	//create a complete graph and calculate the nodes distances.
	//the nodes of initial graph are read from input file.
	//Graph(): input(*new Input){ };
	Graph(Input &input_, int nsubset):input(input_){
		cvl_subset_num = nsubset;
		reset();
		buildGraph();
		splitHGraph();
		mapRobotTypeGroups();
		setMapGroupOfRobot();

		//inserir depots nos targets
		insertDepotsOnNodesSets();
		//setAllNodesCosts();
		SplitSubGraph();

		map_nodes_on_cl = GetALLCLines();
	};

	Graph();

	//get costs form i to j at graph k on coverageSet
	double getCost(unsigned int k, unsigned int i, unsigned int j);
	double getCost(unsigned int i, unsigned int j);

	double getCostOnGraph(unsigned int robotID, unsigned int i, unsigned int j);

	//get graphs number
	int getNOfGraphs();

	//get number os nodes from graph k
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

	//bool swapCLine(uint k1, uint k2, int l1, int l2);
	void swapCLine(uint k1, uint k2, int l1, int l2);

	//bool shiftCLine (uint k1, uint k2, uint l);
	void shiftCLine (uint k1, uint k2, uint l);

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
	bool insertNewCVLine(int k1,int k2, int targetID, std::vector<int>&depotsK1, std::vector<int>&depotsK2);

	void updateNodesSets(std::vector<Set> ns);
	void updateCoverageSets(std::vector<Set> ns);
	void Convert_NS_to_CS(Set ns);

	void updateAllSets(std::vector<Set> ns);
	void updateCoverageSet(int k);
	void insertNSDepots(int id, std::vector<int> depots);

	void swapRobotsNodesSets(int g1,int g2 );
	void UpdateDepotsOnSubSet(int nodeset_id , int subset_id,std::vector<int> depot);
	void UpdateDepots(int id, std::vector<int> depots);

	int getMapRobotGroupSize();
	std::set<int> getRobotGroups(int k);

	void swapRobotsGroups(int k1, int g1, int k2, int g2);

	int getTargetsGraphIndexNum();

	std::map<int,int> GetALLCLines();

	bool IsCLine(int node1, int node2);

	double get_min_fuel_2_depot(int i);

	//retorna o o tipo do robô
	int GetGroupOfRobot(int path_id);

	void setMapGroupOfRobot();


	virtual ~Graph();
	virtual void reset();

};


#endif /* SRC_GRAPH_H_ */
