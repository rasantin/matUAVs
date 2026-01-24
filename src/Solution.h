/*
 * Solution.h
 *
 *  Created on: 26 de jul de 2018
 *      Author: rsantin
 */

#ifndef SRC_SOLUTION_H_
#define SRC_SOLUTION_H_

#include "Input.h"
#include "gurobi_c++.h"
#include "Graph.h"
#include "Rand.h"
#include "Utils.h"
#include "SolverContext.h"

#include <stack>
#include <limits>

#include <vector>
#include <list>
#include <tuple>
#include <queue>
#include <unordered_map>
#include <utility>  // std::pair

	// Functor para permitir unordered_map com std::pair<int, int> como chave
	struct pair_hash
	{
		std::size_t operator()(const std::pair<int, int> &p) const
		{
			return std::hash<int>()(p.first) ^ (std::hash<int>()(p.second) << 1);
		}
	};

	class Solution : public Graph
	{

	private:
		struct edge
		{
			int node_a;
			int node_b;
			double time;
			double cost;
		};

		struct path
		{
			// pair<int,int> id nodes A->B, pair<double, double> (fcost, time)
			std::vector<std::pair<std::pair<int, int>, std::pair<double, double>>> nodes;
			std::vector<edge> edges;
			std::map<int, double> fuelOnTarget;
			int robotID ;
			int pID ;
			// sum of fcosts
			double pCost;
			std::set<int> depots;
			int depotsNum;
			int targetsNum;

			path() : pCost(0.0), robotID(-1), pID(-1), depotsNum(0), targetsNum(0) {}
		};

		struct solution
		{
			std::vector<path> paths;
			double sCost;
			double maxCost;
			int depotsNum;
			int targetsNum;
			int maxCostPathID;
			std::set<int> depots;
		};

		std::string itos(int i)
		{
			std::stringstream s;
			s << i;
			return s.str();
		}
		Rand rand;
		path bestPath(SolverContext& ctx,int gID);
		path milpSolver(SolverContext& ctx, const Set& coverage_set, const path& initial_sol = path());
		path improvePath(SolverContext& ctx, path p);

		path removeCL(path p, int cl);
		path remove_cl_new(path p, int cl);
		path insertCL(path p, int from, int to);
		// path bestInsertionCL(path p, int cl);

		std::pair<path, std::string> bestInsertionCL(path p, int cl);

		std::pair<int, int> getCLDirection(path p, int cl);

		std::vector<edge> GetEdgesCosts(std::vector<std::pair<int, int>> path, int RobotID);

		path AdjustRefuelingFromNode(std::vector<edge> path_cost, path p, int node);
		path AdjustPathRefueling(std::vector<edge> path_costs, path p);

		path GetPathFromBaseToNewCL(path p, int pos);
		path GetPathFromNodeAToNodeB(path p, int node_a, int node_b);
		path GetPathFromNewCLToBase(path p, int pos);
		path GetPathFromEdgeToNextDepot(path p, edge e);
		path GetInvPathFromNewCLToBase(path p, int pos);
		path GetInvPath(path p);

		std::vector<std::pair<int, int>> GetInvCLs(std::vector<std::pair<int, int>> cls_vec);
		std::vector<std::pair<int, int>> GetALLCLsFromBaseToNewCL(path p, int cl);
		std::vector<std::pair<int, int>> GetALLCLsFromBase(path p);
		std::vector<std::pair<int, int>> GetALLCLsFromNewCLToBase(path p, int node);

		path PathUnion(path path_a, path path_b);

		void Iteractive_DFS(path p);
		void SortMultiplesAdjacentOnDepots(path *p);

		std::pair<path, std::string> GetLeastCostPath(path p1, path p2, path p3, path p4);

		std::vector<std::pair<int, int>> GetEdgesFromPreviousCLToCL(path p, int cl);
		std::vector<std::pair<int, int>> GetEdgesFromCLToPosteriorCL(path p, int cl);

		path new_insert(path p, int out, int new_cl);

		path link_out_cl_in(path p, int out, int new_cl);
		path Link_Sets_Node_out_in(path p, int out, int in);
		bool IsCircuit(path p);

		edge GetNextCL(path p, int pos);

		int GetNearOpenedDepot(path p, int node_id);
		path GetSPTOverOpenDepots(path p, int node_start, int node_goal);

		void testeInvWalk(path p);
		path insertCLTest(path p, int from, int to);
		std::pair<path, std::string> EvalOthersOrientationsOnPath(path p, int cl);
		path Union_Solutions(solution sol_vec);

		std::list<std::pair<std::pair<std::pair<int, int>, double>, std::vector<int>>> GetTargetsLinkingCLs(path p);

		// obter o grafo dos depots que estão ente os targets a e b
		std::map<int, std::vector<std::pair<int, double>>> GetGraphOfDepotsBetween_T2T(int a, int b, int robot_id);
		std::map<int, std::vector<std::pair<int, double>>> GetOpenDepotGraphFromPath(path p);

		std::map<int, std::vector<std::pair<int, double>>> AddTargetOnGraph(std::map<int, std::vector<std::pair<int, double>>> list_adj,
															 int node_a, int node_b, int robot_id);
		std::map<int, std::vector<std::pair<int, double>>> AddNodesOnGraph(std::map<int, std::vector<std::pair<int, double>>> list_adj, path p,
															int node_a, int node_b);

		std::vector<int> SPT_A_Star(std::map<int, std::vector<std::pair<int, double>>> list_adj, int start, int goal, int robot_id);

		bool HasRepeatedCLines(path p);

		std::vector<std::pair<int, int>> GetPrevNextNodes(path p, int node_id);
		path LinkPairsOnPath(path p, std::vector<std::pair<int, int>> p_nodes);

		path RemoveDepotFromPath(path p, int node_id);

		void OpenRandomDepotsOnTargetsPath(path *p);

		double maxSolCost;
		void initSol(SolverContext& ctx, solution *s);
		int depotsNumInit = 0;

		double gurobi_time_limit = 0;
		double gurobi_optimize_time;
		bool initialSolution = true;
		int call_num = 0;
		int pred_num = 0;

	public:
		struct alog
		{
			int g1, g2, l1, l2;
		};

		struct pred
		{
			int pred_id;
			int pred_num;
			int pred_improv_sol;
			bool best_pred;
			std::string operation;
			std::string path_op_g1;
			std::string path_op_g2;
			double pred_time_g1;
			double pred_time_g2;
		};

		std::vector<pred> vec_predictions;

		struct gurobi_call
		{
			int call_id;
			int T;
			int D;
			double optimize_time;
			bool feasible;
			std::string type;
		};

		std::vector<gurobi_call> vec_call;
		alog vars;

		Solution(SolverContext& ctx, Input& input_, int max_cvl_subset_num);
    	
		// defensivo: Solution não pode ser copiada
    	Solution(const Solution&) = delete;
    	Solution& operator=(const Solution&) = delete;

		virtual ~Solution();

		solution currentSol;
		solution best_sol;

		std::vector<solution> vecSol;
		std::vector<std::pair<bool, solution>> paretoSet;
		std::pair<std::pair<int, int>, std::pair<int, int>> changeLog;
		std::map<int, std::set<int>> mapDG;

		// defining neighborhood functions
		bool swap(solution *s);
		//bool shift(solution *s);
		bool shift();
		void pshift(solution*s);
		bool swapRobots(solution *s);
		void openRandomDepot(solution *s);
		bool improveSol(SolverContext& ctx,solution *s);

		// junction of pertubation methods
		void perturbation(solution *s, int maxDepots);

		bool IsBetterSol(solution new_sol, solution incumbent_sol);

		void printSol(solution s);
		double getMaxSolCost();

		// return all depots open depots
		std::vector<int> getOpenDepots();

		void mapOpenDepotsToGroup();

		// return the odd indexes of coverage lines in path p
		std::vector<int> getOddCVLIndexes(const path& p);

		// return the map from depots and theirs groups
		std::map<int, std::set<int>> getMapDG(solution s);

		void updatePathCurrenteSol(int gID, path p);

		void updateAllPathCosts(solution *s);

		void updateSolCosts(solution *s);

		// close random  depot
		bool closeRandomDepot(solution *s);

		int GetCloserDepot(path p, int depot_closed);

		path GetPathWithDepotClosure(path p, int depotID);

		path CloseDepotLinkToNext(path p, int depot_to_close, int next_depot);

		std::vector<int> getClosedDepots(solution s);

		void depotsOperation();
		void updateSolutionWithGlobalDepots(SolverContext& ctx,solution *s);

		// insert solution on paretoSet.
		void insert_solution(solution s);

		// insert solution on vecSol
		void insert_vecSol(solution s);

		void clear_vecSol();

		// remove solution pointed with iterator from paretoSet.
		void erase_solution(std::vector<std::pair<bool, solution>>::iterator solIt);

		// return a solution from solID position from paretoSet
		solution get_solution(int solID);

		// return a solution not visited from paretoSet
		solution get_solution_not_visited();

		// return a random solution from paretoSet
		solution get_random_solution();

		// Insert sol at paretoSet if solution is non-dominated and remove all solutions dominated
		void update_paretoSet(std::map<int, std::vector<int>> mapEval, solution s);

		// return a map from  dominance evaluation of s with all other solution on paretoSet
		std::map<int, std::vector<int>> eval_solution(solution s);

		// Perform evaluation of solution and update paretoSet and return true if the solution is at paretoSet
		bool isAtParetoSet(solution s);

		// return iterator of  paretSet solution at solID position;
		std::vector<std::pair<bool, solution>>::iterator getSolIterator(int solID);
		// print all solution at paretoSet
		void print_paretoSet();

		// convert solution info into nodeSet at graph
		void solutionToNodesSet(solution s);

		bool HasSolutionNotVisited();

		Set PathToNodesSet(path p);

		void eval_VecSol();

		bool hasAllTargets(solution s);
		void checkTargetPath(int gid, path p);

		int getDepotsNumInit();

		int getTargetsNum();
		bool validation(solution s);
		std::vector<bool> paretoSetValidation();
		bool robotCapacity_val(path p);
		path RobotCostInPath(path p);

		bool checkRCFromNodeToNextDepot(path p, int node);

		bool checkRestrictions(solution s);
		bool PathRestrictions(path p);

		void ClearGurobiCallInfo()
		{
			vec_call.clear();
		}

		void ClearPredictionInfo()
		{
			vec_predictions.clear();
		}

		void SetCVLSubSetNum(int i)
		{
			cvl_subset_num = i;
		}

		bool best_prediction = true;

	};


#endif /* SRC_SOLUTION_H_ */
