/*
 * Solution.cpp
 *
 *  Created on: 26 de jul de 2018
 *      Author: rsantin
 */

#include "Solution.h"
#include <chrono>
#include <iomanip>

namespace std
{

	void findsubset(int n, double **sol, vector<vector<int>> *subsets);
	void DFS(double **g, int v, int n, bool *seen, vector<int> *subset, bool *inserted);

	class subtourelim : public GRBCallback
	{
	public:
		GRBVar **vars;
		GRBVar *vars_d;

		int n;
		int nd;
		int wm;
		subtourelim(GRBVar **xvars, int xn, GRBVar *yvars, int yn, int warm_start)
		{
			vars = xvars;
			n = xn;
			vars_d = yvars;
			nd = yn;
			wm = warm_start;
		}

	protected:
		void callback()
		{
			try
			{
				if (where == GRB_CB_MIPSOL)
				{
					// 1. Recupera solução atual
					// 2. Identifica subconjuntos desconectados (subtours)
					// 3. Para cada subconjunto sem a base, adiciona restrição lazy
					//    para forçar ligação com o restante do grafo
					// 4. Libera memória alocada

					// Found an integer feasible solution - does it visit every node?
					double **x = new double *[n];
					int i, j;
					for (i = 0; i < n; i++)
						x[i] = getSolution(vars[i], n);

					double *y = new double[nd];

					for (i = 0; i < nd; i++)
						y[i] = getSolution(vars_d[i]);

					vector<vector<int>> subsets;
					findsubset(n, x, &subsets);

					vector<int> depots_on_subset;

					// all strong connect components tha  does not contain the depot
					vector<vector<int>> subsets_S;

					// subset with base node
					vector<int> not_S;

					double obj_bnd = getDoubleInfo(GRB_CB_MIPSOL_OBJBND);
					// se não for warm start e obj_bnd não definido
					if (!(wm == 1 && obj_bnd < 0))
					{

						if (subsets.size() > 1)
						{

							vector<vector<int>> ordered_subsets;
							// inserire o subset que possui a base no final
							bool found_base = false;

							for (auto set : subsets)
							{
								// encontrar o grupo que contém a base
								for (auto vert : set)
								{
									if (vert == nd)
										found_base = true;
								}
								if (!found_base)
								{ // caso o subset não tenha a base insere no vetor
									subsets_S.emplace_back(set);
								}
								else
								{ // armazenar o subset que contém a base
									not_S.insert(not_S.end(), set.begin(), set.end());
									found_base = false;
								}
							}
							ordered_subsets.insert(ordered_subsets.begin(), subsets_S.begin(), subsets_S.end());

							// for(auto it_subset_a = subsets_S.begin(); it_subset_a !=subsets_S.end();it_subset_a++){

							ordered_subsets.push_back(not_S);

							// obter as arestas entre os subconjuntos S e o não S.
							for (auto it_subset_a = ordered_subsets.begin(); it_subset_a != ordered_subsets.end(); it_subset_a++)
							{

								if (it_subset_a == ordered_subsets.end() - 1)
									continue;

								depots_on_subset.clear();
								for (auto it_id = it_subset_a->begin(); it_id != it_subset_a->end(); it_id++)
								{
									if (*it_id < nd)
										depots_on_subset.emplace_back(*it_id);
								}

								if (depots_on_subset.empty())
									continue;

								GRBLinExpr expr = 0;
								for (auto it_subset_b = ordered_subsets.begin(); it_subset_b != ordered_subsets.end(); it_subset_b++)
								{
									if (it_subset_b == it_subset_a)
										continue;
									for (auto it_id_i = it_subset_a->begin(); it_id_i != it_subset_a->end(); it_id_i++)
									{
										i = *it_id_i;
										for (auto it_id_j = it_subset_b->begin(); it_id_j != it_subset_b->end(); it_id_j++)
										{
											j = *it_id_j;
											expr += vars[i][j];
										}
									}
								}
								// adicinar as restrições para cada depot em S
								for (int d : depots_on_subset)
									addLazy(expr >= y[d]);
							}
						}
					}

					for (int i = 0; i < n; i++)
						delete[] x[i];
					delete[] y;
				}
			}
			catch (GRBException &e)
			{
				cout << "Error number: " << e.getErrorCode() << endl;
				cout << e.getMessage() << endl;
			}
			catch (...)
			{
				cout << "Error during callback" << endl;
			}
		}
	};

	// busca em profundidade para encontrar os componentes conectados
	void DFS(double **g, int v, int n, bool *seen, vector<int> *subset, bool *inserted)
	{
		seen[v] = true;
		for (int u = 0; u < n; u++)
		{
			if (g[v][u] > 0.5)
			{
				if (!inserted[v])
				{
					subset->push_back(v);
					inserted[v] = true;
				}
				if (!seen[u])
				{
					DFS(g, u, n, seen, subset, inserted);
				}
			}
		}
	}

	// Dado uma solução inteira-viável'sol',
	// retorne os componetes conectados.
	void findsubset(int n,
					double **sol,
					vector<vector<int>> *subsets)
	{
		int i;

		subsets->clear();
		vector<int> subset;

		bool *node_seen = new bool[n];
		for (i = 0; i < n; i++)
			node_seen[i] = false;

		bool *inserted = new bool[n];
		for (i = 0; i < n; i++)
			inserted[i] = false;

		// encontrar os componetes conectados
		for (int v = 0; v < n; v++)
		{
			if (!node_seen[v])
				DFS(sol, v, n, node_seen, &subset, inserted);

			if (!subset.empty())
			{
				subsets->push_back(subset);
				subset.clear();
			}
		}
		delete[] node_seen;
		delete[] inserted;
	}

	/**
	 * @brief Calcula a solução inicial para todos os grupos de cobertura.
	 *
	 * - Encontra o melhor caminho (rota) para cada grupo usando bestPath().
	 * - Calcula algumas métricas (custo total, caminho mais custoso, tempo de execução).
	 * - Prepara a estrutura de solução inicial para otimização posterior.
	 *
	 * @param s Ponteiro para a estrutura solution que será preenchida com os resultados.
	 * @throws std::runtime_error Se bestPath() retornar um custo negativo (falha na construção).
	 * @note O vetor de depots não inclui a base, pois esta não sofrerá modificações.
	 */

	void Solution::initSol(solution *s)
	{
		// 1. Inicializa variáveis para rastrear:
		// - maxValue: maior custo individual de caminho (para identificar gargalos)
		// - maxTime: maior tempo de otimização do Gurobi (para definir limites)
		// - cost: custo acumulado de todos os caminhos

		double maxValue = numeric_limits<double>::min(); // Menor valor possível para comparação
		double maxTime = -1;
		double cost = 0;
		int maxCostPathID = 0;

		set<int> globalDepots; // set<int> para globalDepots para evitar duplicatas automaticamente

		// path p;
		vector<path> paths;
		solution solTemp;

		if (getNumberOfSets() == 0)
		{
			throw runtime_error("No coverage groups defined");
		}

		// 2. Para cada grupo de cobertura:
		// - Obtém o melhor caminho usando bestPath() (resolve subproblema via Gurobi)
		// - Armazena o caminho e atualiza métricas globais
		for (int i = 0; i < getNumberOfSets(); ++i)
		{
			path p = bestPath(i); // resolve a subrota via gurobi

			if (p.pCost < 0)
			{
				throw runtime_error("Failed to build path for group " + to_string(i));
			}

			paths.emplace_back(p); // Obter o melhor caminho para o grupo k

			globalDepots.insert(paths.back().depots.begin(), paths.back().depots.end()); // Coleta depots únicos (exceto base)

			cost += paths.back().pCost; // Somar o custo de cada grupo

			// Identificar o maior custo e o índice da rota
			if (paths.back().pCost > maxValue)
			{
				maxValue = paths.back().pCost;
				maxCostPathID = paths.back().pID;
			}

			// obter o maior tempo da solução inicial
			if (gurobi_optimize_time > maxTime)
				maxTime = gurobi_optimize_time;
		}

		// Atribuir como limite tempo o tempo do grupo que demandou mais tempo para executar
		gurobi_time_limit = maxTime;

		// solTemp.paths.insert(solTemp.paths.begin(), paths.begin(), paths.end());
		solTemp.paths = move(paths);

		// O vetor depots não inclui a base (não será modificada em shift/swap)
		solTemp.depots = move(globalDepots);
		solTemp.depotsNum = globalDepots.size() + 1; // +1 para incluir a base (não presente em globalDepots)
		solTemp.maxCost = maxValue;
		solTemp.sCost = cost;
		solTemp.maxCostPathID = maxCostPathID;
		*s = solTemp;
	}

	Solution::~Solution()
	{
		// TODO Auto-generated destructor stub
	}

	// find best path for graph gID, returning Solution
	//  path's cost and the edges of path.
	Solution::path Solution::bestPath(int gID)
	{
		int T, D, robotID, setID;
		int subset_id = 0;
		std::chrono::time_point<std::chrono::system_clock> start, end;
		gurobi_call call_info;

		path sol;

		solution vec_sol;
		vec_sol.targetsNum = 0;
		Set set_temp;
		set<int> depots;

		T = nodesSets[gID].cvLines.size() * 2;
		D = nodesSets[gID].depots.size();
		// N = T+D;

		robotID = nodesSets[gID].robotID;
		setID = nodesSets[gID].set_id;

		auto it_sub_set = nodesSets[gID].sub_set.begin();

		if (it_sub_set == nodesSets[gID].sub_set.end())
		{
			set_temp = nodesSets[gID];

			sol = MILP(set_temp);

			// muitas chamadas do gurobi, manteremos apenas a informação da última
			// vec_call.clear();

			call_info.call_id = ++call_num;
			call_info.T = T;
			call_info.D = D;
			call_info.optimize_time = gurobi_optimize_time;
			call_info.type = "normal";
			if (sol.pCost > 0)
				call_info.feasible = true;
			else
				call_info.feasible = false;

			vec_call.emplace_back(call_info);
		}
		else
		{

			while (it_sub_set != nodesSets[gID].sub_set.end())
			{

				// passar as informações de um sub_set como um grupo
				set_temp.set_id = setID;
				set_temp.robotID = robotID;
				set_temp.cvLines = it_sub_set->cvLines;
				set_temp.depots = it_sub_set->depots;

				// calcular a solução
				sol = MILP(set_temp);

				// muitas chamadas do gurobi, materemos apenas a informação da última
				// vec_call.clear();
				call_info.call_id = ++call_num;
				call_info.T = set_temp.cvLines.size() * 2;
				call_info.D = set_temp.depots.size() + 1;
				call_info.optimize_time = gurobi_optimize_time;
				call_info.type = "normal";

				if (sol.pCost > 0)
					call_info.feasible = true;
				else
					call_info.feasible = false;

				// adicionar as informções da chamada do gurobi no vetor
				vec_call.emplace_back(call_info);

				if (sol.pCost < 0)
					return sol;

				vec_sol.paths.emplace_back(sol);
				vec_sol.targetsNum += sol.targetsNum;

				subset_id++;
				++it_sub_set;
			}

			sol = Union_Solutions(vec_sol);

			if (sol.pCost < 0)
				return sol;

			// covert path to nodeSets
			// set_temp = PathToNodesSet(sol);
			set_temp = nodesSets[gID];

			// pass to gurobi as warm start
			sol = MILP_Warm_Start(set_temp, sol);

			// vec_call.clear();
			call_info.call_id = ++call_num;
			call_info.T = set_temp.cvLines.size() * 2;
			call_info.D = set_temp.depots.size() + 1;
			call_info.optimize_time = gurobi_optimize_time;
			call_info.type = "Warm_Start";
			if (sol.pCost > 0)
				call_info.feasible = true;
			else
				call_info.feasible = false;

			vec_call.emplace_back(call_info);
		}
		//---------------------teste temporário saída -------------------------------------
		/*if(!PathRestrictions(sol))
					cout << "solução não validada:Best Path" <<endl;*/

		return sol;
	}

	// find best path for graph gID, returning Solution
	//  path's cost and the edges of path.
	Solution::path Solution::improvePath(path p)
	{
		std::chrono::time_point<std::chrono::system_clock> start, end;

		gurobi_call call_info;
		path sol;

		solution vec_sol;
		vec_sol.targetsNum = 0;
		Set set_temp;
		set<int> depots;

		int gID = p.pID;

		// setID = nodesSets[gID].set_id;
		set_temp = nodesSets[gID];

		// pass to gurobi as warm start
		sol = MILP_Warm_Start(set_temp, p);

		call_info.call_id = ++call_num;
		call_info.T = set_temp.cvLines.size() * 2;
		call_info.D = set_temp.depots.size() + 1;
		call_info.optimize_time = gurobi_optimize_time;
		call_info.type = "Warm_Start";
		if (sol.pCost > 0)
			call_info.feasible = true;
		else
			call_info.feasible = false;

		vec_call.emplace_back(call_info);

		//---------------------teste temporário saída -------------------------------------
		// if (!PathRestrictions(sol))
		// cout << "solução não validada:Best Path" << endl;

		return sol;
	}

	Solution::path Solution::MILP_Warm_Start(Set nodes_set, path initial_sol)
	{
		int i, j, T, D;
		int baseId = 0;
		std::chrono::time_point<std::chrono::system_clock> start, end;

		path sol;
		sol.pID = -1;
		// colocar as informações do node_set no formato da entrada da programação inteira, chamada de coverage_set
		Convert_NS_to_CS(nodes_set);

		T = getTargetNum(); // set of targets
		D = getDepotNum();	// fueling depots

		const int N = T + D; // total nodes

		int robotID = nodes_set.robotID;

		// base position on coverageSet;
		baseId = getBaseID();

		try
		{
			// GRBEnv env = GRBEnv();
			GRBModel model = GRBModel(env);
			model.set(GRB_IntParam_OutputFlag, 0);
			model.set(GRB_IntParam_LazyConstraints, 1);

			// model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
			GRBVar *Elem = new GRBVar[2];

			Elem[0] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Pmax");
			Elem[1] = model.addVar(0, D - 1, 0.0, GRB_INTEGER, "Dmax");

			Elem[0].set(GRB_StringAttr_VarName, "vars_Pmax");
			Elem[0].set(GRB_CharAttr_VType, GRB_CONTINUOUS);

			Elem[1].set(GRB_StringAttr_VarName, "vars_Dmin");
			Elem[1].set(GRB_CharAttr_VType, GRB_INTEGER);

			GRBLinExpr objn = 0;

			// setar e configurar o objetivo 0
			objn = Elem[0];
			model.setObjectiveN(objn, 0, 0, 0.9999);

			// setar e configurar o objetivo 1
			objn = Elem[1];
			model.setObjectiveN(objn, 1, 0, 0.0001);

			/***************** Objective Function *****************/
			model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);

			model.update();

			GRBVar **vars_x = NULL;
			vars_x = new GRBVar *[N];
			for (i = 0; i < N; i++)
				vars_x[i] = new GRBVar[N];

			GRBVar *vars_d = nullptr;
			if (D > 1)
				vars_d = new GRBVar[D - 1];

			// Create decision variables x_ijk for all edges except self-loops (i != j)
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{
					//if (i != j)
					vars_x[i][j] = model.addVar(0, 1, 0, GRB_BINARY, "x_i_" + itos(i) + "_j_" + itos(j));
				}
			}
			// Create decision variables z_ij for all edges (including self-loops)
			GRBVar **vars_z = new GRBVar *[N];
			for (i = 0; i < N; i++)
				vars_z[i] = new GRBVar[N];
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{
					vars_z[i][j] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_i" + itos(i) + "_j_" + itos(j));
				}
			}
			// Create decision variables d_i for depots (D-1 depots, excluding base)
			for (i = 0; i < D - 1; i++)
			{
				vars_d[i] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "d_" + itos(i));
			}

			/******************** Constraints *****************/
			// Constraint 1: SUM_{i,j} (1 + q_k) * c_ij * x_ij <= Pmax, for all nodes i, j (robotID is fixed)
			GRBLinExpr rest1 = 0;
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{
					if (i != j)
					{
						double cost_ij = getCost(i, j);
						rest1 += (cost_ij * vars_x[i][j]) + (input.getRobotProp(robotID) * cost_ij * vars_x[i][j]);
					}
				}
			}
			string s = "Rest1";
			model.addConstr(rest1 <= Elem[0], s);

			/**************** Degree Constraints ***********************/
			// Constraint 2: SUM x_di = SUM x_id para qualquer d em D diferente da base
			//  From any target(i) there is only one robot(k) going to the vertice(j)
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base
				GRBLinExpr rest2_1 = 0;
				GRBLinExpr rest2_2 = 0;
				for (i = 0; i < N; i++)
				{
					if (i != d)
					{
						rest2_1 += vars_x[d][i];
						rest2_2 += vars_x[i][d];
					}
				}
				string s = "Rest2_d_" + itos(d);
				model.addConstr(rest2_1 == rest2_2, s);
			}

			// Constraint 3a: SUM x_id >= y_d para qualquer d em D diferente da base
			// Constraint 3b: SUM x_di >= y_d * N para qualquer d em D diferente da base
			//  N = (0.5*(D*(D-1))+T) quantidade de ligações entre depots + target e depots
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base
				GRBLinExpr rest3_b = 0;
				for (i = 0; i < N; i++) // para target
					rest3_b += vars_x[i][d];

				string s = "Rest3_a_d_" + itos(d);
				model.addConstr(rest3_b >= vars_d[d], s);
				s = "Rest3_b_d_" + itos(d);
				model.addConstr(rest3_b <= (0.5 * (D * (D - 1)) + D * T) * vars_d[d], s);
				// model.addConstr(rest3_b <= N*vars_d[d], s);
			}

			// Constraint 4: SUM x_di <= y_d para qualquer d em D diferente da base
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base
				string s = "Rest4_a_" + itos(d);
				model.addConstr(0 <= vars_d[d], s);

				s = "Rest4_b_" + itos(d);
				model.addConstr(vars_d[d] <= 1, s);
			}

			// Constraint 5: [SUM x_id0 == m)
			GRBLinExpr rest5 = 0;
			for (i = 0; i < N; i++)
				if (i != baseId)
					rest5 += vars_x[i][baseId];
			s = "Rest5";
			model.addConstr(rest5 == 1, s);

			// Constraint 6: [SUM x_d0i == 1]
			GRBLinExpr rest6 = 0;
			for (i = 0; i < N; i++)
				if (i != baseId)
					rest6 += vars_x[baseId][i];
			s = "Rest6";
			model.addConstr(rest6 == 1, s);

			// Constraint 7: SUM x_ij = 1 i em V e j em T
			for (int j = D; j < N; j++)
			{ // cada target
				GRBLinExpr rest7 = 0;
				for (i = 0; i < N; i++)
					if (i != j)
						rest7 += vars_x[i][j];
				string s = "Rest7_j_" + itos(j);
				model.addConstr(rest7 == 1, s);
			}

			// Constraint 8: SUM x_ji = 1 i em V e j em T
			for (int j = D; j < N; j++)
			{ // cada target
				GRBLinExpr rest8 = 0;
				for (i = 0; i < N; i++)
					if (i != j)
						rest8 += vars_x[j][i];

				string s = "Rest8_j_" + itos(j);
				model.addConstr(rest8 == 1, s);
			}

			//-----------------------Fuel restriction---------------------------------------------
			// Constraint 9: SUM z_ij - SUM z_ji = SUM f_ij * x_ij, sendo i  qualquer target
			for (i = D; i < N; i++)
			{
				GRBLinExpr rest9_1 = 0;
				GRBLinExpr rest9_2 = 0;
				GRBLinExpr rest9_3 = 0;

				for (j = 0; j < N; j++)
				{
					if (i != j)
					{
						rest9_1 += vars_z[i][j];
						rest9_2 += vars_z[j][i];
						rest9_3 += getCost(i, j) * vars_x[i][j];
					}
				}
				string s = "Rest9_i" + itos(i);
				model.addConstr(rest9_1 - rest9_2 == rest9_3, s);
			}

			// constraint 10: z_di = f_di * x_di , sendo i qualquer target e d em D
			// no artigo está definido  de depot para target, isso exclui a limitação da aresta no caso de depot para depot
			// por isso alteramos o índice i para empregar qualquer vértice
			for (i = 0; i < N; ++i)
			{
				GRBLinExpr rest10_1 = 0;
				GRBLinExpr rest10_2 = 0;

				for (int d = 0; d < D; d++)
				{
					if (i != d)
					{
						rest10_1 = vars_z[d][i];
						rest10_2 = getCost(d, i) * vars_x[d][i];
						string s = "Rest10_d" + itos(d) + "_i_" + itos(i);
						model.addConstr(rest10_1 == rest10_2, s);
					}
				}
			}
			// constraint 11  z_i_j <(F-t_j)x_ij para qualquer j em T, (i,j) em E
			for (i = 0; i < N; i++)
			{
				GRBLinExpr rest11_1 = 0;
				GRBLinExpr rest11_2 = 0;
				for (j = D; j < N; j++)
				{
					if (i != j)
					{
						rest11_1 = vars_z[i][j];
						rest11_2 = (input.getRobotFuel(robotID) - get_min_fuel_2_depot(j)) * vars_x[i][j];
						s = "Rest11_i_" + itos(i) + "_j_" + itos(j);
						model.addConstr(rest11_1 <= rest11_2, s);
					}
				}
			}
			// constraint 12  z_i_d <(F*x_ij) para qualquer i em V, d em D
			for (i = 0; i < N; i++)
			{
				GRBLinExpr rest12_1 = 0;
				GRBLinExpr rest12_2 = 0;
				for (int d = 0; d < D; d++)
				{
					if (i != d)
					{
						rest12_1 = vars_z[i][d];
						rest12_2 = input.getRobotFuel(robotID) * vars_x[i][d];
						s = "Rest12_i_" + itos(i) + "_d_" + itos(d);
						model.addConstr(rest12_1 <= rest12_2, s);
					}
				}
			}

			// constraint 13  z_i_j >= (s_i + f_ij)*x_ij para qualquer i em T, (i,j) em E
			for (i = D; i < N; i++)
			{
				GRBLinExpr rest13_1 = 0;
				GRBLinExpr rest13_2 = 0;
				for (j = 0; j < N; j++)
				{
					if (i != j)
					{
						rest13_1 = vars_z[i][j];
						rest13_2 = (get_min_fuel_2_depot(i) + getCost(i, j)) * vars_x[i][j];
						s = "Rest14_i_" + itos(i) + "_j_" + itos(j);
						model.addConstr(rest13_1 >= rest13_2, s);
					}
				}
			}

			//******************* Coverage Constraints **************************/
			// constraint 14: sum X_i_i+1_k + sum X_i+1_i_k = 1, i=D,D+2,...N-1
			for (i = D; i < N - 1; i = i + 2)
			{
				GRBLinExpr rest14 = 0;
				rest14 += vars_x[i][i + 1] + vars_x[i + 1][i];
				string s = "Rest14_i_" + itos(i);
				model.addConstr(rest14 == 1, s);
			}
			// constraint 15: sum X_i_i+1_k = Sum Sum X_i_j_k, i = 2,4,N
			for (i = D; i < N; i = i + 2)
			{
				GRBLinExpr rest15_1 = 0;
				GRBLinExpr rest15_2 = 0;
				rest15_1 += vars_x[i][i + 1];
				for (j = D + 1; j < N; j = j + 2)
					if (i != j)
						rest15_2 += vars_x[i][j];

				string s = "Rest15_i_" + itos(i);
				model.addConstr(rest15_1 == rest15_2, s);
			}

			// constraint 16: sum X_i_i-1_k = Sum Sum X_i_j_k, i =
			for (i = D + 1; i < N; i = i + 2)
			{
				GRBLinExpr rest16_1 = 0;
				GRBLinExpr rest16_2 = 0;
				rest16_1 += vars_x[i][i - 1];
				for (j = D; j < N; j = j + 2)
					if (i != j)
						rest16_2 += vars_x[i][j];

				string s = "Rest16_i_" + itos(i);
				model.addConstr(rest16_1 == rest16_2, s);
			}
			/********************Depot Constraint ***********************************************/
			// Contraint 17: Dmin ==SumD[j]
			GRBLinExpr rest17 = 0;
			for (j = 0; j < D - 1; j++)
			{ // vertice
				rest17 += vars_d[j];
			}
			// model.addConstr(rest17 <= var_d, s);
			model.addConstr(rest17 <= Elem[1], "Rest17");

			/******************************************************************************************/
			// initial solution
			vector<vector<GRBVar>> vars_x_temp(N, vector<GRBVar>(N));
			for (i = 0; i < N; i++)
				for (j = 0; j < N; j++)
				{
					if (i != j)
						vars_x_temp[i][j] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "x_temp_" + itos(i) + "_" + itos(j));
				}
			double fuel;
			vector<vector<double>> start_vals(N, vector<double>(N, 0.0)); // todos 0 inicialmente

			for (auto edge : initial_sol.edges)
			{
				// Verifica se os nós do edge existem no mapa de IDs
				auto it_a = map_cvset_id_to_node_id.find(edge.node_a);
				auto it_b = map_cvset_id_to_node_id.find(edge.node_b);

				// Se algum dos nós não estiver no mapa, ignora esse edge
				if (it_a == map_cvset_id_to_node_id.end() || it_b == map_cvset_id_to_node_id.end())
					continue;

				i = it_a->second;
				j = it_b->second;

				// Evita a criação de variáveis com loop (i == j), que não foram adicionadas ao modelo
				if (i == j)
					continue;

				// Marca a variável de decisão (i,j) como parte da solução inicial
				start_vals[i][j] = 1.0;

				// Se o nó de chegada for um target (j >= D), define a dica de valor para variável z
				if (j >= D)
				{
					// Verifica se existe informação de combustível associada ao target
					auto fuelIt = initial_sol.fuelOnTarget.find(edge.node_b);
					if (fuelIt != initial_sol.fuelOnTarget.end())
					{
						// Calcula o combustível restante no target e define como dica (hint) para z_ij
						fuel = input.getRobotFuel(robotID) - fuelIt->second;
						vars_z[i][j].set(GRB_DoubleAttr_VarHintVal, fuel);
					}
				}
			}

			for (i = 0; i < N; i++)
				for (j = 0; j < N; j++)
				{
					if (i != j)
						vars_x[i][j].set(GRB_DoubleAttr_Start, start_vals[i][j]);
				}

			for (i = 0; i < D - 1; i++)
				vars_d[i].set(GRB_DoubleAttr_VarHintVal, 0.0);

			for (auto depot : initial_sol.depots)
			{
				i = map_cvset_id_to_node_id.find(depot)->second;
				vars_d[i].set(GRB_DoubleAttr_VarHintVal, 1.0);
			}

			/******************************************************************************************/
			// model.set(GRB_IntParam_StartNodeLimit,0);
			// model.set(GRB_IntParam_Method, GRB_METHOD_BARRIER);
			// model.set(GRB_IntParam_StartNodeLimit,2000000000);

			// model.set(GRB_DoubleParam_Heuristics,0);

			model.update();

			model.set(GRB_IntParam_LogToConsole, 0);
			model.set(GRB_DoubleParam_MIPGap, 0.01);

			// desabilitado para o cut off no callback;
			// model.set(GRB_IntParam_PreCrush,1);

			subtourelim cb = subtourelim(vars_x, N, vars_d, D - 1, 1);
			model.setCallback(&cb);

			start = std::chrono::system_clock::now();
			model.optimize();
			end = std::chrono::system_clock::now();

			// obter o tempo necessário para otimizar a rota;
			gurobi_optimize_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

			model.update();

			int optimstatus = model.get(GRB_IntAttr_Status);

			if (optimstatus == GRB_OPTIMAL)
			{
				sol.robotID = robotID;
				sol.pID = nodes_set.set_id;
				sol.depotsNum = 0;
				sol.targetsNum = 0;

				double fcost = 0;
				double time = 0;
				double total_path_cost = 0.0;

				vector<pair<int, pair<pair<int, int>, pair<double, double>>>> output;
				vector<pair<int, edge>> grb_out;
				edge e;

				int commodity = 0;

				double v_x = 0;

				// set the path to the solution
				for (i = 0; i < N; i++)
				{
					for (j = 0; j < N; j++)
					{
						if (i == j)
							continue;

						v_x = vars_x[i][j].get(GRB_DoubleAttr_X);

						if (v_x >= 0.99)
						{
							double v_z = vars_z[i][j].get(GRB_DoubleAttr_X);
							int index_i = getIndex(i);
							int index_j = getIndex(j);
							int commodity = static_cast<int>(v_z);

							// não inserimos a base que é referenciada pelo índice D
							if (j >= D)
							{
								sol.fuelOnTarget.emplace(getIndex(j), input.getRobotFuel(robotID) - v_z);
								sol.targetsNum++;
							}

							if (i < D - 1)
							{
								int id = getIndex(i);

								if (mapNodesTypes[id] != 0)
									cerr << "problem warm !\n";
								sol.depots.insert(getIndex(i));
							}

							time = getCost(i, j);
							fcost = time + (input.getRobotProp(robotID) * time);

							e.cost = fcost;
							e.time = time;
							e.node_a = index_i;
							e.node_b = index_j;
							total_path_cost += e.cost;

							if (e.node_a == e.node_b)
							{
								cout << "Loop detected!\n";
							}

							// Arredonda a quantidade de vezes que a aresta é percorrida
							int n_arcs = static_cast<int>(round(v_x));

							// insere a quantidade de vezes que a aresta é percorrida
							for (int x = 0; x < n_arcs; ++x)
								grb_out.emplace_back(make_pair(commodity, e));
						}
					}
				}

				// Atribui o custo total da solução, calculado como a soma dos custos de cada aresta do caminho.
				sol.pCost = total_path_cost;

				// ordernar pelo valor da commodity do maior para o menor
				sort(grb_out.begin(), grb_out.end(),
					 [](pair<int, edge> el_1, pair<int, edge> el_2)
					 {
						 return el_1.first > el_2.first;
					 });

				// inserir o caminho na solução
				for (const auto &itvec : grb_out)
					sol.edges.emplace_back(itvec.second);

				// caso o caminho tenha aresta fora de ordem, não configurando um circuito, ordenar
				// isso pode ocorrer em depots, visto a possíbilidade de loops, ordenar os diversos loops que passam pelos depots.
				// if(!IsCircuit(sol))
				SortMultiplesAdjacentOnDepots(&sol);

				//----------------------------------------- teste temporário para verificar se a capacidade do robô é mantida
				if (!PathRestrictions(sol))
				{
					cout << "solução não validada: warm_start" << endl;
					sol.pCost = -1;
					sol.robotID = -1;
					sol.pID = -1;
					return sol;
				}
				//--------------------------------------------------

				// contabilizar o base do robô
				sol.depotsNum = sol.depots.size() + 1;
			}
			else
			{ // se for inviável
				sol.pCost = -1;
				sol.robotID = -1;
				sol.pID = -1;
				//	model.computeIIS();
				//	model.write("model.ilp");
				//	model.write("model.lp");
			}
			for (int i = 0; i < N; i++)
				delete[] vars_x[i];
			delete[] vars_x;
			delete[] vars_d;
			for (int i = 0; i < N; i++)
				delete[] vars_z[i];
			delete[] vars_z;
		}
		catch (GRBException &e)
		{
			cout << "Error number: " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
		}
		catch (...)
		{
			cout << "Error during optimization" << endl;
		}

		return sol;
	}

	Solution::path Solution::MILP(Set nodes_set)
	{
		int i, j, T, D, N;
		int baseId = 0;
		std::chrono::time_point<std::chrono::system_clock> start, end;

		path sol;
		sol.pID = -1;
		// colocar as informações do node_set no formato da entrada da programação inteira, chamada de coverage_set

		Convert_NS_to_CS(nodes_set);

		T = getTargetNum(); // set of targets
		D = getDepotNum();	// fueling depots

		N = T + D; // total nodes

		int robotID = nodes_set.robotID;

		// base position on coverageSet;
		baseId = getBaseID();

		try
		{
			// GRBEnv env = GRBEnv();

			GRBModel model = GRBModel(env);
			model.set(GRB_IntParam_OutputFlag, 0);

			// Must set LazyConstraints parameter when using lazy constraints
			model.set(GRB_IntParam_LazyConstraints, 1);

			// model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
			GRBVar *Elem = 0;

			Elem = model.addVars(2);

			Elem[0] = model.addVar(0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "Pmax");
			Elem[1] = model.addVar(0, D - 1, 0.0, GRB_INTEGER, "Dmax");

			Elem[0].set(GRB_StringAttr_VarName, "vars_Pmax");
			Elem[0].set(GRB_CharAttr_VType, GRB_CONTINUOUS);

			Elem[1].set(GRB_StringAttr_VarName, "vars_Dmin");
			Elem[1].set(GRB_CharAttr_VType, GRB_INTEGER);

			GRBLinExpr objn = 0;

			// setar e configurar o objetivo 0
			objn = Elem[0];
			model.setObjectiveN(objn, 0, 0, 0.9999);

			// setar e configurar o objetivo 1
			objn = Elem[1];
			model.setObjectiveN(objn, 1, 0, 0.0001);

			/***************** Objective Function *****************/
			model.set(GRB_IntAttr_ModelSense, GRB_MINIMIZE);
			model.update();

			GRBVar **vars_x = NULL;
			vars_x = new GRBVar *[N];
			for (i = 0; i < N; i++)
				vars_x[i] = new GRBVar[N];

			GRBVar *vars_d = new GRBVar[D - 1];

			// vars_x[i][j] = model.addVar(0, 1, 0, GRB_BINARY, "x_i_"+itos(i)+"_j_"+itos(j));
			//  Create decision variables x_ijk for all edges ...alteração do upper bound de T para 1
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{ // depot to depot
					vars_x[i][j] = model.addVar(0, 1, 0, GRB_BINARY, "x_i_" + itos(i) + "_j_" + itos(j));
				}
			}

			vector<vector<GRBVar>> vars_z(N, vector<GRBVar>(N));
			// GRBVar vars_z[N][N];
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{
					// vars_z[i][j] = model.addVar(-GRB_INFINITY, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_i" + itos(i) + "_j_" + itos(j));
					vars_z[i][j] = model.addVar(0.0, GRB_INFINITY, 0.0, GRB_CONTINUOUS, "z_i" + itos(i) + "_j_" + itos(j));
				}
			}

			// GRBVar vars_d[D-1];
			for (i = 0; i < D - 1; i++)
			{
				vars_d[i] = model.addVar(0.0, 1.0, 0, GRB_BINARY, "d_" + itos(i));
			}
			/******************** Constraints *****************/
			// Constraint 1: SUM SUM(1+ q_k)c_ij * x_ijk <= Pmax  k=0, ..., M-1
			GRBLinExpr rest1 = 0;
			for (i = 0; i < N; i++)
			{
				for (j = 0; j < N; j++)
				{
					rest1 += (getCost(i, j) * vars_x[i][j]) + (input.getRobotProp(robotID) * getCost(i, j) * vars_x[i][j]);
				}
			}
			string s = "Rest1";
			model.addConstr(rest1 <= Elem[0], s);

			/**************** Degree Constraints ***********************/
			// Constraint 2: SUM x_di = SUM x_id para qualquer d em D diferente da base
			//  From any target(i) there is only one robot(k) going to the vertice(j)
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base
				GRBLinExpr rest2_1 = 0;
				GRBLinExpr rest2_2 = 0;
				for (i = 0; i < N; i++)
				{
					rest2_1 += vars_x[d][i];
					rest2_2 += vars_x[i][d];
				}
				string s = "Rest2_d_" + itos(d);
				model.addConstr(rest2_1 == rest2_2, s);
			}

			// Constraint 3a: SUM x_id >= y_d para qualquer d em D diferente da base
			// Constraint 3b: SUM x_di >= y_d * N para qualquer d em D diferente da base
			//  N = (0.5*(D*(D-1))+T) quantidade de ligações entre depots + target e depots
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base
				GRBLinExpr rest3_b = 0;
				for (i = 0; i < N; i++)
				{
					rest3_b += vars_x[i][d];
				}
				string s = "Rest3_a_d_" + itos(d);
				model.addConstr(rest3_b >= vars_d[d], s);
				s = "Rest3_b_d_" + itos(d);
				// model.addConstr(rest3_b <= N*vars_d[d], s);
				model.addConstr(rest3_b <= (0.5 * (D * (D - 1)) + D * T) * vars_d[d], s);
			}

			// Constraint 4: SUM x_di <= y_d para qualquer d em D diferente da base
			for (int d = 0; d < D - 1; d++)
			{ // apenas depots sem a base

				string s = "Rest4_a_" + itos(d);
				model.addConstr(0 <= vars_d[d], s);

				s = "Rest4_b_" + itos(d);
				model.addConstr(vars_d[d] <= 1, s);
			}

			// Constraint 5: [SUM x_id0 == m)
			GRBLinExpr rest5 = 0;
			for (i = 0; i < N; i++)
			{
				rest5 += vars_x[i][baseId];
			}
			s = "Rest5";
			model.addConstr(rest5 == 1, s);

			// Constraint 6: [SUM x_d0i == 1]
			GRBLinExpr rest6 = 0;
			for (i = 0; i < N; i++)
			{
				rest6 += vars_x[baseId][i];
			}
			s = "Rest6";
			model.addConstr(rest6 == 1, s);

			// Constraint 7: SUM x_ij = 1 i em V e j em T
			for (int j = D; j < N; j++)
			{ // cada target
				GRBLinExpr rest7 = 0;
				for (i = 0; i < N; i++)
				{
					rest7 += vars_x[i][j];
				}
				string s = "Rest7_j_" + itos(j);
				model.addConstr(rest7 == 1, s);
			}

			// Constraint 8: SUM x_ji = 1 i em V e j em T
			for (int j = D; j < N; j++)
			{ // cada target
				GRBLinExpr rest8 = 0;
				for (i = 0; i < N; i++)
				{
					rest8 += vars_x[j][i];
				}
				string s = "Rest8_j_" + itos(j);
				model.addConstr(rest8 == 1, s);
			}

			// Constraint 9
			GRBLinExpr rest9 = 0;
			for (int i = 0; i < N; i++)
			{ // cada target
				// skip self_loop
				rest9 += vars_x[i][i];
			}
			model.addConstr(rest9 == 0, "Rest9");

			// Constraint 10: SUM z_ij - SUM z_ji = SUM f_ij * x_ij, sendo i  qualquer target
			for (i = D; i < N; i++)
			{
				GRBLinExpr rest10_1 = 0;
				GRBLinExpr rest10_2 = 0;
				GRBLinExpr rest10_3 = 0;

				for (j = 0; j < N; j++)
				{
					rest10_1 += vars_z[i][j];
					rest10_2 += vars_z[j][i];
					rest10_3 += getCost(i, j) * vars_x[i][j];
				}
				string s = "Rest10_i" + itos(i);
				model.addConstr(rest10_1 - rest10_2 == rest10_3, s);
			}

			// constraint 11: z_di = f_di * x_di , sendo i qualquer target e d em D
			// no artigo está definido  de depot para target, isso exclui a limitação da aresta no caso de depot para depot
			// por isso alteramos o índice i para empregar qualquer vértice
			for (i = 0; i < N; ++i)
			{
				GRBLinExpr rest11_1 = 0;
				GRBLinExpr rest11_2 = 0;

				for (int d = 0; d < D; d++)
				{
					rest11_1 = vars_z[d][i];
					rest11_2 = getCost(d, i) * vars_x[d][i];
					string s = "Rest11_d" + itos(d) + "_i_" + itos(i);
					model.addConstr(rest11_1 == rest11_2, s);
				}
			}

			//------------------------------------------------------------------------------
			// constraint 12  z_i_j <(F-t_j)x_ij para qualquer j em T, (i,j) em E
			for (i = 0; i < N; i++)
			{
				GRBLinExpr rest12_1 = 0;
				GRBLinExpr rest12_2 = 0;
				for (j = D; j < N; j++)
				{

					rest12_1 = vars_z[i][j];
					rest12_2 = (input.getRobotFuel(robotID) - get_min_fuel_2_depot(j)) * vars_x[i][j];
					s = "Rest12_i_" + itos(i) + "_j_" + itos(j);
					model.addConstr(rest12_1 <= rest12_2, s);
				}
			}

			// constraint 13  z_i_d <(F*x_ij) para qualquer i em V, d em D
			for (i = 0; i < N; i++)
			{
				GRBLinExpr rest13_1 = 0;
				GRBLinExpr rest13_2 = 0;
				for (int d = 0; d < D; d++)
				{
					rest13_1 = vars_z[i][d];
					rest13_2 = input.getRobotFuel(robotID) * vars_x[i][d];
					s = "Rest13_i_" + itos(i) + "_d_" + itos(d);
					model.addConstr(rest13_1 <= rest13_2, s);
				}
			}

			// constraint 14  z_i_j >= (s_i + f_ij)*x_ij para qualquer i em T, (i,j) em E
			for (i = D; i < N; i++)
			{
				GRBLinExpr rest14_1 = 0;
				GRBLinExpr rest14_2 = 0;

				for (j = 0; j < N; j++)
				{
					rest14_1 = vars_z[i][j];
					rest14_2 = (get_min_fuel_2_depot(i) + getCost(i, j)) * vars_x[i][j];

					s = "Rest14_i_" + itos(i) + "_j_" + itos(j);
					model.addConstr(rest14_1 >= rest14_2, s);
				}
			}

			//******************* Coverage Constraints **************************/
			// constraint 15: sum X_i_i+1_k + sum X_i+1_i_k = 1, i=D,D+2,...N-1
			for (i = D; i < N; i = i + 2)
			{
				GRBLinExpr rest15 = 0;
				rest15 += vars_x[i][i + 1] + vars_x[i + 1][i];
				string s = "Rest15_i_" + itos(i);
				model.addConstr(rest15 == 1, s);
			}
			// constraint 16: sum X_i_i+1_k = Sum Sum X_i_j_k, i = 2,4,N
			for (i = D; i < N; i = i + 2)
			{
				GRBLinExpr rest16_1 = 0;
				GRBLinExpr rest16_2 = 0;
				rest16_1 += vars_x[i][i + 1];
				for (j = D + 1; j < N; j = j + 2)
					rest16_2 += vars_x[i][j];

				string s = "Rest16_i_" + itos(i);
				model.addConstr(rest16_1 == rest16_2, s);
			}

			// constraint 17: sum X_i_i-1_k = Sum Sum X_i_j_k, i =
			for (i = D + 1; i < N; i = i + 2)
			{
				GRBLinExpr rest17_1 = 0;
				GRBLinExpr rest17_2 = 0;
				rest17_1 += vars_x[i][i - 1];
				for (j = D; j < N; j = j + 2)
					rest17_2 += vars_x[i][j];

				string s = "Rest17_i_" + itos(i);
				model.addConstr(rest17_1 == rest17_2, s);
			}
			/********************Depot Constraint ***********************************************/
			// Contraint 18: Dmin ==SumD[j]
			GRBLinExpr rest18 = 0;
			for (j = 0; j < D - 1; j++)
			{ // vertice
				rest18 += vars_d[j];
			}
			// model.addConstr(rest18 == Elem[1], "Rest18");
			model.addConstr(rest18 <= Elem[1], "Rest18");

			/******************************************************************************************/

			// for(i =0;i<N;++i)
			// vars_x[i][i].set(GRB_DoubleAttr_UB,0);//
			model.set(GRB_IntParam_LogToConsole, 0);
			model.set(GRB_DoubleParam_MIPGap, 0.01);

			subtourelim cb = subtourelim(vars_x, N, vars_d, D - 1, 0);
			model.setCallback(&cb);

			model.update();

			start = std::chrono::system_clock::now();
			model.optimize();

			end = std::chrono::system_clock::now();
			// obter o tempo necessário para otimizar a rota;
			gurobi_optimize_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

			model.update();

			int optimstatus = model.get(GRB_IntAttr_Status);

			if (optimstatus == GRB_OPTIMAL)
			{

				// set cost and robot to the solution
				sol.pCost = Elem[0].get(GRB_DoubleAttr_X);
				sol.robotID = robotID;
				sol.pID = nodes_set.set_id;
				sol.depotsNum = Elem[1].get(GRB_DoubleAttr_X);
				sol.targetsNum = 0;

				double fcost = 0;
				double time = 0;
				vector<pair<int, pair<pair<int, int>, pair<double, double>>>> output;
				vector<pair<int, edge>> grb_out;
				edge e;

				int commodity = 0;

				// set the path to the solution
				for (i = 0; i < N; i++)
				{

					// insere a quantidade de combustível no target i
					for (j = 0; j < N; j++)
					{
						if (vars_x[i][j].get(GRB_DoubleAttr_X) >= 0.98)
						{

							if (j >= D)
							{
								sol.fuelOnTarget.emplace(getIndex(j), input.getRobotFuel(robotID) - vars_z[i][j].get(GRB_DoubleAttr_X));
							}
							commodity = vars_z[i][j].get(GRB_DoubleAttr_X);
							// não inserimos a base que é referenciada pelo índice D
							if (i < D - 1)
							{
								sol.depots.insert(getIndex(i));
							}
							else if (i >= D)
							{
								sol.targetsNum++;
							}

							time = getCost(i, j);
							fcost = time + (input.getRobotProp(robotID) * time);

							e.cost = fcost;
							e.time = time;
							e.node_a = getIndex(i);
							e.node_b = getIndex(j);

							grb_out.emplace_back(make_pair(commodity, e));
						}
					}
				}

				// ordernar pelo valor da commodity do menor para a maior
				sort(grb_out.begin(), grb_out.end(),
					 [](pair<int, edge> el_1, pair<int, edge> el_2)
					 {
						 return el_1.first > el_2.first;
					 });

				// inserir o caminho na solução
				for (auto itvec : grb_out)
					sol.edges.emplace_back(itvec.second);

				// caso o caminho tenha aresta fora de ordem, não configurando um circuito, ordenar
				// isso pode ocorrer em depots, visto a possíbilidade de loops, ordenar os diversos loops que passam pelos depots.
				// if(!IsCircuit(sol))
				SortMultiplesAdjacentOnDepots(&sol);

				//----------------------------------------- teste temporário para verificar se a capacidade do robô é mantida
				if (!PathRestrictions(sol))
					cout << "solução não validada: milp" << endl;
				//--------------------------------------------------

				// contabilizar o base do robô
				sol.depotsNum = sol.depots.size() + 1;
			}
			else
			{ // se for inviável
				// model.computeIIS();
				// model.write("model.ilp");
				// model.write("model.lp");
				sol.pCost = -1;
				sol.robotID = -1;
				sol.pID = -1;
			}

			for (int i = 0; i < N; i++)
				delete[] vars_x[i];
			delete[] vars_x;
			delete[] vars_d;
		}
		catch (GRBException &e)
		{
			cout << "Error number: " << e.getErrorCode() << endl;
			cout << e.getMessage() << endl;
		}
		catch (...)
		{
			cout << "Error during optimization" << endl;
		}
		return sol;
	}

	// realizar a união dos subsets
	Solution::path Solution::Union_Solutions(solution vec_sol)
	{
		path union_paths;
		path union_paths_temp;

		list<pair<pair<edge, edge>, int>> in_out_vec;
		list<pair<edge, edge>> in_out_vec_temp;

		list<pair<pair<edge, edge>, int>> set_order;
		list<pair<edge, edge>> set_order_temp;

		union_paths.robotID = vec_sol.paths.front().robotID;
		union_paths.pID = vec_sol.paths.front().pID;

		int group_id = 0;
		// inserir as entradas e saídas dos caminhos na lista
		for (path p : vec_sol.paths)
			in_out_vec.emplace_back(make_pair(make_pair(p.edges.front(), p.edges.back()), group_id++));

		// obter a saída que está mais a esquerda;
		// posicionar o par que tem a linha de saída mais a esquerda no eixo x, no início do vetor
		// most left line
		double mll = numeric_limits<double>::max();
		auto it_pair = in_out_vec.begin();
		auto it_in_out = in_out_vec.begin();

		pair<pair<edge, edge>, int> pair_mli;

		double x_coord;
		int in_node;
		int out_node;

		it_pair = in_out_vec.begin();
		// encontrar o par na lista de pares em que o nó de entrada está mais à esquerda
		while (it_pair != in_out_vec.end())
		{
			// obter o índice do nó de entrada no grupo, saindo da base
			in_node = it_pair->first.first.node_b;
			// obter a posição x do nó
			x_coord = link_nid_to_ninfo.find(in_node)->second.getX();
			// armazernar apenas o mais a esquerda
			if (mll > x_coord)
			{
				pair_mli = *it_pair;
				mll = x_coord;
				it_in_out = it_pair;
			}
			it_pair++;
		}
		// se o pair no início da lista não for o mais à esquerda, inserir o pair_mll no início da lista
		if (it_in_out != in_out_vec.begin())
		{
			set_order.push_back(pair_mli);
			in_out_vec.erase(it_in_out);
		}

		// classificar os pares em relação a saída e entrada de dois pares diferentes,
		// iniciar do início da lista, buscar o par que possui a entrada mais próxima da saída do par anterior.
		double cost;
		double min_cost = numeric_limits<double>::max();
		it_in_out = in_out_vec.begin();
		auto it_set_order = set_order.begin();
		auto it_near_set = in_out_vec.begin();
		while (!in_out_vec.empty())
		{

			if (it_in_out != in_out_vec.end())
			{
				out_node = it_set_order->first.second.node_a;
				in_node = it_in_out->first.first.node_b;

				cost = getCostOnGraph(union_paths.robotID, out_node, in_node);
				if (cost < min_cost)
				{
					min_cost = cost;
					it_near_set = it_in_out;
				}
			}
			else
			{
				// it_near_set->second = set_order.size();
				set_order.emplace_back(*it_near_set);
				in_out_vec.erase(it_near_set);
				it_in_out = in_out_vec.begin();
				min_cost = numeric_limits<double>::max();
				it_set_order++;
				continue;
			}
			it_in_out++;
		}
		// unir os caminho seguindo a ordem dos pares no vetor set_order.
		it_set_order = set_order.begin();
		int in = 0;
		int out = 0;
		auto it_path = vec_sol.paths.begin();
		auto it_next_edge = vec_sol.paths.begin()->edges.begin();

		int insert_num = 1;
		while (!set_order.empty())
		{

			it_path = vec_sol.paths.begin() + set_order.front().second;

			union_paths.edges.insert(union_paths.edges.end(), it_path->edges.begin(), it_path->edges.end());
			union_paths.fuelOnTarget.insert(it_path->fuelOnTarget.begin(), it_path->fuelOnTarget.end());
			union_paths.depots.insert(it_path->depots.begin(), it_path->depots.end());

			it_next_edge = it_path->edges.begin();

			if (vec_sol.paths.size() - set_order.size() == 0)
				out = union_paths.edges.back().node_a;

			if (vec_sol.paths.size() - set_order.size() >= 1)
			{

				in = it_path->edges.front().node_b;

				union_paths = Link_Sets_Node_out_in(union_paths, out, in);

				// se for inviável
				if (union_paths.pCost < 0)
					return union_paths;
				out = union_paths.edges.back().node_a;
			}

			++insert_num;
			// vec_sol.paths.erase(it_path);
			set_order.pop_front();
		}

		union_paths.targetsNum = union_paths.fuelOnTarget.size();
		union_paths.depotsNum = union_paths.depots.size() + 1;
		union_paths.pCost = 0;

		for (auto e : union_paths.edges)
			union_paths.pCost += e.cost;

		return union_paths;
	}

	bool Solution::swap(solution *s)
	{

		struct swapSol
		{
			path p1;
			path p2;
			int g1Index;
			int g2Index;
			int g1LID;
			int g2LID;
			string path_op_g1;
			string path_op_g2;
			double time_op_g1;
			double time_op_g2;
		};

		path pathG1;
		path pathG2;

		solution solTemp = *s;

		vector<path> swapPathLines;
		swapPathLines = s->paths;
		vector<int> depots;

		map<double, swapSol> mapCostSwapSol;

		std::chrono::time_point<std::chrono::system_clock> start1, end1;
		std::chrono::time_point<std::chrono::system_clock> start2, end2;

		// vetor de previsões
		vector<pair<double, swapSol>> predictions_sol;
		swapSol sol;

		// get index of graphs in random order
		vector<int> graphsIndex = rand.randVector(getNumberOfSets());
		vector<int> g1LinesIndex, g2LinesIndex;

		int lineG1;
		int lineG2;

		int g1Index = 0;
		int nLineCoverage = 0;

		double maxCost = 0;

		pair<path, string> path_op_g1;
		pair<path, string> path_op_g2;

		path p1R, p1I, p2R, p2I;

		bool found_best_sol = false;

		for (path p : s->paths)
		{
			for (int id : p.depots)
				if (id < 31)
					cerr << "#1 Erro id em na função swap!\n";
		}

		// copy current solution groups operators: nodesSets and coverageSets
		copyNSets();

		// enquanto existe pelo menos dois conjuntos para realizar o swap
		// while(graphsIndex.size() > 1){

		g1Index = solTemp.maxCostPathID;

		// obter ordem aleatória das linhas de g1
		// se o número de linhas de cobertura for maior que zero atribuir os índices embaralhados
		// à g1LineCoverage
		nLineCoverage = getNumberOfLines(g1Index);
		if (nLineCoverage < 1)
		{
			return false;
		}

		g1LinesIndex = rand.randVector(nLineCoverage);

		// variar todos os conjuntos desconsiderando o conjunto já seleciondo (g1Index)
		for (int g2Index : graphsIndex)
		{

			if (found_best_sol)
				break;

			if (g2Index == g1Index)
				continue;

			nLineCoverage = getNumberOfLines(g2Index);
			if (nLineCoverage < 1)
				continue;

			g2LinesIndex = rand.randVector(nLineCoverage);

			for (int g1LID : g1LinesIndex)
			{

				if (found_best_sol)
					break;

				// linha escolhida de g1
				lineG1 = getCVLIndex(g1Index, g1LID);

				// remover aresta lineG1 de p1
				p1R = removeCL(solTemp.paths[g1Index], lineG1);

				if (p1R.pCost < 0)
					continue;

				// percorrer as linhas de g2
				for (int g2LID : g2LinesIndex)
				{

					if (found_best_sol)
						break;

					// linha escolhida de g2
					lineG2 = getCVLIndex(g2Index, g2LID);

					// inserir a aresta lineG2 de p2 em p1
					start1 = std::chrono::system_clock::now();
					path_op_g1 = bestInsertionCL(p1R, lineG2);
					end1 = std::chrono::system_clock::now();

					p1I = path_op_g1.first;
					// se o valor após remover for igual ou maior que o valor anterior, pegar outra linha
					if (p1I.pCost < 0)
						continue;

					// remover uma aresta lineG2 de p2
					p2R = removeCL(solTemp.paths[g2Index], lineG2);

					if (p2R.pCost < 0)
						continue;

					// inserir a linha lineG1 de p1 em p2
					start2 = std::chrono::system_clock::now();
					path_op_g2 = bestInsertionCL(p2R, lineG1);
					end2 = std::chrono::system_clock::now();

					p2I = path_op_g2.first;

					if (p2I.pCost < 0)
						continue;

					// limpar operações antigas
					sol.path_op_g1.clear();
					sol.path_op_g2.clear();

					// obter o maior valor entre os novos caminho p1 e p2 após as devidas remoções e inserções
					maxCost = max(p1I.pCost, p2I.pCost);

					// obter as informação das modificações
					sol.g1Index = g1Index;
					sol.g2Index = g2Index;
					sol.g1LID = g1LID;
					sol.g2LID = g2LID;
					sol.p1 = p1I;
					sol.p2 = p2I;
					sol.path_op_g1.assign(path_op_g1.second);
					sol.path_op_g2.assign(path_op_g2.second);
					sol.time_op_g1 = std::chrono::duration_cast<std::chrono::milliseconds>(end1 - start1).count();
					sol.time_op_g2 = std::chrono::duration_cast<std::chrono::milliseconds>(end2 - start2).count();

					// inserir as informações e o resultado no vetor
					mapCostSwapSol.emplace(maxCost, sol);
					predictions_sol.emplace_back(make_pair(maxCost, sol));

					if (maxCost < solTemp.maxCost)
					{
						found_best_sol = true;
					}
				}
			}
		}

		// ordenar o vetor de previsões com base no custo.
		sort(predictions_sol.begin(), predictions_sol.end(),
			 [](pair<double, swapSol> el_1, pair<double, swapSol> el_2)
			 {
				 return el_1.first < el_2.first;
			 });
		// guiar o swap pela previsão de menor custo
		//  aplica o método exato para verificar o melhor caminho para a previsão selecionada.
		if (!predictions_sol.empty())
		{

			// obter a previsão da solução de menor custo
			auto itMap = predictions_sol.begin();

			pred prediction;

			prediction.pred_num = predictions_sol.size();
			prediction.operation = "Swap";
			prediction.best_pred = best_prediction;

			// caso não haja melhor manter -1
			prediction.pred_improv_sol = -1;

			int n_of_prediction = 0;

			if (best_prediction)
				n_of_prediction = 1;
			else
				n_of_prediction = predictions_sol.size();

			// realizar o swap das k melhores previsões.
			for (int i = 0; i < n_of_prediction; i++)
			{
				if (itMap != predictions_sol.end())
				{
					int g1 = itMap->second.g1Index;
					int g2 = itMap->second.g2Index;

					pathG1 = itMap->second.p1;
					// se o caminho em g1 for inviável
					if (pathG1.pCost < 0 || pathG1.pID == -1 || pathG1.edges.empty())
					{
						// std::cout << "Caminho G1 inválido: pID = " << pathG1.pID << ", nodes = " << pathG1.nodes.size() << std::endl;
						continue;
					}

					pathG2 = itMap->second.p2;
					// se o caminho em g2 for inviável
					if (pathG2.pCost < 0 || pathG2.pID == -1 || pathG2.edges.empty())
					{
						// std::cout << " Caminho G2 inválido: pID = " << pathG2.pID << ", nodes = " << pathG2.nodes.size() << std::endl;
						continue;
					}

					// change the paths of solTemp
					solTemp.paths[g1] = pathG1;
					solTemp.paths[g2] = pathG2;

					for (int id : pathG1.depots)
						if (id < 31)
							cerr << "#2 Erro id em na função swap!\n";

					for (int id : pathG2.depots)
						if (id < 31)
							cerr << "#3 Erro id em na função swap!\n";

					// changed the paths:update tempSol cost and depotsNum;
					updateSolCosts(&solTemp);
					// se o maior custo de shiftPath  for menor que o maior custo da solução corrente
					if (isDefinitelyGreaterThan(max(s->paths[g1].pCost, s->paths[g2].pCost),
												max(solTemp.paths[g1].pCost, solTemp.paths[g2].pCost), 1.0) ||
						(s->depotsNum > solTemp.depotsNum))
					{

						// insere na fronteira e retorna apenas se melhorar o custo
						if (isDefinitelyGreaterThan(max(s->paths[g1].pCost, s->paths[g2].pCost),
													max(solTemp.paths[g1Index].pCost, solTemp.paths[g2].pCost), 1.0))
						{

							if (isDefinitelyGreaterThan(s->maxCost, solTemp.maxCost, 1.0))
								insert_vecSol(solTemp);

							// atualiza solução corrente
							*s = solTemp;

							depots.clear();
							depots.insert(depots.end(), s->depots.begin(), s->depots.end());
							for (uint i = 0; i < s->paths.size(); ++i)
								UpdateDepots(i, depots);

							prediction.pred_improv_sol = i;
							prediction.path_op_g1 = itMap->second.path_op_g1;
							prediction.path_op_g2 = itMap->second.path_op_g2;
							prediction.pred_time_g1 = itMap->second.time_op_g1;
							prediction.pred_time_g2 = itMap->second.time_op_g2;
							prediction.pred_id = pred_num++;

							vec_predictions.emplace_back(prediction);

							// update nodeset
							swapCLine(g1, g2, itMap->second.g1LID, itMap->second.g2LID);

							// cout << "swaped groups: " << g1 << ", " << g2;
							checkRestrictions(*s);

							return true;
						}

						// insere na fronteira a solução temporária, que é melhor apenas na quantidade de depots
						// não atualizamos a solução corrente se o swap não melhora o custo, apenas inserimos
						// a solução na fronteira.
						insert_vecSol(solTemp);
					}
					// restoreNSets();
					solTemp = *s;
				}
				++itMap;
			}
		}
		checkRestrictions(*s);
		// cout << "swap done! Sol not improved!";
		return false;
	}

	// for each graph, choose a line to remove and insert it in an other graph randomly choosed.
	bool Solution::shift(solution *s)
	{
		path pathG1;
		path pathG2;
		vector<int> depots;
		pair<path, string> path_op;

		struct pathIndex
		{
			path p1;
			path p2;
			int g1Index;
			int g2Index;
			int g1LID;
			string path_op;
			double shift_time;
		};

		pathIndex pIndex;

		std::chrono::time_point<std::chrono::system_clock> start, end;

		map<double, pathIndex> mapCostPaths;

		int line = -1;
		int iLine = 0;
		solution solTemp = *s;

		int g1, g2, g1LID;

		bool found_best_sol = false;

		// get index of graphs in random order
		vector<int> graphsIndex = rand.randVector(getNumberOfSets());
		int nOfCLines;

		// copy current solution groups operators: nodesSets and coverageSets
		copyNSets();

		// obter o índice do grupo com maior custo
		int g1Index = solTemp.maxCostPathID;

		// obter a quantidade de linhas de cobertura do grupo
		nOfCLines = getNumberOfLines(g1Index);
		// caso não haja nº linha de cobertura suficiente no grupo, testar o próximo
		if (nOfCLines <= 1)
			return false;

		// obter os índices das linhas de cobertura de maneira embaralhada.
		vector<int> vLines = rand.randVector(nOfCLines);
		while (!vLines.empty())
		{ // enquanto não testar todas as linhas

			if (found_best_sol)
				break;

			// get a coverage line on  g1 group
			iLine = vLines.back();
			// remove from vector
			vLines.pop_back();

			// obter o índice da linha em g1
			line = getCVLIndex(g1Index, iLine);

			pathG1 = removeCL(solTemp.paths[g1Index], line);

			if (pathG1.pCost <= 0)
				continue;

			// para todos os outro grupos da solução
			for (int g2Index : graphsIndex)
			{

				if (found_best_sol)
					break;
				if (g1Index != g2Index)
				{ // com exceção de g1Index

					start = std::chrono::system_clock::now();
					path_op = bestInsertionCL(solTemp.paths[g2Index], line);
					end = std::chrono::system_clock::now();

					pathG2 = path_op.first;

					if (pathG2.pCost <= 1)
						continue;

					double maxCost = max(pathG2.pCost, pathG1.pCost);
					pIndex.g1Index = g1Index;
					pIndex.g2Index = g2Index;
					pIndex.g1LID = iLine;
					pIndex.p1 = pathG1;
					pIndex.p2 = pathG2;
					pIndex.path_op = path_op.second;
					pIndex.shift_time = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();

					mapCostPaths.emplace(maxCost, pIndex);

					if (maxCost < solTemp.maxCost)
					{
						found_best_sol = true;
					}
				}
			}
		}

		if (!mapCostPaths.empty())
		{

			auto itMap = mapCostPaths.begin();
			int n_of_prediction = 0;

			pred prediction;

			prediction.pred_num = mapCostPaths.size();
			prediction.operation = "Shift";
			prediction.best_pred = best_prediction;

			// caso não haja melhor manter -1
			prediction.pred_improv_sol = -1;

			if (best_prediction)
				n_of_prediction = 1;
			else
				n_of_prediction = mapCostPaths.size();

			for (int i = 0; i < n_of_prediction; ++i)
			{
				if (itMap != mapCostPaths.end())
				{

					g1 = itMap->second.g1Index;
					g2 = itMap->second.g2Index;
					g1LID = itMap->second.g1LID;

					pathG2 = itMap->second.p2;
					if (pathG2.pCost <= 0)
					{
						// solTemp = *s;
						// restoreNSets();
						continue;
					}

					solTemp.paths[g2] = pathG2;

					if (isDefinitelyGreaterThan(solTemp.paths[g1].pCost, solTemp.paths[g2].pCost, 1.0))
					{
						pathG1 = itMap->second.p1;

						if (pathG1.pCost <= 0)
						{
							solTemp = *s;
							// restoreNSets();
							continue;
						}

						// update solTemp with new path
						solTemp.paths[g1] = pathG1;

						// changed the paths:update tempSol cost and depotsNum;
						updateSolCosts(&solTemp);

						insert_vecSol(solTemp);

						// update current solution s
						*s = solTemp;

						depots.clear();
						// inserir um possível atualização de depots no nodesSet
						depots.insert(depots.end(), s->depots.begin(), s->depots.end());
						for (uint i = 0; i < s->paths.size(); ++i)
						{
							UpdateDepots(i, depots);
						}

						// atribuir o índice da predição que melhorou a solução
						prediction.pred_improv_sol = i;
						prediction.path_op_g1.assign("none");
						prediction.path_op_g2 = itMap->second.path_op;
						prediction.pred_time_g1 = -1;

						prediction.pred_time_g2 = itMap->second.shift_time;
						prediction.pred_id = pred_num++;

						// inserir no vetor
						vec_predictions.emplace_back(prediction);

						// atualizar nodesSets
						shiftCLine(g1, g2, g1LID);

						// cout << "Shifted  line: " << getCVLIndex(g1,g1LID) << " from: " << g1 <<" to " << g2;
						return true;
					}
					// back solTemp to current solution
					solTemp = *s;
					// restore current solution groups operators: nodesSets and coverageSets
					// restoreNSets();
				}
				++itMap;
			}
		}
		// cout << "shift done! Sol not improved!";

		return false;
	}

	bool Solution::improveSol(solution *s)
	{
		path imp_path;
		vector<path> imp_sol;
		solution solTemp;
		double cost = 0;
		int maxCostPathID = -1;
		double maxCost = numeric_limits<double>::min();

		set<int> globalDepots;
		vector<int> depots;
		int targets_num = 0;
		;

		for (uint i = 0; i < s->paths.size(); ++i)
		{
			imp_path = improvePath(s->paths[i]);
			if (imp_path.pID == -1 || imp_path.pCost < 0 || imp_path.edges.empty())
				return false;

			if (!PathRestrictions(imp_path))
			{
				// cout << "Caminho inválido gerado em improveSol (i = " << i << ", pID = " << imp_path.pID << ", robotID = " << imp_path.robotID << ")";
				return false;
			}
			imp_sol.emplace_back(imp_path);
			cost += imp_path.pCost;

			if (isDefinitelyGreaterThan(imp_path.pCost, maxCost))
			{
				maxCost = imp_path.pCost;
				maxCostPathID = i;
			}

			globalDepots.insert(imp_path.depots.begin(), imp_path.depots.end());
			targets_num += imp_path.fuelOnTarget.size();
		}

		// Verificação de integridade
		if (maxCostPathID == -1)
			return false;

		solTemp.paths.clear();
		solTemp.paths.insert(solTemp.paths.begin(), imp_sol.begin(),
							 imp_sol.end());

		solTemp.depots.insert(globalDepots.begin(), globalDepots.end());
		solTemp.depotsNum = globalDepots.size() + 1;
		solTemp.maxCost = maxCost;
		solTemp.sCost = cost;
		solTemp.maxCostPathID = maxCostPathID;
		solTemp.targetsNum = targets_num;

		// caso a maior solução seja melhorada ou redução de posto, se reduzir o maior custo atribui-lo ao vetor
		// de soluções e considerá-la como a solução corrente, caso reduza apenas o número de posto atualizar a solução corrente
		if (isDefinitelyGreaterThan(s->maxCost, solTemp.maxCost, 1.0) || (s->depotsNum > solTemp.depotsNum))
		{

			// inserir no vetor de soluções
			if (isDefinitelyGreaterThan(s->maxCost, solTemp.maxCost, 1.0))
			{
				// inserção pela melhora do custo do maior caminho
				insert_vecSol(solTemp);

				// atualiza solução corrente
				*s = solTemp;

				// atualizar a estrutura do grafo com os depots encontrados
				depots.clear();
				depots.insert(depots.end(), s->depots.begin(), s->depots.end());
				for (uint i = 0; i < s->paths.size(); ++i)
					UpdateDepots(i, depots);

				// cout << "improved max_sol ";
				return true;
			}
			else
			{
				for (uint i = 0; i < s->paths.size(); ++i)
				{
					// realizar a comparação entre solTemp e a solução corrente.
					// modificar se a melhora for de pelo menos 1s
					if (isDefinitelyGreaterThan(s->paths[i].pCost, solTemp.paths[i].pCost, 1.0))
					{

						// a redução no custo de qualquer solução que não seja a maior, atualizará apenas a solução corrente.
						// mas se o improve_sol diminuir o número de postos, colocamos a solução no vetor de soluções.
						if (s->depotsNum > solTemp.depotsNum)
							insert_vecSol(solTemp); // inserção pela redução de postos

						// atualiza solução corrente, devido a melhor no custo
						*s = solTemp;

						// atualizar a estrutura do grafo com os depots encontrados
						depots.clear();
						depots.insert(depots.end(), s->depots.begin(), s->depots.end());
						for (uint i = 0; i < s->paths.size(); ++i)
							UpdateDepots(i, depots);

						// cout << "improved current_sol ";
						return true;
					}
				}
			}
			// caso haja somente redução no número de postos, inserir no vetor de soluções.
			insert_vecSol(solTemp);
		}
		// cout << "improveSol done ! sol not improved !";

		return false;
	}

	// change types of robot at the groups
	bool Solution::swapRobots(solution *s)
	{

		solution solTemp = *s;

		// obter quantidade  de tipos de robôs;
		int robotsTypes = getMapRobotGroupSize();

		vector<int> t1Groups;
		vector<int> t2Groups;
		set<int> setT1;
		set<int> setT2;

		path pathG1;
		path pathG2;
		vector<int> depots;

		copyNSets();

		// max cost path
		int max_p = s->maxCostPathID;

		// id do robô do maior caminho
		int robot_id = s->paths[max_p].robotID;

		// tipo do robô do maior caminho
		int t1 = GetGroupOfRobot(max_p);

		vector<int> randRobotTypesIDs = rand.randVector(robotsTypes);

		// testar se é possível substituir em dois caminhos mc)id e algum de g2 o tipo de robô.
		for (int t2 : randRobotTypesIDs)
		{
			if (t1 == t2)
				continue;
			// obter todos os caminhos do segundo tipo t2
			setT2 = getRobotGroups(t2);
			// inserir o grupo de caminhos no vetor t2Groups
			t2Groups.clear();
			t2Groups.insert(t2Groups.end(), setT2.begin(), setT2.end());
			// aleatorizar o vetor
			rand.sufferVector(&t2Groups);

			// verificar para cada g2, se é possível trocar o tipo do robô com mc_id
			// for(int g1:t1Groups){
			for (int g2 : t2Groups)
			{

				// trocar os robôs dos caminhos mc_id e g2
				robot_id = solTemp.paths[max_p].robotID;
				solTemp.paths[max_p].robotID = solTemp.paths[g2].robotID;
				solTemp.paths[g2].robotID = robot_id;

				// obter o custo do robô de g2 em percorrer mc_id
				solTemp.paths[max_p] = RobotCostInPath(solTemp.paths[max_p]);

				// caso não seja possível percorrer o caminho mc_pid com o robô do caminho g2
				if (solTemp.paths[max_p].pCost < 0)
				{
					solTemp = *s;
					restoreNSets();
					continue;
				}

				// obter o custo do robô de mc_pid em percorrer g2
				solTemp.paths[g2] = RobotCostInPath(solTemp.paths[g2]);

				// caso não seja possível percorrer o caminho g2 como o robô do caminho mc_pid
				if (solTemp.paths[g2].pCost < 0)
				{
					solTemp = *s;
					restoreNSets();
					continue;
				}

				if (solTemp.paths[g2].pCost > 0 && solTemp.paths[max_p].pCost > 0)
				{
					// changed the paths:update tempSol cost and depotsNum;

					// atualiza a solução para a troca de robôs em mc_pid e g2
					updateSolCosts(&solTemp);
					// se o maior custo solTemp for menor que o maior custo da solução corrente
					// insere na fronteira e retorna apenas se melhorar o maior custo
					// o seja se os custos mc_pid ou g2 de soltem forem melhores que em s.
					if (isDefinitelyGreaterThan(s->maxCost, solTemp.maxCost, 1.0))
					{
						insert_vecSol(solTemp);

						// atualizar a solução atual
						*s = solTemp;

						// aplicar mudança em nodesSet
						swapRobotsGroups(t1, max_p, t2, g2);

						// cout << "swaped robots groups: " << max_p <<" " << g2 <<endl;
						return true;
					}
				}
				solTemp = *s;
				restoreNSets();
			}
		}
		// cout << "swapRobots done! Sol not improved!";
		return false;
	}

	// obter o índice do depot mais próximo.
	int Solution::GetCloserDepot(path p, int depot_to_close)
	{
		vector<pair<int, double>> depot_distances;
		double distance;

		// encontrar o depot na lista
		auto it_depot_to_close = p.depots.find(depot_to_close);

		if (it_depot_to_close != p.depots.end())
		{

			// obter a distância do depot fechado ao outro depots do caminho
			for (int depot_id : p.depots)
			{

				if (depot_id != depot_to_close)
				{
					distance = getCostOnGraph(p.robotID, depot_to_close, depot_id);
					// inserir a distância e o id do depot no vetor
					depot_distances.emplace_back(make_pair(depot_id, distance));
				}
			}

			// ordenar da menor distância para a maior
			sort(depot_distances.begin(), depot_distances.end(),
				 [](pair<int, double> el_1, pair<int, double> el_2)
				 { return el_1.second < el_2.second; });

			// retorna o id do depot mais próximo
			if (!depot_distances.empty())
				return depot_distances.front().first;

			else // caso não tenha depots próximo
				return -1;
		}
		// se não encontrou o depot
		return -1;
	}

	// fechar o depósito e desvia para o mais próximo.
	Solution::path Solution::GetPathWithDepotClosure(path p, int depot_to_close)
	{

		int next_depot = GetCloserDepot(p, depot_to_close);
		path path_temp;

		if (next_depot < 0)
		{
			p.pCost = -1;
			return p;
		}
		path_temp = CloseDepotLinkToNext(p, depot_to_close, next_depot);

		if (!robotCapacity_val(path_temp))
		{
			path_temp.pCost = -1;
			return path_temp;
		}

		//-------------------------teste temporário da saída------------------------------------
		if (path_temp.pCost > 0)
			if (!PathRestrictions(path_temp))
				cout << "Problema restrição getPath with depot closure" << endl;
		//------------------------------------------------------------------------------------

		return path_temp;
	}

	// remover o depósito do caminho e desviar a rota para o depot mais próximo
	Solution::path Solution::CloseDepotLinkToNext(path p, int depot_to_close, int next_depot)
	{
		double pcost = 0;

		auto itfuel = p.fuelOnTarget.begin();
		double fuel_required;
		double fuel_remaining;
		double rcapacity = input.getRobotFuel(p.robotID);

		auto it_edge_fuel = p.edges.begin();
		for (auto it_edge = p.edges.begin(); it_edge != p.edges.end(); ++it_edge)
		{

			if (it_edge->node_b == depot_to_close)
			{

				if (it_edge->node_a == next_depot)
				{
					p.edges.erase(it_edge);
					--it_edge;
					continue;
				}

				it_edge->node_b = next_depot;
				it_edge->time = getCostOnGraph(p.robotID, it_edge->node_a, it_edge->node_b);
				it_edge->cost = it_edge->time + (it_edge->time * input.getRobotProp(p.robotID));

				// combústive necessário para chegar no novo depot
				fuel_required = it_edge->time;

				// obter o combustível disponível no robô
				itfuel = p.fuelOnTarget.find(it_edge->node_a);
				if (itfuel != p.fuelOnTarget.end()) // se a for target
					fuel_remaining = itfuel->second;

				else
					fuel_remaining = rcapacity - fuel_required;

				// o desvio do caminho é inviável, o robô não tem combustível suficiente para chegar no depot
				if (isDefinitelyLessThan(fuel_remaining, fuel_required))
				{
					p.pCost = -1;
					return p;
				}
			}
			if (it_edge->node_a == depot_to_close)
			{

				if (it_edge->node_b == next_depot)
				{
					p.edges.erase(it_edge);
					--it_edge;
					continue;
				}

				it_edge->node_a = next_depot;
				it_edge->time = getCostOnGraph(p.robotID, it_edge->node_a, it_edge->node_b);
				it_edge->cost = it_edge->time + (it_edge->time * input.getRobotProp(p.robotID));

				fuel_required = it_edge->time;

				// combustível que restará em node_b ao sair do novo posto
				fuel_remaining = rcapacity - fuel_required;

				// caso o robô não tenha capacidade de chegar no node_b
				if (fuel_remaining < 0)
				{
					p.pCost = -1;
					return p;
				}
				// se o node_b for target, atualizar a quantidade de combustível que o robô chegará no target após o desvio.
				itfuel = p.fuelOnTarget.find(it_edge->node_b);

				it_edge_fuel = it_edge;
				while (itfuel != p.fuelOnTarget.end())
				{
					// atualizar o combustível restante no robô ao chegar em node_b
					itfuel->second = fuel_remaining;

					++it_edge_fuel;

					fuel_required = it_edge_fuel->time;

					fuel_remaining = fuel_remaining - fuel_required;

					if (fuel_remaining < 0)
					{
						p.pCost = -1;
						return p;
					}

					itfuel = p.fuelOnTarget.find(it_edge_fuel->node_b);
				}
			}

			pcost += it_edge->cost;
		}

		p.pCost = pcost;
		p.depots.erase(depot_to_close);

		//-------------------------------------------teste temporário da saída---------------------------------------
		// if(!PathRestrictions(p))
		// cout << "problema restrições closedepot link to next" <<endl;
		//-------------------------------------------------------------------------------------------------

		return p;
	}

	bool Solution::closeRandomDepot(solution *s)
	{
		solution solTemp = *s;

		// get map of depots open to every group(path)
		map<int, set<int>> mapDG = getMapDG(solTemp);

		vector<int> openDepots;

		vector<int> depots;
		vector<int> udepots;

		depots.clear();

		// get depots IDs
		depots.insert(depots.end(), solTemp.depots.begin(), solTemp.depots.end());

		// obter a quantidade de depots abertos
		int nDepots = solTemp.depots.size();

		// get a index of opendepots in random order
		vector<int> idVector = rand.randVector(nDepots);

		// save depots opened
		copyNSets();

		// try close one depot
		while (!idVector.empty())
		{
			// get last id from random vector of ids
			int id = idVector.back();

			// remove last id
			idVector.pop_back();

			// get depot id
			int depotID = depots.at(id);

			// get all opened depots
			openDepots = depots;

			// close depot.
			openDepots.erase(openDepots.begin() + id);

			// quantity of groups feasible when the depot is close
			uint closure = 0;

			// get all groups that use the depot
			set<int> gSet = mapDG.find(depotID)->second;

			// get depots graph

			// check if closure of the depot is feasible for all groups
			for (int gID : gSet)
			{

				path p = solTemp.paths[gID];
				p.depots.clear();
				p.depots.insert(openDepots.begin(), openDepots.end());

				path p_temp = p;

				p = RemoveDepotFromPath(p, depotID);

				// p is feasible
				if (p.pCost > 0)
				{
					// atualizar o custo da solução temporária;
					solTemp.paths[gID] = p;

					closure++;
					udepots.clear();
					udepots.insert(udepots.end(), p.depots.begin(), p.depots.end());

					// update nodesSet
					UpdateDepots(gID, udepots);

					//-------------------------------teste temporário da saída-----------------------------
					// if(!PathRestrictions(p))
					// cout <<"Restrição CloseRandomDepot_2" <<endl;
					//-------------------------------------------------------------
				}
				else
				{
					// back to currente solution
					solTemp = *s;
					restoreNSets();
					closure = 0;
					break;
				}
			}
			// if all groups can handle with the closure of depot
			if (closure == gSet.size())
			{
				// update current solution
				updateSolCosts(&solTemp);

				*s = solTemp;

				udepots.clear();
				udepots.insert(udepots.end(), s->depots.begin(), s->depots.end());
				for (uint i = 0; i < s->paths.size(); ++i)
				{
					UpdateDepots(i, udepots);
				}

				// insere no vetor de soluções
				insert_vecSol(*s);

				// close only one depot that is feasible.
				return true;
			}
		}
		// cout << "closeDepot done! Sol not improved!";
		return false;
	}

	vector<pair<int, int>> Solution::GetPrevNextNodes(path p, int node_id)
	{

		pair<int, int> p_nodes;
		vector<pair<int, int>> v_pnodes;
		for (edge e : p.edges)
		{
			if (e.node_b == node_id)
				p_nodes.first = e.node_a;
			else if (e.node_a == node_id)
			{
				p_nodes.second = e.node_b;
				v_pnodes.emplace_back(p_nodes);
			}
		}

		return v_pnodes;
	}

	Solution::path Solution::RemoveDepotFromPath(path p, int node_id)
	{
		// vector<pair<int,int>> v_pnodes = GetPrevNextNodes(p,node_id);

		list<edge> edges;

		// aresta para controlar possível loop caso a aresta anterior já tenha sido modificada
		edge prev_changed_edge;
		prev_changed_edge.node_a = -1;
		prev_changed_edge.node_b = -1;
		edges.insert(edges.begin(), p.edges.begin(), p.edges.end());
		auto it_edge = edges.begin();
		auto it_edge_first = edges.begin();
		auto it_edge_second = edges.begin();

		path new_path = p;
		path p_segment;
		edge e;
		int node_a_id;

		// auto it_fuel = p.fuelOnTarget.begin();
		double fuel_available_node_a;
		double fuel_available_node_b;

		double robot_capacity = input.getRobotFuel(p.robotID);

		while (it_edge != edges.end())
		{
			node_a_id = it_edge->node_a;

			if (node_a_id < 0)
			{
				p.pCost = -1;
				return p;
			}

			// verificar se os combustíveis em cada nó condizem com os gastos de cada aresta
			auto it_fuel = p.fuelOnTarget.find(node_a_id);
			if (it_fuel != p.fuelOnTarget.end())
			{
				fuel_available_node_a = it_fuel->second;
			}
			else
				fuel_available_node_a = robot_capacity;

			fuel_available_node_b = fuel_available_node_a - it_edge->time;

			// atualizar o combustível do node_b
			it_fuel = p.fuelOnTarget.find(it_edge->node_b);
			if (it_fuel != p.fuelOnTarget.end())
				it_fuel->second = fuel_available_node_b;

			// se a quantidade de combustível não for suficiente, pesquisar novas possibilidades de abastecimento
			// para sair de node_a e atingir node_b
			if (isDefinitelyLessThan(fuel_available_node_b, 0.0) && it_edge->node_b != node_id)
			{

				// se não for linha de cobertura verificar a possibilidade de realizar novo abastecimento
				// entre node_a e node_b
				if (!IsCLine(it_edge->node_a, it_edge->node_b))
				{

					// get last edge to prevent loop if it was infeasible to cover next coverage line.
					if (it_edge->node_a == prev_changed_edge.node_a && it_edge->node_b == prev_changed_edge.node_b)
					{
						p.pCost = -1;
						return p;
					}

					else
					{ // se for diferente atualizar para a nova aresta
						prev_changed_edge.node_a = it_edge->node_a;
						prev_changed_edge.node_b = it_edge->node_b;
					}

					p_segment = GetSPTOverOpenDepots(p, it_edge->node_a, it_edge->node_b);

					if (p_segment.edges.empty())
					{
						p.pCost = -1;
						return p;
					}

					// remover o custo da aresta
					p.pCost -= it_edge->cost;

					// remover a aresta da lista
					it_edge = edges.erase(it_edge);

					// it_edge = edges.erase(it_edge_second);

					// adicionar o custo do segmento
					p.pCost += p_segment.pCost;

					// inserir o segmento na lista
					edges.insert(it_edge, p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target dado os valores do segmento
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel != p.fuelOnTarget.end())
							it_fuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}
					continue;
				}
				else
				{ // se for linha de cobertura verificar a possibilidade de abastecer antes de
					// chegar no primeiro nó da linha de cobertura. voltar uma aresta para verificar

					auto prev = it_edge;

					--prev; // obter o link anterior

					p_segment = GetSPTOverOpenDepots(p, prev->node_a, prev->node_b);

					if (p_segment.edges.empty())
					{
						p.pCost = -1;
						return p;
					}

					// caso já tenhamos tentado obter um abastecimento a aresta anterior, iremos realizar a mesma atribuição
					// assim, o caminho será inviável.
					if (prev->node_a == prev_changed_edge.node_a && prev->node_b == prev_changed_edge.node_b)
					{
						p.pCost = -1;
						return p;
					}

					else
					{ // se for diferente atualizar para a última aresta do trecho, que será
						/// a aresta prev caso não seja viável percorrer a linha de cobertura
						prev_changed_edge.node_a = p_segment.edges.back().node_a;
						prev_changed_edge.node_b = p_segment.edges.back().node_b;
					}

					// remover o custo da aresta
					p.pCost -= prev->cost;

					// remover a aresta da lista
					it_edge = edges.erase(prev);

					// adicionar o custo do segmento
					p.pCost += p_segment.pCost;

					// inserir o segmento na lista
					edges.insert(it_edge, p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target dado os valores do segmento
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel != p.fuelOnTarget.end())
							it_fuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}
					continue;
				}
			}

			// realizar a alteração da rota desviando para outro depósito.
			if (it_edge->node_b == node_id)
			{

				// se o nó anterior for o depot de desvio, a substituição de node_b pelo mesmo nó
				// irá gerar um loop. remover e voltar uma posição;
				if (it_edge->node_a == node_id)
				{
					it_edge = edges.erase(it_edge);

					if (it_edge != edges.begin())
						--it_edge;
					continue;
				}

				e.node_a = it_edge->node_a;
				it_edge_first = it_edge;
				p.pCost -= it_edge->cost;
				it_edge = edges.erase(it_edge_first);
			}
			if (it_edge->node_a == node_id)
			{

				// se o nó posterior for o depot de desvio, a substituição de node_a pelo mesmo nó
				// irá gerar um loop. remover e voltar uma posição;
				if (it_edge->node_b == node_id)
				{
					it_edge = edges.erase(it_edge);

					if (it_edge != edges.begin())
						--it_edge;
					continue;
				}

				e.node_b = it_edge->node_b;
				it_edge_second = it_edge;

				p_segment = GetSPTOverOpenDepots(p, e.node_a, e.node_b);

				if (p_segment.edges.empty())
				{
					p.pCost = -1;
					return p;
				}

				p.pCost -= it_edge->cost;
				it_edge = edges.erase(it_edge_second);

				p.pCost += p_segment.pCost;
				edges.insert(it_edge, p_segment.edges.begin(), p_segment.edges.end());

				// atualizar o vetor de combustível em cada target dado os valores do segmento
				for (auto fuel_segment : p_segment.fuelOnTarget)
				{
					it_fuel = p.fuelOnTarget.find(fuel_segment.first);
					if (it_fuel != p.fuelOnTarget.end())
						it_fuel->second = fuel_segment.second;
					else
						p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
				}
				continue;
			}
			++it_edge;
		}

		p.edges.clear();
		p.edges.insert(p.edges.begin(), edges.begin(), edges.end());

		// update depots to only the ones used;
		set<int> depots;
		// for(auto e:p.edges){
		// if(!input.isTarget(e.node_b) && e.node_b != input.getRobotBaseId(p.robotID))
		// depots.insert(e.node_b);
		//}

		// if(p.depots.size()!= depots.size())
		// cout << "problema removedepot" <<endl;
		// p.depots = depots;

		//----------------------------teste temporário da saída-----------------------------------------
		// if(!PathRestrictions(p))
		// cout << "problema capacidade do robô removedepot" <<endl;
		//-------------------------------------------------------------------------------------------

		return p;
	}

	void Solution::updateSolCosts(solution *s)
	{
		// update depotsNum;
		double maxCost = numeric_limits<double>::min();

		double sCost = 0;
		int maxCostPathID = 0;
		int sTargetsNum = 0;
		s->depots.clear();

		for (path p : s->paths)
		{
			for (int id : p.depots)
				if (id < 31)
					cerr << "Problema id depot!\n";
			s->depots.insert(p.depots.begin(), p.depots.end()); // obter todos os depósito abertos
			sCost += p.pCost;
			sTargetsNum += p.fuelOnTarget.size();
			if (p.pCost > maxCost)
			{
				maxCost = p.pCost;
				maxCostPathID = p.pID;
			}
		}
		s->sCost = sCost;
		s->maxCost = maxCost;
		s->maxCostPathID = maxCostPathID;
		// contabilizar a base como depot
		s->depotsNum = s->depots.size() + 1;
		s->targetsNum = sTargetsNum;
	}

	// print all graphs costs. This is a solution.
	void Solution::printSol(solution s)
	{
		cout << "Sol:" << " dNum:" << s.depotsNum << " tNum:" << s.targetsNum << " mCost:" << s.maxCost << "\n";
		for (path i : s.paths)
			cout << "P_" << i.pID << "[" << i.pCost << ", " << i.robotID << ", " << i.targetsNum << "];";
		cout << "\n";
	}

	double Solution::getMaxSolCost()
	{
		return maxSolCost;
	}

	// get depots open for refueling
	vector<int> Solution::getOpenDepots()
	{

		vector<int> solNodes;
		vector<int> result(graphDepotsIndexes.size());
		vector<int>::iterator it;

		for (path s : currentSol.paths)
		{
			for (edge e : s.edges)
			{
				solNodes.push_back(e.node_a);
			}

			sort(solNodes.begin(), solNodes.end());
		}

		it = set_intersection(solNodes.begin(), solNodes.end(), graphDepotsIndexes.begin(), graphDepotsIndexes.end(), result.begin());
		result.resize(it - result.begin());

		/* std::cout << "The intersection has " << (result.size()) << " elements:\n";
		  for (it=result.begin(); it!=result.end(); ++it)
			std::cout << ' ' << *it;
		  std::cout << '\n';*/

		return result;
	}

	void Solution::mapOpenDepotsToGroup()
	{
		vector<int> solNodes;
		vector<int> result(graphDepotsIndexes.size());
		vector<int>::iterator it;

		// map node into group
		map<int, set<int>> mapNG;

		auto itMap = mapNG.end();

		for (path s : currentSol.paths)
		{
			for (edge e : s.edges)
			{
				solNodes.push_back(e.node_a);
				auto a = make_pair(e.node_a, solNodes);
				itMap = mapNG.emplace_hint(itMap, e.node_a, set<int>());
				itMap->second.insert(s.pID);
			}
		}
		sort(solNodes.begin(), solNodes.end());

		it = set_intersection(solNodes.begin(), solNodes.end(), graphDepotsIndexes.begin(), graphDepotsIndexes.end(), result.begin());
		result.resize(it - result.begin());

		// create a map only with depot to the groups ids: mapDG
		for (it = result.begin(); it != result.end(); ++it)
		{
			auto itm = mapNG.find(*it);
			// insert <depot:group> into mapDG
			mapDG.insert(*itm);
		}

		// clear all nodes mapping
		mapNG.clear();

		/*std::cout << "The intersection has " << (mapDG.size()) << " elements:\n";
		for (auto it=mapDG.begin(); it!=mapDG.end(); ++it){
			std::cout << "Depot: " << (*it).first <<" Groups:";
			for(auto itv = (*it).second.begin(); itv!= (*it).second.end(); ++itv)
				std::cout << " " << (*itv);
			std::cout << '\n';

		}*/
	}

	/*return map of depots with their groups ids
	 * one depot can be accessed by more than one robot
	 * the robots bases are not included at map */
	map<int, set<int>> Solution::getMapDG(solution s)
	{
		// map only depots into group
		map<int, set<int>> mapDG;
		auto itMap = mapDG.end();
		// map all nodes to solution ID
		for (path p : s.paths)
		{
			for (int depotID : p.depots)
			{
				// Verificação: ignorar IDs inválidos (por exemplo, targets ou erro de construção)
				if (depotID < 31)
				{
					cerr << "Aviso: ID inválido detectado no mapa de depósitos: " << depotID
						 << " (pID = " << p.pID << ")\n";
					continue;
				}
				itMap = mapDG.emplace_hint(itMap, depotID, set<int>());
				itMap->second.insert(p.pID);
			}
		}
		return mapDG;
	}

	/*Get all open depots and calculate the routes considering all depots at all groups
	 * Update the nodeSet and currentSol
	 * */
	void Solution::updateSolutionWithGlobalDepots(solution *s)
	{
		double maxTime = -1;
		double gurobiTime = gurobi_time_limit;
		vector<int> depots;
		set<int> groups;
		path pathTemp;

		// number of paths
		uint nPaths = s->paths.size();

		// change the time to initial solution
		gurobi_time_limit = 3600;

		// get map of depots open to every group
		map<int, set<int>> mapDG = getMapDG(*s);

		// insert all open depots ids on depots vector
		for (auto itmap = mapDG.begin(); itmap != mapDG.end(); ++itmap)
		{
			depots.emplace_back((*itmap).first);
			// set up the groups with more than one depot (base plus others depots)
			for (auto itvec = (*itmap).second.begin(); itvec != (*itmap).second.end(); ++itvec)
				groups.insert(*itvec);
		}

		// update nodesSets with groups with more than one depot and calculate the cost
		for (auto itset = groups.begin(); itset != groups.end(); ++itset)
		{
			// change nodesSet adding all open depots to all groups
			UpdateDepots(*itset, depots);
			// updateCoverageSet(*itset);

			// update only the groups with more than one depots
			// considering new depots
			pathTemp = bestPath(*itset);

			// se a solução não for viável;
			if (pathTemp.pCost < 0)
				return;

			s->paths[*itset] = pathTemp;

			// obter o maior tempo para obter a solução
			if (gurobi_optimize_time > maxTime)
				maxTime = gurobi_optimize_time;
		}

		// ajustar tempo para o maior entre a solução inicia e a solução após o merge dos depots
		if (gurobiTime > maxTime)
			gurobi_time_limit = gurobiTime;
		else
			gurobi_time_limit = maxTime;

		// after the new routing its possible change the depots used by the solutions
		// get opened depots at mapDG
		mapDG.clear();
		mapDG = getMapDG(*s);
		depots.clear();
		for (auto itmap = mapDG.begin(); itmap != mapDG.end(); ++itmap)
			depots.emplace_back((*itmap).first);

		// update all nodesSets with opened depots
		// its not necessary routing again, because previous routing choose this solution
		// that is the best, with the same depots.

		for (uint i = 0; i < nPaths; i++)
			UpdateDepots(i, depots);

		// update paths costs and open depot of the solution s.
		updateSolCosts(s);
		// não contabilizar a base, somente os depots que poderão ser trocados ou fechados
		depotsNumInit = s->depots.size();
	}

	void Solution::updatePathCurrenteSol(int gID, path p)
	{
		currentSol.sCost = currentSol.sCost + p.pCost - currentSol.paths[gID].pCost;
		currentSol.paths[gID] = p;
	}

	void Solution::eval_VecSol()
	{
		while (!vecSol.empty())
		{
			solution s = vecSol.back();
			vecSol.pop_back();
			if (paretoSet.empty())
			{
				insert_solution(s);
				continue;
			}
			map<int, vector<int>> mapEval = eval_solution(s);
			if (mapEval.find(1) != mapEval.end() || mapEval.find(2) != mapEval.end())
				update_paretoSet(mapEval, s);
		}
	}

	// insert solution s at paretoSet
	void Solution::insert_solution(solution s)
	{
		paretoSet.emplace_back(make_pair(false, s));
	}

	// insert solution at vecSol
	void Solution::insert_vecSol(solution s)
	{
		vecSol.emplace_back(s);
	}
	void Solution::clear_vecSol()
	{
		vecSol.clear();
	}

	// erase paretoSet solution pointed with an iterator
	void Solution::erase_solution(vector<pair<bool, solution>>::iterator solIt)
	{
		paretoSet.erase(solIt);
	}

	// return a paretoSet solution on solID position
	Solution::solution Solution::get_solution(int solID)
	{
		return paretoSet[solID].second;
	}

	// return a random paretoSet soltuion
	Solution::solution Solution::get_random_solution()
	{
		int solID = rand.randNum(paretoSet.size() - 1);
		return paretoSet[solID].second;
	}

	bool Solution::HasSolutionNotVisited()
	{
		sort(paretoSet.begin(), paretoSet.end(), [](const pair<bool, solution> el1, const pair<bool, solution> el2)
			 { return el1.first < el2.first; });
		for (pair<bool, solution> p : paretoSet)
		{
			// se não foi visitado, retornar true;
			if (!p.first)
				return true;
		}
		return false;
	}

	// return a paretoSet solution on solID position
	Solution::solution Solution::get_solution_not_visited()
	{
		solution s;
		s.maxCost = -1;
		sort(paretoSet.begin(), paretoSet.end(), [](const pair<bool, solution> el1, const pair<bool, solution> el2)
			 { return el1.first < el2.first; });
		if (!paretoSet.front().first)
		{
			// set as visited
			paretoSet.front().first = true;
			s = paretoSet.front().second;
		}
		return s;
	}

	// evaluate a newSol with all paretoSet solutions and return for each objective 0, 1 or 2
	// 0 if newSol is dominated by paretoSet solution.
	// 1 if newSol doesn't dominate the paretoSet solution, but neither solution dominate newSol
	// 2 if newSol dominates some paretoSet soltution's. At this case all ids of dominated solutions are returned within the map vector.
	map<int, vector<int>> Solution::eval_solution(Solution::solution newSol)
	{
		map<int, vector<int>> mapEval;
		auto itMap = mapEval.end();
		int position = 0;
		bool almostEqual, greaterThan, lessThan;

		for (pair<bool, solution> ps : paretoSet)
		{
			solution psSol = ps.second;

			almostEqual = isApproximatelyEqual(round(newSol.maxCost), round(psSol.maxCost));
			greaterThan = round(newSol.maxCost) > round(psSol.maxCost);
			lessThan = round(newSol.maxCost) < round(psSol.maxCost);

			// if solution is dominated for psSol
			if ((almostEqual || greaterThan) && newSol.depotsNum >= psSol.depotsNum)
			{
				itMap = mapEval.emplace_hint(itMap, 0, vector<int>());
				itMap->second.emplace_back(position);
				break;
			}

			// if solution is non-dominated
			else
			{
				// if solution dominates psSol: insert all dominated solution at map
				if ((almostEqual || lessThan) && newSol.depotsNum <= psSol.depotsNum)
				{
					itMap = mapEval.emplace_hint(itMap, 2, vector<int>());
					itMap->second.emplace_back(position);
				}
				// if solution does not dominate psSol, but isn't dominated.
				else
				{
					itMap = mapEval.emplace_hint(itMap, 1, vector<int>());
					itMap->second.emplace_back(position);
				}
			}

			position++;
		}
		return mapEval;
	}

	// return an iterator of paretoSet's solution at solID position
	vector<pair<bool, Solution::solution>>::iterator Solution::getSolIterator(int solID)
	{
		return paretoSet.begin() + solID;
	}

	// update paretoSet if necessary.
	void Solution::update_paretoSet(map<int, vector<int>> mapEval, solution s)
	{

		for (pair<int, vector<int>> map : mapEval)
		{
			// if solution s is dominated break
			if (map.first == 0)
				break;

			// if s is not dominated
			else
			{
				// if s dominate some solution at pareto set
				auto it = mapEval.find(2);
				if (it != mapEval.end())
				{
					// removel all solutions
					while (!(*it).second.empty())
					{
						int p = (*it).second.back();
						erase_solution(getSolIterator(p));
						(*it).second.pop_back();
					}
					// insert solution at pareto set
					insert_solution(s);
					break;
				}
				// if s is independent, insert at pareto set
				else
					insert_solution(s);
			}
		}
	}

	// print all solutions at paretoSet
	void Solution::print_paretoSet()
	{
		for (pair<bool, solution> sol : paretoSet)
		{
			cout << "depotNum: " << sol.second.depotsNum << " Maior custo: " << sol.second.maxCost << endl;
		}
	}

	// check if solution is non-dominated and insert at paretoSet.
	// if the solution is non-dominated return true or false otherwise.
	bool Solution::isAtParetoSet(solution s)
	{
		bool status = false;

		if (paretoSet.empty())
		{
			insert_solution(s);
			return true;
		}

		map<int, vector<int>> mapEval = eval_solution(s);
		if (mapEval.find(1) != mapEval.end() || mapEval.find(2) != mapEval.end())
		{
			update_paretoSet(mapEval, s);
			status = true;
		}
		return status;
	}

	// build a nodesSet from solution and update de coverageSet
	void Solution::solutionToNodesSet(solution s)
	{

		vector<Set> nwNodesSets;
		vector<int> nodes;
		vector<int> depots;
		vector<int> targets;
		// int base =0;
		int type = 0;

		// get depots IDs
		depots.insert(depots.end(), s.depots.begin(), s.depots.end());

		// for each path
		for (path p : s.paths)
		{
			nwNodesSets.push_back(Set());
			// assign robot id to path
			nwNodesSets.back().robotID = p.robotID;

			// insert depots
			nwNodesSets.back().depots = depots;

			nwNodesSets.back().set_id = p.pID;

			// clear targets vector
			targets.clear();
			// push all nodes at vector
			for (auto e : p.edges)
			{
				nodes.emplace_back(e.node_a);
				type = mapNodesTypes.find(e.node_a)->second;

				// if(type == 1)
				// base = node.first.first;
				if (type == 2)
					targets.emplace_back(e.node_a);
			}

			// it's not necessary push the baseID on nodesSet, this operation is done at updateSetNodesCosts
			// depots.emplace_back(base);
			sort(targets.begin(), targets.end());

			// if smaller target is even
			if (targets.front() % 2 == 0)
			{
				for (int id : targets)
				{
					if (id % 2 == 0)
					{
						nwNodesSets.back().cvLines.emplace_back(id);
					}
				}
			}

			// if smaller target is odd
			else
			{
				for (int id : targets)
				{
					if (id % 2 != 0)
						nwNodesSets.back().cvLines.emplace_back(id);
				}
			}
		}
		// update nodesSets
		updateNodesSets(nwNodesSets);
	}

	Solution::Set Solution::PathToNodesSet(path p)
	{
		Set nwNodesSets;
		vector<int> nodes;
		vector<int> depots;
		vector<int> targets;
		// int base =0;
		int type = 0;

		// insert set id
		nwNodesSets.set_id = p.pID;
		// insert depots IDs
		nwNodesSets.depots.insert(nwNodesSets.depots.begin(), p.depots.begin(), p.depots.end());

		// nwNodesSets.cvLines.push_back(e);
		// assign robot id to path
		nwNodesSets.robotID = p.robotID;

		// insert depots
		// nwNodesSets.depots = depots;

		// push all nodes at vector
		for (auto e : p.edges)
		{
			nodes.emplace_back(e.node_a);
			type = mapNodesTypes.find(e.node_a)->second;

			// base = node.first.first;
			if (type == 2)
				targets.emplace_back(e.node_a);
		}

		// it's not necessary push the baseID on nodesSet, this operation is done at updateSetNodesCosts
		// depots.emplace_back(base);
		sort(targets.begin(), targets.end());

		// if smaller target is even
		if (targets.front() % 2 == 0)
		{
			for (int id : targets)
			{
				if (id % 2 == 0)
				{
					nwNodesSets.cvLines.emplace_back(id);
				}
			}
		}

		// if smaller target is odd
		else
		{
			for (int id : targets)
			{
				if (id % 2 != 0)
					nwNodesSets.cvLines.emplace_back(id);
			}
		}

		return nwNodesSets;
	}

	vector<int> Solution::getClosedDepots(solution s)
	{
		vector<int> openDepots;
		vector<int> closedDepots(graphDepotsIndexes.size());
		vector<int>::iterator it;

		// get depots IDs
		openDepots.insert(openDepots.end(), s.depots.begin(), s.depots.end());

		// get the complement of openDepots
		it = set_difference(graphDepotsIndexes.begin(), graphDepotsIndexes.end(),
							openDepots.begin(), openDepots.end(), closedDepots.begin());

		// resize the closedDepots
		closedDepots.resize(it - closedDepots.begin());
		return closedDepots;
	}

	void Solution::perturbation(solution *s, int maxDepots)
	{
		int choose = rand.randNum(2);
		// número da quantidade de depots da solução inicial + 50% //passar como parâmetro
		// ajustar a execução das duas perturbações.

		for (path p : s->paths)
		{
			for (int id : p.depots)
				if (id < 31)
					cerr << "#1 Erro id em na função perturbation!\n";
		}

		if (choose == 0)
		{
			pshift(s);
			for (path p : s->paths)
			{
				for (int id : p.depots)
					if (id < 31)
						cerr << "#2 Erro id em na função perturbation!\n";
			}
		}
		else if (choose == 1)
		{
			openRandomDepot(s);
			for (path p : s->paths)
			{
				for (int id : p.depots)
					if (id < 31)
						cerr << "#3 Erro id em na função perturbation!\n";
			}
		}
		else if (choose == 2)
		{
			openRandomDepot(s);
			for (path p : s->paths)
			{
				for (int id : p.depots)
					if (id < 31)
						cerr << "# 4Erro id em na função perturbation!\n";
			}
			pshift(s);
			for (path p : s->paths)
			{
				for (int id : p.depots)
					if (id < 31)
						cerr << "#5 Erro id em na função perturbation!\n";
			}
		}
	}

	bool Solution::hasAllTargets(solution s)
	{
		vector<int> vnodes;
		vector<int> diff(graphTargetsIndexes.size());
		vector<int>::iterator it;

		for (path p : s.paths)
		{
			for (edge e : p.edges)
			{
				vnodes.emplace_back(e.node_a);
			}
		}

		sort(vnodes.begin(), vnodes.end());
		// get the complement of openDepots
		it = set_difference(graphTargetsIndexes.begin(), graphTargetsIndexes.end(),
							vnodes.begin(), vnodes.end(), diff.begin());
		// resize the closedDepots
		diff.resize(it - diff.begin());

		if (!diff.empty())
			return false;

		return true;
	}

	void Solution::checkTargetPath(int gid, path p)
	{
		vector<int> vnodes;
		vector<int> targets;
		vector<int> diff(graphTargetsIndexes.size());
		vector<int>::iterator it;

		for (edge e : p.edges)
		{
			vnodes.emplace_back(e.node_a);
		}

		int line = -1;
		for (auto it = nodesSets[gid].cvLines.begin(); it != nodesSets[gid].cvLines.end(); ++it)
		{
			line = *it;
			targets.emplace_back(line);
			targets.emplace_back(line + 1);
		}

		sort(vnodes.begin(), vnodes.end());
		sort(targets.begin(), targets.end());
		// get the complement of openDepots
		it = set_difference(targets.begin(), targets.end(),
							vnodes.begin(), vnodes.end(), diff.begin());
		// resize the closedDepots
		diff.resize(it - diff.begin());

		if (!diff.empty())
		{
			cout << "índices perdidos: ";
			for (int i : diff)
			{
				cout << " " << i;
			}
			cout << "\n";
		}
	}

	int Solution::getDepotsNumInit()
	{
		return depotsNumInit;
	}

	int Solution::getTargetsNum()
	{
		return getTargetsGraphIndexNum();
	}

	// verifica se a capacidade do robô no caminho p respeita a restrição de combustível
	// retorna true se a capacidade é satisfeita.
	bool Solution::robotCapacity_val(path p)
	{
		double fuel_required = 0.0;
		double robot_capacity = 0.0;
		double max_fuel_required = 0.0;

		// obtém a base do robô
		int baseID = input.getRobotBaseId(p.robotID);

		robot_capacity = input.getRobotFuel(p.robotID);

		for (const edge &e : p.edges)
		{							 // percorrer todas as arestas
			fuel_required += e.time; // obter o tempo(combustível)

			// se o nós de chegada da aresta for depot ou a base
			if (p.depots.find(e.node_b) != p.depots.end() || e.node_b == baseID)
			{
				if (fuel_required > max_fuel_required)
					max_fuel_required = fuel_required;

				fuel_required = 0.0;
			}
		}
		// Verificar o último trecho caso ele não termine em depot
		if (fuel_required > max_fuel_required)
			max_fuel_required = fuel_required;

		const double tolerance = 1e-6;

		if (robot_capacity + tolerance >= max_fuel_required)
			return (true);

		else
			return (false);
	}

	// verificar o custo do caminho para o novo robô, caso não tenha capacidade de percorrer o caminho retorna o custo negativo (-1)
	Solution::path Solution::RobotCostInPath(path p)
	{
		double fuel_required, robot_capacity, pcost;
		double max_fuel_required = numeric_limits<double>::min();

		// obtém a base do robô
		int baseID = input.getRobotBaseId(p.robotID);
		double fuel_on_a = 0.0;

		robot_capacity = input.getRobotFuel(p.robotID);
		fuel_required = 0;
		pcost = 0;

		// atualizar o custo de cada aresta para o custo do novo robô
		for (auto it_edges = p.edges.begin(); it_edges != p.edges.end(); ++it_edges)
		{
			it_edges->time = getCostOnGraph(p.robotID, it_edges->node_a, it_edges->node_b);
			it_edges->cost = it_edges->time + (it_edges->time * input.getRobotProp(p.robotID));

			auto it_fuel_a = p.fuelOnTarget.find(it_edges->node_a);
			auto it_fuel_b = p.fuelOnTarget.find(it_edges->node_b);

			// se houver a entrada do target
			if (it_fuel_a != p.fuelOnTarget.end())
				fuel_on_a = it_fuel_a->second;
			else
				fuel_on_a = robot_capacity;

			if (it_fuel_b != p.fuelOnTarget.end())
				it_fuel_b->second = fuel_on_a - it_edges->time;

			fuel_required += it_edges->time;
			pcost += it_edges->cost;

			// verificar se o combustível necessário entre depots incluindo a base
			if (p.depots.find(it_edges->node_b) != p.depots.end() || it_edges->node_b == baseID)
			{
				if (fuel_required > max_fuel_required)
					max_fuel_required = fuel_required;
				fuel_required = 0;
			}
		}

		// se capacidade do robô for menor que o custo do maior sub caminho, retonar false
		if (robot_capacity < max_fuel_required)
		{
			// atribuir custo negativo para o caminho, ou seja o robô não tem capacidade de percorrer o caminho
			p.pCost = -1;
			return p;
		}

		// o robô tem capacidade de percorrer o mesmo caminho do robô anterior.
		// atribuir o custo do caminho para o novo robô;
		p.pCost = pcost;

		return p;
	}

	// verifica se o robô tem combustível suficiente para atingir o próximo depot
	// retorna true se a restrição de combustível é satisfeita
	bool Solution::checkRCFromNodeToNextDepot(path p, int node)
	{
		bool node_found = false;
		vector<edge> sub_path;
		double fuel_required_on_path, fuel_remaining = 0;
		double fuel_on_node = 0;
		int baseID = input.getRobotBaseId(p.robotID);
		for (auto it_edges = p.edges.begin(); it_edges != p.edges.end(); ++it_edges)
		{
			// buscar pelo nó no início da aresta
			if (node == it_edges->node_a)
				node_found = true;
			// obter o combustível passando por todas as arestas até chegar no depot ou na base.
			if (node_found)
			{
				fuel_required_on_path += it_edges->time; // obtem o combustível necessário
				// caso atinja algum depot
				if (p.depots.find(it_edges->node_b) != p.depots.end() || it_edges->node_b == baseID)
					break; // saia do loop
			}
		}
		// obter o combustível disponível no robô em node.
		auto itR = p.fuelOnTarget.find(node);
		if (itR != p.fuelOnTarget.end()) // se node for target
			fuel_on_node = itR->second;
		else // se o node for depot, o robô sai com capacidade total
			fuel_on_node = input.getRobotFuel(p.robotID);

		// check if remaining fuel is sufficient to reach next depot
		fuel_remaining = fuel_on_node - fuel_required_on_path;
		// if there is no fuel
		if (fuel_remaining < 0)
			return false;
		return true;
	}

	// verificar a restrição da solução
	bool Solution::checkRestrictions(solution s)
	{
		map<int, pair<int, int>> mapInOut;
		double pathCost = 0;
		double maxCost = 0;

		for (path p : s.paths)
		{
			mapInOut.clear();
			// vector<pair<pair<int,int>,pair<double,double>>> tempNodes = p.nodes;

			vector<edge> edge_temp = p.edges;
			auto it_edges = edge_temp.begin();

			// auto itTemp = tempNodes.begin();
			auto itMap = mapInOut.end();
			pathCost = 0;
			maxCost = numeric_limits<double>::min();
			// get map of all nodes that enter and departure from a node;
			while (it_edges != edge_temp.end())
			{
				// obter a quantidade de saída e entrada de cada vértice
				itMap = mapInOut.emplace_hint(itMap, it_edges->node_a, make_pair(0, 0));
				itMap->second.first++;

				itMap = mapInOut.emplace_hint(itMap, it_edges->node_b, make_pair(0, 0));
				itMap->second.second++;

				// obter a soma de cada aresta
				pathCost += it_edges->cost;

				// obter o maior custo;
				if (maxCost < it_edges->cost)
					maxCost = it_edges->cost;

				++it_edges;
			}

			// check if all targets ids are on solution
			if (!hasAllTargets(s))
			{
				cout << "missing targets";
				return false;
			}
			// check robot capacity restriction is satisfied for path.
			if (!robotCapacity_val(p))
			{
				cout << "Robot Capacity problem\n";
				return false;
			}
			// check path cost
			if (abs(pathCost - p.pCost) > 0.1)
			{
				cout << "Path cost problem";
				return false;
			}
			// check if targets nodes has only one input and one output
			for (int i : graphTargetsIndexes)
			{
				auto elem = mapInOut.find(i);
				if (elem != mapInOut.end())
				{
					if (elem->second.first > 1 || elem->second.second > 1)
					{
						cout << "Problema de visita única";
						return false;
					}
				}
			}
			// check if the paths is eulerian circuit
			for (auto &map : mapInOut)
			{
				if (map.second.first != map.second.second)
				{
					cout << "Não forma um circuito euleriano";
					return false;
				}
			}
		}
		return true;
	}

	// verificar as restrições do problema no caminho
	/*bool Solution::PathRestrictions(path p)
	{
		map<int, pair<int, int>> mapInOut;
		double pathCost = 0;
		double maxCost = 0;

		// for(path p:s.paths){
		mapInOut.clear();
		// vector<pair<pair<int,int>,pair<double,double>>> tempNodes = p.nodes;

		vector<edge> edge_temp = p.edges;
		auto it_edges = edge_temp.begin();

		// auto itTemp = tempNodes.begin();
		auto itMap = mapInOut.end();
		pathCost = 0;
		maxCost = numeric_limits<double>::min();
		// get map of all nodes that enter and departure from a node;
		while (it_edges != edge_temp.end())
		{
			// obter a quantidade de saída e entrada de cada vértice
			itMap = mapInOut.emplace_hint(itMap, it_edges->node_a, make_pair(0, 0));
			itMap->second.first++;

			itMap = mapInOut.emplace_hint(itMap, it_edges->node_b, make_pair(0, 0));
			itMap->second.second++;

			// obter a soma de cada aresta
			pathCost += it_edges->cost;

			// obter o maior custo;
			if (maxCost < it_edges->cost)
				maxCost = it_edges->cost;

			// se tiver loop
			if (it_edges->node_a == it_edges->node_b)
			{
				return false;
			}

			++it_edges;
		}

		// check robot capacity restriction is satisfied for path.
		if (!robotCapacity_val(p))
			return false;
		// check path cost
		if (abs(pathCost - p.pCost) > 0.1)
			return false;
		// check if targets nodes has only one input and one output
		for (int i : graphTargetsIndexes)
		{
			auto elem = mapInOut.find(i);
			if (elem != mapInOut.end())
			{
				if (elem->second.first > 1 || elem->second.second > 1)
					return false;
			}
		}
		// check if the paths is eulerian circuit
		for (auto &map : mapInOut)
		{
			if (map.second.first != map.second.second)
				return false;
		}

		if (HasRepeatedCLines(p))
		{
			return false;
		}

		if (!IsCircuit(p))
			return false;
		return true;
	}*/

	bool Solution::PathRestrictions(path p)
	{
		map<int, pair<int, int>> mapInOut;
		double pathCost = 0;
		double maxCost = 0;

		mapInOut.clear();
		vector<edge> edge_temp = p.edges;
		auto it_edges = edge_temp.begin();
		pathCost = 0;
		maxCost = numeric_limits<double>::min();

		// 1. Contabiliza entradas e saídas de cada nó e soma o custo
		while (it_edges != edge_temp.end())
		{
			// Loop inválido
			if (it_edges->node_a == it_edges->node_b)
				return false;

			// Incrementa contador de saída
			auto resA = mapInOut.emplace(it_edges->node_a, make_pair(0, 0));
			resA.first->second.first++;

			// Incrementa contador de entrada
			auto resB = mapInOut.emplace(it_edges->node_b, make_pair(0, 0));
			resB.first->second.second++;

			pathCost += it_edges->cost;
			if (maxCost < it_edges->cost)
				maxCost = it_edges->cost;

			++it_edges;
		}

		// 2. Verifica capacidade do robô
		if (!robotCapacity_val(p))
			return false;

		// 3. Verifica se o custo acumulado bate com p.pCost
		if (abs(pathCost - p.pCost) > 0.1)
			return false;

		// 4. Cada target deve ter exatamente 1 entrada e 1 saída
		for (int i : graphTargetsIndexes)
		{
			auto elem = mapInOut.find(i);
			if (elem != mapInOut.end())
			{
				if (elem->second.first != 1 || elem->second.second != 1)
					return false;
			}
		}

		// 5. Todos os nós devem ter entradas = saídas
		for (auto &map : mapInOut)
		{
			if (map.second.first != map.second.second)
				return false;
		}

		// 6. Checa se há repetição de linhas de cobertura
		if (HasRepeatedCLines(p))
			return false;

		// 7. Confirma se é um circuito válido
		if (!IsCircuit(p))
			return false;

		return true;
	}

	vector<bool> Solution::paretoSetValidation()
	{
		vector<bool> val;
		for (pair<bool, solution> s : paretoSet)
		{
			val.push_back(checkRestrictions(s.second));
		}
		return val;
	}

	// Remover uma "linha de cobertura" (CL - Coverage Line) de um caminho e recalcular todos os parâmetros afetados.
	Solution::path Solution::removeCL(path p, int cl)
	{
		double pcost = 0;
		int t_1 = cl;
		int t_2 = cl + 1;

		list<edge> edges(p.edges.begin(), p.edges.end());

		double fuel_on_node_a = 0;
		double fuel_on_node_b = 0;

		double robot_capacity = input.getRobotFuel(p.robotID);

		// Verifica se o caminho tem arestas
		if (edges.empty())
		{
			p.pCost = -1; // Marca como inválido
			return p;
		}

		// Encontrar a aresta da linha de cobertura (t1, t2)
		auto edge_b = edges.end();
		auto it_edge = edges.begin();
		while (it_edge != edges.end())
		{
			if ((it_edge->node_a == t_1 && it_edge->node_b == t_2) ||
				(it_edge->node_b == t_1 && it_edge->node_a == t_2))
			{
				edge_b = it_edge;
				break;
			}
			++it_edge;
		}

		// Se não encontrou a CL, retorna caminho inválido
		if (edge_b == edges.end())
		{
			p.pCost = -1;
			return p;
		}

		// Identificar edge_a (anterior) e edge_c (posterior)
		auto edge_a = edge_b;
		auto edge_c = edge_b;
		bool has_a = (edge_b != edges.begin());
		bool has_c = (++edge_c != edges.end());

		if (has_a)
			--edge_a;
		else
			edge_a = edges.end();

		// Caso especial: não há anterior ou posterior suficiente
		if (!has_a || !has_c)
		{
			p.pCost = -1;
			return p;
		}

		// Conectar nó da aresta anterior com o da posterior
		edge_a->node_b = edge_c->node_b;

		// Se virou loop, remover o triângulo (a, b, c)
		if (edge_a->node_a == edge_a->node_b)
		{
			// Remove combustível dos nós da CL
			p.fuelOnTarget.erase(edge_b->node_a);
			p.fuelOnTarget.erase(edge_b->node_b);

			edges.erase(edge_b);
			edges.erase(edge_a); // edge_a antes de edge_b
			edges.erase(edge_c);

			for (const auto &e : edges)
				pcost += e.cost;

			p.pCost = pcost;
			p.targetsNum = p.fuelOnTarget.size();

			p.edges.assign(edges.begin(), edges.end());

			if (!robotCapacity_val(p))
				p.pCost = -1;

			return p;
		}

		// Atualiza tempo e custo da nova aresta combinada
		edge_a->time = getCostOnGraph(p.robotID, edge_a->node_a, edge_a->node_b);
		edge_a->cost = edge_a->time + (input.getRobotProp(p.robotID) * edge_a->time);

		// Atualiza combustível
		auto it_fuel = p.fuelOnTarget.find(edge_a->node_a);
		fuel_on_node_a = (it_fuel != p.fuelOnTarget.end()) ? it_fuel->second : robot_capacity;
		fuel_on_node_b = fuel_on_node_a - edge_a->time;

		// Atualizar combustível no novo destino (node_b da nova edge)
		if ((it_fuel = p.fuelOnTarget.find(edge_a->node_b)) != p.fuelOnTarget.end())
			it_fuel->second = fuel_on_node_b;

		// Remover targets da LC
		p.fuelOnTarget.erase(edge_b->node_a);
		p.fuelOnTarget.erase(edge_b->node_b);

		// Atualizar combustíveis após edge_a até próximo depósito (ou fim)
		it_edge = ++edge_a; // começa da próxima após edge_a
		fuel_on_node_a = fuel_on_node_b;

		while (it_edge != edges.end())
		{
			auto next_target = p.fuelOnTarget.find(it_edge->node_b);
			if (next_target != p.fuelOnTarget.end())
			{
				fuel_on_node_b = fuel_on_node_a - it_edge->time;
				next_target->second = fuel_on_node_b;

				fuel_on_node_a = fuel_on_node_b;
			}
			else
			{
				break;
			}
			++it_edge;
		}

		// Remover edge_b e edge_c
		edges.erase(edge_b);
		edges.erase(edge_c);

		for (const auto &e : edges)
			pcost += e.cost;

		p.pCost = pcost;
		p.targetsNum = p.fuelOnTarget.size();
		p.edges.assign(edges.begin(), edges.end());

		if (!robotCapacity_val(p))
			p.pCost = -1;

		return p;
	}

	// return true if the path is circuit....antigo
	/*bool Solution::IsCircuit(path p)
	{
		int baseID = input.getRobotBaseId(p.robotID);
		list<edge> edges_list;

		// convert vector to list
		edges_list.insert(edges_list.begin(), p.edges.begin(), p.edges.end());

		// caso o nó de saída não seja o mesmo de retorno e não seja a base do robô, retorna falso
		if (edges_list.front().node_a != edges_list.back().node_b || edges_list.front().node_a != baseID)
			return false;

		// remover a base
		edges_list.pop_front();

		// para cada aresta do caminho verificar se é possível realizar o caminhamento até a volta à base.
		for (auto e : p.edges)
		{
			if (e.node_b == edges_list.front().node_a)
				edges_list.pop_front();
		}

		if (!edges_list.empty())
			return false;

		return true;
	}*/

	bool Solution::IsCircuit(path p)
	{
		int baseID = input.getRobotBaseId(p.robotID);

		if (p.edges.empty())
			return false;

		// Verifica se começa e termina na base
		if (p.edges.front().node_a != baseID || p.edges.back().node_b != baseID)
			return false;

		// Verifica continuidade entre arestas
		for (size_t i = 1; i < p.edges.size(); ++i)
		{
			if (p.edges[i - 1].node_b != p.edges[i].node_a)
				return false;
		}

		return true;
	}

	Solution::path Solution::new_insert(path p, int out, int new_cl)
	{
		list<edge> edges_list;
		set<int> depots;
		path path_temp;

		// obter a lista de arestas
		edges_list.insert(edges_list.begin(), p.edges.begin(), p.edges.end());

		// obter o novo caminho do nó de saída, ligando a nova linha de cobertura e a próxima linha de cobertura do nó de saída.
		// o reabastecimento é ajustado caso necessário.
		path_temp = link_out_cl_in(p, out, new_cl);

		for (int id : path_temp.depots)

			if (id < 31)
				cerr << "#1 Problema id new insert!\n";

		if (path_temp.pCost <= 0)
		{
			p.pCost = -1;
			return p;
		}

		// obter a posição da aresta de saída da linha de cobertura, referente ao nó out.
		auto it_first = edges_list.begin();
		while (it_first != edges_list.end())
		{
			if (it_first->node_a == path_temp.edges.front().node_a)
				break;
			++it_first;
		}
		// obter a próxima linha de cobertura, posterior ao nó out

		edge temp_next_cv;

		auto path_temp_it = path_temp.edges.crbegin();
		auto fuel_it = path_temp.fuelOnTarget.begin();
		while (path_temp_it != path_temp.edges.crend())
		{
			fuel_it = path_temp.fuelOnTarget.find(path_temp_it->node_b);
			if (fuel_it != path_temp.fuelOnTarget.end())
			{
				temp_next_cv.node_a = path_temp_it->node_b;
				break;
			}
			++path_temp_it;
		}

		temp_next_cv.node_b = path_temp.edges.back().node_b;

		auto it_second = it_first;
		while (it_second != edges_list.end())
		{
			if (it_second->node_a == temp_next_cv.node_a && it_second->node_b == temp_next_cv.node_b)
			{
				++it_second;
				break;
			}
			++it_second;
		}

		// apagar as arestas intermediárias de out a proxima cl inclusive.
		it_first = edges_list.erase(it_first, it_second);

		// inserir o novo caminho com a nova aresta em edge_list
		edges_list.insert(it_first, path_temp.edges.begin(), path_temp.edges.end());

		// apagar o caminho antigo em path
		p.edges.clear();

		// inserir o novo caminho em path
		p.edges.insert(p.edges.begin(), edges_list.begin(), edges_list.end());

		// atualizar os combustíveis nos targets
		auto p_fuel = p.fuelOnTarget.begin();
		for (auto path_temp_fuel : path_temp.fuelOnTarget)
		{
			p_fuel = p.fuelOnTarget.find(path_temp_fuel.first);
			if (p_fuel != p.fuelOnTarget.end()) // se existir atualizar
				p_fuel->second = path_temp_fuel.second;
			else // caso contrário inserir o novo target
				p.fuelOnTarget.emplace(path_temp_fuel.first, path_temp_fuel.second);
		}

		// atualizar os depósitos e o custo em cada target
		p_fuel = p.fuelOnTarget.begin();
		double cost = 0;
		for (auto e : p.edges)
		{
			cost += e.cost;
			p_fuel = p.fuelOnTarget.find(e.node_a);
			if (p_fuel == p.fuelOnTarget.end() && e.node_a != input.getRobotBaseId(p.robotID))
			{
				if (e.node_a < 31)
					cerr << "Problema: ID \n";
				depots.insert(e.node_a);
			}
		}

		// atualizar as informações do caminho.
		p.depots.clear();
		p.depots = depots;
		p.depotsNum = p.depots.size() + 1;
		p.targetsNum = p.fuelOnTarget.size();
		p.pCost = cost;

		for (int id : p.depots)
			if (id < 31)
				cerr << "1 Erro id em na função perturbation!\n";

		//---------------------------------------------------teste temporário da saída-----------------------
		if (!PathRestrictions(p))
		{
			cout << "restrição não respeitada: new_insert" << endl;
			p.pCost = -1;
		}
		//-----------------------------------------------------------------------------------------

		return p;
	}

	Solution::path Solution::Link_Sets_Node_out_in(path p, int out, int in)
	{
		double fuel_remaining = 0;
		double fuel_required = 0;
		double fuel_on_arrival;
		pair<int, int> dir_out;

		edge edge_temp;

		// informações temporária do caminho para ao término ser possível atualizar p.
		path path_temp;
		path p_segment;

		// lista de aresta para ir do nó out até o próximo depot apos o nó in
		list<edge> edge_list;

		// lista d controle
		// contém as arestas que deverão constar em edge_list e o custo e combustível para percorrê-las deverão ser calculados
		list<edge> new_edges;

		path_temp.robotID = p.robotID;
		path_temp.pID = p.pID;
		path_temp.edges.clear();

		int sl1 = 0;
		auto it_list = edge_list.begin();

		// obter o combustível do robô no nó de saída
		auto it_fuel_p = p.fuelOnTarget.find(out);
		// if(it_fuel_p != p.fuelOnTarget.end())
		// path_temp.fuelOnTarget.emplace(out, it_fuel_p->second);

		// combustível necessário para sair de out e atingir o nó  in
		fuel_required = getCostOnGraph(p.robotID, out, in);

		// arco que liga out e in
		edge_temp.node_a = out;
		edge_temp.node_b = in;
		edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
		edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));

		if (out != in)
			new_edges.emplace_back(edge_temp);

		// obter os arcos após a ligação (out,in) até o próximo depot
		// objetivo é ajustar os gastos de combustível até o próximo abastecimento
		auto it_first = p.edges.begin();
		auto it_second = p.edges.begin();
		auto it_edges = p.edges.begin();
		auto it_depot = p.depots.begin();

		int baseID = input.getRobotBaseId(p.robotID);
		int n_b = in;

		auto it_after = p.edges.begin();
		while (it_edges != p.edges.end())
		{

			if (it_edges->node_a == out && it_edges->node_b == baseID)
			{
				it_first = it_edges;
				it_after = it_first;
				// it_after--;
				// if(out == in)
				// edge_list.emplace_back(*it_after);
				break;
			}
			it_edges++;
		}

		it_edges = it_first;
		while (it_edges != p.edges.end())
		{
			if (it_edges->node_a == baseID && it_edges->node_b == n_b)
			{
				it_depot = p.depots.find(it_edges->node_b);
				while (it_depot == p.depots.end())
				{
					it_edges++;
					new_edges.emplace_back(*it_edges);
					it_depot = p.depots.find(it_edges->node_b);
				}

				if (it_depot != p.depots.end())
				{ // se for depot
					it_second = ++it_edges;
					break;
				}
			}
			it_edges++;
		}

		auto it_new_edges = new_edges.begin();
		while (!new_edges.empty())
		{

			fuel_required = it_new_edges->time;

			it_fuel_p = p.fuelOnTarget.find(it_new_edges->node_a);
			if (it_fuel_p != p.fuelOnTarget.end())
				fuel_remaining = it_fuel_p->second;
			else
				fuel_remaining = input.getRobotFuel(path_temp.robotID);

			fuel_on_arrival = fuel_remaining - fuel_required;

			// se o robô não tiver capacidade para percorrer o caminho
			if (fuel_required > input.getRobotFuel(path_temp.robotID) && !initialSolution)
			{
				path_temp.pCost = -1; // inviável
				return path_temp;
			}

			// inserir o abastecimento no posto associado ao nó out
			if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
			{
				// se for link entre linhas de cobertura, não coverage line
				if (!IsCLine(it_new_edges->node_a, it_new_edges->node_b))
				{
					// obter o segmento do menor caminho possível lingando node_a e node_b
					// passando pelos postos.
					p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);

					if (p_segment.edges.empty())
					{
						int depot_id = input.getDepotIdOnTarget(it_new_edges->node_a);

						auto it_depot = p.depots.find(depot_id);
						if (it_depot == p.depots.end() && initialSolution)
						{
							if (depot_id != baseID)
								p.depots.insert(depot_id);
							p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);
						}
						else
						{
							path_temp.pCost = -1;
							return path_temp;
						}
					}
					// atualizar o custo do segmento em path_temp;
					path_temp.pCost += p_segment.pCost;

					// inserir as arestas do segmento na lista de arestas
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel_p != p.fuelOnTarget.end())
							it_fuel_p->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}
				}
				else
				{ // se for linha de cobertura
					// auto edge_list_temp = edge_list;
					// desviar a aresta anterior para o posto associado ao primeiro nó da nova linha de cobertura
					edge link_e = edge_list.back();

					path_temp.pCost -= edge_list.back().cost; // remover o custo antigo da variável de custo

					// edge_list.back().node_b = input.getDepotIdOnTarget(it_new_edges->node_a);
					// obter o segmento o menor caminho possível lingando node_a e node_b

					p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);

					if (p_segment.edges.empty())
					{
						int depot_id = input.getDepotIdOnTarget(edge_list.back().node_a);

						auto it_depot = p.depots.find(depot_id);
						// se o for a solução inicial permitir que o depot na saída seja aberto para
						// obter uma rota viável na solução inicial. Para as demais soluçõe,
						// não permitir que seja aberto novos depots, visto que podemos entrar em
						// loop com a abertura e fechamento de depot
						if (it_depot == p.depots.end() && initialSolution)
						{
							p.depots.insert(depot_id);
							p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);
							if (p_segment.edges.empty())
							{
								path_temp.pCost = -1;
								return path_temp;
							}
						}
						else
						{
							path_temp.pCost = -1;
							return path_temp;
						}
					}

					path_temp.pCost += p_segment.pCost;
					// remove a tentative anterior, que é inviável para a capacidade do robô
					edge_list.pop_back();

					// armazenar a quantidade de elementos existentes após a remoção;
					// o objetivo é manter esses elementos na lista
					sl1 = edge_list.size() + 1; // tamanho da lista após a remoção
					// inserir as arestas do segmento na lista de aresta
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel_p != p.fuelOnTarget.end())
							it_fuel_p->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// inserir a linha de cobertura
					edge_temp.node_a = it_new_edges->node_a;
					edge_temp.node_b = it_new_edges->node_b;
					edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
					edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
					edge_list.emplace_back(edge_temp);
					path_temp.pCost += edge_temp.cost;

					// obter o combustível restante no nó de partida node_a.
					it_fuel_p = p.fuelOnTarget.find(edge_temp.node_a);
					if (it_fuel_p != p.fuelOnTarget.end())
						fuel_remaining = it_fuel_p->second;

					// combustível necessário para o deslocamento
					fuel_required = edge_temp.time;
					// combustível que restará no robô após o deslocamento
					fuel_on_arrival = fuel_remaining - fuel_required;

					// se o combustível necessário para cobrir a aresta for insuficiente
					if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
					{

						if (initialSolution)
						{
							path_temp.pCost = 0;

							// verificar a viabilidade é alcançada
							// se abrir um depot no primeiro target
							int depot_id = input.getDepotIdOnTarget(link_e.node_b);
							p.depots.insert(depot_id);
							p_segment = GetSPTOverOpenDepots(p, link_e.node_a, link_e.node_b);

							if (p_segment.edges.empty())
							{
								path_temp.pCost = -1;
								return path_temp;
							}

							path_temp.pCost += p_segment.pCost;
							// ante-penultimo p_segement. Removemos só estes, substituindo pelo valor do novo p_segment
							it_list = edge_list.begin();
							// remove a tentative anterior, que foi considerada inviável para a capacidade do robô
							advance(it_list, sl1); // saltar sl1 elementos;
							// remove a tentative anterior, que foi considerada inviável para a capacidade do robô
							edge_list.erase(it_list, edge_list.end());

							// inserir as arestas do segmento na lista de aresta
							edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());
							// atualizar o vetor de combustível em cada target
							for (auto fuel_segment : p_segment.fuelOnTarget)
							{
								it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
								if (it_fuel_p != p.fuelOnTarget.end())
									it_fuel_p->second = fuel_segment.second;
								else
									p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
							}

							// inserir a linha de cobertura
							edge_temp.node_a = it_new_edges->node_a;
							edge_temp.node_b = it_new_edges->node_b;
							edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
							edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
							edge_list.emplace_back(edge_temp);
							path_temp.pCost += edge_temp.cost;

							// obter o combustível restante no nó de partida node_a.
							it_fuel_p = p.fuelOnTarget.find(edge_temp.node_a);
							if (it_fuel_p != p.fuelOnTarget.end())
								fuel_remaining = it_fuel_p->second;

							// combustível necessário para o deslocamento
							fuel_required = edge_temp.time;
							// combustível que restará no robô após o deslocamento
							fuel_on_arrival = fuel_remaining - fuel_required;
							if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
							{
								path_temp.pCost = -1;
								return path_temp;
							}
						}

						else
						{
							path_temp.pCost = -1;
							return path_temp;
						}
					}
					// atualizar o combustível em node_b após o deslocamento
					it_fuel_p = p.fuelOnTarget.find(edge_temp.node_b);
					if (it_fuel_p != p.fuelOnTarget.end()) // se o target já existir em fuelontarget
						it_fuel_p->second = fuel_on_arrival;
					else // caso o nó não tenha sido alterado
						p.fuelOnTarget.emplace(edge_temp.node_b, fuel_on_arrival);
				}
			}

			else
			{
				// atualizar o combustível no nó
				if (input.isTarget(it_new_edges->node_b))
				{
					it_fuel_p = p.fuelOnTarget.find(it_new_edges->node_b);
					if (it_fuel_p != p.fuelOnTarget.end())
						it_fuel_p->second = fuel_on_arrival;
					else // caso não tenha o target na lista
						p.fuelOnTarget.emplace(it_new_edges->node_b, fuel_on_arrival);
				}

				// edge_list.emplace_back(*it_edge);
				edge_list.push_back(*it_new_edges);
				path_temp.pCost += it_new_edges->cost;
			}
			// remover o elemento processado
			new_edges.pop_front();
			// atualizar o iterator para o novo início da lista
			it_new_edges = new_edges.begin();
		}

		// remover o caminho do nó out até o próximo depot após o nó in
		it_first = p.edges.erase(it_first, it_second);

		// inserir o novo caminho
		p.edges.insert(it_first, edge_list.begin(), edge_list.end());

		// atualizar targetNum
		p.targetsNum = p.fuelOnTarget.size();

		return p;
	}

	/*
		Solution::path Solution::Link_Sets_Node_out_in(path p, int out, int in)
		{
			double fuel_remaining = 0;
			double fuel_required = 0;
			double fuel_on_arrival;
			pair<int, int> dir_out;

			edge edge_temp;

			// informações temporária do caminho para ao término ser possível atualizar p.
			path path_temp;
			path p_segment;

			// lista de aresta para ir do nó out até o próximo depot apos o nó in
			list<edge> edge_list;

			// lista d controle
			// contém as arestas que deverão constar em edge_list e o custo e combustível para percorrê-las deverão ser calculados
			list<edge> new_edges;

			path_temp.robotID = p.robotID;
			path_temp.pID = p.pID;
			path_temp.edges.clear();

			int sl1 = 0;
			auto it_list = edge_list.begin();

			// obter o combustível do robô no nó de saída
			auto it_fuel_p = p.fuelOnTarget.find(out);
			// if(it_fuel_p != p.fuelOnTarget.end())
			// path_temp.fuelOnTarget.emplace(out, it_fuel_p->second);

			// combustível necessário para sair de out e atingir o nó  in
			fuel_required = getCostOnGraph(p.robotID, out, in);

			// arco que liga out e in
			edge_temp.node_a = out;
			edge_temp.node_b = in;
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));

			if (out != in)
				new_edges.emplace_back(edge_temp);

			// obter os arcos após a ligação (out,in) até o próximo depot
			// objetivo é ajustar os gastos de combustível até o próximo abastecimento
			auto it_first = p.edges.begin();
			auto it_second = p.edges.begin();
			auto it_edges = p.edges.begin();
			auto it_depot = p.depots.begin();

			int baseID = input.getRobotBaseId(p.robotID);
			int n_b = in;
			while (it_edges != p.edges.end())
			{

				if (it_edges->node_a == out && it_edges->node_b == baseID)
				{
					it_first = it_edges;
					break;
				}
				it_edges++;
			}

			it_edges = it_first;
			while (it_edges != p.edges.end())
			{
				if (it_edges->node_a == baseID && it_edges->node_b == n_b)
				{
					it_depot = p.depots.find(it_edges->node_b);
					while (it_depot == p.depots.end())
					{
						it_edges++;
						new_edges.emplace_back(*it_edges);
						it_depot = p.depots.find(it_edges->node_b);
					}

					if (it_depot != p.depots.end())
					{ // se for depot
						it_second = ++it_edges;
						break;
					}
				}
				it_edges++;
			}
			auto it_new_edges = new_edges.begin();
			while (!new_edges.empty())
			{

				fuel_required = it_new_edges->time;

				it_fuel_p = p.fuelOnTarget.find(it_new_edges->node_a);
				if (it_fuel_p != p.fuelOnTarget.end())
					fuel_remaining = it_fuel_p->second;
				else
					fuel_remaining = input.getRobotFuel(path_temp.robotID);

				fuel_on_arrival = fuel_remaining - fuel_required;

				// se o robô não tiver capacidade para percorrer o caminho
				if (fuel_required > input.getRobotFuel(path_temp.robotID) && !initialSolution)
				{
					path_temp.pCost = -1; // inviável
					return path_temp;
				}

				// inserir o abastecimento no posto associado ao nó out
				if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
				{
					// se for link entre linhas de cobertura, não coverage line
					if (!IsCLine(it_new_edges->node_a, it_new_edges->node_b))
					{
						// obter o segmento do menor caminho possível ligando node_a e node_b
						// passando pelos postos.
						p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);

						if (p_segment.edges.empty())
						{
							int depot_id = input.getDepotIdOnTarget(it_new_edges->node_a);

							auto it_depot = p.depots.find(depot_id);
							if (it_depot == p.depots.end() && initialSolution)
							{

								if (depot_id != baseID)
									p.depots.insert(depot_id);
								p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);
							}
							else
							{
								path_temp.pCost = -1;
								return path_temp;
							}
						}
						// atualizar o custo do segmento em path_temp;
						path_temp.pCost += p_segment.pCost;

						// inserir as arestas do segmento na lista de arestas
						edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

						// atualizar o vetor de combustível em cada target
						for (auto fuel_segment : p_segment.fuelOnTarget)
						{
							it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
							if (it_fuel_p != p.fuelOnTarget.end())
								it_fuel_p->second = fuel_segment.second;
							else
								p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
						}
					}
					else
					{ // se for linha de cobertura
						// auto edge_list_temp = edge_list;
						// desviar a aresta anterior para o posto associado ao primeiro nó da nova linha de cobertura
						edge link_e = edge_list.back();

						path_temp.pCost -= edge_list.back().cost; // remover o custo antigo da variável de custo

						// edge_list.back().node_b = input.getDepotIdOnTarget(it_new_edges->node_a);
						// obter o segmento o menor caminho possível lingando node_a e node_b

						p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);

						if (p_segment.edges.empty())
						{
							int depot_id = input.getDepotIdOnTarget(edge_list.back().node_a);

							auto it_depot = p.depots.find(depot_id);
							// se o for a solução inicial permitir que o depot na saída seja aberto para
							// obter uma rota viável na solução inicial. Para as demais soluçõe,
							// não permitir que seja aberto novos depots, visto que podemos entrar em
							// loop com a abertura e fechamento de depot
							if (it_depot == p.depots.end() && initialSolution)
							{
								p.depots.insert(depot_id);
								p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);
								if (p_segment.edges.empty())
								{
									path_temp.pCost = -1;
									return path_temp;
								}
							}
							else
							{
								path_temp.pCost = -1;
								return path_temp;
							}
						}

						path_temp.pCost += p_segment.pCost;
						// remove a tentative anterior, que é inviável para a capacidade do robô
						edge_list.pop_back();

						// armazenar a quantidade de elementos existentes após a remoção;
						// o objetivo é manter esses elementos na lista
						sl1 = edge_list.size() + 1; // tamanho da lista após a remoção
						// inserir as arestas do segmento na lista de aresta
						edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

						// atualizar o vetor de combustível em cada target
						for (auto fuel_segment : p_segment.fuelOnTarget)
						{
							it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
							if (it_fuel_p != p.fuelOnTarget.end())
								it_fuel_p->second = fuel_segment.second;
							else
								p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
						}

						// inserir a linha de cobertura
						edge_temp.node_a = it_new_edges->node_a;
						edge_temp.node_b = it_new_edges->node_b;
						edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
						edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
						edge_list.emplace_back(edge_temp);
						path_temp.pCost += edge_temp.cost;

						// obter o combustível restante no nó de partida node_a.
						it_fuel_p = p.fuelOnTarget.find(edge_temp.node_a);
						if (it_fuel_p != p.fuelOnTarget.end())
							fuel_remaining = it_fuel_p->second;

						// combustível necessário para o deslocamento
						fuel_required = edge_temp.time;
						// combustível que restará no robô após o deslocamento
						fuel_on_arrival = fuel_remaining - fuel_required;

						// se o combustível necessário para cobrir a aresta for insuficiente
						if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
						{

							if (initialSolution)
							{
								path_temp.pCost = 0;

								// verificar a viabilidade é alcançada
								// se abrir um depot no primeiro target
								int depot_id = input.getDepotIdOnTarget(link_e.node_b);
								p.depots.insert(depot_id);
								p_segment = GetSPTOverOpenDepots(p, link_e.node_a, link_e.node_b);

								if (p_segment.edges.empty() || isDefinitelyLessThan(p_segment.pCost, 0.0))
								{
									path_temp.pCost = -1;
									return path_temp;
								}

								path_temp.pCost += p_segment.pCost;
								it_list = edge_list.begin();
								advance(it_list, sl1); // saltar sl1 elementos;
								// remove a tentative anterior, que foi considerada inviável para a capacidade do robô
								edge_list.erase(it_list, edge_list.end());
								// inserir as arestas do segmento na lista de aresta
								edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());
								// atualizar o vetor de combustível em cada target
								for (auto fuel_segment : p_segment.fuelOnTarget)
								{
									it_fuel_p = p.fuelOnTarget.find(fuel_segment.first);
									if (it_fuel_p != p.fuelOnTarget.end())
										it_fuel_p->second = fuel_segment.second;
									else
										p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
								}

								// inserir a linha de cobertura
								edge_temp.node_a = it_new_edges->node_a;
								edge_temp.node_b = it_new_edges->node_b;
								edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
								edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
								edge_list.emplace_back(edge_temp);
								path_temp.pCost += edge_temp.cost;

								// obter o combustível restante no nó de partida node_a.
								it_fuel_p = p.fuelOnTarget.find(edge_temp.node_a);
								if (it_fuel_p != p.fuelOnTarget.end())
									fuel_remaining = it_fuel_p->second;

								// combustível necessário para o deslocamento
								fuel_required = edge_temp.time;
								// combustível que restará no robô após o deslocamento
								fuel_on_arrival = fuel_remaining - fuel_required;
								if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
								{
									path_temp.pCost = -1;
									return path_temp;
								}
							}

							else
							{
								path_temp.pCost = -1;
								return path_temp;
							}
						}
						// atualizar o combustível em node_b após o deslocamento
						it_fuel_p = p.fuelOnTarget.find(edge_temp.node_b);
						if (it_fuel_p != p.fuelOnTarget.end()) // se o target já existir em fuelontarget
							it_fuel_p->second = fuel_on_arrival;
						else // caso o nó não tenha sido alterado
							p.fuelOnTarget.emplace(edge_temp.node_b, fuel_on_arrival);
					}
				}

				else
				{
					// atualizar o combustível no nó
					if (input.isTarget(it_new_edges->node_b))
					{
						it_fuel_p = p.fuelOnTarget.find(it_new_edges->node_b);
						if (it_fuel_p != p.fuelOnTarget.end())
							it_fuel_p->second = fuel_on_arrival;
						else // caso não tenha o target na lista
							p.fuelOnTarget.emplace(it_new_edges->node_b, fuel_on_arrival);
					}

					// edge_list.emplace_back(*it_edge);
					edge_list.push_back(*it_new_edges);
					path_temp.pCost += it_new_edges->cost;
				}
				// remover o elemento processado
				new_edges.pop_front();
				// atualizar o iterator para o novo início da lista
				it_new_edges = new_edges.begin();
			}

			// remover o caminho do nó out até o próximo depot após o nó in
			it_first = p.edges.erase(it_first, it_second);

			// inserir o novo caminho
			p.edges.insert(it_first, edge_list.begin(), edge_list.end());

			// atualizar targetNum
			p.targetsNum = p.fuelOnTarget.size();

			return p;
		}*/

	/**
	 * @brief Gera um segmento de caminho para inserção de uma nova linha de cobertura após uma já existente.
	 *
	 * A função recebe o caminho atual `p`, um ponto de desconexão após a linha de cobertura `out`,
	 * e uma nova linha de cobertura (`new_cl`) a ser inserida. Ela constrói e retorna o trecho de rota
	 * necessário para visitar a nova linha de cobertura e reconectar ao caminho original,
	 * seja na próxima linha de cobertura ou diretamente à base, respeitando os limites de combustível do robô.
	 *
	 * Caso não exista um segmento viável — seja por falta de combustível ou ausência de desvios possíveis —
	 * a função retorna um caminho com `pCost = -1`.
	 *
	 * @param p Caminho atual do robô, contendo a linha de cobertura `out`.
	 * @param out Identificador da linha de cobertura já visitada, antes da desconexão.
	 * @param new_cl Início da nova linha de cobertura a ser inserida após `out`.
	 * @return path Segmento de caminho válido para reconexão ou caminho inválido (pCost = -1).
	 */
	Solution::path Solution::link_out_cl_in(path p, int out, int new_cl)
	{
		double fuel_remaining = 0;
		double fuel_required = 0;
		double fuel_on_arrival;
		int new_cl_n1; // nó de entra da cl
		int new_cl_n2; // nó de saída
		edge next_cl;
		pair<int, int> dir_out;

		edge edge_temp;
		path path_temp = p;
		path p_segment;

		vector<edge> edge_vec;
		list<edge> edge_list;
		list<edge> new_edges;

		path_temp.edges.clear();
		path_temp.pCost = 0;
		path_temp.fuelOnTarget.clear();

		// obter a direção da linha de cobertura de saída
		dir_out = getCLDirection(p, out);

		// descobrir qual nó da nova linha de cobertura está mais próximo de out
		double fuel_out_to_new_cl_n1 = getCostOnGraph(p.robotID, dir_out.second, new_cl);
		double fuel_out_to_new_cl_n2 = getCostOnGraph(p.robotID, dir_out.second, new_cl + 1);

		// combustível necessário para sair de out e atingir o nó mais próximo de new_cl;
		fuel_required = isDefinitelyLessThan(fuel_out_to_new_cl_n1, fuel_out_to_new_cl_n2) ? fuel_out_to_new_cl_n1 : fuel_out_to_new_cl_n2;

		// obter o nó da nova linha de cobertura mais próximo do nó out.
		if (isDefinitelyLessThan(fuel_out_to_new_cl_n1, fuel_out_to_new_cl_n2))
		{
			new_cl_n1 = new_cl;
			new_cl_n2 = new_cl + 1;
		}
		else
		{
			new_cl_n1 = new_cl + 1;
			new_cl_n2 = new_cl;
		}

		// linha de cobertura anterior a nova linha de cobertura
		edge_temp.node_a = dir_out.second;
		edge_temp.node_b = new_cl_n1;
		edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
		edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
		edge_vec.emplace_back(edge_temp);
		new_edges.emplace_back(edge_temp);

		// nova linha de cobertura direcionada de n1 a n2
		edge_temp.node_a = new_cl_n1;
		edge_temp.node_b = new_cl_n2;
		edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
		edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
		edge_vec.emplace_back(edge_temp);
		new_edges.emplace_back(edge_temp);

		// obter a próxima linha de cobertura, antiga ligação com out
		next_cl = GetNextCL(p, dir_out.first);

		// se existir próxima linha de cobertura
		if (next_cl.cost > 0)
		{
			// linha que liga a nova linha de cobertura à proxima linha
			edge_temp.node_a = new_cl_n2;
			edge_temp.node_b = next_cl.node_a;
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);

			// linha de cobertura posterior a linha de cobertua
			edge_temp.node_a = next_cl.node_a;
			edge_temp.node_b = next_cl.node_b;
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);

			// obter o caminho após a linha de cobetura em questão, até o próximo abastecimento.
			path next_cl_to_next_depot = GetPathFromEdgeToNextDepot(p, next_cl);
			for (auto e : next_cl_to_next_depot.edges)
			{
				edge_vec.emplace_back(e);
				new_edges.emplace_back(e);
			}
		}
		else
		{ // caso a linha contendo o nó out seja a última inserir um aresta retornando para a base
			edge_temp.node_a = new_cl_n2;
			edge_temp.node_b = input.getRobotBaseId(p.robotID);
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);
		}
		auto it_new_edges = new_edges.begin();
		auto it_fuel = p.fuelOnTarget.begin();
		while (!new_edges.empty())
		{

			fuel_remaining = 0.0;

			it_fuel = p.fuelOnTarget.find(it_new_edges->node_a);
			if (it_fuel != p.fuelOnTarget.end())
				fuel_remaining = it_fuel->second;

			fuel_required = it_new_edges->time;
			fuel_on_arrival = fuel_remaining - fuel_required;

			// se o robô não tiver capacidade para percorrer o caminho
			if (fuel_required > input.getRobotFuel(path_temp.robotID))
			{
				path_temp.pCost = -1; // inviável
				return path_temp;
			}

			// inserir o abastecimento no posto associado ao nó out
			if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
			{ // se o robô não tem combustível suficiente
				// se for link entre linhas de cobertura, não coverage line
				if (!IsCLine(it_new_edges->node_a, it_new_edges->node_b))
				{
					// obter o segmento do menor caminho possível lingando node_a e node_b
					// passando pelos postos.
					p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);

					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}

					// atualizar o custo do segmento em path_temp;
					path_temp.pCost += p_segment.pCost;

					// inserir as arestas do segmento na lista de arestas
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel != p.fuelOnTarget.end())
							it_fuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}
				}
				else
				{ // se for linha de cobertura
					// auto edge_list_temp = edge_list;
					// desviar a aresta anterior para o posto associado ao primeiro nó da nova linha de cobertura

					path_temp.pCost -= edge_list.back().cost; // remover o custo antigo da variável de custo

					if (edge_list.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}

					// edge_list.back().node_b = input.getDepotIdOnTarget(it_new_edges->node_a);
					// obter o segmento o menor caminho possível lingando node_a e node_b
					p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);

					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}

					path_temp.pCost += p_segment.pCost;

					// remove a tentative anterior, que é inviável para a capacidade do robô
					edge_list.pop_back();
					// inserir as arestas do segmento na lista de aresta
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						it_fuel = p.fuelOnTarget.find(fuel_segment.first);
						if (it_fuel != p.fuelOnTarget.end())
							it_fuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// inserir a linha de cobertura
					edge_temp.node_a = it_new_edges->node_a;
					edge_temp.node_b = it_new_edges->node_b;
					edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
					edge_temp.cost = edge_temp.time + (edge_temp.time * input.getRobotProp(p.robotID));
					edge_list.emplace_back(edge_temp);
					path_temp.pCost += edge_temp.cost;

					// obter o combustível restante no nó de partida node_a.
					it_fuel = p.fuelOnTarget.find(edge_temp.node_a);
					if (it_fuel != p.fuelOnTarget.end())
						fuel_remaining = it_fuel->second;

					// combustível necessário para o deslocamento
					fuel_required = edge_temp.time;
					// combustível que restará no robô após o deslocamento
					fuel_on_arrival = fuel_remaining - fuel_required;

					// se o combustível necessário para cobrir a aresta for insuficiente
					if (isDefinitelyLessThan(fuel_on_arrival, 0.0))
					{
						path_temp.pCost = -1;
						return path_temp;
					}

					// atualizar o combustível em node_b após o deslocamento
					it_fuel = p.fuelOnTarget.find(edge_temp.node_b);
					if (it_fuel != p.fuelOnTarget.end()) // se o target já existir em fuelontarget
						it_fuel->second = fuel_on_arrival;
					else // caso o nó não tenha sido alterado
						p.fuelOnTarget.emplace(edge_temp.node_b, fuel_on_arrival);
				}
			}
			else
			{

				if (input.isTarget(it_new_edges->node_b))
				{
					it_fuel = p.fuelOnTarget.find(it_new_edges->node_b);
					if (it_fuel != p.fuelOnTarget.end())
						it_fuel->second = fuel_on_arrival;
					else // caso não tenha o target na lista
						p.fuelOnTarget.emplace(it_new_edges->node_b, fuel_on_arrival);
				}

				// edge_list.emplace_back(*it_edge);
				edge_list.push_back(*it_new_edges);
				path_temp.pCost += it_new_edges->cost;
			}
			// remover o elemento processado
			new_edges.pop_front();
			// atualizar o iterator para o novo início da lista
			it_new_edges = new_edges.begin();
			//++it_new_edges;
		}

		path_temp.fuelOnTarget = p.fuelOnTarget;
		path_temp.edges.insert(path_temp.edges.begin(), edge_list.begin(), edge_list.end());
		path_temp.targetsNum = path_temp.fuelOnTarget.size();

		//---------------------------------------------------teste temporário da saída-----------------------
		if (!robotCapacity_val(path_temp))
			cout << "Robot Capacity problem\n";

		return path_temp;
	}

	// gerada pelo chatgpt
	/*
	Solution::path Solution::link_out_cl_in(path p, int out, int new_cl)
	{
		double fuel_remaining = 0;
		double fuel_required = 0;
		double fuel_on_arrival;
		int new_cl_n1;
		int new_cl_n2;

		edge next_cl;
		pair<int, int> dir_out;

		edge edge_temp;
		path path_temp = p;
		path p_segment;

		vector<edge> edge_vec;
		list<edge> edge_list;
		list<edge> new_edges;

		path_temp.edges.clear();
		path_temp.pCost = 0;
		path_temp.fuelOnTarget.clear();

		// Direção da CL de onde o robô está saindo
		dir_out = getCLDirection(p, out);

		// Escolhe qual nó da nova CL está mais próximo de 'out'
		double fuel_out_to_new_cl_n1 = getCostOnGraph(p.robotID, dir_out.second, new_cl);
		double fuel_out_to_new_cl_n2 = getCostOnGraph(p.robotID, dir_out.second, new_cl + 1);

		fuel_required = std::min(fuel_out_to_new_cl_n1, fuel_out_to_new_cl_n2);

		if (fuel_out_to_new_cl_n1 < fuel_out_to_new_cl_n2)
		{
			new_cl_n1 = new_cl;
			new_cl_n2 = new_cl + 1;
		}
		else
		{
			new_cl_n1 = new_cl + 1;
			new_cl_n2 = new_cl;
		}

		// Cria a ligação entre a CL atual e a nova CL
		edge_temp = {dir_out.second, new_cl_n1};
		edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
		edge_temp.cost = edge_temp.time + edge_temp.time * input.getRobotProp(p.robotID);
		edge_vec.emplace_back(edge_temp);
		new_edges.emplace_back(edge_temp);

		// Adiciona a nova CL (new_cl_n1 -> new_cl_n2)
		edge_temp = {new_cl_n1, new_cl_n2};
		edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
		edge_temp.cost = edge_temp.time + edge_temp.time * input.getRobotProp(p.robotID);
		edge_vec.emplace_back(edge_temp);
		new_edges.emplace_back(edge_temp);

		// Verifica se há próxima CL após a atual
		next_cl = GetNextCL(p, dir_out.first);
		if (next_cl.cost > 0)
		{
			// Liga nova CL à próxima
			edge_temp = {new_cl_n2, next_cl.node_a};
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + edge_temp.time * input.getRobotProp(p.robotID);
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);

			// Adiciona a próxima CL
			edge_temp = next_cl;
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);

			// Adiciona caminho até próximo posto após próxima CL
			path next_cl_to_next_depot = GetPathFromEdgeToNextDepot(p, next_cl);
			for (const auto &e : next_cl_to_next_depot.edges)
			{
				edge_vec.emplace_back(e);
				new_edges.emplace_back(e);
			}
		}
		else
		{
			// Retorna para a base se não houver próxima CL
			edge_temp = {new_cl_n2, input.getRobotBaseId(p.robotID)};
			edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
			edge_temp.cost = edge_temp.time + edge_temp.time * input.getRobotProp(p.robotID);
			edge_vec.emplace_back(edge_temp);
			new_edges.emplace_back(edge_temp);
		}

		auto it_new_edges = new_edges.begin();
		while (!new_edges.empty())
		{
			// Obtém combustível restante no nó de partida
			auto it_fuel = p.fuelOnTarget.find(it_new_edges->node_a);
			fuel_remaining = (it_fuel != p.fuelOnTarget.end()) ? it_fuel->second : input.getRobotFuel(p.robotID);

			fuel_required = it_new_edges->time;
			fuel_on_arrival = fuel_remaining - fuel_required;

			if (fuel_required > input.getRobotFuel(p.robotID))
			{
				path_temp.pCost = -1;
				return path_temp;
			}

			// Caso necessite de abastecimento
			if (fuel_on_arrival < 0.0)
			{
				if (!IsCLine(it_new_edges->node_a, it_new_edges->node_b))
				{
					p_segment = GetSPTOverOpenDepots(p, it_new_edges->node_a, it_new_edges->node_b);
					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}
					path_temp.pCost += p_segment.pCost;
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());
				}
				else
				{
					if (edge_list.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}
					path_temp.pCost -= edge_list.back().cost;
					p_segment = GetSPTOverOpenDepots(p, edge_list.back().node_a, edge_list.back().node_b);
					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}
					path_temp.pCost += p_segment.pCost;
					edge_list.pop_back();
					edge_list.insert(edge_list.end(), p_segment.edges.begin(), p_segment.edges.end());

					edge_temp = *it_new_edges;
					edge_temp.time = getCostOnGraph(p.robotID, edge_temp.node_a, edge_temp.node_b);
					edge_temp.cost = edge_temp.time + edge_temp.time * input.getRobotProp(p.robotID);
					edge_list.emplace_back(edge_temp);
					path_temp.pCost += edge_temp.cost;
				}
			}
			else
			{
				if (input.isTarget(it_new_edges->node_b))
				{
					p.fuelOnTarget[it_new_edges->node_b] = fuel_on_arrival;
				}
				edge_list.emplace_back(*it_new_edges);
				path_temp.pCost += it_new_edges->cost;
			}
			new_edges.pop_front();
			it_new_edges = new_edges.begin();
		}

		path_temp.fuelOnTarget = p.fuelOnTarget;
		path_temp.edges.insert(path_temp.edges.begin(), edge_list.begin(), edge_list.end());
		path_temp.targetsNum = path_temp.fuelOnTarget.size();

		return path_temp;
	}*/

	Solution::path Solution::GetPathFromEdgeToNextDepot(path p, edge edge_cl)
	{
		path path_temp;

		bool found_node = false;
		vector<edge> edges_vec;

		for (auto edge : p.edges)
		{
			// ao encontrar a linha de cobertura
			if (!found_node && (edge.node_a == edge_cl.node_a && edge.node_b == edge_cl.node_b))
			{
				found_node = true;
				continue;
			}
			if (found_node)
			{ // obter todas as aresta até encontrar uma que liga um depot
				edges_vec.emplace_back(edge);
				auto it_depot = p.depots.find(edge.node_b);
				if (it_depot != p.depots.end())
					break;
			}
		}
		path_temp.edges = edges_vec;
		return path_temp;
	}

	vector<pair<int, int>> Solution::GetEdgesFromPreviousCLToCL(path p, int cl)
	{
		vector<pair<int, int>> rlinks_vec;
		vector<pair<int, int>> links_vec;

		// posto associado na entrada
		path path_temp = GetPathFromBaseToNewCL(p, cl);

		auto rit = path_temp.edges.rbegin();

		// jump cl
		rit++;
		while (rit != path_temp.edges.rend())
		{

			// se for a CL anterior
			if (IsCLine(rit->node_a, rit->node_b))
				break;

			// inserir as linhas de ligações entre as cls
			rlinks_vec.emplace_back(make_pair(rit->node_a, rit->node_b));

			rit++;
		}

		links_vec.insert(links_vec.begin(), rlinks_vec.rbegin(), rlinks_vec.rend());

		// erro
		return links_vec;
	}

	vector<pair<int, int>> Solution::GetEdgesFromCLToPosteriorCL(path p, int cl)
	{
		vector<pair<int, int>> rlinks_vec;
		vector<pair<int, int>> links_vec;

		// posto associado na entrada
		path path_temp = GetPathFromNewCLToBase(p, cl);

		auto it = path_temp.edges.begin();

		// jump cl
		it++;
		while (it != path_temp.edges.end())
		{

			// sair se for a próxima CL
			if (IsCLine(it->node_a, it->node_b))
				break;

			// inserir as linhas de ligações entre as cls
			links_vec.emplace_back(make_pair(it->node_a, it->node_b));

			it++;
		}

		return links_vec;
	}

	pair<Solution::path, string> Solution::bestInsertionCL(path p, int cl)
	{
		// obter as linhas de cobertura de p
		int nCL = getNumberOfLines(p.pID);
		int pos = 0;
		path new_path, pathTemp, minPath;
		double minPCost = numeric_limits<double>::max();

		map<int, pair<int, int>> mapcl;
		pair<int, int> dir;

		pair<path, string> path_op;

		for (int i = 0; i < nCL; i++)
		{
			// position to insert
			pos = getCVLIndex(p.pID, i);

			// new_path = insertCL(p,pos,cl);
			new_path = new_insert(p, pos, cl);

			// se o caminho for inviável tentar próxima inserção
			if (new_path.pCost < 1)
			{
				minPath.pCost = -1;
				continue;
			}

			path_op = EvalOthersOrientationsOnPath(new_path, cl);

			pathTemp = path_op.first;

			// if(!PathRestrictions(pathTemp))
			// cout <<"Restrição bestInsertionCL" <<endl;

			// se o caminho for inviável tentar próxima inserção
			if (pathTemp.pCost < 0)
			{
				minPath.pCost = -1;
				continue;
			}

			if (minPCost > pathTemp.pCost)
			{
				minPCost = pathTemp.pCost;
				minPath = pathTemp;

				path_op.first = minPath;
			}
		}
		/*//verificar inserção da nova linha ao sair da base
		int baseID = input.getRobotBaseId(p.robotID);
		pathTemp = insertCL(p,baseID,cl);
		if(minPCost > pathTemp.pCost){
			minPCost = pathTemp.pCost;
			minPath = pathTemp;
		}*/

		return path_op;
	}

	pair<Solution::path, string> Solution::EvalOthersOrientationsOnPath(path p_a_b, int cl)
	{
		pair<int, int> dir;
		path least_cost_path;

		path p_ia_ib, p_a_ib, p_ia_b;
		path p_a, p_b, p_ib, p_ia;

		pair<path, string> path_op;

		// get first segment from base to new cl called p_a
		p_a = GetPathFromBaseToNewCL(p_a_b, cl);
		// get second segment cl to base and invert the start point on the cl, called p_ib
		p_ib = GetInvPathFromNewCLToBase(p_a_b, cl);

		// verificar se as alterações foram viáveis
		if (p_a.pCost > 0 && p_ib.pCost > 0)
			p_a_ib = PathUnion(p_a, p_ib);
		else // inviável
			p_a_ib.pCost = -1;

		p_ia_ib = GetInvPath(p_a_b);

		// get first segment from base to new cl,
		p_ia = GetPathFromBaseToNewCL(p_ia_ib, cl);
		// get second segment inverted, the they will restore the original orientation
		p_b = GetInvPathFromNewCLToBase(p_ia_ib, cl);

		// check if it is possible to join the paths
		if (p_ia.pCost > 0 && p_b.pCost > 0)
			p_ia_b = PathUnion(p_ia, p_b);
		else // inviável
			p_ia_b.pCost = -1;

		path_op = GetLeastCostPath(p_a_b, p_a_ib, p_ia_ib, p_ia_b);

		least_cost_path = path_op.first;

		//-----------------------------------------------------------------------------------
		// manutenção para testes...verificar se o caminho gerado é circuito e se a capacidade do robô é respeitada
		if (!PathRestrictions(least_cost_path))
			cout << "solução NÃO validada eval" << endl;
		//-------------------------------------------------------------------------------------------------------

		return path_op;
	}

	pair<Solution::path, string> Solution::GetLeastCostPath(path p_a_b, path p_a_ib, path p_ia_ib, path p_ia_b)
	{

		path path_temp;
		pair<path, string> path_op;

		// caso todos os caminhos sejam inviáveis atribuir -1 ao custo
		path_temp.pCost = -1;

		if ((p_a_b.pCost < p_a_ib.pCost || p_a_ib.pCost < 0) &&
			(p_a_b.pCost < p_ia_ib.pCost || p_ia_ib.pCost < 0) &&
			(p_a_b.pCost < p_ia_b.pCost || p_ia_b.pCost < 0) &&
			p_a_b.pCost > 0)
		{
			return make_pair(p_a_b, "p_a_b");
		}
		else if ((p_a_ib.pCost < p_ia_ib.pCost || p_ia_ib.pCost < 0) &&
				 (p_a_ib.pCost < p_ia_b.pCost || p_ia_b.pCost < 0) &&
				 p_a_ib.pCost > 0)
		{
			return make_pair(p_a_ib, "p_a_ib");
		}
		else if ((p_ia_ib.pCost < p_ia_b.pCost || p_ia_b.pCost < 0) &&
				 p_ia_ib.pCost > 0)
		{
			return make_pair(p_ia_ib, "p_ia_ib");
		}
		else if (p_ia_b.pCost > 0)
		{
			return make_pair(p_ia_b, "p_ia_b");
		}

		return make_pair(path_temp, "none");
	}

	Solution::path Solution::PathUnion(path path_a, path path_b)
	{
		path p_union;

		p_union.edges.clear();
		p_union.pCost = -1;

		if (path_a.pCost < 0 || path_b.pCost < 0)
			return p_union;

		p_union.pID = path_a.pID;
		p_union.robotID = path_a.robotID;

		p_union.edges.insert(p_union.edges.begin(), path_a.edges.begin(),
							 path_a.edges.end());
		p_union.edges.insert(p_union.edges.end(), path_b.edges.begin(),
							 path_b.edges.end());

		p_union.pCost = path_a.pCost + path_b.pCost;
		p_union.fuelOnTarget.insert(path_a.fuelOnTarget.begin(),
									path_a.fuelOnTarget.end());
		p_union.fuelOnTarget.insert(path_b.fuelOnTarget.begin(),
									path_b.fuelOnTarget.end());
		p_union.depots.insert(path_a.depots.begin(),
							  path_a.depots.end());
		p_union.depots.insert(path_b.depots.begin(),
							  path_b.depots.end());

		p_union.depotsNum = p_union.depots.size() + 1;
		p_union.targetsNum = p_union.fuelOnTarget.size();

		for (int id : p_union.depots)
			if (id < 31)
				cerr << "Problem id PathUnion!\n";

		return p_union;
	}

	vector<pair<int, int>> Solution::GetALLCLsFromNewCLToBase(path p, int node)
	{
		vector<pair<int, int>> cls_vec;

		bool node_found = false;

		for (auto it_edges = p.edges.begin(); it_edges != p.edges.end(); ++it_edges)
		{

			// encontrar a aresta partindo de node
			if (it_edges->node_a == node)
				node_found = true;

			// inserit todos as arestas que são CL no vetor
			if (node_found)
				if (IsCLine(it_edges->node_a, it_edges->node_b))
					cls_vec.emplace_back(make_pair(it_edges->node_a, it_edges->node_b));
		}

		return cls_vec;
	}

	pair<int, int> Solution::getCLDirection(path p, int cl)
	{
		auto it_edges = p.edges.begin();
		pair<int, int> cl_nodes;

		while (it_edges != p.edges.end())
		{
			if (it_edges->node_a == cl && it_edges->node_b == cl + 1)
			{
				cl_nodes = make_pair(it_edges->node_a, it_edges->node_b);
				break;
			}
			else if (it_edges->node_a == cl + 1 && it_edges->node_b == cl)
			{
				cl_nodes = make_pair(it_edges->node_a, it_edges->node_b);
				break;
			}
			++it_edges;
		}
		return cl_nodes;
	}

	vector<pair<int, int>> Solution::GetInvCLs(vector<pair<int, int>> cls_vec)
	{
		vector<pair<int, int>> path;
		auto itMap = cls_vec.begin();
		int node_first, node_second;

		// inverter a direção das linhas de cobertura e adicionar as linhas de ligação
		while (itMap != cls_vec.end())
		{
			node_first = itMap->first;
			node_second = itMap->second;

			// adicionar as arestas de ligação das linhas de cobertura
			if (!path.empty())
				path.emplace_back(path.back().second, node_second);

			path.emplace_back(make_pair(node_second, node_first));
			itMap++;
		}
		return path;
	}

	// calcula o combustível e o custo para percorrer cada aresta do caminho.
	vector<Solution::edge> Solution::GetEdgesCosts(vector<pair<int, int>> path, int robotID)
	{
		vector<edge> edges_costs;
		double fuel, cost;
		edge edge_temp;
		for (pair<int, int> e : path)
		{
			// adiciona combustível para ir do nó A ao B
			fuel = getCostOnGraph(robotID, e.first, e.second);
			// adiciona o custo
			cost = fuel + (input.getRobotProp(robotID) * fuel);
			// insere no vetor
			edge_temp.node_a = e.first;
			edge_temp.node_b = e.second;
			edge_temp.time = fuel;
			edge_temp.cost = cost;
			edges_costs.emplace_back(edge_temp);
		}
		return edges_costs;
	}

	// recebe um vetor com um possível caminho e um nó de partida, realiza os ajustes de reabastecimento se possível e retorna
	// o caminho ajustado.
	Solution::path Solution::AdjustRefuelingFromNode(vector<edge> path_costs, path p, int cl_node)
	{

		edge link_cl_node_to_path;
		edge link_path_to_base;
		double fuel_required = 0;
		double fuel_remaning = 0;
		double fuel_on_node = 0;
		path path_temp = p;
		path_temp.edges.clear();
		path p_segment;

		// utilizar a lista para inserir novas arestas no caminho. O uso de vetor traz problemas na realocação;
		list<edge> edge_list;
		edge_list.insert(edge_list.begin(), path_costs.begin(), path_costs.end());

		// vetor para retornar o resultado.
		vector<pair<pair<int, int>, pair<double, double>>> walkVector;

		// obter o combustível restante nó de saída da linha de cobertura inserida.
		if (p.fuelOnTarget.find(cl_node) != p.fuelOnTarget.end())
			fuel_remaning = p.fuelOnTarget.find(cl_node)->second;

		// inicializar as infos do path_temp.
		path_temp.pCost = 0;
		path_temp.fuelOnTarget.clear();

		// link second node from newCL inserted to next sector  of the path
		// after the newCL.
		link_cl_node_to_path.node_a = cl_node;
		link_cl_node_to_path.node_b = path_costs.front().node_a;
		link_cl_node_to_path.time = getCostOnGraph(p.robotID, link_cl_node_to_path.node_a, link_cl_node_to_path.node_b);
		link_cl_node_to_path.cost = link_cl_node_to_path.time + (link_cl_node_to_path.time * input.getRobotProp(p.robotID));

		// inserir no início
		edge_list.insert(edge_list.begin(), link_cl_node_to_path);

		// link last edg to base
		link_path_to_base.node_a = path_costs.back().node_b;
		link_path_to_base.node_b = input.getRobotBaseId(p.robotID);
		link_path_to_base.time = getCostOnGraph(p.robotID, link_path_to_base.node_a, link_path_to_base.node_b);
		link_path_to_base.cost = link_path_to_base.time + (link_path_to_base.time * input.getRobotProp(p.robotID));

		// inserir no fim
		edge_list.emplace_back(link_path_to_base);

		// adicinoar o combustível disponível no robô no node pos.
		auto itFuel = p.fuelOnTarget.find(cl_node);
		if (itFuel != p.fuelOnTarget.end())
			itFuel->second = fuel_remaning;

		// inicializar os ponteiros
		auto it_list = edge_list.begin();
		while (it_list != edge_list.end())
		{

			// se o tempo para percorrer a aresta for maior que a capacidade de combustível
			if (it_list->time > input.getRobotFuel(p.robotID))
			{
				path_temp.pCost = -1;
				// retornar caminho vazio;
				path_temp.nodes.clear();
				return path_temp;
			}

			// obter o combustível requerido para percorrer a aresta
			fuel_required = it_list->time;

			// se o nó de saída for target, obter o combustível disponível
			if (p.fuelOnTarget.find(it_list->node_a) != p.fuelOnTarget.end())
				fuel_on_node = p.fuelOnTarget.find(it_list->node_a)->second;
			// caso o nó de saída não seja target, o robô está saindo de algum depot
			else // verificar se a a capcidade do robô é sufciente
				fuel_on_node = input.getRobotFuel(path_temp.robotID);

			// obter o combustíve que restará no robô ao atingir o nó de chegada
			fuel_remaning = fuel_on_node - fuel_required;

			// caso o combustível no robô seja insuficiente para chegar no nó de chegada. Adicionar depot no nó de saída.
			if (isDefinitelyLessThan(fuel_remaning, 0.0))
			{

				// se não for linha de cobetura e o primeiro nó for target(ligação entre linhas de cobertura)
				if (!IsCLine(it_list->node_a, it_list->node_b) && input.isTarget(it_list->node_a))
				{

					// obter o segmento do menor caminho possível lingando node_a e node_b
					// passando pelos postos.
					p_segment = GetSPTOverOpenDepots(p, it_list->node_a, it_list->node_b);

					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}

					// atualizar o custo do segmento em path_temp;
					path_temp.pCost += p_segment.pCost;

					it_list = edge_list.erase(it_list);

					edge_list.insert(it_list, p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						itFuel = p.fuelOnTarget.find(fuel_segment.first);
						if (itFuel != p.fuelOnTarget.end())
							itFuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// a operação de inserção já posiciona o it_list no próximo elemento a ser analisado
					continue;
				}
				// se for linha de cobertura
				else if (IsCLine(it_list->node_a, it_list->node_b))
				{

					auto prev_edge = it_list;

					--prev_edge;
					// como a linha anterior irá mudar para visitar o posto
					// remover o valor inserido no custo
					path_temp.pCost -= prev_edge->cost; // remover o custo antido da variável de custo

					p_segment = GetSPTOverOpenDepots(p, prev_edge->node_a, prev_edge->node_b);

					if (p_segment.edges.empty())
					{
						path_temp.pCost = -1;
						return path_temp;
					}
					path_temp.pCost += p_segment.pCost;

					it_list = edge_list.erase(prev_edge);

					// inserir as arestas do segmento na lista de aresta
					edge_list.insert(it_list, p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						itFuel = p.fuelOnTarget.find(fuel_segment.first);
						if (itFuel != p.fuelOnTarget.end())
							itFuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// atualizar o combustível nos targets da LC
					itFuel = p.fuelOnTarget.find(it_list->node_a);
					if (itFuel != p.fuelOnTarget.end())
						fuel_remaning = itFuel->second;

					itFuel = p.fuelOnTarget.find(it_list->node_b);
					if (itFuel != p.fuelOnTarget.end())
					{
						fuel_remaning = fuel_remaning - fuel_required;
						if (isDefinitelyLessThan(fuel_remaning, 0.0))
						{
							path_temp.pCost = -1;
							return path_temp;
						}
						itFuel->second = fuel_remaning;
					}

					// adicionar o custo da linha de cobertura
					path_temp.pCost += it_list->cost;
				}
			}
			// caso o combustível seja suficiente, atualizar o combustível disponivel após percorrer a aresta.
			else
			{
				auto itFuel = p.fuelOnTarget.find(it_list->node_b);
				if (itFuel != p.fuelOnTarget.end())
					itFuel->second = fuel_remaning;
				// somar o custo da aresta na variável de custo total
				path_temp.pCost += it_list->cost;
			}
			if (it_list != edge_list.end())
				++it_list;
		}

		path_temp.fuelOnTarget = p.fuelOnTarget;
		// adiciona a base na contagem de depot;

		// inserir a quantidade de targets.
		path_temp.targetsNum = path_temp.fuelOnTarget.size();

		// insere o caminho armazenado na lista em path_temp.
		path_temp.edges.insert(path_temp.edges.begin(), edge_list.begin(), edge_list.end());

		return path_temp;
	}

	Solution::path Solution::GetPathFromBaseToNewCL(path p, int cl)
	{
		path path_temp;
		pair<int, int> dir;

		dir = getCLDirection(p, cl);
		int node_a = input.getRobotBaseId(p.robotID);
		int node_b = dir.second;

		path_temp = GetPathFromNodeAToNodeB(p, node_a, node_b);

		return path_temp;
	}

	// recebe um circuito e realiza os ajustes de reabastecimento se possível
	// retorna a o circuito ajustado
	Solution::path Solution::AdjustPathRefueling(vector<edge> path_costs, path p)
	{
		path path_t = p;
		path p_segment;

		edge link_base_to_path;
		edge link_path_to_base;

		double fuel_required = 0;
		double fuel_remaning = 0;
		double fuel_on_node = 0;
		int baseID = input.getRobotBaseId(p.robotID);

		// utilizar a lista para inserir novas arestas no caminho. O uso de vetor traz problemas na realocação;
		list<edge> edge_list;

		if (path_costs.empty())
		{
			path_t.pCost = -1;
			return path_t;
		}
		edge_list.insert(edge_list.begin(), path_costs.begin(), path_costs.end());

		// limpar a informação dos combustíveis em cada nó.
		path_t.fuelOnTarget.clear();

		// limpar o custo da rota.
		path_t.pCost = 0;

		// limpar a rota em path_t;
		path_t.edges.clear();

		// link base to first CL
		link_base_to_path.node_a = baseID;
		link_base_to_path.node_b = path_costs.front().node_a;
		link_base_to_path.time = getCostOnGraph(path_t.robotID, link_base_to_path.node_a, link_base_to_path.node_b);
		link_base_to_path.cost = link_base_to_path.time + (link_base_to_path.time * input.getRobotProp(path_t.robotID));
		// inserir no início
		edge_list.insert(edge_list.begin(), link_base_to_path);

		// obter o combustível disponível ao acessar o primeiro nó da primeira CL(saindo da base com tanque cheio)
		fuel_remaning = input.getRobotFuel(path_t.robotID) - link_base_to_path.time;

		// atualizar o map com a informação do combustível disponível no target de início da rota.
		p.fuelOnTarget.find(link_base_to_path.node_b)->second = fuel_remaning;

		// link last edg to base
		link_path_to_base.node_a = path_costs.back().node_b;
		link_path_to_base.node_b = input.getRobotBaseId(path_t.robotID);
		link_path_to_base.time = getCostOnGraph(path_t.robotID, link_path_to_base.node_a, link_path_to_base.node_b);
		link_path_to_base.cost = link_path_to_base.time + (link_path_to_base.time * input.getRobotProp(path_t.robotID));
		// inserir no fim
		edge_list.emplace_back(link_path_to_base);

		// caso o robô não tenha capacidade de atingir o target, retornar o vetor vazio.
		if (fuel_remaning < 0)
		{
			path_t.pCost = -1;
			path_t.edges.clear();
			return path_t;
		}

		// inicializar os ponteiros
		auto it_list_temp = edge_list.begin();
		auto it_list = edge_list.begin();

		// obter o custo da base ao primeiro target.
		path_t.pCost += it_list->cost;

		// como  primeiro nó é depot(base), não iremos adicionar outro posto antes,
		// assim, obter a próxima aresta que é a primeira CL.
		it_list++;
		while (it_list != edge_list.end())
		{
			// se o tempo para percorrer a aresta for maior que a capacidade de combustível
			if (it_list->time > input.getRobotFuel(path_t.robotID))
			{
				path_t.pCost = -1;
				// retornar caminho vazio;
				path_t.edges.clear();
				return path_t;
			}
			// obter o combustível requerido para percorrer a aresta
			fuel_required = it_list->time;

			// se o nó de saída for target, obter o combustível disponível
			auto itFuel = p.fuelOnTarget.find(it_list->node_a);
			if (itFuel != p.fuelOnTarget.end())
				fuel_on_node = itFuel->second;
			// caso o nó de saída não seja target, o robô está saindo de algum depot
			else // verificar se a a capcidade do robô é sufciente
				fuel_on_node = input.getRobotFuel(path_t.robotID);

			// obter o combustíve que restará no robô ao atingir o nó de chegada
			fuel_remaning = fuel_on_node - fuel_required;

			// caso o combustível no robô seja insuficiente para chegar no nó de chegada. Adicionar depot no nó de saída.
			if (fuel_remaning < 0)
			{
				// se for linha de ligação(entre linhas de cobertura)
				// se não for linha de cobetura e o primeiro nó for target(ligação entre linhas de cobertura)
				if (!IsCLine(it_list->node_a, it_list->node_b) && input.isTarget(it_list->node_a))
				{
					// obter o segmento do menor caminho possível lingando node_a e node_b
					// passando pelos postos.
					p_segment = GetSPTOverOpenDepots(p, it_list->node_a, it_list->node_b);

					if (p_segment.edges.empty())
					{
						path_t.pCost = -1;
						return path_t;
					}

					it_list = edge_list.erase(it_list);
					// inserir as arestas na lista
					edge_list.insert(it_list, p_segment.edges.begin(), p_segment.edges.end());

					// somar o custo na variável de custo total do caminho
					path_t.pCost += p_segment.pCost;

					// atualizar o vetor de combustível em cada target
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						itFuel = p.fuelOnTarget.find(fuel_segment.first);
						if (itFuel != p.fuelOnTarget.end())
							itFuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// a operação de inserção posiciona o iterator na próxima aresta, não precisamor movê-lo
					continue;
				}
				// se for linha de cobertura
				else if (IsCLine(it_list->node_a, it_list->node_b))
				{
					it_list_temp = it_list;

					// obter a aresta anterior
					it_list_temp--;

					// obter a menor rota passando por depots
					p_segment = GetSPTOverOpenDepots(p, it_list_temp->node_a, it_list_temp->node_b);

					if (p_segment.edges.empty())
					{
						path_t.pCost = -1;
						return path_t;
					}

					// remover o custo da aresta anterior já contabilizado no custo de path_t
					path_t.pCost -= it_list_temp->cost;

					// remover a aresta anterior que chega na LC da lista de arestas
					it_list_temp = edge_list.erase(it_list_temp);

					/// adicionar o custo do segmento;
					path_t.pCost += p_segment.pCost;

					// inserir o segmento na lista de aresta
					edge_list.insert(it_list_temp, p_segment.edges.begin(), p_segment.edges.end());

					// atualizar o vetor de combustível em cada target dado os valores do segmento
					for (auto fuel_segment : p_segment.fuelOnTarget)
					{
						itFuel = p.fuelOnTarget.find(fuel_segment.first);
						if (itFuel != p.fuelOnTarget.end())
							itFuel->second = fuel_segment.second;
						else
							p.fuelOnTarget.emplace(fuel_segment.first, fuel_segment.second);
					}

					// atualizar o combustível nos targets da LC
					itFuel = p.fuelOnTarget.find(it_list->node_a);
					if (itFuel != p.fuelOnTarget.end())
						fuel_remaning = itFuel->second;

					// atualizar o combustível dispoível no segundo target da lc
					itFuel = p.fuelOnTarget.find(it_list->node_b);
					if (itFuel != p.fuelOnTarget.end())
						itFuel->second = fuel_remaning - fuel_required;

					// adicionar o custo da linha e cobertura
					path_t.pCost += it_list->cost;
				}
			}
			// caso o combustível seja suficiente, atualizar o combustível disponivel após percorrer a aresta.
			else
			{
				itFuel = p.fuelOnTarget.find(it_list->node_b);
				if (itFuel != p.fuelOnTarget.end())
					itFuel->second = fuel_remaning;
				// somar o custo da aresta na variável de custo total
				path_t.pCost += it_list->cost;
			}
			if (it_list != edge_list.end())
				++it_list;
		}
		path_t.fuelOnTarget = p.fuelOnTarget;

		// insere o caminho armazenado na lista em path_t.
		path_t.edges.insert(path_t.edges.begin(), edge_list.begin(), edge_list.end());

		//-------------------------------teste temporário da saída---------------------------------------------
		if (!robotCapacity_val(path_t))
		{
			cout << "capacidade violada adjsPath" << endl;
		}
		//---------------------------------------------------------------------------
		return path_t;
	}

	// retorna todas a CL da base até a CL anterior à nova Cl.
	vector<pair<int, int>> Solution::GetALLCLsFromBaseToNewCL(path p, int node)
	{
		int base, newcl;
		vector<pair<int, int>> cls_vec;

		base = input.getRobotBaseId(p.robotID);
		newcl = node;

		bool found_node_base = false;
		bool found_new_cl = false;

		for (auto edge : p.edges)
		{
			if (edge.node_a == base) // habilita a inserção de cls no vetor
				found_node_base = true;
			else if (edge.node_a == newcl) // se achar a newcl, parar de inserir cl no vetor.
				found_new_cl = true;

			// não inclui a newcl no vetor
			if (found_node_base && !found_new_cl)
				if (IsCLine(edge.node_a, edge.node_b))
					cls_vec.emplace_back(make_pair(edge.node_a, edge.node_b));
		}

		return cls_vec;
	}

	vector<pair<int, int>> Solution::GetALLCLsFromBase(path p)
	{
		int base = input.getRobotBaseId(p.robotID);
		vector<pair<int, int>> cls_vec;
		bool found_node_base = false;

		for (auto edge : p.edges)
		{
			if (edge.node_a == base) // habilita a inserção de cls no vetor
				found_node_base = true;

			// não inclui a newcl no vetor
			if (found_node_base)
				if (IsCLine(edge.node_a, edge.node_b))
					cls_vec.emplace_back(make_pair(edge.node_a, edge.node_b));
		}

		return cls_vec;
	}

	Solution::path Solution::GetPathFromNewCLToBase(path p, int cl)
	{
		pair<int, int> dir;
		dir = getCLDirection(p, cl);
		int node_a = dir.second;
		int node_b = input.getRobotBaseId(p.robotID);

		path path_temp = GetPathFromNodeAToNodeB(p, node_a, node_b);

		return path_temp;
	}

	Solution::path Solution::GetInvPathFromNewCLToBase(path p, int cl)
	{
		pair<int, int> dir;
		pair<int, int> next_cl;
		vector<pair<int, int>> cls_vec;
		vector<pair<int, int>> inv_path;
		vector<edge> edges_costs;
		path path_temp;

		map<int, pair<int, int>> mapcls;

		// obter a direção da linha de cobertura
		dir = getCLDirection(p, cl);

		// obter todas as CL a partir da nova linha de cobertura.
		cls_vec = GetALLCLsFromNewCLToBase(p, dir.first);

		// verificar a inexistência de linhas a partir de newCL

		// remover a linha de cobertura do map. A direção da nova linha é mantinda
		// todas as linhas posteriores serão invertidas até o retorna à base.

		if (!cls_vec.empty())
			cls_vec.erase(cls_vec.begin());

		if (cls_vec.empty())
		{
			path_temp.pCost = -1;
			return path_temp;
		}

		// inverter todas as cls
		inv_path = GetInvCLs(cls_vec);

		// calcular os custos
		edges_costs = GetEdgesCosts(inv_path, p.robotID);

		// ajustar se necessário o reabastecimento do caminho
		// a partir do nó de saída de nova cl.
		path_temp = AdjustRefuelingFromNode(edges_costs, p, dir.second);

		return path_temp;
	}

	Solution::path Solution::GetInvPath(path p)
	{
		vector<pair<int, int>> cls_vec;
		vector<pair<int, int>> inv_path;
		vector<edge> path_costs;
		path path_temp;

		// obter as linhas de coberturas a partir de cl.
		cls_vec = GetALLCLsFromBase(p);

		// inverter todas as cls
		inv_path = GetInvCLs(cls_vec);

		// calcular os custos
		path_costs = GetEdgesCosts(inv_path, p.robotID);

		// verificar a restrição de combustíve a partir da cl.
		path_temp = AdjustPathRefueling(path_costs, p);

		return path_temp;
	}

	// verificar o erro que alguns nós não estão na lista de target e são considerados depot.
	// alguma função não está atualizando os fuelontarget e ao passar o caminho para esta função
	// ocorre que o id da aresta é inserido como depot.
	Solution::path Solution::GetPathFromNodeAToNodeB(path p, int node_a, int node_b)
	{
		path path_temp;

		path_temp.pCost = 0;
		path_temp.fuelOnTarget.clear();
		path_temp.depots.clear();
		path_temp.depotsNum = 0;
		path_temp.targetsNum = 0;
		path_temp.nodes.clear();

		path_temp.pID = p.pID;
		path_temp.robotID = p.robotID;

		auto it_fuel = p.fuelOnTarget.begin();

		bool found_node_a = false;
		edge edge_temp;

		for (edge e : p.edges)
		{
			if (e.node_a == node_a)
				found_node_a = true;
			else if (e.node_a == node_b && found_node_a)
				break;

			if (found_node_a)
			{
				// adicionar os atributos da aresta
				edge_temp.node_a = e.node_a;
				edge_temp.node_b = e.node_b;
				edge_temp.cost = e.cost;
				edge_temp.time = e.time;
				path_temp.edges.emplace_back(edge_temp);
				path_temp.pCost += edge_temp.cost;

				// obter o valor do combustíve disponível no caminho
				it_fuel = p.fuelOnTarget.find(e.node_a);
				// se for target
				if (it_fuel != p.fuelOnTarget.end())
				{
					path_temp.fuelOnTarget.emplace(it_fuel->first,
												   it_fuel->second);
				}
				else if (e.node_a != input.getRobotBaseId(path_temp.robotID))
				{ // se for depot
					int depot_id = input.getDepotIdOnTarget(e.node_a);

					path_temp.depots.emplace(e.node_a);
					if (e.node_a < 31)
						cerr << "Problem id GetPathFromNodeAtoNodeB!\n";
				}
			}
		}
		path_temp.targetsNum = path_temp.fuelOnTarget.size();

		// adicionar a base na contagem
		path_temp.depotsNum = path_temp.depots.size() + 1;

		return path_temp;
	}

	Solution::edge Solution::GetNextCL(path p, int node)
	{

		bool found_node = false;
		edge edge_temp;
		edge_temp.time = 0;
		edge_temp.cost = -1;

		for (edge e : p.edges)
		{
			if (!found_node && e.node_a == node)
			{
				found_node = true;
				continue;
			}

			if (found_node)
				if (IsCLine(e.node_a, e.node_b))
				{
					edge_temp = e;
					break;
				}
		}

		return edge_temp;
	}

	void Solution::Iteractive_DFS(path p)
	{
		int prox = 0;
		map<int, pair<pair<vector<pair<int, double>>, int>, int>> ladj;
		vector<pair<pair<int, int>, pair<double, double>>> walk;
		pair<int, int> edge;
		int node_a = input.getRobotBaseId(p.robotID);

		vector<pair<pair<int, int>, pair<double, double>>> tempNodes = p.nodes;
		auto itTemp = tempNodes.begin();
		auto itLAdj = ladj.end();
		// transforma pnodes em lista de adjacência
		while (itTemp != tempNodes.end())
		{
			itLAdj = ladj.emplace_hint(itLAdj, itTemp->first.first, make_pair(make_pair(vector<pair<int, double>>(), 0), 1));
			itLAdj->second.first.first.push_back(make_pair(itTemp->first.second, itTemp->second.second));
			++itTemp;
		}
		// Aplicar a busca em profundidade para verificar o custo do maior caminho entre depots.

		ladj.find(node_a)->second.second = 2;
		stack<int> stack;
		// inicializa com a base
		stack.push(node_a);
		itLAdj = ladj.begin();
		// posição do próximo na lista de adjacência
		int pos = 0;
		// iterador para o vetor de pares <nodeID, custo>
		auto v = itLAdj->second.first.first.begin();
		while (!stack.empty())
		{
			// obter o id do nó no topo da pilha
			node_a = stack.top();
			// encontrar a posição do nó  na lista de adjacência
			itLAdj = ladj.find(node_a);
			// se o nó existir
			if (itLAdj != ladj.end())
			{
				// obter a posição do próximo da lista de adjacência do nó nodeA
				pos = itLAdj->second.first.second;
				// obter o par <nodeID, custo> do vetor dada a posição pos
				v = itLAdj->second.first.first.begin() + pos;
				// se o par existir no vetor
				if (v != itLAdj->second.first.first.end())
				{
					// apontar o pos para o próximo elemento da lista de adjacência
					itLAdj->second.first.second++;
					// prox recebe o ID do próxino nó na lista de adjacência
					prox = v->first;
					// se o prox não foi visitado
					if (ladj.find(prox)->second.second == 1)
					{
						// marcar o prox como visitado
						ladj.find(prox)->second.second = 2;
						// inserir o prox na pilha
						stack.push(prox);
					}
				}
				// se não houver vizinho do node_a
				else
				{
					// marcar nodeA como visitado todas os adjacentes
					ladj.find(node_a)->second.second = 0;
					// remover o nodeA da pilha
					stack.pop();
				}
			}
		}
	}

	/*void Solution::SortMultiplesAdjacentOnDepots(path *p)
	{

		// map de adjacências <node,tupla>
		// a tupla contêm <lista, visitação, posição do adjacente visitado na lista>
		// lista < nó adjacente, posição do nó da lista de adjacências>
		// visitação, 1 não foi visitado, 2 vistado e 0 encerradas as visitas em todos os adjacentes do nó;
		map<int, tuple<list<pair<int, int>>, int, int>> map_adj;
		auto it_map = map_adj.begin();
		int pos = 0;

		// obter a base
		int baseID = input.getRobotBaseId(p->robotID);
		int u = baseID;
		int v = 0;
		int size;
		int prox;
		uint branch_id;

		auto it_map_before = map_adj.begin();

		// criar o map de adjacências
		for (auto edge : p->edges)
		{
			// criar lista de adjacentes (nó, visitado, posição)
			list<pair<int, int>> adj_list;

			// inserir o adjacente na lista e inicializar a posição
			adj_list.emplace_back(make_pair(edge.node_b, 0));

			// inicializar a tupla com o a lista de adjacentes, visitado = 1, próximo adjacente é o adjacente 0;
			tuple<list<pair<int, int>>, int, int> tuple_adj(make_tuple(adj_list, 1, 0));

			it_map_before = map_adj.find(edge.node_a);
			// inserir no map de adjacência a lista de adjacências e as informação de vistação do nó e a posição do adjacente a ser visitado.
			it_map = map_adj.emplace_hint(it_map, edge.node_a, tuple_adj);

			// caso já exista nós adjacentes inserir na mesma posição do map na próxima posição da lista de adjacência
			if (it_map_before != map_adj.end())
			{
				pos = get<0>(it_map->second).back().second;							  // obter a posição do anterior
				get<0>(it_map->second).emplace_back(make_pair(edge.node_b, pos + 1)); // inserir o adjacente na lista e a sua posição;
			}
		}

		// executar o DFS para obter a lista de branches no caminho.

		// map do identificador do nó para os seus branches
		// cada adjacente origina um branch logo teremos uma lista de listas
		map<int, pair<int, list<pair<int, list<int>>>>> branches_list;

		// iterator para cada elemento do map, acessar o conjunto de branches de cada nó.
		auto it_branches_list = branches_list.begin();

		auto exit_branch = branches_list.begin();

		// criar branch para a unição
		list<int> union_branches;

		// iterator para a lista de adjacente de cada nó
		list<pair<int, int>>::iterator adj;

		// encontrar o nó u no map de adjacência
		it_map = map_adj.find(u);
		if (it_map != map_adj.end())
			get<1>(it_map->second) = 2; // marcar o primeiro nó como visitado

		// criar a pilha para controle da busca em profundidade
		stack<int> stack_dfs;

		// criar pilha para controle dos branches abertos
		stack<int> branches_opened;

		// inicializa com a base
		stack_dfs.push(u);

		branches_opened.push(u);
		while (!stack_dfs.empty())
		{
			// obter o elemento do topo da pilha
			u = stack_dfs.top();

			// encontrar o nó u no map de adjacência
			it_map = map_adj.find(u);

			if (it_map != map_adj.end())
			{
				// posição do próximo adjacente
				prox = get<2>(it_map->second);

				// verifica se ainda há adjacentes a visitar
				if (prox < get<0>(it_map->second).size())
				{
					adj = get<0>(it_map->second).begin();
					advance(adj, prox);

					// incrementa apenas depois de acessar o adjacente
					get<2>(it_map->second)++;

					v = adj->first;
					size = get<0>(it_map->second).size();

					if (size > 1 || u == baseID)
					{
						if (branches_opened.empty() || u != branches_opened.top())
							branches_opened.push(u);

						pos = adj->second;

						it_branches_list = branches_list.emplace_hint(it_branches_list, u, make_pair(0, list<pair<int, list<int>>>()));
						it_branches_list->second.first = pos;
					}

					if (!branches_list.empty())
					{
						branch_id = it_branches_list->second.first;

						if (it_branches_list->second.second.size() == branch_id)
							it_branches_list->second.second.emplace_back(pair<int, list<int>>());

						auto branch_it = it_branches_list->second.second.begin();
						advance(branch_it, branch_id);

						if (u == baseID && u != v)
						{
							branch_it->second.emplace_back(u);
							branch_it->first = it_branches_list->second.second.size();
						}
						else if (v == baseID)
						{
							exit_branch = it_branches_list;
						}

						branch_it->second.emplace_back(v);
						branch_it->first = it_branches_list->second.second.size();
					}

					// visitar o nó v, se ele ainda não foi visitado
					auto it_v = map_adj.find(v);
					if (it_v != map_adj.end() && get<1>(it_v->second) == 1)
					{
						get<1>(it_v->second) = 2; // marca como visitado
						stack_dfs.push(v);
					}
				}
				else
				{
					// se não houver mais adjacentes, encerrar e remover da pilha
					get<1>(it_map->second) = 0;
					stack_dfs.pop();
				}
			}
			else
			{
				// proteção extra se nó não estiver no mapa
				stack_dfs.pop();
			}
		}

		// maior ordem é a quantidade de branch
		int order = branches_list.size();
		map<int, int> branch_order;
		// o último branch a ser visitado é o branch da base
		branch_order.emplace(baseID, order); // inserir a base como último nó de retorno

		// o penúltimo é conjunto de branches, ou seja o tronco que leva a base.
		branch_order.emplace(exit_branch->first, --order); // inserir o último branch de retorno

		// lista para identificar todos os branches de um tronco que tem o nó de saída que leva à base ou a um branch que tem um link com a base.
		list<int> branches_with_node;

		// percorrer todos os branches e criar o map com a ordem (branch_order) em que o branches devem ser explorados.
		// Os branches são ordenados de acordo com o seu acesso ao nó base. O branch que tem como adjacente a base é colocado no final da fila.
		// O próximo branch tem um link a esse que leva á base e colocado por penúltimo e assim por diante.
		//  todos os branches recebem um número de ordem, começando do mais distânte em termos de branches para retornar à base.
		it_branches_list = branches_list.begin();
		auto it_list = it_branches_list->second.second.begin();

		// percorrer todos os branches enquanto a lista de ordem não contemplar todos os branches
		while (it_branches_list != branches_list.end() && branches_list.size() > branch_order.size())
		{
			// obter a lista de branches de cada tronco
			it_list = it_branches_list->second.second.begin();
			while (it_list != it_branches_list->second.second.end())
			{													   // enquanto não percorrer todos os branches do tronco
				if (it_branches_list->first == exit_branch->first) // se o branch de saída analisado é o mesmo que a lista de branches aponta
					break;										   // obter o próximo tronco, ou seja conjunto de branches
				if (it_list->second.back() == exit_branch->first)
				{ // se o nó de saída da lista é igual ao nó de saída procurado
					// inserir todos os branches que possuem o nó de saída.
					// se o branch não está na lista de branch
					if (branch_order.find(it_branches_list->first) == branch_order.end()) // inserir na lista apenas os que não já estão em branch_order
						branches_with_node.emplace_back(it_branches_list->first);		  // inserir todos os branches  que o nó de saída que são o tronco procurado.
					break;																  // apos  encontrado o branch de saída no tronco, obter o próximo.
				}

				++it_list; // obter a próxima lista do conjunto de branches
			}

			++it_branches_list; // obter o próximo, tronco, conjunto de branches

			if (it_branches_list == branches_list.end())
			{ // se a lista de troncos chegou ao final, ou seja percorreu todas os branches procurando o branch de saída.
				branch_order.emplace(branches_with_node.front(), --order);
				// inserir o identificador do primeiro branch encontrado que tem o link para o branch de saída e a sua ordem de exploração
				exit_branch = branches_list.find(branches_with_node.front());
				branches_with_node.pop_front(); // remover o branch do início da fila
				// if(!branches_with_node.empty())// se a lista  de branches não é vazia, inicializar a busca do próximo branch de saída do início.
				it_branches_list = branches_list.begin();
			}
		}

		// atualizar a ordem de saída de cada branch.
		// cada branch possui um campo com a ordem que foi explorado na visita durante o DFS. Atualizar o campo com a ordem que devem ser explorados
		// essa ordem foi obtida no passo anterior e armazenada no map branch_order.
		it_branches_list = branches_list.begin();
		while (it_branches_list != branches_list.end())
		{
			it_list = it_branches_list->second.second.begin();
			while (it_list != it_branches_list->second.second.end())
			{
				it_list->first = branch_order.find(it_list->second.back())->second;
				++it_list;
			}

			++it_branches_list;
		}

		// ordenar os branches de acordo com a ordem de saída
		it_branches_list = branches_list.begin();

		// imprimir o valores de branches_list

		vector<pair<int, list<int>>> temp; // vetor para armazenar temporariamente o conjunto de branches de um tronco.
		// tivemos que utilizar o vetor pois a função sort não trabalha com o tipo do iterator do list
		// o vector utiliza iteradores do tipo random_access iterator, já list utiliza o biderecional iterator.
		while (it_branches_list != branches_list.end())
		{

			temp.clear();

			// obter a lista de branches
			temp.insert(temp.begin(), it_branches_list->second.second.begin(), it_branches_list->second.second.end());

			// ordenar as listas de acordo com a ordem em seu cabeçalho.
			sort(temp.begin(), temp.end(),
				 [](pair<int, const list<int>> el1, pair<int, const list<int>> el2)
				 {
					 return el1.first < el2.first;
				 });

			// remover a lista de branches
			it_branches_list->second.second.clear();

			// inserir a lista de branches ordenada
			it_branches_list->second.second.insert(it_branches_list->second.second.begin(), temp.begin(), temp.end());

			// obter a próxima lista
			++it_branches_list;
		}

		// montar a lista ordenada, inicializando da base
		// it_branches_list = branches_list.begin();
		// it_list = it_branches_list->second.second.begin();

		// cout << "branches_list.size após ordenação: " << branches_list.size();

		it_branches_list = branches_list.begin(); // ← se falhar aqui, o map foi invalidado externamente
		if (it_branches_list != branches_list.end())
		{
			auto it_list = it_branches_list->second.second.begin();
		}

		int root_node = baseID;
		while (!branches_list.empty())
		{

			// obter o ponteiro para a raiz do branch
			it_branches_list = branches_list.find(root_node);
			// apontar para a primeira lista
			if (it_branches_list != branches_list.end())
				it_list = it_branches_list->second.second.begin();
			else
				break;

			// inserir inicialmente todas os branches que volta para a raiz
			while (it_list != it_branches_list->second.second.end())
			{
				if (it_list->second.back() == root_node)
				{
					union_branches.insert(union_branches.end(), it_list->second.begin(), it_list->second.end()); // inserir o ramo na lista de ordenação
					it_list = it_branches_list->second.second.erase(it_list);									 // remover o ramo da lista de branches
					// se só tivermos branches que retornam para o tronco, todos serão inseridos em union_branches e removido da lista.
					// logo a lista de branches para esse tronco será vazia.
					if (it_list == it_branches_list->second.second.end()) // se não tiver mais lista
						branches_list.erase(it_branches_list);			  // remover o tronco da lista
					continue;
				}
				++it_list;
			}
			it_list = it_branches_list->second.second.begin();
			// inserir os branches que não retonam para raiz na lista ordenada e desviar para a raiz apontada pelo nó de saída
			if (!it_branches_list->second.second.empty())
			{
				// manter o branch que volta para base enquanto houver outros branches
				if (it_list->second.back() == baseID && it_branches_list->second.second.size() > 1)
					++it_list;

				if (it_list->second.begin() != it_list->second.end())
				{
					union_branches.insert(union_branches.end(), it_list->second.begin(), it_list->second.end()); // inserir o ramo na lista de ordenação
					root_node = it_list->second.back();
				}
				else
					cout << "Problema  na ordenação !!" << endl;

				it_branches_list->second.second.erase(it_list); // remover o ramo da lista de branches
				it_list = it_branches_list->second.second.begin();
				// se não houver mais branches, apagar esse nó, excluí-lo do map
				if (it_list == it_branches_list->second.second.end())
					branches_list.erase(it_branches_list);
			}
		}

		// converter a lista de nós sequência para a sequência de arestas, utilizada por p.edges;
		list<edge> edges;
		edges.insert(edges.begin(), p->edges.begin(), p->edges.end());
		p->edges.clear();
		edge edge_temp;

		auto p_base = edges.begin();
		auto edge_find = edges.begin();

		// enquanto não remover todos os nós da lista inion_branches
		if (!union_branches.empty())
		{
			while (!union_branches.empty())
			{

				// obter o primeiro nó da aresta
				edge_temp.node_a = union_branches.front();

				// remover o nó obtido
				union_branches.pop_front();

				// obter o próximo nó da aresta
				edge_temp.node_b = union_branches.front();

				// procurar a aresta na lista de aresta para encontrar as informações de custo e tempo
				while ((edge_find->node_a != edge_temp.node_a || edge_find->node_b != edge_temp.node_b) && edge_find != edges.end())
					++edge_find;

				// se encontrou a aresta,
				if (edge_find != edges.end())
				{
					// atribuir as informações
					edge_temp.cost = edge_find->cost;
					edge_temp.time = edge_find->time;

					// apagar a aresta da lista de arestas
					edges.erase(edge_find);
					// inserir a aresta em p->edges, na ordem correta
					p->edges.emplace_back(edge_temp);

					// apontar novamento para o início da lista
					p_base = edges.begin();
				}

				edge_find = p_base;
			}
		}

		// se a listas de arestas não foi toda utilizada, tornar o caminho inviável
		if (!edges.empty())
			p->pCost = -1;

		//--------------------------------------------------------funções temporário para verificar possíveis falhas no caminho.
		// if(!PathRestrictions(*p))
		// cout << "Problema Ordenação 1" << endl;
		//-------------------------------------------------------------------------------------------------------------------------
	}*/

	/**
	 * @brief Ordena os caminhos em p->edges quando há múltiplas arestas adjacentes conectadas a depósitos.
	 *
	 * A função lida com casos em que um depósito se conecta a múltiplos nós (ramificações),
	 * o que pode gerar caminhos desordenados ou inconsistentes. O algoritmo realiza:
	 *
	 * 1. Construção de um grafo auxiliar de adjacência para representar os caminhos como ramificações (branches);
	 * 2. Busca em profundidade (DFS) para explorar e agrupar os caminhos a partir da base do robô;
	 * 3. Organização dos branches com base em sua distância lógica à base;
	 * 4. Reordenação e reconstrução do caminho original de forma consistente e linear;
	 * 5. Verificação de consistência com base nas arestas originais.
	 *
	 * Em caso de inconsistência (arestas faltando ou fora de ordem), o custo total do caminho é setado como -1,
	 * indicando caminho inválido.
	 *
	 * @param p Ponteiro para o caminho a ser ordenado.
	 */
	void Solution::SortMultiplesAdjacentOnDepots(path *p)
	{
		using AdjTuple = tuple<list<pair<int, int>>, int, int>; // <adjacentes, visitado, proximo_adj_index>
		map<int, AdjTuple> map_adj;

		// Obtém o identificador da base do robô para iniciar a DFS
		const int baseID = input.getRobotBaseId(p->robotID);
		int u = baseID, v = 0;
		uint branch_id;

		// Constrói o grafo auxiliar de adjacência (map_adj) a partir das arestas originais
		for (const auto &edge : p->edges)
		{
			auto &adj_list = get<0>(map_adj[edge.node_a]);
			int pos = adj_list.empty() ? 0 : adj_list.back().second + 1;
			adj_list.emplace_back(edge.node_b, pos);
			get<1>(map_adj[edge.node_a]) = 1; // Visitado = 1
			get<2>(map_adj[edge.node_a]) = 0; // Prox adj = 0
		}

		// Inicializa as estruturas de controle para a DFS, incluindo pilha e marcação de visitados
		stack<int> stack_dfs;
		stack<int> branches_opened;
		stack_dfs.push(u);
		branches_opened.push(u);
		get<1>(map_adj[u]) = 2;

		map<int, pair<int, list<pair<int, list<int>>>>> branches_list;
		auto it_branches_list = branches_list.begin();
		auto exit_branch = branches_list.begin();

		// Realiza DFS a partir da base para construir os branches (subcaminhos adjacentes à base)
		while (!stack_dfs.empty())
		{
			u = stack_dfs.top();
			auto &tupla = map_adj[u];
			auto &adj_list = get<0>(tupla);
			auto &visitado = get<1>(tupla);
			auto &prox = get<2>(tupla);

			if (prox < (int)adj_list.size())
			{
				auto adj = next(adj_list.begin(), prox);
				++prox;

				v = adj->first;
				int size = adj_list.size();

				if (size > 1 || u == baseID)
				{
					if (branches_opened.empty() || u != branches_opened.top())
						branches_opened.push(u);

					int pos = adj->second;
					it_branches_list = branches_list.emplace_hint(it_branches_list, u, make_pair(0, list<pair<int, list<int>>>()));
					it_branches_list->second.first = pos;
				}

				if (!branches_list.empty())
				{
					branch_id = it_branches_list->second.first;
					auto &branch_list = it_branches_list->second.second;
					if ((int)branch_list.size() == branch_id)
						branch_list.emplace_back(pair<int, list<int>>());

					auto branch_it = next(branch_list.begin(), branch_id);

					if (u == baseID && u != v)
					{
						branch_it->second.emplace_back(u);
						branch_it->first = branch_list.size();
					}
					else if (v == baseID)
					{
						exit_branch = it_branches_list;
					}

					branch_it->second.emplace_back(v);
					branch_it->first = branch_list.size();
				}

				if (map_adj.count(v) && get<1>(map_adj[v]) == 1)
				{
					get<1>(map_adj[v]) = 2;
					stack_dfs.push(v);
				}
			}
			else
			{
				visitado = 0;
				stack_dfs.pop();
			}
		}

		// Ordena os branches com base na proximidade lógica em relação à base (simulando distância)
		int order = branches_list.size();
		map<int, int> branch_order;
		branch_order[baseID] = order--;
		branch_order[exit_branch->first] = order--;

		list<int> branches_with_node;
		it_branches_list = branches_list.begin();

		while (branch_order.size() < branches_list.size())
		{
			for (auto &bl : branches_list)
			{
				for (auto &br : bl.second.second)
				{
					if (!br.second.empty() && br.second.back() == exit_branch->first)
					{
						if (!branch_order.count(bl.first))
							branches_with_node.push_back(bl.first);
					}
				}
			}

			if (!branches_with_node.empty())
			{
				branch_order[branches_with_node.front()] = order--;
				exit_branch = branches_list.find(branches_with_node.front());
				branches_with_node.pop_front();
			}
		}

		// Atualiza o identificador de ordem dos branches com base na ordenação anterior
		for (auto &bl : branches_list)
		{
			for (auto &br : bl.second.second)
			{
				if (!br.second.empty())
					br.first = branch_order[br.second.back()];
			}
		}

		// Ordenar cada lista de branches
		for (auto &bl : branches_list)
		{
			vector<pair<int, list<int>>> temp(bl.second.second.begin(), bl.second.second.end());
			sort(temp.begin(), temp.end(), [](auto &a, auto &b)
				 { return a.first < b.first; });
			bl.second.second.assign(temp.begin(), temp.end());
		}

		// Constrói a sequência final de nós (union_branches) a partir dos branches ordenados
		list<int> union_branches;
		int root_node = baseID;

		while (!branches_list.empty())
		{
			it_branches_list = branches_list.find(root_node);
			if (it_branches_list == branches_list.end())
				break;

			auto &blist = it_branches_list->second.second;
			auto it_list = blist.begin();

			while (it_list != blist.end())
			{
				if (!it_list->second.empty() && it_list->second.back() == root_node)
				{
					union_branches.splice(union_branches.end(), list<int>(it_list->second.begin(), it_list->second.end()));
					it_list = blist.erase(it_list);
				}
				else
					++it_list;
			}

			if (!blist.empty())
			{
				auto &seq = blist.front().second;
				if (!seq.empty())
				{
					union_branches.insert(union_branches.end(), seq.begin(), seq.end());
					root_node = seq.back();
				}
				blist.pop_front();
			}
			if (blist.empty())
				branches_list.erase(it_branches_list);
		}

		// Reconstrói a lista de arestas original (p->edges) a partir da sequência de nós ordenada
		list<edge> original_edges(p->edges.begin(), p->edges.end());
		p->edges.clear();
		edge edge_temp;
		unordered_map<pair<int, int>, edge, pair_hash> original_edges_map;

		for (const auto &e : original_edges)
		{
			original_edges_map[{e.node_a, e.node_b}] = e;
		}

		while (union_branches.size() >= 2)
		{
			edge_temp.node_a = union_branches.front();
			union_branches.pop_front();
			edge_temp.node_b = union_branches.front();

			auto it = original_edges_map.find({edge_temp.node_a, edge_temp.node_b});
			if (it != original_edges_map.end())
			{
				edge_temp.cost = it->second.cost;
				edge_temp.time = it->second.time;
				original_edges_map.erase(it);
				p->edges.push_back(edge_temp);
			}
			else
			{
				p->pCost = -1;
				return;
			}
		}
		// Se restarem arestas não utilizadas, o caminho está inconsistente
		if (!original_edges_map.empty())
			p->pCost = -1;
	}

	list<pair<pair<pair<int, int>, double>, vector<int>>> Solution::GetTargetsLinkingCLs(path p)
	{

		list<pair<pair<pair<int, int>, double>, vector<int>>> nodes_between_cl;
		vector<int> depots_ids;

		pair<pair<int, int>, double> t2t;
		pair<pair<pair<int, int>, double>, vector<int>> t2t_depot;

		vector<edge>::iterator it_edges = p.edges.begin();
		int base_id = input.getRobotBaseId(p.robotID);

		// buscar o caminho entre linhas de cobertura
		while (it_edges != p.edges.end() && it_edges->node_b != base_id)
		{

			if (IsCLine(it_edges->node_a, it_edges->node_b))
			{
				++it_edges;
				continue;
			}
			auto it_target = p.fuelOnTarget.find(it_edges->node_a);

			// se o nó for target, obter todas as arestas até o node_b chegar num target
			if (it_target != p.fuelOnTarget.end())
			{

				// inicialize nodes
				t2t.first.first = 0;  // node_a
				t2t.first.second = 0; // node_b
				t2t.second = 0;		  // time node_a to node_b
				depots_ids.clear();	  // depots between targets

				it_target = p.fuelOnTarget.find(it_edges->node_b);
				// se não for target
				if (it_target == p.fuelOnTarget.end())
				{
					t2t.second += it_edges->time;
					t2t.first.first = it_edges->node_a;
					// depots_ids.emplace_back(it_edges->node_b);
					while (it_target == p.fuelOnTarget.end())
					{ // enquanto b não for target
						depots_ids.emplace_back(it_edges->node_b);
						++it_edges;
						if (it_edges->node_b == base_id)
							return nodes_between_cl;
						it_target = p.fuelOnTarget.find(it_edges->node_b);
						t2t.second += it_edges->time;
					}
					t2t.first.second = it_edges->node_b;
					t2t_depot.first = t2t;
					t2t_depot.second = depots_ids;
					// inserir o target inicial e o final e o tempo que é gasto
					nodes_between_cl.emplace_back(t2t_depot);
				}
			}
			++it_edges;
		}
		return nodes_between_cl;
	}

	// retornar os ids de todos os targets existentes entre node_a e node_b
	map<int, vector<pair<int, double>>> Solution::GetGraphOfDepotsBetween_T2T(int node_a, int node_b, int robot_id)
	{
		map<int, vector<pair<int, double>>> list_adj;
		vector<int> depots = getDepotsBetweenNodes(node_a, node_b);
		unsigned int nvertices = depots.size();
		double flight_time = 0;

		auto it_list_adj = list_adj.begin();

		for (unsigned int i = 0; i < nvertices; i++)
		{
			it_list_adj = list_adj.emplace_hint(it_list_adj, depots[i], vector<pair<int, double>>());

			for (unsigned int j = 0; j < nvertices; j++)
			{
				flight_time = getCostOnGraph(robot_id, depots[i], depots[j]);
				// caso o robô tenha capacidade de voar de i para j, adicione ao grafo dg
				if (isDefinitelyLessThan(flight_time, input.getRobotFuel(robot_id)))
				{
					it_list_adj->second.emplace_back(make_pair(depots[j], flight_time));
				}
			}
		}

		return list_adj;
	}

	map<int, vector<pair<int, double>>> Solution::AddTargetOnGraph(map<int, vector<pair<int, double>>> list_adj, int node_a, int node_b, int robot_id)
	{
		map<int, vector<pair<int, double>>> list_adj_temp = list_adj;
		int n1 = 0;
		double time_2_a, time_2_b;

		auto it_list_adj = list_adj.begin();
		// auto it_list_adj_temp = list_adj_temp.begin();

		double robot_capacity = input.getRobotFuel(robot_id);

		vector<pair<double, int>> dist_depots_2_node_a; // distância e depotID em relação ao node_a
		vector<pair<double, int>> dist_depots_2_node_b; // distância e depotID em relação ao node_b

		// obter a distância entre os node_a e node_b, e os depots da lista de adjacência
		while (it_list_adj != list_adj.end())
		{
			n1 = it_list_adj->first; // nós da lista

			// distância n1 em relaão ao node_a
			time_2_a = getCostOnGraph(robot_id, n1, node_a);
			if (time_2_a < robot_capacity)
			{
				dist_depots_2_node_a.emplace_back(make_pair(time_2_a, n1));
			}

			// distância n1 em relaão ao node_b;
			time_2_b = getCostOnGraph(robot_id, n1, node_b);
			if (time_2_b < robot_capacity)
			{
				dist_depots_2_node_b.emplace_back(make_pair(time_2_b, n1));
			}

			++it_list_adj;
		}

		sort(dist_depots_2_node_a.begin(), dist_depots_2_node_a.end());
		sort(dist_depots_2_node_b.begin(), dist_depots_2_node_b.end());

		// inserir na lista de adj ligação do node_a ao depot mais próximo e
		int node1 = dist_depots_2_node_a.front().second;
		double dist1 = dist_depots_2_node_a.front().first;
		vector<pair<int, double>> vec1;
		vec1.emplace_back(make_pair(node1, dist1));
		// insere o node_a como vizinho do node1
		list_adj.find(node1)->second.emplace_back(make_pair(node_a, dist1));

		// insere o node_a como uma entrada na lista de adjacência
		// como o viznho node1
		list_adj.emplace(node_a, vec1);

		// inserir na lista de adj ligação do node_b ao depot mais próximo.
		int node2 = dist_depots_2_node_b.front().second;
		int dist2 = dist_depots_2_node_b.front().first;
		vector<pair<int, double>> vec2;
		vec2.emplace_back(make_pair(node2, dist2));

		// insere o node_b como uma entrada na lista de adjacência
		// como o viznho node2
		list_adj.find(node2)->second.emplace_back(make_pair(node_b, dist2));
		list_adj.emplace(node_b, vec2);

		return list_adj;
	}

	vector<int> Solution::SPT_A_Star(map<int, vector<pair<int, double>>> list_adj, int start, int goal, int robot_id)
	{
		vector<edge> edges;
		// custo h(dist(node,goal)+custo do node), id do node e o custo do node
		priority_queue<pair<double, pair<int, double>>, vector<pair<double, pair<int, double>>>, greater<pair<double, pair<int, double>>>> pq;

		map<int, double> path_info;
		auto it_path = path_info.begin();

		vector<int> path_nodes;
		pair<double, pair<int, double>> pair_info;
		int node;
		double path_cost, cost_start_2_node, cost_node_2_goal, h;

		double robot_capacity = input.getRobotFuel(robot_id);

		double robot_fuel = 0;

		// insert new
		pq.push(make_pair(0.0, make_pair(start, 0.0)));

		auto it_adj = list_adj.begin()->second.begin();
		auto it_list = list_adj.begin();

		while (!pq.empty())
		{
			pair_info = pq.top();		   // obter o elemento de maior prioridade, menor valor
			node = pair_info.second.first; // obter o índice do nó em node

			pq.pop(); // remover o elemento de maior prioridade da fila

			it_path = path_info.find(node); // buscar o nó no caminho
			if (it_path == path_info.end())
			{										  // se o nó não está no caminho
				path_cost += pair_info.second.second; // adicionar o custo do node,saindo do nó anterior
				path_info.emplace(node, path_cost);	  // adicione o nó ao caminho e custo para chegar neste nó em relação ao start

				// resultado
				path_nodes.emplace_back(node);
				// se o nó inserido for o goal parar.
				if (node == goal)
					break;
				// buscar os vizinhos de node que não estão em path e inserir em pq
				// it_list =  list_adj.find(node);
				// it_adj = it_list->second.begin();//obter a lista de adjacentes à node
				// path_cost += it_adj->second;
				it_list = list_adj.find(node);
				if (it_list != list_adj.end())
				{
					auto &adj_list = it_list->second;
					if (!adj_list.empty())
					{
						it_adj = adj_list.begin();

						while (it_adj != it_list->second.end())
						{						  // para cada adjacente
							node = it_adj->first; // obter o indice do adjacente
							// verificar se o adjacente não está em path
							it_path = path_info.find(node);
							if (it_path == path_info.end())
							{ // caso não esteja
								// obter o custo do caminho do start até node

								cost_start_2_node = path_cost + it_adj->second;			 // custo path + custo do nó
								cost_node_2_goal = getCostOnGraph(robot_id, node, goal); // custo do nó até goal

								// forçar o robô se deslocar o máximo com o combustível, até o próximo posto.
								// caso o posto esteja no mesmo local do node
								if (cost_node_2_goal < 1)
								{ // caso o posto seja o goal
									if (node == goal)
										robot_fuel = 0;
									else
										robot_fuel = 1; // caso o objetivo seja o target com posto assiciado.
								}
								else
									robot_fuel = robot_capacity - it_adj->second;

								h = cost_start_2_node + cost_node_2_goal + robot_fuel;
								pq.push(make_pair(h, make_pair(node, it_adj->second))); // inserir na fila de prioridades
							}

							++it_adj; // obter o próximo adjacente
						}
					}
				}
			}
		}

		// caso não seja possível atingir o goal, limpar o vetor
		if (path_nodes.back() != goal)
			path_nodes.clear();

		return path_nodes;
	}

	/*void Solution::OpenRandomDepotsOnTargetsPath(path *p)
	{
		map<int, int> map_depots_on_targets;
		for (auto e : p->fuelOnTarget) // obter dos targets de p quais são os possíveis postos que podem estar abertos
			map_depots_on_targets.emplace(input.getDepotIdOnTarget(e.first), e.first);

		// remover do map os depots de targets associados ao caminho
		// o que sobrar são os depots não abertos;
		for (int depot : p->depots)
		{
			map_depots_on_targets.erase(depot);
		}

		if (map_depots_on_targets.empty())
			return;

		// obter o targets que não tem postos.
		vector<int> targets_without_depots;
		for (auto m : map_depots_on_targets)
			targets_without_depots.emplace_back(m.second);

		// embaralhar o vetor
		rand.sufferVector(&targets_without_depots);

		// selecionar os targets que terão postos abertos
		int t_size = rand.randNum(targets_without_depots.size());

		if (t_size > 2) // abrir 50% dos depósitos fechados
			t_size = floor(t_size * 0.5);
		else if (t_size == 0)
			return;

		targets_without_depots.resize(t_size);

		// obter a lista de targets, mais seguro que vetor para manter a consistência após novas inserções;
		list<int> ltargets;
		ltargets.insert(ltargets.begin(), targets_without_depots.begin(), targets_without_depots.end());
		list<edge> ledges;
		ledges.insert(ledges.begin(), p->edges.begin(), p->edges.end());

		int target = 0;
		edge new_edge;
		int robot_id = p->robotID;

		double robot_capacity = input.getRobotFuel(robot_id);
		double fuel_remaining = 0;
		double fuel_required = 0;

		auto it_fuel = p->fuelOnTarget.begin();
		auto it_depot = p->depots.begin();
		auto it_edges = ledges.begin();
		auto it_edge_adjfuel = ledges.begin();

		while (!ltargets.empty())
		{

			target = ltargets.front();

			it_edges = ledges.begin();
			while (it_edges != ledges.end())
			{
				if (!IsCLine(it_edges->node_a, it_edges->node_b))
				{
					if (it_edges->node_a == target)
					{ // se sair do target, deviar para o posto antes de sair

						new_edge.node_a = target;
						new_edge.node_b = input.getDepotIdOnTarget(target);

						new_edge.time = getCostOnGraph(robot_id, new_edge.node_a, new_edge.node_b);
						new_edge.cost = new_edge.time + (input.getRobotProp(robot_id) * new_edge.time);
						it_edge_adjfuel = ledges.insert(it_edges, new_edge);

						it_edges->node_a = new_edge.node_b;
						it_edges->time = getCostOnGraph(robot_id, it_edges->node_a, it_edges->node_b);
						it_edges->cost = it_edges->time + (input.getRobotProp(robot_id) * it_edges->time);

						p->depots.insert(new_edge.node_b);
						p->depotsNum = p->depotsNum + 1;

						// atualizar o combústive dos nos targets do segmento iniciando no novo posto até o próximo já existente no caminho
						fuel_remaining = robot_capacity;

						it_edge_adjfuel++;
						it_depot = p->depots.find(it_edge_adjfuel->node_b);
						while (it_depot == p->depots.end() && it_edge_adjfuel != ledges.end())
						{ // enquanto não for depot

							fuel_required = it_edge_adjfuel->time;
							fuel_remaining = fuel_remaining - fuel_required;

							it_fuel = p->fuelOnTarget.find(it_edge_adjfuel->node_b);

							if (it_fuel != p->fuelOnTarget.end())
								it_fuel->second = fuel_remaining;

							it_edge_adjfuel++;

							if (it_edge_adjfuel == ledges.end())
								break;

							it_depot = p->depots.find(it_edge_adjfuel->node_b);
						}

						ltargets.pop_front();
						if (ltargets.empty())
							break;
						target = ltargets.front();
					}
					// se chegar no target, deviar aresta de chegada para o posto antes de chegar
					else if (it_edges->node_b == target)
					{

						new_edge.node_a = it_edges->node_a;
						new_edge.node_b = input.getDepotIdOnTarget(target);
						new_edge.time = getCostOnGraph(robot_id, new_edge.node_a, new_edge.node_b);
						new_edge.cost = new_edge.time + (input.getRobotProp(robot_id) * new_edge.time);

						it_edge_adjfuel = ledges.insert(it_edges, new_edge);

						it_edges->node_a = new_edge.node_b;
						it_edges->time = getCostOnGraph(robot_id, it_edges->node_a, it_edges->node_b);
						it_edges->cost = it_edges->time + (input.getRobotProp(robot_id) * it_edges->time);

						p->depots.insert(new_edge.node_b);
						p->depotsNum = p->depotsNum + 1;

						// atualizar o combústive dos nos targets do segmento iniciando no novo posto até o próximo já existente no caminho
						fuel_remaining = robot_capacity;

						it_edge_adjfuel++;
						it_depot = p->depots.find(it_edge_adjfuel->node_b);
						while (it_depot == p->depots.end() && it_edge_adjfuel != ledges.end())
						{ // enquanto não for depot
							fuel_required = it_edge_adjfuel->time;
							fuel_remaining = fuel_remaining - fuel_required;

							it_fuel = p->fuelOnTarget.find(it_edge_adjfuel->node_b);
							if (it_fuel != p->fuelOnTarget.end())
								it_fuel->second = fuel_remaining;

							it_edge_adjfuel++;

							if (it_edge_adjfuel == ledges.end())
								break;

							it_depot = p->depots.find(it_edge_adjfuel->node_b);
						}

						ltargets.pop_front();
						if (ltargets.empty())
							break;
						target = ltargets.front();
					}
				}
				it_edges++;
			}
		}
		p->edges.clear();
		p->edges.insert(p->edges.begin(), ledges.begin(), ledges.end());

		// atualizar nodesSets
		vector<int> depots;
		depots.insert(depots.begin(), p->depots.begin(), p->depots.end());
		UpdateDepots(p->pID, depots);
	}*/

	void Solution::OpenRandomDepotsOnTargetsPath(path *p)
	{
		map<int, int> map_depots_on_targets;

		// Obter os depots associados aos targets usados no caminho
		for (auto &e : p->fuelOnTarget)
			map_depots_on_targets.emplace(input.getDepotIdOnTarget(e.first), e.first);

		// Remover os depots já abertos
		for (int depot : p->depots)
			map_depots_on_targets.erase(depot);

		if (map_depots_on_targets.empty())
			return;

		// Targets sem depósito aberto
		vector<int> targets_without_depots;
		for (const auto &m : map_depots_on_targets)
			targets_without_depots.emplace_back(m.second);

		// Embaralhar aleatoriamente
		rand.sufferVector(&targets_without_depots);

		int t_size = rand.randNum(targets_without_depots.size());
		if (t_size > 2)
			t_size = floor(t_size * 0.5);
		else if (t_size == 0)
			return;

		targets_without_depots.resize(t_size);
		list<int> ltargets(targets_without_depots.begin(), targets_without_depots.end());
		list<edge> ledges(p->edges.begin(), p->edges.end());

		int robot_id = p->robotID;
		double robot_capacity = input.getRobotFuel(robot_id);
		double fuel_remaining, fuel_required;

		set<int> processed_targets;

		auto it_edges = ledges.begin();

		while (!ltargets.empty())
		{
			int target = ltargets.front();
			if (processed_targets.count(target))
			{
				ltargets.pop_front();
				continue;
			}

			it_edges = ledges.begin();
			while (it_edges != ledges.end())
			{
				if (IsCLine(it_edges->node_a, it_edges->node_b))
				{
					++it_edges;
					continue;
				}

				edge new_edge;

				// Desvio ao sair do target
				if (it_edges->node_a == target)
				{
					int depot_id = input.getDepotIdOnTarget(target);
					if (depot_id == -1)
						break; // segurança

					new_edge.node_a = target;
					new_edge.node_b = depot_id;
					new_edge.time = getCostOnGraph(robot_id, new_edge.node_a, new_edge.node_b);
					new_edge.cost = new_edge.time + input.getRobotProp(robot_id) * new_edge.time;

					auto it_edge_adjfuel = ledges.insert(it_edges, new_edge);

					it_edges->node_a = new_edge.node_b;
					it_edges->time = getCostOnGraph(robot_id, it_edges->node_a, it_edges->node_b);
					it_edges->cost = it_edges->time + input.getRobotProp(robot_id) * it_edges->time;

					if (new_edge.node_b < 31)
						cerr << "openRDepotOnTargetPath ... problema id \n";

					p->depots.insert(new_edge.node_b);
					p->depotsNum++;

					fuel_remaining = robot_capacity;
					++it_edge_adjfuel;

					while (it_edge_adjfuel != ledges.end() && p->depots.find(it_edge_adjfuel->node_b) == p->depots.end())
					{
						fuel_required = it_edge_adjfuel->time;
						fuel_remaining -= fuel_required;
						p->fuelOnTarget[it_edge_adjfuel->node_b] = fuel_remaining;
						++it_edge_adjfuel;
					}

					processed_targets.insert(target);
					break;
				}

				// Desvio ao chegar no target
				else if (it_edges->node_b == target)
				{
					int depot_id = input.getDepotIdOnTarget(target);
					if (depot_id == -1)
						break;

					new_edge.node_a = it_edges->node_a;
					new_edge.node_b = depot_id;
					new_edge.time = getCostOnGraph(robot_id, new_edge.node_a, new_edge.node_b);
					new_edge.cost = new_edge.time + input.getRobotProp(robot_id) * new_edge.time;

					auto it_edge_adjfuel = ledges.insert(it_edges, new_edge);

					it_edges->node_a = new_edge.node_b;
					it_edges->time = getCostOnGraph(robot_id, it_edges->node_a, it_edges->node_b);
					it_edges->cost = it_edges->time + input.getRobotProp(robot_id) * it_edges->time;

					p->depots.insert(new_edge.node_b);
					p->depotsNum++;

					fuel_remaining = robot_capacity;
					++it_edge_adjfuel;

					while (it_edge_adjfuel != ledges.end() && p->depots.find(it_edge_adjfuel->node_b) == p->depots.end())
					{
						fuel_required = it_edge_adjfuel->time;
						fuel_remaining -= fuel_required;
						p->fuelOnTarget[it_edge_adjfuel->node_b] = fuel_remaining;
						++it_edge_adjfuel;
					}

					processed_targets.insert(target);
					break;
				}
				++it_edges;
			}
			ltargets.pop_front();
		}

		p->edges.assign(ledges.begin(), ledges.end());

		// Atualiza os depots associados ao path
		vector<int> depots(p->depots.begin(), p->depots.end());
		UpdateDepots(p->pID, depots);
	}

	void Solution::openRandomDepot(solution *s)
	{
		double new_cost_a2b = 0;
		int paths_num = s->paths.size();
		double pcost = 0;
		list<edge> l_edges;
		set<int> depots;
		vector<int> paths_id = rand.randVector(paths_num);

		// procurar um caminho que se possar abrir algum depot e diminuir o custo, iniciar a escolha do caminho de maneira aleatória
		for (int i : paths_id)
		{
			path *p = &s->paths[i];
			pcost = 0;

			OpenRandomDepotsOnTargetsPath(p);
			updateSolCosts(s);

			// obter os targets iniciais e finais que conectam as linhas de cobertura
			list<pair<pair<pair<int, int>, double>, vector<int>>> tls = GetTargetsLinkingCLs(*p);

			list<pair<pair<pair<int, int>, double>, vector<int>>> tls_temp = tls;

			// ordenar os pares em ordem crescente
			tls.sort([](const pair<pair<pair<int, int>, double>, vector<int>> &el1,
						const pair<pair<pair<int, int>, double>, vector<int>> &el2)
					 { return (isDefinitelyGreaterThan(el1.first.second, el2.first.second)); });

			// procurar abrir depots entre os diversos targets de ligação da rota, começando da maior.
			while (!tls.empty())
			{

				l_edges.clear();
				// obter as referências dos trechos de maior custo
				int node_a = tls.front().first.first.first;
				int node_b = tls.front().first.first.second;
				vector<int> current_link;

				// caso não haja nenhum depot
				if (tls.front().second.empty())
				{
					// cout <<"não foi possível abrir depósito" <<endl;
					return;
				}

				// inserir o primeiro target
				current_link.emplace_back(node_a);

				// inserir os depots
				current_link.insert(current_link.end(), tls.front().second.begin(), tls.front().second.end());

				// inserir o último target
				current_link.emplace_back(node_b);

				// remover o par da lista
				tls.pop_front();

				// inserir na lista, pois iremos remover parte do caminho
				l_edges.insert(l_edges.begin(), p->edges.begin(), p->edges.end());

				// obter o grafo (lista de adjacências) dos depósitos existente entre os targets
				map<int, vector<pair<int, double>>> list_adj = GetGraphOfDepotsBetween_T2T(node_a, node_b, p->robotID);

				// adicionar os target no grafo, ligar o node_a e node_b aos respectivos depósitos
				map<int, vector<pair<int, double>>> list_with_targets = AddTargetOnGraph(list_adj, node_a, node_b, p->robotID);

				int depot_start = list_with_targets.find(node_a)->second.begin()->first;
				int depot_goal = list_with_targets.find(node_b)->second.begin()->first;

				vector<int> new_link;
				new_link.emplace_back(node_a);
				// obter o menor caminho entre node_a e node_b
				vector<int> spt = SPT_A_Star(list_adj, depot_start, depot_goal, p->robotID);

				// se não for possível ligar node_a com node_b
				if (spt.empty())
				{
					// cout <<"não foi possível abrir depósito" <<endl;
					return;
				}

				new_link.insert(new_link.end(), spt.begin(), spt.end());
				new_link.emplace_back(node_b);

				// verificar se existe depots diferentes no novo link
				// ordenar os vetores, necessário para a operação de diferença
				// como não podemos perder a ordem dos vetores, criamos vetores temporários
				vector<int> current_link_temp = current_link;
				vector<int> new_link_temp = new_link;

				sort(current_link_temp.begin(), current_link_temp.end());
				sort(new_link_temp.begin(), new_link_temp.end());

				// criar o vetor de intersecção
				vector<int> v_inter(current_link.size() + new_link.size());
				auto it_v_inter = v_inter.begin();

				// obter os elementos que estão em new_link e não no vetor corrente;
				it_v_inter = set_difference(new_link_temp.begin(), new_link_temp.end(),
											current_link_temp.begin(), current_link_temp.end(), v_inter.begin());
				v_inter.resize(it_v_inter - v_inter.begin());

				// caso exista algum depot diferente no novo link, alterar o trecho no caminho.
				if (v_inter.size() > 0)
				{

					// montar o caminho no vetor de arestas
					edge e;
					vector<edge> edges;

					// atribuir os nós do trechos aos arcos e capturar os custos
					for (auto i = new_link.begin(); (i + 1) != new_link.end(); ++i)
					{
						e.node_a = *i;
						e.node_b = *(i + 1);
						e.time = getCostOnGraph(p->robotID, e.node_a, e.node_b);
						e.cost = e.time + (e.time * input.getRobotProp(p->robotID));
						edges.emplace_back(e);
						new_cost_a2b += e.cost;
					}

					// remover o trecho antigo do node_a ao node_b do caminho
					auto it_edge = l_edges.begin();
					auto f_edge2cut = l_edges.begin();
					auto s_edge2cut = l_edges.begin();
					// encontrar a posições target node_a e target node_b no caminho para remover a ligação antiga
					while (it_edge != l_edges.end())
					{
						if (it_edge->node_a == node_a)
							f_edge2cut = it_edge;

						else if (it_edge->node_b == node_b)
						{ // se encontrar o fim do trecho sair;
							it_edge++;
							s_edge2cut = it_edge;
							break;
						}
						++it_edge;
					}

					// inserir o novo trecho (edges) no caminho
					l_edges.erase(f_edge2cut, s_edge2cut);

					// inserir o novo trecho (edges) no caminho
					l_edges.insert(it_edge, edges.begin(), edges.end());

					double fuel_required = 0;
					double robot_capacity = input.getRobotFuel(p->robotID);
					auto it_fuel = p->fuelOnTarget.begin();
					double fuel_remaining = robot_capacity;
					pcost = 0;

					// mesmo o novo trecho sendo viável, quando inserido no caminho,
					// pode ser que se torne inviável, devido a quantidade de combustível que o robô
					// atinge o target node_b, seria importante procurar abrir depot até o caminho ser viável novamente.
					// isto é corrigir os abastecimento até o próximo depot após o novo trecho do caminho.
					it_edge = l_edges.begin();
					while (it_edge != l_edges.end())
					{
						fuel_required = it_edge->time;

						// fuel remaining on first arc node (node_a) of next arc
						fuel_remaining = fuel_remaining - fuel_required;

						// abrir depot no target inicial

						// caso o combustível no robô seja insuficiente para percorrer a aresta.
						// Abrir depot no nó de partida, considerando a particularidade se for linha de cobertura
						//  ou linha de ligação..
						if (fuel_remaining < 0)
						{

							// se for linha de ligação(entre linhas de cobertura)
							// se não for linha de cobetura e o primeiro nó for target(ligação entre linhas de cobertura)
							if (!IsCLine(it_edge->node_a, it_edge->node_b) && input.isTarget(it_edge->node_a))
							{
								// obter o segmento do menor caminho possível lingando node_a e node_b
								// passando pelos postos.

								// pcost -= it_edge->cost;
								int depot_id = input.getDepotIdOnTarget(it_edge->node_a);

								// obter  a aresta que liga o posto ao node_b
								edge e;
								e.node_a = depot_id;
								e.node_b = it_edge->node_b;
								e.time = getCostOnGraph(p->robotID, e.node_a, e.node_b);
								e.cost = e.time + (e.time * input.getRobotProp(p->robotID));

								// adicionar o custo dessa aresta
								pcost += e.cost;

								// desviar a aresta para o posto localizado no próprio node_a
								it_edge->node_b = e.node_a;
								it_edge->time = 0;
								it_edge->cost = 0;

								// avançar uma aresta
								it_edge++;

								// insere antes a nova aresta
								it_edge = l_edges.insert(it_edge, e);

								// se for target atualizar o combustível
								it_fuel = p->fuelOnTarget.find(it_edge->node_b);
								if (it_fuel != p->fuelOnTarget.end())
								{
									it_fuel->second = robot_capacity - e.time;
									// atualizar o combustível disponível ao percorre a nova aresta
									fuel_remaining = robot_capacity - e.time;
								}
								else // caso o nó de chegada seja posto
									fuel_remaining = robot_capacity;

								// inserir o depot caso não esteja na lista
								auto it_depot = p->depots.find(e.node_a);
								if (it_depot == p->depots.end() && mapNodesTypes[e.node_a] == 0)
								{
									p->depots.insert(e.node_a);
								}
							}
							// se for linha de cobertura
							else if (IsCLine(it_edge->node_a, it_edge->node_b))
							{

								int depot_id = input.getDepotIdOnTarget(it_edge->node_a);

								auto it_edge_temp = it_edge;

								// obter a aresta anterior
								it_edge_temp--;

								pcost -= it_edge_temp->cost;

								// montar a aresta do depósito ao node_b
								edge e;
								e.node_a = depot_id;
								e.node_b = it_edge->node_a;
								e.time = getCostOnGraph(p->robotID, e.node_a, e.node_b);
								e.cost = e.time + (e.time * input.getRobotProp(p->robotID));

								// altera a aresta do caminho, desviando para o posto localizado em node_b
								it_edge_temp->node_b = e.node_a;
								it_edge_temp->time = getCostOnGraph(p->robotID, it_edge_temp->node_a, it_edge_temp->node_b);
								;
								it_edge_temp->cost = it_edge_temp->time + (it_edge_temp->time * input.getRobotProp(p->robotID));

								// adicionar o tempo de deslocamento dessa arestra
								pcost += it_edge_temp->cost;

								// inserir a nova aresta que liga o posto ao primeiro nó da linha de cobertura
								it_edge_temp = l_edges.insert(it_edge, e);

								// inserir o depot caso não esteja na lista
								auto it_depot = p->depots.find(e.node_a);
								if (it_depot == p->depots.end() && mapNodesTypes[e.node_a] == 0)
								{
									p->depots.insert(e.node_a);
								}

								// atualizar o combustível nos targets da LC
								it_fuel = p->fuelOnTarget.find(it_edge->node_a);
								if (it_fuel != p->fuelOnTarget.end())
									it_fuel->second = robot_capacity;

								// atualizar o combustível nos targets da LC
								it_fuel = p->fuelOnTarget.find(it_edge->node_b);
								if (it_fuel != p->fuelOnTarget.end())
									it_fuel->second = robot_capacity - it_edge->time;

								// adicionar o tempo de deslocamento dessa LC
								pcost += it_edge->cost;

								// atualizar o combustível disponível apos percorrer a LC
								fuel_remaining = robot_capacity - it_edge->time;
							}
						}
						else
						{
							pcost += it_edge->cost;
							it_fuel = p->fuelOnTarget.find(it_edge->node_b);
							if (it_fuel != p->fuelOnTarget.end())
								it_fuel->second = fuel_remaining;
							else
							{
								// se não for target
								auto it_depot = p->depots.find(it_edge->node_b);
								if (it_depot == p->depots.end() && it_edge->node_b != input.getRobotBaseId(p->robotID) && mapNodesTypes[it_edge->node_b] == 0)
									p->depots.insert(it_edge->node_b);
								fuel_remaining = robot_capacity;
							}
						}
						it_edge++;
					}

					// limpar antigo caminho
					p->edges.clear();

					// inserir o novo
					p->edges.insert(p->edges.begin(), l_edges.begin(), l_edges.end());

					p->depotsNum = p->depots.size() + 1;

					p->pCost = pcost;

					//-----teste temporário da saída-------------------------------------------------------

					/*if(!IsCircuit(*p))
						cout <<"não circuito não validada openDepots " <<endl;
					if(!PathRestrictions(*p))
						cout <<"solução não validada openDepots " <<endl;

					checkTargetPath(i,*p);*/

					//------------------------------------------------------------

					// atualizar o custo da solução;
					updateSolCosts(s);

					vector<int> all_depots;
					all_depots.insert(all_depots.end(), s->depots.begin(), s->depots.end());
					for (uint i = 0; i < s->paths.size(); ++i)
						UpdateDepots(i, all_depots);
					return;
				}
			}
		}

		// cout <<"não foi possível abrir depósito" <<endl;
	}

	// for each graph, choose a line to remove and insert it in an other graph randomly choosed.
	void Solution::pshift(solution *s)
	{
		path pathG1;
		path pathG2;
		vector<int> depots;
		pair<path, string> path_op;

		struct pathIndex
		{
			path p1;
			path p2;
			int g1Index;
			int g2Index;
			int g1LID;
			string path_op;
		};

		pathIndex pIndex;
		vector<pair<double, pathIndex>> mapCostPaths;

		int line = -1;
		// int lastLine =-1;
		int iLine = 0;
		solution solTemp = *s;

		int p_chances = rand.randNum(s->paths.size());

		for (int pert = 0; pert <= p_chances; pert++)
		{

			// get index of graphs in random order
			vector<int> graphsIndex = rand.randVector(getNumberOfSets());
			int nOfCLines = 0;

			int g1Index = graphsIndex.back();
			graphsIndex.pop_back();

			// obter a quantidade de linhas de cobertura do grupo
			nOfCLines = getNumberOfLines(g1Index);
			// caso não haja nº linha de cobertura suficiente no grupo, testar o próximo
			if (nOfCLines <= 1)
				return;

			// obter os índices das linhas de cobertura de maneira embaralhada.
			vector<int> vLines = rand.randVector(nOfCLines);
			while (!vLines.empty())
			{ // enquanto não testar todas as linhas

				// get a coverage line on  g1 group
				iLine = vLines.back();
				// remove from vector
				vLines.pop_back();

				// obter o índice da linha em g1
				line = getCVLIndex(g1Index, iLine);

				pathG1 = removeCL(solTemp.paths[g1Index], line);

				if (pathG1.pCost <= 0)
					continue;
				// para todos os outro grupos da solução
				for (int g2Index : graphsIndex)
				{
					if (g1Index != g2Index)
					{ // com exceção de g1Index

						path_op = bestInsertionCL(solTemp.paths[g2Index], line);
						pathG2 = path_op.first;

						if (pathG2.pCost <= 1)
							continue;

						else
						{
							// atualizar nodesSets
							shiftCLine(g1Index, g2Index, iLine);

							// update solTemp with new path
							solTemp.paths[g1Index] = pathG1;
							solTemp.paths[g2Index] = pathG2;

							// changed the paths:update tempSol cost and depotsNum;
							updateSolCosts(&solTemp);

							// inserir no vetor de soluções
							insert_vecSol(solTemp);

							// update current solution s
							*s = solTemp;

							depots.clear();
							// inserir um possível atualização de depots no nodesSet
							depots.insert(depots.end(), s->depots.begin(), s->depots.end());
							for (uint i = 0; i < s->paths.size(); ++i)
							{
								UpdateDepots(i, depots);
							}

							//-------------------------------- Testes saída ----------------------------------

							/*checkTargetPath(g1Index,solTemp.paths[g1Index]);
							checkTargetPath(g2Index,solTemp.paths[g2Index]);

							if(!IsCircuit(pathG1))
								cout <<"g1 não circuito " <<endl;
							if(!IsCircuit(pathG2))
								cout <<" g2 não circuito " <<endl;
							if(!PathRestrictions(pathG1))
								cout <<"solução não validada pshift 1 pathG1" <<endl;

							if(!PathRestrictions(pathG2))
								//cout <<"solução não validada pshift 2 pathG2" <<endl;*/
							//---------------------------------------------------------------------------

							// cout << "Pshifted:" << pert <<" line: " << getCVLIndex(g1Index,iLine) << " from: " << g1Index <<" to " << g2Index ;
							return;
						}
					}
				}
			}
		}

		// cout << "pshift not done!!";
	}

	// obtém o posto aberto mais próximo que seja acessível, dada a quantidade de combustível no robô.
	// caso não seja possível atingir qualquer posto, retornar -1;
	int Solution::GetNearOpenedDepot(path p, int node_id)
	{

		// começar com número negativo, caso não haja depot próximo ao nó.
		int depot_id = -1;
		vector<pair<double, int>> depots_info;

		double flight_time = 0;
		double robot_fuel_available = 0;
		auto it_fuel = p.fuelOnTarget.find(node_id);
		auto it_depot = p.depots.find(node_id);

		// obter o combustível disponível antes de partir para o posto.
		if (it_fuel != p.fuelOnTarget.end())
		{											// se for target
			robot_fuel_available = it_fuel->second; // obter a quantidade disponível
		}
		else
		{
			if (it_depot != p.depots.end())
			{ // se o nó for um posto, o robô parte com capacidade máxima
				robot_fuel_available = input.getRobotFuel(p.robotID);
			}
			else
			{ // se o depot for um nó for um depot não presente na lista de depot.
				return -1;
			}
		}

		// obter as distâncias entre o nó e os postos abertos.
		// armazenar apenas as distância viáveis.
		for (int i : p.depots)
		{
			flight_time = getCostOnGraph(p.robotID, node_id, i);
			// se o combustível no robô for suficiente para atingir o posto
			if (!isDefinitelyLessThan(robot_fuel_available, flight_time))
				depots_info.emplace_back(make_pair(flight_time, i));
		}

		// ordenar o vetor em relaçao as distância do menor ao maior
		sort(depots_info.begin(), depots_info.end(),
			 [](pair<double, int> el1, pair<double, int> el2)
			 { return isDefinitelyLessThan(el1.first, el2.first); });

		// se houver depot, obter o índice do primeiro
		if (!depots_info.empty())
			depot_id = depots_info.begin()->second;

		return depot_id;
	}
	// criar o grafo entre o depots do caminho, as ligações são realizada levando em conta
	// a capacidade do robô.
	map<int, vector<pair<int, double>>> Solution::GetOpenDepotGraphFromPath(path p)
	{
		int robot_id = p.robotID;
		map<int, vector<pair<int, double>>> list_adj;
		vector<int> depots;
		depots.insert(depots.begin(), p.depots.begin(), p.depots.end());

		unsigned int nvertices = depots.size();
		double flight_time = 0;
		auto it_list_adj = list_adj.begin();

		// obter os custos entre os  depots
		for (unsigned int i = 0; i < nvertices; i++)
		{
			it_list_adj = list_adj.emplace_hint(it_list_adj, depots[i], vector<pair<int, double>>());

			for (unsigned int j = 0; j < nvertices; j++)
			{
				flight_time = getCostOnGraph(robot_id, depots[i], depots[j]);
				// caso o robô tenha capacidade de voar de i para j, adicione ao grafo
				if (isDefinitelyGreaterThan(input.getRobotFuel(robot_id), flight_time, 1.0))
				{
					it_list_adj->second.emplace_back(make_pair(depots[j], flight_time));
				}
			}
		}

		return list_adj;
	}

	Solution::path Solution::GetSPTOverOpenDepots(path p, int node_start, int node_goal)
	{
		vector<edge> edges;
		edge e;
		map<int, vector<pair<int, double>>> adj_list;
		path path_result;
		double robot_capacity = input.getRobotFuel(p.robotID);
		double fuel_remaining;

		path_result.pCost = 0;

		if (p.edges.empty())
		{
			return path_result;
		}

		// obter os grafo entre os depots do caminho p.
		adj_list = GetOpenDepotGraphFromPath(p);

		// inserir a ligação dos nós start e goal ao grafo;
		adj_list = AddNodesOnGraph(adj_list, p, node_start, node_goal);

		// obter o menor caminho entre node_a e node_b
		vector<int> new_link = SPT_A_Star(adj_list, node_start, node_goal, p.robotID);

		// caso não haja caminho entre start e goal.
		if (new_link.empty())
		{
			path_result.pCost = -1;
			path_result.edges.clear();
			return path_result;
		}

		auto it_fuel = p.fuelOnTarget.find(node_start);

		// combustível nó inicial presente no caminho p
		if (it_fuel != p.fuelOnTarget.end())
		{
			path_result.fuelOnTarget.emplace(node_start, it_fuel->second);
		}
		else if (mapNodesTypes[node_start] == 0)
		{ // se o nó for depot
			path_result.depots.insert(node_start);
		}

		// auto it_depot = path_result.fuelOnTarget.begin();
		// atribuir os nós do trechos ao arcos e capturar os custos
		for (auto i = new_link.begin(); (i + 1) != new_link.end(); ++i)
		{
			e.node_a = *i;
			e.node_b = *(i + 1);
			e.time = getCostOnGraph(p.robotID, e.node_a, e.node_b);
			e.cost = e.time + (e.time * input.getRobotProp(p.robotID));
			path_result.edges.emplace_back(e);

			// atualizar o custo do segmento
			path_result.pCost += e.cost;

			it_fuel = p.fuelOnTarget.find(e.node_a);

			// combustível disponível no node_a
			if (it_fuel != p.fuelOnTarget.end())
				fuel_remaining = it_fuel->second;

			else // se o node_a for posto
				fuel_remaining = robot_capacity;

			fuel_remaining = fuel_remaining - e.time;

			if (isDefinitelyLessThan(fuel_remaining, 0.0))
			{
				path_result.pCost = -1;
				path_result.edges.clear();
				return path_result;
			}

			// o nó goal não está em p..dado que o robô sai do depot e atinge o target
			it_fuel = p.fuelOnTarget.find(e.node_b);
			if (input.isTarget(e.node_b))
			{
				path_result.fuelOnTarget.emplace(e.node_b, fuel_remaining);
			}
			else if (mapNodesTypes[e.node_b] == 0)
				path_result.depots.insert(e.node_b);
		}

		//----------------------teste temporário saída--------------------------------
		/*if(HasRepeatedCLines(path_result)){
					cout <<"sptOver repeatedline pshift" <<endl;
		}*/
		//------------------------------------------------------
		return path_result;
	}

	map<int, vector<pair<int, double>>> Solution::AddNodesOnGraph(map<int, vector<pair<int, double>>> list_adj,
																  path p, int node_a, int node_b)
	{
		map<int, vector<pair<int, double>>> list_adj_temp = list_adj;
		int n1 = 0;
		int n2 = 0;
		double time_2_a, time_2_b;

		int robot_id = p.robotID;
		int base_id = input.getRobotBaseId(robot_id);
		int depot_on_target_id = 0;

		auto it_list_adj = list_adj.begin();
		auto it_list_adj_temp = list_adj_temp.begin();

		double robot_capacity = input.getRobotFuel(robot_id);
		double robot_fuel_available = 0;
		double fuel_consumed = -1;

		auto it_fuel = p.fuelOnTarget.find(node_a);
		auto it_depot = p.depots.find(node_a);

		bool has_depot_on_target = false;

		// obter o combustível disponível no nó de partida, antes de seguir para o posto.
		if (it_fuel != p.fuelOnTarget.end())
		{											// se for target
			robot_fuel_available = it_fuel->second; // obter a quantidade disponível
			fuel_consumed = robot_capacity - robot_fuel_available;
		}
		else
		{
			if (it_depot != p.depots.end() || node_a == base_id) // se o nó for um posto, o robô parte com capacidade máxima
				robot_fuel_available = robot_capacity;

			else
			{ // se o depot for um nó for um depot não presente na lista de depot.
				list_adj.clear();
				return list_adj;
			}
		}

		// se node b for target
		if (input.isTarget(node_b))
			depot_on_target_id = input.getDepotIdOnTarget(node_b);

		if (it_fuel != p.fuelOnTarget.end() || node_a == base_id)
		{ // se node_a for target ligar a todos os depots respeitando a capacidade do robô

			// ou seja o combustível presente no node_a
			// código para ligar os dois targets a todos os depots da lista de adj
			while (it_list_adj != list_adj.end())
			{
				n1 = it_list_adj->first;

				// se existir o depot de node_b
				if (n1 == depot_on_target_id)
					has_depot_on_target = true;

				// tempo de cada depot no grafo ao nó primeiro nó que teremos inserir no segmento
				// chamemos esse nó de partida
				time_2_a = getCostOnGraph(robot_id, n1, node_a);

				// obter a lista n1
				it_list_adj_temp = list_adj_temp.find(n1);

				// sendo o nó de partida target a autonomia do robô provavelmente será reduzida.
				// verificar se a autonomia é suficiente para realizar o movimento, caso não,
				// continuar a cálculo para os demais depots
				if (time_2_a < robot_fuel_available)
				{

					// inserir um novo vizinho (node_a) na lista n1

					// it_list_adj_temp->second.emplace_back(make_pair(node_a,time_2_a));

					// criar uma nova lista (node_a) e inserir o vizinho n1
					it_list_adj_temp = list_adj_temp.emplace_hint(it_list_adj_temp, node_a, vector<pair<int, double>>());
					it_list_adj_temp->second.emplace_back(make_pair(n1, fuel_consumed + time_2_a));
				}
				++it_list_adj;
			}
			/*if(!has_depot_on_target){
				//inserir ligação de a para b se não existir um depot na posição de node_b
				time_2_a = getCostOnGraph(robot_id,node_a, node_b);
				if(time_2_a < robot_fuel_available){
					it_list_adj_temp = list_adj_temp.emplace_hint(it_list_adj_temp,node_a,vector<pair<int,double>>());
					it_list_adj_temp->second.emplace_back(make_pair(node_b,time_2_a));
				}
			}*/
		}

		auto it_depot_b = p.depots.find(depot_on_target_id);

		// realizar apenas uma a ligação do depot ao target de saída;
		if (it_depot_b != p.depots.end())
		{ // caso haja o depósito associado ao target
			// obter a lista n2
			it_list_adj_temp = list_adj_temp.find(depot_on_target_id);
			// inserir um novo vizinho (node_b) na lista n2
			it_list_adj_temp->second.emplace_back(make_pair(node_b, time_2_b));
		}

		else
		{ // caso contrário ligar todos os depots ao target

			it_list_adj = list_adj.begin();

			// ou seja o combustível presente no node_a
			// código para ligar os dois targets a todos os depots da lista de adj
			while (it_list_adj != list_adj.end())
			{
				n2 = it_list_adj->first;

				// tempo de cada depot no grafo ao nó primeiro nó que teremos inserir no segmento
				// chamemos esse nó de partida
				time_2_b = getCostOnGraph(robot_id, n2, node_b);

				// obter a lista n1
				it_list_adj_temp = list_adj_temp.find(n2);

				// sendo o nó de partida target a autonomia do robô provavelmente será reduzida.
				// verificar se a autonomia é suficiente para realizar o movimento, caso não,
				// continuar a cálculo para os demais depots
				if (time_2_b < robot_capacity)
				{

					// inserir um novo vizinho (node_a) na lista n1
					it_list_adj_temp->second.emplace_back(make_pair(node_b, time_2_b));
				}
				++it_list_adj;
			}
		}
		return list_adj_temp;
	}

	/*bool Solution::HasRepeatedCLines(path p)
	{

		bool has_repeated_line = false;
		map<int, double> target_temp = p.fuelOnTarget;
		auto p_target1 = target_temp.begin();
		auto p_target2 = target_temp.begin();

		list<edge> edges;
		edges.insert(edges.begin(), p.edges.begin(), p.edges.end());
		auto p_edges = edges.begin();
		/// verificar arestas repetidas
		while (p_edges != edges.end())
		{
			if (IsCLine(p_edges->node_a, p_edges->node_b))
			{

				p_target1 = target_temp.find(p_edges->node_a);
				p_target2 = target_temp.find(p_edges->node_b);

				if (p_target1 != target_temp.end() && p_target2 != target_temp.end())
				{
					target_temp.erase(p_target1);
					target_temp.erase(p_target2);
					p_edges = edges.erase(p_edges);
					continue;
				}
			}
			p_edges++;
		}

		p_edges = edges.begin();
		/// verificar arestas repetidas
		while (p_edges != edges.end())
		{
			if (IsCLine(p_edges->node_a, p_edges->node_b))
			{
				has_repeated_line = true;
				break;
			}

			p_edges++;
		}
		return has_repeated_line;
	}*/

	bool Solution::HasRepeatedCLines(path p)
	{
		std::set<std::pair<int, int>> seen;

		for (const edge &e : p.edges)
		{
			if (IsCLine(e.node_a, e.node_b))
			{
				// Normalizar ordem dos nós
				int u = std::min(e.node_a, e.node_b);
				int v = std::max(e.node_a, e.node_b);

				auto edge_pair = std::make_pair(u, v);

				// Se já foi vista, é repetida
				if (seen.count(edge_pair))
					return true;

				seen.insert(edge_pair);
			}
		}

		return false; // Nenhuma linha de cobertura repetida
	}

	bool Solution ::IsBetterSol(solution new_sol, solution incumbent_sol)
	{
		if (new_sol.depots.size() < incumbent_sol.depots.size() ||
			isDefinitelyGreaterThan(incumbent_sol.maxCost, new_sol.maxCost, 1.0))
		{
			return true;
		}
		return false;
	}

} /* namespace std */
