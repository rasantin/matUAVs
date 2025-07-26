/*
 * MHCP.cpp
 *
 *  Created on: 15 de julho de 2019
 *      Author: rsantin
 * MHCP.cpp - Solver para o problema MHCP (Multi-Robot Heterogeneous Covering Problem)
 * 
 * Estratégia: Algoritmo híbrido VNS/VND (Variable Neighborhood Search / Variable Neighborhood Descent)
 * 
 * - VNS (Busca em Vizinhança Variável): 
 *   - Gera soluções perturbadas para escapar de ótimos locais.
 *   - Utiliza a função `perturbation()` para explorar diferentes vizinhanças.
 * 
 * - VND (Descida em Vizinhança Variável):  
 *   - Aplica operadores locais (shift, swap, swapRobots, closeRandomDepot) em ordem.
 *   - Se uma melhoria é encontrada, reinicia o processo; caso contrário, passa para o próximo operador.
 * 
 * - Pareto Front: 
 *   - Mantém um conjunto de soluções não-dominadas (`paretoSet`).
 *   - Avalia trade-offs entre custo e cobertura.
 * 
 * Fluxo principal:
 * 1. Perturbação (VNS) → 2. Busca Local (VND) → 3. Atualização do Pareto Front → Repete até critério de parada.
 */

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <ctime>

#include "Input.h"
#include "Solution.h"
#include "Output.h"

using namespace std;


string datetime()
{
    time_t rawtime;
    struct tm * timeinfo;
    char buffer[80];

    time (&rawtime);
    timeinfo = localtime(&rawtime);

    strftime(buffer,80,"%d-%m-%Y-%H-%M-%S",timeinfo);
    return string(buffer);
}

int main(int argc, char **argv) {
	int exec = 1;
	int m = 1;
	int n=1;
	//máximo de quantidade de linhas de cobertura por subset

	//bool best_prediction = false;
	bool best_prediction = true;

	std::chrono::time_point<std::chrono::system_clock> start, end;
	std::chrono::time_point<std::chrono::system_clock> start_op, end_op;
	chrono::duration <std::chrono::system_clock> el_s;

	double elapsed_seconds;

	string program_name = argv[0];
	string fileName = argv[1];

	size_t  found = program_name.find_last_of("/\\");
	program_name = program_name.substr(found+1);

	cout << "Program: "<< program_name <<endl;
	cout << "Start Reading:" << fileName <<"\n";

	Input input (fileName);
	input.printNodes();
	input.printRobots();
	Output output(input);

	int cvl_subset_num = input.getMaxCVLSubSet();

// =============================================
// Algoritmo Principal: VNS/VND Híbrido
// =============================================
// 1. Gera solução inicial (Solution s).
// 2. Enquanto houver soluções não visitadas:
//    a. Perturbação (VNS): Gera uma nova solução vizinha.
//    b. VND: Aplica operadores locais até não haver melhorias:
//       i.   Shift: Move tarefas entre robôs.
//       ii.  Swap: Troca tarefas entre robôs.
//       iii. SwapRobots: Troca rotas inteiras entre robôs.
//       iv.  CloseRandomDepot: Remove um depósito aleatório.
//    c. Se a solução é melhor que a atual, atualiza.
//    d. Se não há melhoria, retorna à melhor solução conhecida.
// 3. Atualiza o Pareto Front com soluções não-dominadas.
// 4. Repete para múltiplas execuções (input.getNExec()).
// =============================================
	while(exec <= input.getNExec()){

		output.createOutput(datetime());
		Solution s(input,cvl_subset_num);
		s.best_prediction = best_prediction;

		int maxDepots = s.getDepotsNumInit();
		int targetsNum = s.getTargetsNum();

		start = std::chrono::system_clock::now();
		while(s.HasSolutionNotVisited() && m <= targetsNum){

			s.currentSol = s.best_sol;
			//vns
			//s.printSol(s.currentSol);
			//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
			//start_op = std::chrono::system_clock::now();
			s.perturbation(&s.currentSol,maxDepots);
			//end_op= std::chrono::system_clock::now();
			//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
			//cout << "Perturbation time:" <<elapsed_seconds <<"segundos" <<endl;

			while(n <= input.getN()){
				//vnd
				//s.printSol(s.currentSol);
				//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				//start_op = std::chrono::system_clock::now();
				if(s.shift(&s.currentSol)){
					//end_op= std::chrono::system_clock::now();
					//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				//end_op= std::chrono::system_clock::now();
				//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				//s.printSol(s.currentSol);
				//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				//start_op = std::chrono::system_clock::now();
				if(s.swap(&s.currentSol)){
					//end_op= std::chrono::system_clock::now();
					//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				//end_op= std::chrono::system_clock::now();
				//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				//s.printSol(s.currentSol);
				//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				//start_op = std::chrono::system_clock::now();
				if(s.swapRobots(&s.currentSol)){
					//end_op= std::chrono::system_clock::now();
					//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				//end_op= std::chrono::system_clock::now();
				//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				//s.printSol(s.currentSol);
				//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				//start_op = std::chrono::system_clock::now();
				if(s.closeRandomDepot(&s.currentSol)){
					//end_op= std::chrono::system_clock::now();
					//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}

				//end_op= std::chrono::system_clock::now();
				//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				//cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				//if an improvement is obtained, the new incumbent is updated
				if(s.IsBetterSol(s.currentSol,s.best_sol))
					s.best_sol = s.currentSol;

				else{
					s.currentSol = s.best_sol;
					//update nodesSet;
					s.solutionToNodesSet(s.best_sol);
				}

				//s.printSol(s.currentSol);
				//cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				//start_op = std::chrono::system_clock::now();
				s.perturbation(&s.currentSol,maxDepots);
				//end_op= std::chrono::system_clock::now();
				//elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				//cout << "Perturbation time:" <<elapsed_seconds <<"segundos" <<endl;


				n++;
			}

			cout << endl;
			s.eval_VecSol();
			s.print_paretoSet();
			if(s.HasSolutionNotVisited()){
				s.best_sol = s.get_solution_not_visited();
				s.solutionToNodesSet(s.best_sol);
			}
			n=1;
			m++;
			output.gurobiCallInfo(s);
			s.ClearGurobiCallInfo();
			output.writePredictions(s);
			s.ClearPredictionInfo();
		}
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_time = end-start;
		std::cout << "secs : " << duration_time.count() << " s\n";

		output.writeParetoSet(s,program_name, fileName, datetime(),duration_time.count(),targetsNum, s.paretoSetValidation(),
				exec, input.getNExec(), input.getM(),input.getN(),cvl_subset_num);
		m=1;
		exec++;

	}
	return 0;
}



