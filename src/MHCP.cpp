/*
 * MHCP.cpp
 *
 *  Created on: 15 de julho de 2019
 *      Author: rsantin
 */

/*algoritmo 3: alg_3
 *
 * Este repositÃģrio ÃĐ originÃĄrio do repositÃģrio algb. O antigo alg3 foi renomeado para Old_alg3
 * O alg_b foi criado para fazer um merge dos novos ajustes do alg2 com as propriedades do alg3.
 *
 */

#include <iostream>
#include <cstdlib>
#include <chrono>
#include <ctime>

#include "Input.h"
#include "Solution.h"
#include "Output.h"
#include "SolverContext.h"

#include <sys/resource.h>
#include <csignal>

// #include <unistd.h>
using namespace std;


string datetime()
{
	time_t rawtime;
	struct tm *timeinfo;
	char buffer[80];

	time(&rawtime);
	timeinfo = localtime(&rawtime);

	strftime(buffer, 80, "%d-%m-%Y-%H-%M-%S", timeinfo);
	return string(buffer);
}

int main(int argc, char **argv)
{

	int exec = 1;
	int m = 1;
	int n = 1;

	struct rlimit rl;
	getrlimit(RLIMIT_STACK, &rl);
	rl.rlim_cur = 256 * 1024 * 1024; // 256MB
	setrlimit(RLIMIT_STACK, &rl);

	// SolverContext solverCtx; // inicializa contexto do solver Gurobi
	//  bool best_prediction = false;
	bool best_prediction = true;

	//std::chrono::time_point<std::chrono::system_clock> start, end;desabilitado para debugar 14/05/26
	//std::chrono::time_point<std::chrono::system_clock> start_op, end_op;desabilitado para debugar 14/05/26
	//chrono::duration<std::chrono::system_clock> el_s;desabilitado para debugar 14/05/26

	double elapsed_seconds;
	string program_name = argv[0];
	string fileName = argv[1];

	size_t found = program_name.find_last_of("/\\");
	program_name = program_name.substr(found + 1);

	cout << "Program: " << program_name << endl;
	cout << "Start Reading:" << fileName << "\n";

	Input input(fileName);
	input.printNodes();
	input.printRobots();
	//Output output(input); desabilitado para debugar 14/05/26

	int cvl_subset_num = input.getMaxCVLSubSet();
	
	auto solverCtx = std::make_unique<SolverContext>();


	while (exec <= input.getNExec())
	{
		std::cout << "\n[EXEC] Start exec " << exec << "/" << input.getNExec() << std::endl;
		n = 1;
		m = 1;
		//output.createOutput(datetime());desabilitado para debugar 14/05/26

		// Construtor de Solution: inicializa best_sol, atualiza depósitos globais,
		// verifica Pareto e prepara o objeto para o VNS, passando SolverContext para solver Gurobi
		Solution s(*solverCtx, input, cvl_subset_num);

		s.best_prediction = best_prediction;

		int maxDepots = s.getDepotsNumInit();
		int targetsNum = s.getTargetsNum();

		//start = std::chrono::system_clock::now(); desabilitado para debugar 14/05/26

		std::cout << "[EXEC " << exec << "] HasSolutionNotVisited = "
				  << s.HasSolutionNotVisited()
				  << ", targetsNum = " << targetsNum << std::endl;
		/*while (s.HasSolutionNotVisited() && m <= targetsNum)
		{

			s.currentSol = s.best_sol;
			// vns
			// s.printSol(s.currentSol);
			// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
			// start_op = std::chrono::system_clock::now();
			//s.perturbation(&s.currentSol, maxDepots); desabilitado para debugar 14/05/26
			// end_op= std::chrono::system_clock::now();
			// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
			// cout << "Perturbation time:" <<elapsed_seconds <<"segundos" <<endl;

			while (n <= input.getN())
			{
				// while(n <= 1){
				// vnd
				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				if (s.shift())
				{
					// end_op= std::chrono::system_clock::now();
					// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				if (s.swap(&s.currentSol))
				{
					// end_op= std::chrono::system_clock::now();
					// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				if (s.improveSol(*solverCtx, &s.currentSol))
				{
					// end_op= std::chrono::system_clock::now();
					// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				if (s.swapRobots(&s.currentSol))
				{
					// end_op= std::chrono::system_clock::now();
					// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}
				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				if (s.closeRandomDepot(&s.currentSol))
				{
					// end_op= std::chrono::system_clock::now();
					// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
					// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;
					continue;
				}

				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << " time:" <<elapsed_seconds <<"segundos" <<endl;

				// if an improvement is obtained, the new incumbent is updated
				if (s.IsBetterSol(s.currentSol, s.best_sol))
					s.best_sol = s.currentSol;

				else
				{
					s.currentSol = s.best_sol;
					// update nodesSet;
					s.solutionToNodesSet(s.best_sol);
				}

				// s.printSol(s.currentSol);
				// cout << "Vector of Solutions:" << s.vecSol.size() <<endl;
				// start_op = std::chrono::system_clock::now();
				//s.perturbation(&s.currentSol, maxDepots);
				// end_op= std::chrono::system_clock::now();
				// elapsed_seconds = std::chrono::duration_cast<std::chrono::seconds> (end_op-start_op).count();
				// cout << "Perturbation time:" <<elapsed_seconds <<"segundos" <<endl;

				n++;
			}
			cout << endl;
			s.eval_VecSol();
			s.print_paretoSet();
			
			if (s.HasSolutionNotVisited())
			{
				s.best_sol = s.get_solution_not_visited();
				s.solutionToNodesSet(s.best_sol);
			}

			//output.gurobiCallInfo(s); desabilitado para debugar 14/05/26
			//s.ClearGurobiCallInfo();desabilitado para debugar 14/05/26
			//output.writePredictions(s); desabilitado para debugar 14/05/26
			//s.ClearPredictionInfo(); //desabilitado para debugar 14/05/26
			n = 1;
			m++;
		}*/
		//end = std::chrono::system_clock::now();desabilitado para debugar 14/05/26
		//std::chrono::duration<double> duration_time = end - start; desabilitado para debugar 14/05/26
		//std::cout << "secs : " << duration_time.count() << " s\n";

		//output.writeParetoSet(s, program_name, fileName, datetime(), duration_time.count(), targetsNum, s.paretoSetValidation(),
		//					  exec, input.getNExec(), input.getM(), input.getN(), cvl_subset_num); desabilitado para debugar 14/05/26
		//					  8/
		m = 1;
		std::cout << "[EXEC " << exec << "] Finished. m=" << m << " n=" << n << std::endl;

		exec++;
	}

	return 0;
}
