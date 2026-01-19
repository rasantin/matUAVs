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

//#include <unistd.h>
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
	//mÃĄximo de quantidade de linhas de cobertura por subset

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

	while(exec <= input.getNExec()){
		std::cout << "\n[EXEC] Start exec " << exec << "/" << input.getNExec() << std::endl;
		n=1;
		m=1;
		output.createOutput(datetime());

		Solution s(input,cvl_subset_num);
		s.best_prediction = best_prediction;

		int maxDepots = s.getDepotsNumInit();
		int targetsNum = s.getTargetsNum();

		start = std::chrono::system_clock::now();

		std::cout << "[EXEC " << exec << "] HasSolutionNotVisited = "
          << s.HasSolutionNotVisited()
          << ", targetsNum = " << targetsNum << std::endl;
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
			//while(n <= 1){
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
				if(s.improveSol(&s.currentSol)){
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

			output.gurobiCallInfo(s);
			s.ClearGurobiCallInfo();
			output.writePredictions(s);
			s.ClearPredictionInfo();
			n=1;
			m++;
		}
		end = std::chrono::system_clock::now();
		std::chrono::duration<double> duration_time = end-start;
		std::cout << "secs : " << duration_time.count() << " s\n";

		output.writeParetoSet(s,program_name, fileName, datetime(),duration_time.count(),targetsNum, s.paretoSetValidation(),
				exec, input.getNExec(), input.getM(),input.getN(),cvl_subset_num);
		m=1;
			std::cout << "[EXEC " << exec << "] Finished. m=" << m << " n=" << n << std::endl;

		exec++;

	}

	return 0;
}


