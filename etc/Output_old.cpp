/*
 * Output.cpp
 *
 *  Created on: 17 de jun de 2017
 *      Author: rsantin
 */

#include "Output.h"


Output::Output() {
	// TODO Auto-generated constructor stub
}

Output::Output(Input input){
	this->input = input;

	createDirOutput();
	createDirInst(input.GetFileName());

}


Output::~Output() {
	// TODO Auto-generated destructor stub
}
void Output::writeNodes(){

	ofstream file;

	string nodesFile = nodesPath;
	nodesFile.append("/nodes.txt");

	file.open(nodesFile);
	if(file.is_open() && !input.nodes.empty()){
		file << "node_id" <<" "<<"x" <<" "<<"y" <<" " << "nodeType" "\n";
		for(uint ind = 0;ind<input.nodes.size();ind++){
			file << input.nodes[ind].getNodeId() << " " << input.nodes[ind].getX() << " " << input.nodes[ind].getY() << " "
					<< input.nodes[ind].getNodeType()<<"\n";
		}
	}
	file.close();
}


void Output::createDir(int it){

	if(!execPath.empty()){
		execPath.clear();
	}

	string command =("mkdir") ;
	execPath.append("exec_");
	execPath.append(to_string(it));
	command.append(" ");
	command.append(execPath);
	system(command.c_str());


	nodesPath.assign(execPath);
	nodesPath.append("/output/nodes");
	command.clear();
	command.append("mkdir -p");
	command.append(" ");
	command.append(nodesPath);
	system(command.c_str());

	solutionPath.assign(execPath);
	solutionPath.append("/output/solutions");
	command.clear();
	command.append("mkdir -p");
	command.append(" ");
	command.append(solutionPath);
	system(command.c_str());
}

void Output::writeSolution(Solution sol, int it, string op, Solution::alog vars){

	string fileName;
	fileName.assign(solutionPath);
	fileName.append("/solution_");

	string s_it = to_string(it);
	fileName.insert(fileName.size(),s_it);
	fileName.insert(fileName.size(),"_");
	fileName.insert(fileName.size(),op);
	if(op.compare("init")!=0 && op.compare("best")!=0 ){
		fileName.append("_G");
		fileName.append(to_string(vars.g1));
		fileName.append("_L");
		fileName.append(to_string(vars.l1));
		fileName.append(":G");
		fileName.append(to_string(vars.g2));
		if(vars.l2 != -1){
			fileName.append("_L");
			fileName.append(to_string(vars.l2));
		}
	}
	fileName.insert(fileName.size(),".txt");

	ofstream file;
	file.open(fileName);

	if(file.is_open()){
		file << "from" << " "<< "to" << " " <<"f"<< " "<<" "<<"objval" <<  " " <<"Group" << " " <<"\n";
		uint f,s;

		for(uint k=0;k<sol.currentSol.paths.size();k++){
			for(uint i=0;i<sol.currentSol.paths[k].nodes.size();i++){
				f = sol.currentSol.paths[k].nodes[i].first.first;
				s = sol.currentSol.paths[k].nodes[i].first.second;
				file << f << " " << s  << " "
						<<sol.currentSol.paths[k].nodes[i].second.first <<"  "
						<<sol.currentSol.paths[k].pCost << " "
						<< k <<" "<<"\n";
			}
		}
	}

	file.close();
}


void Output::writeOutput(double maxCost, int iter, string op ){
	string outputPath;
	ofstream outputFile;

	outputPath.assign(execPath);
	outputPath.append("/output.txt");

	string s_iter = to_string(iter);
	string s_maxCost = to_string(maxCost);

	outputFile.open(outputPath, std::ofstream::out | std::ofstream::app);
	outputFile <<  s_iter << " " << s_maxCost << " " << op <<"\n";
	outputFile.close();
}

void Output::writeSolutions(Solution sol, string op){
	string solutionsPath;
	ofstream solutionsFile;

	solutionsPath.assign(execPath);
	solutionsPath.append("/solutions.txt");

	solutionsFile.open(solutionsPath, std::ofstream::out | std::ofstream::app);

	solutionsFile << op << " ";
	for(uint i = 0; i < sol.currentSol.paths.size();i++){
		solutionsFile << sol.currentSol.paths[i].pCost << " ";
	}
	solutionsFile <<"\n";
	solutionsFile.close();
}


void Output::writeParetoSet(Solution sol, string pname, string iName,string date,
		double elapsed_time, int targetsNum, vector<bool> validation,int nexec, int total_exec, int m, int n, int cvl_subset_num){
	int nsol = 1;
	int npath =1;

	string solutions;
	ofstream solutionsFile;

	string solution;
	string solutionName;

	ofstream solutionFile;

	string path;
	ofstream pathFile;

	bool val = all_of(validation.begin(),validation.end(),[](bool v){return v;});

	solutions.append(execPath);
	solutions.append("solutions.txt");
	solutionsFile.open(solutions, std::ofstream::out | std::ofstream::app);

	solutionsFile << "Algorithm: " << pname << "\n" ;
	solutionsFile << "Instance: " << iName << "\n" ;
	solutionsFile << "Number of executions: " << total_exec <<"\n" ;
	solutionsFile << "Number of current execution: " << nexec <<"\n" ;
	solutionsFile << "M: " << m  <<"\n" ;
	solutionsFile << "N: " << n <<"\n" ;
	solutionsFile << "Max Coverage Lines on subset: " << cvl_subset_num << "\n";
	solutionsFile << "Targets Nodes: " << targetsNum << "\n" ;
	solutionsFile << "Execution Time: " <<  elapsed_time<< "\n" ;
	solutionsFile << "Date: " << date << "\n" ;
	solutionsFile << "All Solutions Validated: " << boolalpha << val << "\n";
	solutionsFile  << "Depots" << "|" << "Largest path cost" << "|" <<
			" Total cost" << "\n" ;

	solutionsFile.close();

	string sname;
	uint valpos = 0;
	for(auto s:sol.paretoSet){
		solutionsFile.open(solutions, std::ofstream::out | std::ofstream::app);
		solutionsFile  << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost <<"\n";
		solutionsFile.close();

		solution.clear();
		solutionName.clear();
		solution = createDirSol(nsol);
		solutionName.append(solution);
		solutionName.append("/sol_");
		solutionName.append(to_string(nsol));
		solutionName.append(".txt");

		solutionFile.open(solutionName, std::ofstream::out | std::ofstream::app);
		solutionFile  << "Depots" << "|" << "Largest path cost" << "|" <<
					" Total cost" << "\n" ;
		solutionFile  << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost <<"\n";
		solutionsFile << "Solution Validated: " << boolalpha << validation[valpos] << "\n";
		solutionFile  << "---------------------------------------------------------------------" <<"\n";
		solutionFile  << "Path_Num[Path cost, Robot_ID]" <<"\n";

		nsol++;
		valpos++;
		npath=0;
		for(auto p:s.second.paths){
			path.clear();
			path.append(solution);
			path.append("/path_");
			path.append(to_string(npath));
			//path = createPathDir(solution, npath);
			path.append(".txt");

			pathFile.open(path, std::ofstream::out | std::ofstream::app);
			pathFile <<"Depots "<< "|" << "Largest paths cost" << "|"<< "Robot ID" << "\n";
			pathFile << p.depotsNum <<";" << p.pCost << ";"<< p.robotID << "\n";
			solutionFile << npath <<"[" << p.pCost << ","<< p.robotID <<"];";
			npath++;
			pathFile << "from" << " "<< "to" << " " <<"f"<< " "<<"\n";
			for(auto edge:p.edges){
				pathFile << edge.node_a << ";" << edge.node_b << ";" << edge.cost <<"\n";
			}

			pathFile.close();
		}
		solutionFile.close();
	}

	execPath = instPath;
}

string Output::createPathDir(string sol, int path){
	string sol_dir;
	sol_dir.append(sol);
	sol_dir.append("/path_");
	sol_dir.append(to_string(path));
	if(!fileExists(sol_dir)){
		string command =("mkdir") ;
		command.append(" -p ");
		command.append(sol_dir);
		system(command.c_str());
	}
	return(sol_dir);
}

string Output::createDirSol(int sol){
	string sol_dir;
	sol_dir.append(execPath);
	sol_dir.append("sol_");
	sol_dir.append(to_string(sol));
	if(!fileExists(sol_dir)){
		string command =("mkdir") ;
		command.append(" -p ");
		command.append(sol_dir);
		system(command.c_str());
	}
	return(sol_dir);

}
void Output::createDirInst(string iName){
	string inst_path;
	std::size_t found_point = iName.find(".");

	//find last bar
	std::size_t found_bar = iName.find_last_of("/\\");

	 if (found_point!=std::string::npos && found_bar!=std::string::npos)
	   inst_path = iName.substr(found_bar,found_point);
	 else if(found_point!=std::string::npos)
		 inst_path = iName.substr(0,found_point);
	 else
		 inst_path =iName;
	execPath.append(inst_path);
	execPath.append("/");
	instPath = execPath;

	if(!fileExists(execPath)){
		string command =("mkdir") ;
		command.append(" ");
		command.append(execPath);
		system(command.c_str());
	}
}

void Output::createDirOutput(){
	execPath.append("output/");
	if(!fileExists(execPath)){
		string command =("mkdir") ;
		command.append(" ");
		command.append(execPath);
		system(command.c_str());
	}
}

void Output::createDataDir(string date){
	execPath.append(date);
	execPath.append("/");
	if(!fileExists(execPath)){
		string command =("mkdir") ;
		command.append(" ");
		command.append(execPath);
		system(command.c_str());
	}
}

void Output::writePredictions(Solution sol){

	string predictions;
	ofstream predictionsFile;

	if(!sol.vec_predictions.empty()){

		predictions.append(execPath);
		predictions.append("predictions.txt");
		predictionsFile.open(predictions, std::ofstream::out | std::ofstream::app);

		if(sol.vec_predictions.front().pred_id ==0)
			predictionsFile << "pred_id" << "pred_num" << ";" << "pred_imp_id" << ";" <<
				"best_pred" << ";" << "op" << ";" << "path_op_g1"<< "time_op_g2"<< "path_op_g2" <<  "time_op_g2"<<"\n";


		for(Solution::pred prediction :sol.vec_predictions ){
			predictionsFile << prediction.pred_id<<";"<< prediction.pred_num << ";" << prediction.pred_improv_sol << ";" <<
					prediction.best_pred << ";" << prediction.operation << ";" <<
					prediction.path_op_g1<<";"<< prediction.pred_time_g1 << ";" << prediction.path_op_g2 <<";"<< prediction.pred_time_g2<<"\n";
		}
	}

}

void Output::gurobiCallInfo(Solution sol){

	string gurobi_call;
	ofstream call_File;

	if(!sol.vec_call.empty()){
		gurobi_call.append(execPath);
		gurobi_call.append("gurobi_info.txt");
		call_File.open(gurobi_call, std::ofstream::out | std::ofstream::app);

		//se for o primeiro
		if(sol.vec_call.front().call_id == 1)
			call_File << "call_id" << ";" << "model_type" << ";" <<
			"optimize_time" << ";" << "feasible" << ";" << "target_num"<<";"<<"depot_num"<<"\n";


		for(Solution::gurobi_call grb_call :sol.vec_call ){
			call_File << grb_call.call_id << ";" << grb_call.type << ";" <<
					grb_call.optimize_time << ";" << grb_call.feasible << ";" << grb_call.T << ";" << grb_call.D << "\n";

		}
	}

}


bool Output::fileExists(const std::string& file) {
    struct stat buf;
    return (stat(file.c_str(), &buf) == 0);
}

void Output::createOutput(string date){
	createDataDir(date);
}



