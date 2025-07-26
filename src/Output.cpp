#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <sys/stat.h>
#include <cstdlib>
#include <filesystem>  // C++17+

#include "Output.h"

using namespace std;
namespace fs = std::filesystem;

Output::Output() {
    // Construtor padrão
}

Output::Output(Input input) {
    this->input = input;
    createDirOutput();
    createDirInst(input.GetFileName());
}

Output::~Output() {
    // Destrutor
}

void Output::writeNodes() {
    ofstream file;
    string nodesFile = nodesPath + "/nodes.txt";

    file.open(nodesFile);
    if(file.is_open() && !input.nodes.empty()) {
        file << "node_id x y nodeType\n";
        for(uint ind = 0; ind < input.nodes.size(); ind++) {
            file << input.nodes[ind].getNodeId() << " "
                 << input.nodes[ind].getX() << " "
                 << input.nodes[ind].getY() << " "
                 << input.nodes[ind].getNodeType() << "\n";
        }
    }
    file.close();
}

void Output::createDir(int it) {
    if(!execPath.empty()) {
        execPath.clear();
    }

    execPath = "exec_" + to_string(it);

    // Cria diretório principal
    if(!fs::exists(execPath)) {
        fs::create_directory(execPath);
    }

    // Cria output/nodes dentro do diretório execPath
    nodesPath = execPath + "/output/nodes";
    if(!fs::exists(nodesPath)) {
        fs::create_directories(nodesPath);  // cria toda a hierarquia
    }

    // Cria output/solutions dentro do diretório execPath
    solutionPath = execPath + "/output/solutions";
    if(!fs::exists(solutionPath)) {
        fs::create_directories(solutionPath);
    }
}

void Output::writeSolution(Solution sol, int it, string op, Solution::alog vars) {
    string fileName = solutionPath + "/solution_" + to_string(it) + "_" + op;

    if(op != "init" && op != "best") {
        fileName += "_G" + to_string(vars.g1) + "_L" + to_string(vars.l1) + ":G" + to_string(vars.g2);
        if(vars.l2 != -1) {
            fileName += "_L" + to_string(vars.l2);
        }
    }
    fileName += ".txt";

    ofstream file(fileName);
    if(file.is_open()) {
        file << "from to f objval Group\n";

        for(uint k = 0; k < sol.currentSol.paths.size(); k++) {
            for(uint i = 0; i < sol.currentSol.paths[k].nodes.size(); i++) {
                auto& edge = sol.currentSol.paths[k].nodes[i];
                uint f = edge.first.first;
                uint s = edge.first.second;

                file << f << " " << s << " "
                     << edge.second.first << "  "
                     << sol.currentSol.paths[k].pCost << " "
                     << k << "\n";
            }
        }
    }
    file.close();
}

void Output::writeOutput(double maxCost, int iter, string op) {
    string outputPath = execPath + "/output.txt";
    ofstream outputFile(outputPath, ios::out | ios::app);
    outputFile << iter << " " << maxCost << " " << op << "\n";
    outputFile.close();
}

void Output::writeSolutions(Solution sol, string op) {
    string solutionsPath = execPath + "/solutions.txt";
    ofstream solutionsFile(solutionsPath, ios::out | ios::app);

    solutionsFile << op << " ";
    for(const auto& path : sol.currentSol.paths) {
        solutionsFile << path.pCost << " ";
    }
    solutionsFile << "\n";
    solutionsFile.close();
}

void Output::writeParetoSet(Solution sol, string pname, string iName, string date,
                            double elapsed_time, int targetsNum, vector<bool> validation,
                            int nexec, int total_exec, int m, int n, int cvl_subset_num) {
    int nsol = 1;

    string solutions = execPath + "solutions.txt";
    ofstream solutionsFile(solutions, ios::out | ios::app);

    bool val = all_of(validation.begin(), validation.end(), [](bool v) { return v; });

    solutionsFile << "Algorithm: " << pname << "\n"
                  << "Instance: " << iName << "\n"
                  << "Number of executions: " << total_exec << "\n"
                  << "Number of current execution: " << nexec << "\n"
                  << "M: " << m << "\n"
                  << "N: " << n << "\n"
                  << "Max Coverage Lines on subset: " << cvl_subset_num << "\n"
                  << "Targets Nodes: " << targetsNum << "\n"
                  << "Execution Time: " << elapsed_time << "\n"
                  << "Date: " << date << "\n"
                  << "All Solutions Validated: " << boolalpha << val << "\n"
                  << "Depots|Largest path cost| Total cost\n";
    solutionsFile.close();

    uint valpos = 0;
    for(auto& s : sol.paretoSet) {
        solutionsFile.open(solutions, ios::out | ios::app);
        solutionsFile << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost << "\n";
        solutionsFile.close();

        string solution = createDirSol(nsol);
        string solutionName = solution + "/sol_" + to_string(nsol) + ".txt";

        ofstream solutionFile(solutionName, ios::out | ios::app);
        solutionFile << "Depots|Largest path cost| Total cost\n"
                     << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost << "\n";

        solutionsFile.open(solutions, ios::out | ios::app);
        solutionsFile << "Solution Validated: " << boolalpha << validation[valpos] << "\n";
        solutionsFile.close();

        solutionFile << "---------------------------------------------------------------------\n"
                     << "Path_Num[Path cost, Robot_ID]\n";

        int npath = 0;
        for(auto& p : s.second.paths) {
            string path = solution + "/path_" + to_string(npath) + ".txt";
            ofstream pathFile(path, ios::out | ios::app);

            pathFile << "Depots|Largest paths cost|Robot ID\n"
                     << p.depotsNum << ";" << p.pCost << ";" << p.robotID << "\n";

            solutionFile << npath << "[" << p.pCost << "," << p.robotID << "];";
            npath++;

            pathFile << "from to f\n";
            for(auto& edge : p.edges) {
                pathFile << edge.node_a << ";" << edge.node_b << ";" << edge.cost << "\n";
            }
            pathFile.close();
        }
        solutionFile.close();
        nsol++;
        valpos++;
    }

    execPath = instPath;
}

string Output::createPathDir(string sol, int path) {
    string sol_dir = sol + "/path_" + to_string(path);
    if(!fs::exists(sol_dir)) {
        fs::create_directories(sol_dir);
    }
    return sol_dir;
}

string Output::createDirSol(int sol) {
    string sol_dir = execPath + "sol_" + to_string(sol);
    if(!fs::exists(sol_dir)) {
        fs::create_directories(sol_dir);
    }
    return sol_dir;
}

void Output::createDirInst(string iName) {
    size_t found_point = iName.find(".");
    size_t found_bar = iName.find_last_of("/\\");

    string inst_path;
    if(found_point != string::npos && found_bar != string::npos)
        inst_path = iName.substr(found_bar + 1, found_point - found_bar - 1);
    else if(found_point != string::npos)
        inst_path = iName.substr(0, found_point);
    else
        inst_path = iName;

    execPath += inst_path + "/";
    instPath = execPath;

    if(!fs::exists(execPath)) {
        fs::create_directories(execPath);
    }
}

void Output::createDirOutput() {
    execPath += "output/";
    if(!fs::exists(execPath)) {
        fs::create_directories(execPath);
    }
}

void Output::createDataDir(string date) {
    execPath += date + "/";
    if(!fs::exists(execPath)) {
        fs::create_directories(execPath);
    }
}

void Output::writePredictions(Solution sol) {
    if(!sol.vec_predictions.empty()) {
        string predictions = execPath + "predictions.txt";
        ofstream predictionsFile(predictions, ios::out | ios::app);

        if(sol.vec_predictions.front().pred_id == 0) {
            predictionsFile << "pred_idpred_num;pred_imp_id;best_pred;op;path_op_g1time_op_g2path_op_g2time_op_g2\n";
        }

        for(auto& prediction : sol.vec_predictions) {
            predictionsFile << prediction.pred_id << ";" << prediction.pred_num << ";" << prediction.pred_improv_sol << ";"
                            << prediction.best_pred << ";" << prediction.operation << ";"
                            << prediction.path_op_g1 << ";" << prediction.pred_time_g1 << ";"
                            << prediction.path_op_g2 << ";" << prediction.pred_time_g2 << "\n";
        }
        predictionsFile.close();
    }
}

void Output::gurobiCallInfo(Solution sol) {
    if(!sol.vec_call.empty()) {
        string gurobi_call = execPath + "gurobi_info.txt";
        ofstream call_File(gurobi_call, ios::out | ios::app);

        if(sol.vec_call.front().call_id == 1) {
            call_File << "call_id;model_type;optimize_time;feasible;target_num;depot_num\n";
        }

        for(auto& grb_call : sol.vec_call) {
            call_File << grb_call.call_id << ";" << grb_call.type << ";"
                      << grb_call.optimize_time << ";" << grb_call.feasible << ";"
                      << grb_call.T << ";" << grb_call.D << "\n";
        }
        call_File.close();
    }
}

bool Output::fileExists(const string& filename) {
    return fs::exists(filename);
}

void Output::createOutput(string date){
	createDataDir(date);
}
