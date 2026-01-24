#include <fstream>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
#include <cstdlib>
#include <filesystem>  // C++17 cross-platform filesystem support

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
    std::string nodesFile = nodesPath + "/nodes.txt";

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

void Output::writeSolution(Solution sol, int it, std::string op, Solution::alog vars) {
    std::string fileName = solutionPath + "/solution_" + std::to_string(it) + "_" + op;

    if(op != "init" && op != "best") {
        fileName += "_G" + std::to_string(vars.g1) + "_L" + std::to_string(vars.l1) + ":G" + std::to_string(vars.g2);
        if(vars.l2 != -1) {
            fileName += "_L" + std::to_string(vars.l2);
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

void Output::writeOutput(double maxCost, int iter, std::string op) {
    std::string outputPath = execPath + "/output.txt";
    std::ofstream outputFile(outputPath, std::ios::out | std::ios::app);
    outputFile << iter << " " << maxCost << " " << op << "\n";
    outputFile.close();
}

void Output::writeSolutions(Solution sol, std::string op) {
    std::string solutionsPath = execPath + "/solutions.txt";
    std::ofstream solutionsFile(solutionsPath, std::ios::out | std::ios::app);

    solutionsFile << op << " ";
    for(const auto& path : sol.currentSol.paths) {
        solutionsFile << path.pCost << " ";
    }
    solutionsFile << "\n";
    solutionsFile.close();
}

void Output::writeParetoSet(const Solution& sol, std::string pname, std::string iName, std::string date,
                            double elapsed_time, int targetsNum, std::vector<bool> validation,
                            int nexec, int total_exec, int m, int n, int cvl_subset_num) {
    int nsol = 1;

    std::string solutions = execPath + "solutions.txt";
    std::ofstream solutionsFile(solutions, std::ios::out | std::ios::app);

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
        solutionsFile.open(solutions, std::ios::out | std::ios::app);
        solutionsFile << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost << "\n";
        solutionsFile.close();

        std::string solution = createDirSol(nsol);
        std::string solutionName = solution + "/sol_" + std::to_string(nsol) + ".txt";

        std::ofstream solutionFile(solutionName, std::ios::out | std::ios::app);
        solutionFile << "Depots|Largest path cost| Total cost\n"
                     << s.second.depotsNum << ";" << s.second.maxCost << ";" << s.second.sCost << "\n";

        solutionsFile.open(solutions, std::ios::out | std::ios::app);
        solutionsFile << "Solution Validated: " << boolalpha << validation[valpos] << "\n";
        solutionsFile.close();

        solutionFile << "---------------------------------------------------------------------\n"
                     << "Path_Num[Path cost, Robot_ID]\n";

        int npath = 0;
        for(auto& p : s.second.paths) {
            std::string path = solution + "/path_" + std::to_string(npath) + ".txt";
            std::ofstream pathFile(path, std::ios::out | std::ios::app);

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

std::string Output::createPathDir(std::string sol, int path) {
    std::string sol_dir = sol + "/path_" + std::to_string(path);
    if(!fs::exists(sol_dir)) {
        fs::create_directories(sol_dir);
    }
    return sol_dir;
}

std::string Output::createDirSol(int sol) {
    std::string sol_dir = execPath + "sol_" + std::to_string(sol);
    if(!fs::exists(sol_dir)) {
        fs::create_directories(sol_dir);
    }
    return sol_dir;
}

void Output::createDirInst(std::string iName) {
    size_t found_point = iName.find(".");
    size_t found_bar = iName.find_last_of("/\\");

    std::string inst_path;
    if(found_point != std::string::npos && found_bar != std::string::npos)
        inst_path = iName.substr(found_bar + 1, found_point - found_bar - 1);
    else if(found_point != std::string::npos)
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

void Output::createDataDir(std::string date) {
    execPath += date + "/";
    if(!fs::exists(execPath)) {
        fs::create_directories(execPath);
    }
}

void Output::writePredictions(const Solution& sol) {
    if(!sol.vec_predictions.empty()) {
        std::string predictions = execPath + "predictions.txt";
        std::ofstream predictionsFile(predictions, std::ios::out | std::ios::app);

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

void Output::gurobiCallInfo(const Solution& sol) {
    if(!sol.vec_call.empty()) {
        std::string gurobi_call = execPath + "gurobi_info.txt";
        std::ofstream call_File(gurobi_call, std::ios::out | std::ios::app);

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

bool Output::fileExists(const std::string& filename) {
    return fs::exists(filename);
}

void Output::createOutput(std::string date){
	createDataDir(date);
}
