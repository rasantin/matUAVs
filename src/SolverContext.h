#pragma once
#include "gurobi_c++.h"

class SolverContext {
public:
    SolverContext(); // declaração
    SolverContext(const SolverContext&) = delete;
    SolverContext& operator=(const SolverContext&) = delete;

    GRBEnv& env();

private:
    GRBEnv env_;
};