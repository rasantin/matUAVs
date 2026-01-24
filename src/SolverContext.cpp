#include "SolverContext.h"

SolverContext::SolverContext() : env_(true) {
    env_.set(GRB_IntParam_OutputFlag, 0);
    env_.set(GRB_IntParam_Threads, 1);
    env_.start();
}

GRBEnv& SolverContext::env() {
    return env_;
}