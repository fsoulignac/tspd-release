//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef LOG_BP_SOLVER_LOG_H
#define LOG_BP_SOLVER_LOG_H

#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "goc/goc.h"
#include "solver_log.h"

namespace tspd
{
    
enum class BPSolverStatus {
	DidNotStart,
    TimeLimitExceeded,
    Optimum
};    
    
/*
 * The log records different statistics about the execution of the branch-and-price algorithm.
 * This log is compatible with the kaleidoscope type tspd_bp_solver.
 */
struct BPSolverLog : public goc::Log
{
	goc::Duration time;
	BPSolverStatus status = BPSolverStatus::DidNotStart;
	size_t nodes_open = 0;
	size_t nodes_closed = 0;
	double lb = 0;
	double ub = goc::INFTY;
	double root_lb = 0;
	double root_ub = goc::INFTY;
    size_t root_exact_labelings = 0;
    size_t root_total_labelings = 0;
    goc::Duration root_time;
	goc::Duration lp_time;
	goc::Duration pricing_time;
	goc::Duration branching_time;
    std::vector<SolverLog> nodes;
    
	BPSolverLog() = default;
	
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, BPSolverStatus status);

} // namespace tspd

#endif //LOG_BP_SOLVER_LOG_H
