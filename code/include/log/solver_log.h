//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef LOG_SOLVER_LOG_H
#define LOG_SOLVER_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/goc.h"
#include "solver_problem_log.h"

namespace tspd
{
    
enum class SolverStatus {
	DidNotStart,
    TimeLimitExceeded,
    Optimum
};    
    
/*
 * The log records different statistics about the execution of the price-fix-and-cut algorithm.
 * This log is compatible with the kaleidoscope type tspd_solver.
 */
struct SolverLog : public goc::Log
{
	goc::Duration time;
	SolverStatus status = SolverStatus::DidNotStart;
    std::string description;
    double initial_lb = 0;
    double initial_ub = goc::INFTY;
    goc::Duration initial_time;
    double root_lb = 0;
    double root_ub = goc::INFTY;
    double root_price_lb = 0;
    LabelingLog root_last_lbl;
    goc::Duration root_time;
    size_t root_exact_labelings = 0;
    size_t root_total_labelings = 0;
	double lb = 0;
	double ub = goc::INFTY;
    size_t DNA_iters = 0;
	goc::Duration DNA_time;
    goc::Duration iterative_fixing_time;
	goc::Duration lp_time;
	goc::Duration pricing_time;
	goc::Duration variable_fixing_time;
    SolverProblemLog main;
	std::vector<SolverProblemLog> subproblems;

	SolverLog() = default;
	virtual nlohmann::json ToJSON() const;
};

std::ostream& operator<<(std::ostream& os, SolverStatus status);

} // namespace tspd

#endif //LOG_SOLVER_LOG_H
