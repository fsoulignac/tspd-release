//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef LOG_SOLVER_PROBLEM_LOG_H
#define LOG_SOLVER_PROBLEM_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/goc.h"
#include "labeling_log.h"

namespace tspd
{
/*
 * The log records different statistics about the execution of one of the subproblems within the
 * price-fix-and-cut algorithm.  For the main problem, some of the pricing iterations are interleaved
 * with fictitious problems.  This log is compatible with the kaleidoscope type tspd_solver_problem.
 */
struct SolverProblemLog : public goc::Log
{
	goc::Duration time;
	double lb = 0;
	double ub = goc::INFTY;
    double start_ub = goc::INFTY;
    size_t start_ng = 0;
    size_t final_ng = 0;
	goc::Duration DNA_time;
    goc::Duration iterative_fixing_time;
	goc::Duration lp_time;
	goc::Duration pricing_time;
	goc::Duration variable_fixing_time;
	std::vector<LabelingLog> labelings;
    size_t exact_labelings = 0;
    size_t exact_labelings_plus = 0;

	SolverProblemLog() = default;
	virtual nlohmann::json ToJSON() const;
};

} // namespace tspd

#endif //LOG_SOLVER_PROBLEM_LOG_H
