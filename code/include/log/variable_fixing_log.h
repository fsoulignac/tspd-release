//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef LOG_VARIABLE_FIXING_LOG_H
#define LOG_VARIABLE_FIXING_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/goc.h"
#include "types.h"

namespace tspd
{

/*
 * The log records different statistics about the execution of the two-pass variable fixing method
 * This log is compatible with the kaleidoscope type tspd_variable_fixing.
 */
struct VariableFixingLog : public goc::Log
{    
    size_t total_arcs;
    DualVector duals;
    goc::Duration lbl_pre_time;
    std::vector<double> gap;
	std::vector<goc::Duration> time;
    std::vector<size_t> arcs;
    std::vector<goc::Duration> lbl_post_time;
    
    VariableFixingLog() = default;
	virtual nlohmann::json ToJSON() const;
};

} // namespace tspd

#endif //LOG_VARIABLE_FIXING_LOG_H
