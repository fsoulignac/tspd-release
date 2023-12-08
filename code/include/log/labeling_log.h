//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef LOG_LABELING_LOG_H
#define LOG_LABELING_LOG_H

#include <iostream>
#include <string>
#include <vector>

#include "goc/goc.h"

namespace tspd
{

/*
 * The log records different statistics about the execution of a bidirectional labeling algorithm.
 * This log is compatible with the kaleidoscope type tspd_labeling.
 */
struct LabelingLog : public goc::Log
{
    struct Monodirectional : public goc::Log
    {
        size_t next_length = 0;
        goc::Duration time;
        size_t enumerated_count = 0;
        size_t r_dominated_count = 0;
        size_t b_dominated_count = 0;
        size_t processed_count = 0;
        size_t bounded_count = 0;
        std::vector<size_t> count_by_length;
        goc::Duration enumeration_time;
        goc::Duration r_domination_time;
        goc::Duration b_domination_time;
        goc::Duration bounding_time;

        Monodirectional() = default;
        
        virtual nlohmann::json ToJSON() const;
    };
    
	goc::Duration time;
    std::string description;
	Monodirectional monolog[2];
	goc::Duration merge_time;
    double pricing_value = 0;

    LabelingLog() = default;

	virtual nlohmann::json ToJSON() const;
};

} // namespace tspd

#endif //LOG_LABELING_LOG_H
