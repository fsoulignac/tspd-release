//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "log/labeling_log.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace tspd
{

json LabelingLog::Monodirectional::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_mono_labeling";
	j["time"] = time;
	j["next_length"] = next_length;
	j["enumerated_count"] = enumerated_count;
	j["r_dominated_count"] = r_dominated_count;
	j["b_dominated_count"] = b_dominated_count;
	j["processed_count"] = processed_count;
	j["bounded_count"] = bounded_count;
	j["count_by_length"] = count_by_length;
	j["enumeration_time"] = enumeration_time;
	j["r_domination_time"] = r_domination_time;
	j["b_domination_time"] = b_domination_time;
	j["bounding_time"] = bounding_time;
	return j;
}

json LabelingLog::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_labeling";
	j["time"] = time;
    j["description"] = description;
	j["forward"] = monolog[0];
	j["backward"] = monolog[1];
	j["merge_time"] = merge_time;
    j["lb"] = pricing_value;
	
	return j;
}

} // namespace tspd
