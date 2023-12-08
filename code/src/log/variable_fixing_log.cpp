//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "log/variable_fixing_log.h"

using namespace std;
using namespace goc;
using namespace nlohmann;

namespace tspd
{

json VariableFixingLog::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_variable_fixing"; // ID of the log type.
	j["duals"] = duals;
	j["total_arcs"] = total_arcs;
	j["lbl_pre_time"] = lbl_pre_time;
	j["lbl_post_time"] = lbl_post_time;
	j["gap"] = gap;
	j["time"] = time;
	j["arcs"] = arcs;
    
	return j;
}

} // namespace tspd
