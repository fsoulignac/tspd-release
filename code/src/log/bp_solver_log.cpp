//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "log/bp_solver_log.h"

using namespace std;
using namespace nlohmann;
using namespace goc;

namespace tspd
{

json BPSolverLog::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_bp_solver";
	j["time"] = time;
	j["status"] = STR(status);
	j["nodes_open"] = nodes_open;
	j["nodes_closed"] = nodes_closed;
	j["lb"] = lb;
	j["ub"] = ub;
	j["root_lb"] = root_lb;
	j["root_ub"] = root_ub;
	j["root_time"] = root_time;
	j["root_exact_labelings"] = root_exact_labelings;
	j["root_total_labelings"] = root_total_labelings;
	j["lp_time"] = lp_time;
	j["pricing_time"] = pricing_time;
	j["branching_time"] = branching_time;
	j["nodes"] = nodes;

	return j;
}

ostream& operator<<(ostream& os, BPSolverStatus status)
{
	unordered_map<BPSolverStatus, string> mapper = {{BPSolverStatus::DidNotStart, "DidNotStart"},
											        {BPSolverStatus::TimeLimitExceeded, "TimeLimitExceeded"},
											        {BPSolverStatus::Optimum, "Optimum"}};
	return os << mapper[status];
}

} // namespace tspd
