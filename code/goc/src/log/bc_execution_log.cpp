//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include "goc/log/bc_execution_log.h"

#include "goc/collection/collection_utils.h"
#include "goc/string/string_utils.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
BCExecutionLog::BCExecutionLog()
{
	screen_output = "";
	time = 0.0_sec;
	status = BCStatus::DidNotStart;
	nodes_open = nodes_closed = 0;
	root_lp_value = root_int_value = best_bound = best_int_value = 0.0;
    cut_time = 0.0_sec;
}

json BCExecutionLog::ToJSON() const
{
	json j;
	j["kd_type"] = "bc";
	j["screen_output"] = screen_output;
	j["time"] = time.Amount(DurationUnit::Seconds);
	j["status"] = STR(status);
	j["nodes_open"] = nodes_open;
	j["nodes_closed"] = nodes_closed;
	j["root_lp_value"] = root_lp_value;
	j["root_int_value"] = root_int_value;
	j["best_bound"] = best_bound;
	j["best_int_value"] = best_int_value;
	j["cut_time"] = cut_time;
	return j;
}

ostream& operator<<(ostream& os, BCStatus status)
{
	unordered_map<BCStatus, string> mapper = {{BCStatus::DidNotStart, "DidNotStart"},
											   {BCStatus::Infeasible, "Infeasible"},
											   {BCStatus::Unbounded, "Unbounded"},
											   {BCStatus::TimeLimitReached, "TimeLimitReached"},
											   {BCStatus::MemoryLimitReached, "MemoryLimitReached"},
											   {BCStatus::Optimum, "Optimum"},
											   {BCStatus::NodeLimitReached, "NodeLimitReached"}};
	return os << mapper[status];
}
} // namespace goc