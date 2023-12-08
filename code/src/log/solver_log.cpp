//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "log/solver_log.h"

using namespace std;
using namespace nlohmann;
using namespace goc;

namespace tspd
{

json SolverLog::ToJSON() const
{
	json j;
	j["kd_type"] = "tspd_solver";
	j["time"] = time;
	j["status"] = STR(status);
    j["description"] = description;
	j["initial_lb"] = initial_lb;
	j["initial_ub"] = initial_ub;
	j["initial_time"] = initial_time;
	j["root_lb"] = root_lb;
	j["root_ub"] = root_ub;
	j["root_price_lb"] = root_price_lb;
	j["root_time"] = root_time;
	j["root_last_lbl"] = root_last_lbl;
    j["root_exact_labelings"] = root_exact_labelings;
    j["root_total_labelings"] = root_total_labelings;
	j["lb"] = lb;
	j["ub"] = ub;
	j["dna_iters"] = DNA_iters;
	j["dna_time"] = DNA_time;
	j["lp_time"] = lp_time;
	j["pricing_time"] = pricing_time;
	j["fix_time"] = variable_fixing_time;
    j["iterative_fix_time"] = iterative_fixing_time;
    auto problems = subproblems;
    if(epsilon_bigger(main.time.Amount(DurationUnit::Milliseconds), 0)) problems.insert(problems.begin(), main);
	j["subproblems"] = problems;
	return j;
}

ostream& operator<<(ostream& os, SolverStatus status)
{
	unordered_map<SolverStatus, string> mapper = {{SolverStatus::DidNotStart, "DidNotStart"},
											    {SolverStatus::TimeLimitExceeded, "TimeLimit"},
											    {SolverStatus::Optimum, "Optimum"}};
	return os << mapper[status];
}

} // namespace tspd
