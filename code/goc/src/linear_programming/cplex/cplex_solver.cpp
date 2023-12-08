//
// Created by Gonzalo Lera Romero.
// Grupo de Optimizacion Combinatoria (GOC).
// Departamento de Computacion - Universidad de Buenos Aires.
//

#include <mutex>

#include "goc/collection/collection_utils.h"
#include "goc/string/string_utils.h"
#include "goc/exception/exception_utils.h"
#include "goc/math/number_utils.h"
#include "goc/time/stopwatch.h"
#include "goc/linear_programming/cuts/separation_algorithm.h"
#include "goc/linear_programming/cplex/cplex_solver.h"
#include "goc/linear_programming/cplex/cplex_wrapper.h"
#include "goc/linear_programming/cplex/cplex_formulation.h"

using namespace std;
using namespace nlohmann;

namespace goc
{
namespace cplex
{
namespace
{
// Function that gets called by cplex when a message is sent to the log.
void tunnel_message(void* handle, const char* message)
{
	auto output_streams = (vector<ostream*>*) handle;
	for (ostream* os: *output_streams) *os << message;
}

// This function adds an observer to the CPLEX logs and sends everything to the streams.
void tunnel_cplex_logs(CplexFormulation* formulation, const vector<ostream*>& streams)
{
	if (streams.empty()) return;
	CPXCHANNELptr result, warning, error, log;
	cplex::getchannels(formulation->Environment(), &result, &warning, &error, &log);
	cplex::addfuncdest(formulation->Environment(), result, (void*) &streams, tunnel_message);
	cplex::addfuncdest(formulation->Environment(), warning, (void*) &streams, tunnel_message);
	cplex::addfuncdest(formulation->Environment(), log, (void*) &streams, tunnel_message);
}

// Removes observers from the CPLEX channels.
void untunnel_cplex_logs(CplexFormulation* formulation, const vector<ostream*>& streams)
{
	if (streams.empty()) return;
	CPXCHANNELptr result, warning, error, log;
	cplex::getchannels(formulation->Environment(), &result, &warning, &error, &log);
	cplex::delfuncdest(formulation->Environment(), result, (void*) &streams, tunnel_message);
	cplex::delfuncdest(formulation->Environment(), warning, (void*) &streams, tunnel_message);
	cplex::delfuncdest(formulation->Environment(), log, (void*) &streams, tunnel_message);
}

// Applies the configuration parameters specified in the config json to CPLEX environment.
void apply_configuration(CplexFormulation* formulation, const json& config)
{
	// Reset parameters of the formulation to the default values.
	cplex::setdefaults(formulation->Environment());
	
	// Add new parameters.
	for (auto it_param = config.begin(); it_param != config.end(); ++it_param)
	{
		string param_name = it_param.key();
		int whichparam, paramtype;
		cplex::getparamnum(formulation->Environment(), param_name.c_str(), &whichparam);
		cplex::getparamtype(formulation->Environment(), whichparam, &paramtype);
		
		if (paramtype == CPX_PARAMTYPE_INT)
			cplex::setintparam(formulation->Environment(), whichparam, it_param.value());
		else if (paramtype == CPX_PARAMTYPE_LONG)
			cplex::setlongparam(formulation->Environment(), whichparam, it_param.value());
		else if (paramtype == CPX_PARAMTYPE_DOUBLE)
			cplex::setdblparam(formulation->Environment(), whichparam, it_param.value());
		else if (paramtype == CPX_PARAMTYPE_STRING)
			cplex::setstrparam(formulation->Environment(), whichparam, STR(it_param.value()).c_str());
		else fail("Unrecognized CPLEX parameter: " + param_name);
	}
}

// Adds the initial solutions provided to the formulation as MIPstarts.
void add_initial_solutions(CplexFormulation* formulation, const vector<Valuation>& initial_solutions)
{
	// Remove previous MIP starts.
	int num_mip_starts = cplex::getnummipstarts(formulation->Environment(), formulation->Problem());
	if (num_mip_starts > 0)
		cplex::delmipstarts(formulation->Environment(), formulation->Problem(), 0, num_mip_starts - 1);
	
	if (!initial_solutions.empty())
	{
		// Add new MIP starts.
		int nzcnt = 0;
		int beg[] = {0};
		vector<int> varindices;
		vector<double> values;
		
		for (auto& solution : initial_solutions)
		{
			for (const Variable& variable: formulation->Variables())
			{
				if (epsilon_different(solution[variable], 0.0))
				{
					nzcnt++;
					varindices.push_back(variable.Index());
					values.push_back(solution[variable]);
				}
			}
			cplex::addmipstarts(formulation->Environment(), formulation->Problem(), 1, nzcnt, beg, &varindices[0],
								&values[0], nullptr, nullptr);
		}
	}
}

// Sets the specified branch priorities to the CPLEX formulation.
void set_branch_priorities(CplexFormulation* formulation, const vector<BranchPriority>& branch_priorities)
{
	for (auto& branch_priority: branch_priorities)
	{
		map<BranchPriority::BranchDirection, int> mapper_to_cplex = {{BranchPriority::Up,   CPX_BRANCH_UP},
																	 {BranchPriority::Down, CPX_BRANCH_DOWN},
																	 {BranchPriority::Any,  CPX_BRANCH_GLOBAL}};
		int indices[] = {branch_priority.variable.Index()};
		int priorities[] = {branch_priority.priority};
		int orders[]{mapper_to_cplex[branch_priority.direction]};
		cplex::copyorder(formulation->Environment(), formulation->Problem(), 1, indices, priorities, orders);
	}
}

// *** Extract information functions *** //

// Extract information about the execution of CPLEX after solving with lpopt and add it to the execution log if necessary.
// Information being handled:
//	* SimplexIterations, Status, IncumbentValue, Incumbent, Duals
void extract_cplex_lp_execution_info(CplexFormulation* formulation, LPExecutionLog* execution_log,
	const unordered_set<LPOption>& options)
{
	CPXENVptr env = formulation->Environment();
	CPXLPptr prob = formulation->Problem();
	
	// Simplex iterations.
	execution_log->simplex_iterations = cplex::getitcnt(env, prob);
	
	// Variable count.
	execution_log->variable_count = cplex::getnumcols(env, prob);
	
	// Constraint count.
	execution_log->constraint_count = cplex::getnumrows(env, prob);
	
	// Status.
	int cplex_status = cplex::getstat(env, prob);
	map<int, LPStatus> cplex_status_mapper = {{CPX_STAT_INForUNBD,               LPStatus::Infeasible},
											  {CPX_STAT_INFEASIBLE,              LPStatus::Infeasible},
											  {CPX_STAT_UNBOUNDED,               LPStatus::Unbounded},
											  {CPX_STAT_CONFLICT_ABORT_TIME_LIM, LPStatus::TimeLimitReached},
											  {CPX_STAT_ABORT_TIME_LIM,          LPStatus::TimeLimitReached},
											  {CPX_STAT_CONFLICT_ABORT_MEM_LIM,  LPStatus::MemoryLimitReached},
											  {CPX_STAT_OPTIMAL_INFEAS,          LPStatus::Optimum},
											  {CPX_STAT_OPTIMAL,                 LPStatus::Optimum}};
	execution_log->status = cplex_status_mapper[cplex_status];
	
	// Incumbent and Incumbent value.
	double objval;
	if (cplex::getobjval(env, prob, &objval)) // If incumbent was found
	{
		// Incumbent value.
		execution_log->incumbent_value = objval;
		// Incumbent.
		if (includes(options, LPOption::Incumbent))
		{
			vector<double> values = vector<double>(formulation->VariableCount(), 0.0);
			cplex::solution(env, prob, nullptr, nullptr, &(values[0]), nullptr, nullptr, nullptr);
			Valuation incumbent;
			for (int i = 0; i < formulation->VariableCount(); ++i)
				incumbent.SetValue(formulation->VariableAtIndex(i), values[i]);
			execution_log->incumbent = incumbent;
		}
	}
	
	// Duals.
	if (includes(options, LPOption::Duals))
	{
		vector<double> duals(formulation->ConstraintCount(), 0.0);
		cplex::getpi(env, prob, &(duals[0]), 0, formulation->ConstraintCount() - 1);
		execution_log->duals = duals;
	}
}
// *** End extract information functions *** //

// *** Callback functions *** //
// The internal structure of CPLEX to store a constraint.
struct CplexRow
{
	int nzcnt, nzind;
	double rhs;
	char sense;
	vector<int> rmatbeg, rmatind;
	vector<double> rmatval;
};

// Returns: a CPLEX row representing the constraint.
// Precondition: constraint must be normalized.
CplexRow constraint_to_cplex_row(const Constraint& constraint)
{
	CplexRow row;
	
	map<enum Constraint::Sense, char> cplex_senses = {{Constraint::LessEqual,    'L'},
													  {Constraint::GreaterEqual, 'G'},
													  {Constraint::Equality,     'E'}};
	row.nzcnt = constraint.LeftSide().NonZeroVariableTermCount();
	row.rhs = constraint.RightSide();
	row.sense = cplex_senses[constraint.Sense()];
	row.rmatbeg = {0};
	row.nzind = 0;
	for (auto& term: constraint.LeftSide().Terms())
	{
		row.rmatind.push_back(term.first.Index());
		row.rmatval.push_back(term.second);
		row.nzind++;
	}
	return row;
}

std::mutex lazy_constraint_lock;

int cplex_generic_callback(CPXCALLBACKCONTEXTptr context, CPXLONG contextid, void* userhandle)
{
	// Parse user handle infromation.
	const SeparationAlgorithm* separation_algorithm;
	CplexFormulation* formulation;
	BCExecutionLog* execution_log;
	tie(separation_algorithm, formulation,
		execution_log) = *(tuple<const SeparationAlgorithm*, CplexFormulation*, BCExecutionLog*>*) userhandle;
	
	// Vertex relaxation solved. Cuts may be introduced here.
	if (contextid == CPX_CALLBACKCONTEXT_RELAXATION)
	{
		if (!separation_algorithm->IsEnabled()) return 0;
		
		// Get nodes solved.
		int nodes_solved;
		CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODECOUNT, &nodes_solved);
		
		// Get relaxation point.
		double objective_value;
		vector<double> relaxation_values(formulation->VariableCount());
		cplex::callbackgetrelaxationpoint(context, &(relaxation_values[0]), 0, formulation->VariableCount() - 1,
										  &objective_value);
		Valuation relaxation_point;
		for (int i = 0; i < formulation->VariableCount(); ++i)
			relaxation_point.SetValue(formulation->VariableAtIndex(i), relaxation_values[i]);
		
		// Cut relaxation point.
		for (auto& cut: separation_algorithm->Separate(relaxation_point, nodes_solved, objective_value))
		{
			CplexRow row = constraint_to_cplex_row(cut);
			int purgeable = CPX_USECUT_FORCE;
			int local = 0;
			cplex::callbackaddusercuts(context, 1, row.nzcnt, &row.rhs, &row.sense, &(row.rmatbeg[0]),
									   &(row.rmatind[0]), &(row.rmatval[0]), &purgeable, &local);
		}
	}
	// Integer solution found. Lazy constraints may be introduced here.
	else if (contextid == CPX_CALLBACKCONTEXT_CANDIDATE)
	{
		// Check that candidate solution is a feasible point and not a ray.
		int is_point;
		cplex::callbackcandidateispoint(context, &is_point);
		if (is_point == 0) return 0;
		
		// Get candidate solution.
		vector<double> candidate_values(formulation->VariableCount());
		double node_bound;
		cplex::callbackgetcandidatepoint(context, &(candidate_values[0]), 0, formulation->VariableCount() - 1, &node_bound);
		Valuation candidate_point;
		for (int i = 0; i < formulation->VariableCount(); ++i)
			candidate_point.SetValue(formulation->VariableAtIndex(i), candidate_values[i]);
		
		// Get nodes solved.
		int nodes_solved;
		CPXcallbackgetinfoint(context, CPXCALLBACKINFO_NODECOUNT, &nodes_solved);
		
		// Add lazy constraints that cut the candidate point.
		for (const SeparationRoutine* lazy: formulation->LazyConstraints())
		{
			lazy_constraint_lock.lock();
			auto violated_constraints = lazy->Separate(candidate_point, nodes_solved, INT_MAX, node_bound);
			lazy_constraint_lock.unlock();
			for (const Constraint& violated_constraint: violated_constraints)
			{
				CplexRow row = constraint_to_cplex_row(violated_constraint);
				cplex::callbackrejectcandidate(context, 1, row.nzcnt, &row.rhs, &row.sense, &(row.rmatbeg[0]),
											   &(row.rmatind[0]), &(row.rmatval[0]));
			}
		}
	}
	else if (contextid == CPX_CALLBACKCONTEXT_GLOBAL_PROGRESS)
	{
		// If we are still solving the root node.
		int node_count;
		cplex::callbackgetinfoint(context, CPXCALLBACKINFO_NODECOUNT, &node_count);
		if (node_count == 0)
		{
			double best_bound;
			cplex::callbackgetinfodbl(context, CPXCALLBACKINFO_BEST_BND, &best_bound);
			execution_log->root_lp_value = best_bound;
			
			// Check if an int solution was found.
			// Cplex Doc.: If there is no incumbent solution, then a very large value will be returned in this buffer.
			double best_int_value;
			cplex::callbackgetincumbent(context, nullptr, 0, -1, &best_int_value);
			if (best_int_value < 10e8)
			{
				execution_log->root_int_value = best_int_value;
			}
		}
	}
	
	return 0;
}
// *** End Callback functions *** //
}

LPExecutionLog solve_lp(CplexFormulation* formulation, ostream* screen_output, Duration time_limit, const json& config,
			   const unordered_set<LPOption>& options)
{
	LPExecutionLog execution_log;
	
	// Tunnel CPLEX logs to the screen and to the log stream if requested.
	vector<ostream*> output_streams;
	stringstream log_stream;
	if (includes(options, LPOption::ScreenOutput)) output_streams.push_back(&log_stream);
	if (screen_output) output_streams.push_back(screen_output);
	tunnel_cplex_logs(formulation, output_streams);
	
	// Apply configurations.
	apply_configuration(formulation, config);
	
	// Apply time limit.
	cplex::setdblparam(formulation->Environment(), CPX_PARAM_TILIM, time_limit.Amount(DurationUnit::Seconds));
    cplex::setintparam(formulation->Environment(), CPX_PARAM_LPMETHOD, CPX_ALG_DUAL);
    cplex::setintparam(formulation->Environment(), CPX_PARAM_THREADS, 1);    
	
	// Change problem to LP, but before save the domain of variables, because changing the problem type to LP erases
	// them.
	vector<VariableDomain> variable_domains(formulation->VariableCount());
	for (int i = 0; i < formulation->VariableCount(); ++i)
		variable_domains[i] = formulation->GetVariableDomain(formulation->VariableAtIndex(i));
	cplex::chgprobtype(formulation->Environment(), formulation->Problem(), CPXPROB_LP);
	
	// Optimize.
	Stopwatch rolex(true);
	cplex::lpopt(formulation->Environment(), formulation->Problem());
	rolex.Pause();
	
	// Remove CPLEX log tunneling from the formulation.
	untunnel_cplex_logs(formulation, output_streams);
	
	// Extract results.
	execution_log.time = rolex.Peek();
	if (includes(options, LPOption::ScreenOutput)) execution_log.screen_output = log_stream.str();
	extract_cplex_lp_execution_info(formulation, &execution_log, options);
	
	// Return the variable domain to all variables.
	for (int i = 0; i < formulation->VariableCount(); ++i)
		formulation->SetVariableDomain(formulation->VariableAtIndex(i), variable_domains[i]);
	
	return execution_log;
}

} // namespace cplex
} // namespace goc
