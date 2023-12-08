//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_SOLVER_H
#define TSPD_SOLVER_H

#include <memory>
#include <vector>
#include <array>

#include "goc/goc.h"
#include "tspd_instance.h"
#include "subinstance.h"
#include "scf.h"
#include "labeling.h"
#include "log/solver_log.h"
#include "log/solver_problem_log.h"
#include "log/variable_fixing_log.h"

namespace tspd
{

/*
 * Options for logging information about the algorithm.
 */ 
enum class LogLevel {
    None, /**< No logging at all */
    Main, /**< Log only the main problem */
    All /**< Log everything */
};

/**
 * To control how each pricing problem is solved within the column generation algorithm
 * Refer to the paper for a description of the heuristics
 */
enum class ColGenState {
    RelaxAll,
    RelaxNG,
    RelaxCost,
    Exact,
    Finished
};
constexpr size_t idx(ColGenState state) {
    return static_cast<size_t>(state);
}
constexpr size_t ColGenStatesCount() {
    return idx(ColGenState::Finished)+1;
}
constexpr ColGenState Next(ColGenState state) {
    return static_cast<ColGenState>(std::min(idx(state) + 1, ColGenStatesCount()));
}
std::ostream& operator<<(std::ostream& os, ColGenState state);

class Solver;
using SolverPtr = std::shared_ptr<Solver>;

/*
 * This class represents the price-fix-and-augment solver, but it also has some functionality to help in the
 * design of a branch-and-price algorithm that builds on top of the column geration algorithm
 * 
 * Refer to the paper for a description of the algorithm
 */
class Solver
{
public:

    LogLevel log_level = LogLevel::None;

    Solver(const Subinstance& instance, const BitGraph& ng);

    SolverLog Run();
    std::pair<bool, DualVector> ColGen(
        Time objective, 
        bool resume,
        bool bidirectional,
        DominanceLevel dominance_level
    );
    std::pair<bool, DualVector> ColGenPlus(
        Time objective,
        bool resume,
        double factor = 0.005
    );
    std::pair<SolverPtr, SolverPtr> Branch();
    void Close();

    const Route& BestSolution() const;
    double LowerBound() const;
    double UpperBound() const;
    bool Solved() const;
    void UpdateBestSolution(const Route& r);
    size_t Id() const;
    std::string Name() const;
    const SolverLog& Log() const;

    VariableFixingLog VariableFixingExperiment(const std::vector<double>& gaps, double time_limit_factor);
private:

    inline static size_t next_id{1};
    inline static BitGraph elemental_ng;
    
    size_t id = 0;
    std::string name = "";
	goc::LPSolver lp_solver;
    std::shared_ptr<goc::TableStream> tstream;
    mutable std::shared_ptr<SolverLog> log;
    std::shared_ptr<SCF> scf;
    BitGraph ng;

    struct Subproblem {
        Subproblem(const Subinstance& instance);
        
        Subproblem* parent = nullptr;
        Subinstance instance;
        double lb = 0;
        Route ub;
        double prev_fixing_gap = goc::INFTY;
        std::shared_ptr<SolverProblemLog> log;
    };
    
    Subproblem main;
    
    void RebuildSPF() {scf = std::move(scf->Restriction(main.instance, ng));}
    Subproblem* NewSubproblem(Subproblem* parent, double lb, double ub);
	std::tuple<std::unique_ptr<Labeling>, double, bool> ColGen(
        Subproblem* problem, 
        bool resume = false,
        Time objective = goc::INFTY, 
        double rmp_objective = 1.1,
        bool fixing = true,
        bool bidirectional = true,
        DominanceLevel dominance_level = DominanceLevel::Fork
    );
	std::pair<std::unique_ptr<Labeling>, double> ColGenPlus(
        Subproblem* problem, 
        double factor, 
        bool resume = false,
        Time objective = goc::INFTY
    );

    void NeighborhoodAugmentation(Subproblem* problem, Labeling* last_pricer);
    std::pair<double, DualVector> SolveRMP(Subproblem* problem);

    std::tuple<std::unique_ptr<Labeling>, std::vector<Route>, double> SolvePricing(
        const std::string& description,
        Subproblem* problem, 
        const DualVector& duals, 
        ColGenState state = ColGenState::Exact,
        double fail_ub = std::numeric_limits<double>::infinity(),
        bool bidirectional = true, 
        DominanceLevel dominance_level = DominanceLevel::Fork,
        size_t route_limit = 1,
        double bound = std::numeric_limits<double>::infinity()
    );

    double UpdateBounds(
        Subproblem* problem, 
        double fail_ub, 
        const DualVector& duals, 
        const std::vector<Route>& routes, 
        ColGenState state
    );

    bool AddNegativeRoutesToSPF(const std::vector<Route>& routes, const DualVector& duals);
    void UpdateBestSolution(Subproblem* problem, Route r);
    void UpdateBestSolution(Subproblem* problem, const std::vector<Route>& routes);
    bool DoSeparate(BitGraph* ng, const std::vector<double>& duals, const std::vector<Route>& routes);

    goc::Duration ArcFixing(
        Subproblem* problem, 
        Labeling* lbl, 
        double price_bound, 
        const goc::Duration& max_price_time = goc::Duration(), 
        const goc::Duration& min_price_time = goc::Duration(), 
        bool iterative=false
    );

    std::unique_ptr<Labeling> RunVariableFixing(
        Subproblem* problem,
        Direction d,
        Labeling* lbl, 
        const BitGraph& ng,
        double bound,
        bool use_associated
    );
    
    void FindSolutionIfOptimumRMPBound(Labeling* lbl, double rmp_bound);
    
    std::pair<SolverPtr, SolverPtr> BranchVehicle(const goc::Valuation& opt) const;
    std::pair<SolverPtr, SolverPtr> BranchTruckArcs(const goc::Valuation& opt) const;
    std::pair<SolverPtr, SolverPtr> BranchDroneArcs(const goc::Valuation& opt) const;

    void StreamInfo(
        Subproblem* problem, 
        const std::string& process, 
        const goc::Duration& time, 
        double pricing_bound, 
        const std::string& rmp_bound = "----"
    ) const;

    inline bool Solved(const Subproblem& problem) const {
        return goc::epsilon_bigger_equal(problem.lb, problem.ub.time);
    }

    void Close(const Subproblem& problem);
        
};
} // namespace tspd


#endif //TSPD_SOLVER_H
