//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include <climits>
#include <cmath>

#include "bp_solver.h"
#include "time_guardian.h"

using namespace std;
using namespace goc;


namespace tspd
{    
    
BPSolver::BPSolver(const Subinstance& instance, const BitGraph& ng)
{
    auto root = make_shared<Solver>(instance, ng);
    q.push({root, false});
    ub = root->BestSolution();
    log.nodes_open++;
}

BPSolverLog BPSolver::Run(
    CGAlgorithm cg_algorithm,
    bool bidirectional,
    DominanceLevel dominance_level
)
{
    Stopwatch rolex_branch;
    SolverPtr node_solver;
    
    try {
        while (!q.empty())
        {

            bool resume;
            tie(node_solver, resume) = q.top();
            q.pop();
            log.lb = lb = min(UpperBound(), node_solver->LowerBound()); // Update z_lb here, because we use Best Bound selection.
            node_solver->UpdateBestSolution(ub);

            if(node_solver->Solved()) {Close(node_solver.get(), true); continue;}
                
            clog << endl << endl << time_guardian.Now() << (resume ? "\tResuming " : "\tProcessing ") << "Node: " << node_solver->Id() << node_solver->Name() << "\tClosed: " << log.nodes_closed << "\tOpen: " << log.nodes_open << endl;
            
            
            auto [rmp_solved, _] = cg_algorithm == CGAlgorithm::Basic ? 
                node_solver->ColGen(q.empty() ? UpperBound() : q.top().first->LowerBound(), resume, bidirectional, dominance_level) : 
                node_solver->ColGenPlus(q.empty() ? UpperBound() : q.top().first->LowerBound(), resume);
    
            ub = node_solver->BestSolution();
                        
            if(node_solver->Solved()) {Close(node_solver.get(), true); continue;}
            
            if(not rmp_solved) {
                q.push({node_solver, true});
                continue;
            }
            Close(node_solver.get(), true);

            if(not solve_only_root) {
                rolex_branch.Resume();
                auto [left_solver, right_solver] = node_solver->Branch();
                q.push({left_solver, false});
                q.push({right_solver, false});
                log.nodes_open += 2;
                rolex_branch.Pause();
            }
        }
    }
    catch(TimeLimitExceeded& tle)
    {
        Close(node_solver.get(), false);
        while(not q.empty()) {
            auto [solver, resume] = q.top();
            if(resume) Close(solver.get(), false);
            q.pop();
        }
        log.branching_time = rolex_branch.Peek();
        log.time = time_guardian.Now();
        log.status = BPSolverStatus::TimeLimitExceeded;
        throw tle;
    }
	
    log.branching_time = rolex_branch.Peek();
    log.time = time_guardian.Now();
    log.lb = UpperBound();
	log.status = BPSolverStatus::Optimum;

	return log;
}


void BPSolver::Close(Solver* node, bool solved) const
{
    node->Close();
    
    if(node->Id() == 0) {
        log.root_lb = node->LowerBound();
        log.root_ub = node->UpperBound();
        log.root_time = time_guardian.Now();
        log.root_exact_labelings = node->Log().root_exact_labelings;
        log.root_total_labelings = node->Log().root_total_labelings;
    }    

    log.ub = min(log.ub, node->UpperBound());
	log.lp_time += node->Log().lp_time; 
	log.pricing_time += node->Log().pricing_time;

    if(log_level == BPLogLevel::All or (log_level == BPLogLevel::Root and node->Id() == 0)) {
        log.nodes.resize(max(log.nodes.size(), node->Id()+1));
        log.nodes[node->Id()] = node->Log();
    }    
    if(solved) {
        log.nodes_closed++;
        log.nodes_open--;
    }
}

std::ostream& operator<<(std::ostream& os, CGAlgorithm cg_algorithm) {
    switch(cg_algorithm) {
        case CGAlgorithm::Basic:
            return os << "Basic (iterative pricing)";
        default:
            return os << "Plus (iterative pricing + variable fixing)";
    }
}

} // namespace tspd

