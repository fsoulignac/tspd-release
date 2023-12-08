//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_BP_H
#define TSPD_BP_H

#include <memory>
#include <vector>

#include "goc/goc.h"
#include "solver.h"
#include "log/bp_solver_log.h"

namespace tspd
{

/**
 * Controls the amount of information logged for the BPSolver
 */
enum class BPLogLevel {
    None, /**< don't log information */
    Root, /**< log only the information for the root node */
    All /**< log everything (huge files are generated) */
};
    
/**
 * Controls the CG algorithm to use at each step
 */
enum class CGAlgorithm {
    Basic, /**< iterative pricing only */
    Plus /**< pricing plus variable fixing and bounding */
};
std::ostream& operator<<(std::ostream& os, CGAlgorithm cg_algorithm);


/*
 * The idea of the branch-and-price solver is to make it easier to evaluate the developed tools in a known
 * setting.  This branch-and-price algorithm follows the ideas described by Roberti and Ruthmair without
 * adding many bells and whistles.  For instance, we may use the neighbhorhood augmentation to implement
 * a robust separation method, but we avoid such a path to keep things simple.  Also, we avoid fancy techniques
 * such as strong branching.  However, these techniques can be incorporated without much effort.
 */
class BPSolver
{
public:

    BPLogLevel log_level = BPLogLevel::None;
    bool solve_only_root = false;

    BPSolver(const Subinstance& instance, const BitGraph& ng);

    BPSolverLog Run(CGAlgorithm cg_algorithm, bool bidirectional, DominanceLevel dominance_level);
    inline const Route& BestSolution() const {return ub;}
    inline double LowerBound() const {return lb;}
    inline double UpperBound() const {return ub.time;}
    inline const BPSolverLog& Log() const {return log;}
    
private:
    
    using Node = std::pair<SolverPtr, bool>;
    struct NodeComparator { 
        inline bool operator() (const Node& n1, const Node& n2) 
        { 
            return goc::epsilon_bigger(n1.first->LowerBound(), n2.first->LowerBound()); 
        } 
    };
    
    void Close(Solver* node, bool solved) const;
	std::priority_queue<Node, std::vector<Node>, NodeComparator> q;
	double lb = 0;
	Route ub;
    mutable BPSolverLog log;
};
} // namespace tspd


#endif //TSPD_BP_H
