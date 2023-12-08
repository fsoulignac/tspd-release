//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#include "scf.h"

using namespace std;
using namespace goc;

namespace tspd
{
SCF::SCF(size_t count, const Route& r0) : n(count)
{
	formulation.reset(LPSolver::NewFormulation());
	formulation->Minimize(Expression());
	formulation->AddConstraint(Expression().LEQ(1.0));
	for (auto i = 1ul; i < n-1; ++i) formulation->AddConstraint(Expression().GEQ(1.0));
	formulation->AddConstraint(Expression().GEQ(0.0));

    AddRoute(r0);
}

const Formulation* SCF::RMPFormulation() const 
{
    return formulation.get();
}

unique_ptr<SCF> SCF::Restriction(const Subinstance& instance, const BitGraph& ng) const
{
    //always keep the trivial route, that can be updated in rmp_routes[1] if it is not forbidden
    auto res = make_unique<SCF>(n, rmp_routes[0]);
    
    for(auto r = 1ul; r < rmp_routes.size(); ++r) 
    if(instance.HasRoute(Direction::Forward, rmp_routes[r]) and rmp_routes[r].NGFeasible(ng))
        res->AddRoute(rmp_routes[r]);

    return res;        
}

void SCF::AddRoute(const Route& r)
{
    if(r.IsElementary() and not rmp_routes.empty()) {
        return;
    }
    
	auto j = rmp_routes.size();
	rmp_routes.push_back(r);

	Variable y_j = formulation->AddVariable("y_" + STR(j), VariableDomain::Binary, 0.0, INFTY);
	y.push_back(y_j);
    
    formulation->SetConstraintCoefficient(0, y_j, 1.0);

	auto a = r.Descriptor(n);
	for (auto v = 0ul; v < n; ++v)
		formulation->SetConstraintCoefficient(static_cast<int>(v), y_j, static_cast<double>(a[v]));

	formulation->SetObjectiveCoefficient(y_j, r.time);
    
}

void SCF::SetSubproblem(const Subinstance& instance, const BitGraph& ng)
{    
    for(auto r = 1ul; r < y.size(); ++r) 
        formulation->SetVariableBound(y[r], 0.0, INFTY);

    for(auto r = 1ul; r < rmp_routes.size(); ++r) 
    if(not instance.HasRoute(Direction::Forward, rmp_routes[r]) or not rmp_routes[r].NGFeasible(ng)) {
        formulation->SetVariableBound(y[r], 0.0, 0.0); 
    }
}

const Route& SCF::RouteOf(const Variable& variable) const
{
	return rmp_routes[variable.Index()];
}
    
vector<Route> SCF::InterpretSolution(const Valuation& z) const
{
	vector<Route> solution;
	for (auto& y_value: z) solution.push_back(rmp_routes[y_value.first.Index()]);
	return solution;
}
} // namespace tspd
