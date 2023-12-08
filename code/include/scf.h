//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_SPF_H
#define TSPD_SPF_H

#include <vector>
#include <memory>

#include "goc/goc.h"

#include "tspd_instance.h"
#include "subinstance.h"
#include "route.h"

namespace tspd
{
/**
 * Represents a set-covering formulation (SCF) for the TSP-D and a corresponding restricted master
 * problem (RMP) for a column-generation algorithm.
 * Refer to the paper for details
 *
 * Warning: this class assumes the starting depot is 0 and the ending depot is n-1.
 */
class SCF
{
public:

    SCF(size_t count, const Route& r0);

    const goc::Formulation* RMPFormulation() const;
    std::unique_ptr<SCF> Restriction(const Subinstance& instance, const BitGraph& ng) const;
	void AddRoute(const Route& r);
	void SetSubproblem(const Subinstance& instance, const BitGraph& ng);
	const Route& RouteOf(const goc::Variable& variable) const;
	std::vector<Route> InterpretSolution(const goc::Valuation& z) const;
    
private:
	std::unique_ptr<goc::Formulation> formulation;
    size_t n;
	std::vector<Route> rmp_routes;
	std::vector<goc::Variable> y;
};
} // namespace tspd

#endif //TSPD_SPF_H
