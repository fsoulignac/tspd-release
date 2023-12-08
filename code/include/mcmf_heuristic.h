//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_MCMF_HEURISTIC_H
#define TSPD_MCMF_HEURISTIC_H

#include "route.h"
#include "subinstance.h"

/*
 * A MinCost-MaxFlow Heuristic that finds an elementary route when a non-elementary
 * route is given
 */

namespace tspd {
    
Route MinCostMaxFlowHeuristic(const Subinstance& instance, const Route& route);
    
}  // namespace tspd

#endif //TSPD_MCMF_HEURISTIC_H
