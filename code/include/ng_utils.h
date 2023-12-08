//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

/*
 * This file contains the functions to define the initial set of neighborhoods and to implement the neighborhood
 * augmentation algorithm described in the paper.  Some additional functionality is provided, that was used
 * in preliminary tests of the algorithm
 */

#ifndef TSPD_NG_UTILS_H
#define TSPD_NG_UTILS_H

#include "subinstance.h"
#include "bit_graph.h"
#include "route.h"

namespace tspd {

using RouteCycle = std::pair<goc::Vertex, goc::GraphPath>; 
BitGraph NearbyNG(const Subinstance& tsp, size_t ng_size);
BitGraph ElementaryNG(const Subinstance& tsp);
std::vector<RouteCycle> Cycles(const Route& route, size_t max_size);
bool AugmentNG(BitGraph* NG, const RouteCycle& cycle, const std::vector<size_t>& max_ng_of);
bool AugmentNG(BitGraph* NG, const Route& route, const std::vector<size_t>& max_ng_of);
bool AugmentNG(BitGraph* NG, const std::vector<Route>& route, const std::vector<size_t>& max_ng_of);

} // namespace tspd

#endif //TSPD_NG_UTILS_H
