//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_ROUTE_H
#define TSPD_ROUTE_H

#include <iostream>

#include "goc/graph/graph_path.h"
#include "goc/lib/json.hpp"
#include "goc/print/printable.h"
#include "tspd_instance.h"
#include "bit_graph.h"

namespace tspd
{
    
enum class Move {
    Truck,
    Drone,
    Combined
};
std::ostream& operator<<(std::ostream& os, Move m);
    
/*
 * This class implements routes and partial routes.
 * Refer to the paper for details
 */
class Route : public goc::Printable
{
public:
    using Bifurcation = std::pair<size_t, size_t>;

    goc::GraphPath truck;
    std::vector<goc::Vertex> drone;
    std::vector<Bifurcation> bifurcation;  
    Time time = 0;
    
    explicit Route(
        double time = 0, 
        const goc::GraphPath& truck = {}, 
        //const std::vector<goc::GraphPath>& drone = {}, 
        const std::vector<goc::Vertex>& drone = {}, 
        const std::vector<Bifurcation>& bifurcation = {}
    );

    std::tuple<goc::Vertex, goc::Vertex, goc::Vertex> Leg(size_t i) const;
    void Reverse();
    void Merge(const Route& other);
	bool IsElementary() const;
    std::vector<size_t> Descriptor(size_t n) const;
    std::vector<std::pair<goc::Vertex, Move>> Operations() const;
    std::vector<std::pair<size_t, size_t>> Positions() const;
    std::vector<std::pair<goc::Vertex, goc::Vertex>> Locations() const;
    bool NGFeasible(const BitGraph& ng) const;
    double ReducedCost(const DualVector& duals) const;
    Time LowerBound(const DualVector& duals) const;

    virtual void Print(std::ostream& os) const;
};

void to_json(nlohmann::json& j, const Route& r);
void from_json(const nlohmann::json& j, Route& r);

} // namespace tspd

#endif //TSPD_ROUTE_H
