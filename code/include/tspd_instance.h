//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_TSPD_INSTANCE_H
#define TSPD_TSPD_INSTANCE_H

#include <vector>
#include <goc/goc.h>
#include "types.h"

namespace tspd
{
/*
 * The input instance of the TSP-D with no preprocessing
 */
class TSPDInput : public goc::Printable
{
    friend void from_json(const nlohmann::json& j, TSPDInput& instance);

public:
    size_t vertex_count;
	goc::Vertex origin; 
    goc::Vertex destination;    
	goc::Matrix<Time> truck_dist; 
	goc::Matrix<Time> drone_dist; 

    virtual void Print(std::ostream& os) const;

};
    
void to_json(nlohmann::json& j, const TSPDInput& input);
void from_json(const nlohmann::json& j, TSPDInput& input);
void from_poi(std::istream& is, TSPDInput& input); 

    
/*
 * The input instance of the TSP-D with minor pre-processing
 */
class TSPDInstance
{    
    goc::Matrix<std::vector<Time>> drone_arcs_by_time_change;  
    goc::Matrix<std::vector<VertexSet>> drone_arcs_by_time_sets;  
    goc::Matrix<std::vector<Time>> legs_by_time_change;  
    goc::Matrix<std::vector<VertexSet>> legs_by_time_sets;  
    
    void UpdatePrecomputedTimes();
public:
    TSPDInstance() = default;
    TSPDInstance(const TSPDInput& input, double drone_speed);

    goc::Digraph network;
    goc::Vertex origin;
    goc::Vertex destination;    
	goc::Matrix<Time> truck_time; 
	goc::Matrix<Time> fork_time;  
    goc::Matrix<Time> join_time;  // _ij = time to join back with the truck from i to j.
    VertexSet vertex_set;
    double drone_speed;

	TSPDInstance Reverse() const;
    VertexSet FartherDroneArcs(goc::Vertex v, goc::Vertex w, Time time) const;
    VertexSet FartherDroneLegs(goc::Vertex d, goc::Vertex t, Time time) const;

};

} // namespace tspd
#endif //TSPD_TSPD_INSTANCE_H
