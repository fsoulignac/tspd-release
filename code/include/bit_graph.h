//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_BIT_GRAPH_H
#define TSPD_BIT_GRAPH_H

#include <vector>
#include <goc/goc.h>
#include "types.h"

namespace tspd {

/*
 * This class implements digraphs (with loops) using an adjacency matrix
 * whose rows are implemented with bitsets.
 */
class BitGraph : public goc::Printable {
    size_t size = 0;
    std::vector<VertexSet> arcs;
    friend void to_json(nlohmann::json&, const BitGraph&);
    friend void from_json(const nlohmann::json&, BitGraph&);
    
public:  
    BitGraph() = default;
    explicit BitGraph(size_t n);
    BitGraph(size_t n, const VertexSet& s);

    BitGraph Reverse() const;
    void AddArc(goc::Vertex v, goc::Vertex w);
    void RemoveArc(goc::Vertex v, goc::Vertex w);
    bool HasArc(goc::Vertex v, goc::Vertex w) const;
    const VertexSet& Neighborhood(goc::Vertex v) const;
    size_t VertexCount() const;
    size_t ArcCount() const;
    virtual void Print(std::ostream& os) const;
};

void to_json(nlohmann::json& j, const BitGraph& G);
void from_json(const nlohmann::json& j, BitGraph& G);

/*
 * This class represents a special kind of digraph that has k copies of a same vertex set {0,...,n-1} such that
 * every arc (v,w) is from a vertex v in an i-th copy to a vertex w in an (i+1)-th copy, for some 1 <= i < k.
 */
class LenBitGraph : public goc::Printable {
    size_t size = 0;
    friend void to_json(nlohmann::json&, const LenBitGraph&);
    friend void from_json(const nlohmann::json&, LenBitGraph&);
    std::vector<BitGraph> graph;
    
public:    
    LenBitGraph() = default;
    LenBitGraph(size_t k, size_t n);
    LenBitGraph(size_t k, const BitGraph& g);

    void AddArc(size_t l, goc::Vertex v, goc::Vertex w);
    void RemoveArc(size_t l, goc::Vertex v, goc::Vertex w);
    bool HasArc(size_t l, goc::Vertex v, goc::Vertex w) const;
    const VertexSet& Neighborhood(size_t l, goc::Vertex v) const;
    size_t Copies() const;
    size_t ArcCount() const;
	virtual void Print(std::ostream& os) const;
};

void to_json(nlohmann::json& j, const LenBitGraph& G);
void from_json(const nlohmann::json& j, LenBitGraph& G);

}  // namespace tspd

#endif //TSPD_SUBINSTANCE
