//
// Grupo de Optimizacion & Logistica (OptLog)
// Departamento de Computacion - Universidad de Buenos Aires.
// Created by Francisco Soulignac
//

#ifndef TSPD_LABELINGTSP_H
#define TSPD_LABELINGTSP_H

#include "goc/goc.h"
#include "tspd_instance.h"
#include "subinstance.h"
#include "route.h"
#include "log/labeling_log.h"

namespace tspd
{
/*
 * Parameters to control which of the dominance rules are applied throught the labeling algorithm
 */
enum class DominanceLevel 
{
    Fork,    /**< all the rules are applied to try to discard a label */
    NoFork  /**< no rule on which p forkes is applied */
};
std::ostream& operator<<(std::ostream& os, DominanceLevel dominance_level);

    
/*
 * The labeling algorithm traverses a label tree whose vertices correspond to the ng-feasible partial routes.
 * For each partial route p, its parent is the partial route obtained by removing the last operation of p.
 * Instead of storing the whole partial route p at each node of the labeling tree, the labeling algorithm
 * stores enough information that allows the reconstruction of p knowing who its parent is.  Also,
 * some additional information can be stored for efficiency reasons or to implement dominance rules.  The
 * actual information that is stored for each partial route p is called the \em label of p, hence the
 * term labeling in labeling algorithms.
 * 
 * Please, refer to the paper for motivations about the different members of this class.
 */
class Label : public goc::Printable
{
public:
    size_t length = 0;
    size_t bifurcation = 0;
    goc::Vertex last = 0;
    goc::Vertex truck = 0; 
    goc::Vertex drone= 0; 
    double cost = 0; // Cost of path p(L), including duals of the truck.
    Time truck_time = 0; 
    VertexSet forbidden_truck;
    mutable VertexSet extensions_lbl = VertexSet().set(); 
    VertexSet extensions = VertexSet().set(); 
    mutable std::vector<goc::Vertex> the_extensions_lbl;
    const Label* prev = nullptr; 
    
    Label() = default;
    virtual ~Label() = default;
    double CostToTruck() const;
    bool TruckCarriesDrone() const;
    Label Associated() const;

    virtual void Print(std::ostream& os) const;
};

using LabelPtr = std::shared_ptr<const Label>;    

/*
 * This class represents different labeling algorithms that can be applied to solve a pricing problem.
 * Refer to the paper for a description of the different algorithms available.
 */
class Labeling {
public:

    // deprecated but never removed
    using ExtensionCallback = std::function<void(Direction d, const Label* l, const Label* m)>;
    
    Labeling(
        const std::string& description,
        const Subinstance* instance,
        const BitGraph* ng,
        const DualVector& duals,
        DominanceLevel dominance_level,
        bool relax_ng = false,
        bool relax_cost = false
    );

    void Run(size_t ext);
    void Run(size_t ext, double bound);
    void MRun(Direction d, size_t ext);
    void MRun(Direction d, size_t ext, double bound);
    std::vector<Route> Merge(size_t ext, double bound, size_t limit);
    std::vector<Route> Merge(double bound, size_t limit);
    void SetBoundingLabels(Direction d, const std::vector<LabelPtr>* labels, bool associated);
    void SetExtensionCallback(const ExtensionCallback& cb);

    Route RouteOf(Direction d, const Label* l) const;
    const std::vector<LabelPtr>* Undominated(Direction d) const;
    const DualVector& Duals() const;

    const std::vector<LabelingLog>& Log() const;
    const LabelingLog& Log(size_t i) const;
    void NewLogPage(const std::string& description);
private:
    
    const Subinstance* instance;
    const BitGraph* ng = nullptr;
    std::vector<double> duals;
    DominanceLevel dominance_level = DominanceLevel::Fork;
    bool relax_cost = false;
    bool relax_ng = false;
    std::shared_ptr<ExtensionCallback> at_each_extension = nullptr;

    std::vector<LabelPtr> labels[2];
    std::vector<const Label*> non_extended[2];
    std::vector<goc::Vertex> V_by_duals;
    std::vector<VertexSet> dual_bigger;
    size_t length[2];
    mutable std::vector<LabelingLog> log;
    std::vector<double> sorted_duals;
    std::vector<VertexSet> bigger_dual;
    
    using BoundingBucketTruck = std::vector<std::vector<const Label*>>;
    using BoundingBucket = std::vector<BoundingBucketTruck>;
    std::vector<BoundingBucket> bounding_buckets[2];
    bool use_associated;

    struct LBLBucket {
        LabelPtr carry;
        std::vector<LabelPtr> alone;
        std::vector<LabelPtr> routes;
        double carry_cost = std::numeric_limits<double>::infinity();
    };
    using LBLForbStorage = std::unordered_map<VertexSet, LBLBucket>;
    using LBLStorage = std::vector<LBLForbStorage>;

    using RangeDominators = std::vector<std::tuple<Time, Time, double>>;
    using SameDominators = std::vector<RangeDominators>;
    struct ForkDominators {
        std::vector<std::tuple<double, bool, goc::Vertex>> no_dual;
        double dual = std::numeric_limits<double>::infinity();
        
        std::vector<double> no_dual_values;
        std::vector<VertexSet> no_dual_survive;
    };
    
    mutable goc::Stopwatch rolex_dominance;
    mutable goc::Stopwatch rolex_extension;
    mutable goc::Stopwatch rolex_bounding;
    
    Route ReversedRouteOf(Direction d, const Label* l) const;
    std::vector<LabelPtr> ExtendUndominated(Direction d, double bound) const;
    LBLStorage ExtendUndominatedCarry(Direction d, double bound) const;
    LBLForbStorage ExtendCarryToBuckets(Direction d, goc::Vertex w, double bound) const;
    void ExtendAloneToBuckets(Direction d, LBLStorage* extensions, goc::Vertex w, double bound) const;
    void ExtendAloneToUnsortedBuckets(Direction d, LBLStorage* extensions, goc::Vertex w, double bound) const;
    std::vector<LabelPtr> ExtensionsCarry(Direction d, const Label* l, goc::Vertex w, double bound = goc::INFTY) const;   
    LabelPtr ExtensionAlone(Direction d, const Label* l, goc::Vertex w, double bound = goc::INFTY) const;
    size_t RemoveSortedDominated(Direction d, std::vector<LabelPtr>* ms) const;
    void RemoveSelfDominated(Direction d, const Label* m) const;
    void RemoveDualDominated(const Label* m, const ForkDominators& dominators, double min_dual) const;
    void RemoveForkDominated(const Label* m, const ForkDominators& dominators) const;
    void RemoveSameDominated(Direction d, const Label* m, const SameDominators& dominators) const;
    void UpdateForkDominators(Direction d, ForkDominators* dominators, const Label* m) const;
    void UpdateSameDominators(Direction d, SameDominators* dominators, const Label* m) const;
    bool Bounded(Direction d, const Label* m, double bound) const;

    using LBLMerge = std::tuple<const Label*, const Label*, goc::Vertex>;

    void DoMerge(
        Direction d,
        const Label& l,
        const BoundingBucket& B,
        double bound,
        size_t limit,
        size_t gen_limit,
        std::multimap<double, LBLMerge>* output
    ) const;
    void DoMerge(
        Direction d,
        const Label* l,
        const BoundingBucket& B,
        double bound,
        size_t limit,
        size_t gen_limit,
        std::multimap<double, LBLMerge>* output
    ) const;
    
    void InsertToRangeDominators(RangeDominators* dominators, const std::tuple<Time, Time, double>& dom) const;
    std::vector<std::pair<VertexSet, LBLBucket*>> SortBySize(LBLForbStorage* buckets) const;
    void UpdateExtensionData(const Label* l) const;
    std::vector<goc::Vertex> MergeExtensions(const Label* l, double min_dual = std::numeric_limits<double>::infinity()) const;
    double ForkCost(Direction d, const Label* l, goc::Vertex v) const;
    VertexSet BucketKey(const Label* l) const;

};  // class Labeling

} // namespace tspd

#endif //TSPD_LABELING_H_NEW
