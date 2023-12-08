class TSPDBPSolverParser extends Parser
{
  gap(lb, ub)
  {
    if (lb == undefined || ub == undefined) return undefined;
    return 100.0*(Math.abs(lb-ub)/Math.abs(lb+10e-6));
  }

  get_attributes(obj)
  {
    return [
      new Attribute("Time", this.f2(obj.time)),
      new Attribute("Status", obj.status),
      new Attribute("#Nodes open", obj.nodes_open),
      new Attribute("#Nodes closed", obj.nodes_closed),
      new Attribute("Root LB", this.f2(obj.root_lb)),
      new Attribute("Root UB", this.f2(obj.root_ub)),
      new Attribute("%Root Gap", this.f2(this.gap(obj.root_lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("Root Time", this.f2(obj.root_time)),
      new Attribute("Root Exact Labelings", obj.root_exact_labelings),
      new Attribute("Root Total Labelings", obj.root_total_labelings),
      new Attribute("Final LB", this.f2(obj.lb)),
      new Attribute("Final UB", this.f2(obj.ub)),
      new Attribute("%Final Gap", this.f2(this.gap(obj.lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("LP time", this.f2(obj.lp_time)),
      new Attribute("Pricing time", this.f2(obj.pricing_time)),
      new Attribute("Branching time", this.f2(obj.branching_time))
    ];
  }

  detail_view_rows(obj, view_section) {

    view_section.add_table_row(
      ["Time", "Status", "#Nodes open", "#Nodes closed"],
      [obj.time, obj.status, obj.nodes_open, obj.nodes_closed],
    );

    view_section.add_table_row(
      ["Root Time", "Root LB", "Root UB", "%Root Gap"],
      [obj.root_time, obj.root_lb, obj.root_ub, this.gap(obj.root_lb, obj.root_ub)],
    );

    view_section.add_table_row(
      ["Final LB", "Final UB", "%Final Gap"],
      [obj.lb, obj.ub, this.gap(obj.lb, obj.ub)],
    );

    view_section.add_table_row(
      ["LP time", "Pricing time", "Branching time"],
      [obj.lp_time, obj.pricing_time, obj.branching_time]
    );
    
    if(obj.nodes.length > 0)
    {
      view_section.add_label_row("Nodes",  " ");
      view_section.add_slider_row(
        obj.nodes,
        (index, node, node_section) => {
          const description = node.description || ", root";
          node_section.add_html_row(`<span class="slider_header">Node #${index}${description}</span>`);
          kd.get_parser(node.subproblems[0].kd_type)?.detail_view_rows(node.subproblems[0], node_section);
        }
      );
    }  
  }
}
kd.add_parser("tspd_bp_solver", new TSPDBPSolverParser());
