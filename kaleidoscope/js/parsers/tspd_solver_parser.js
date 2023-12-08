class TSPDSolverParser extends Parser
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
      new Attribute("Initial LB", this.f2(obj.initial_lb)),
      new Attribute("Initial UB", this.f2(obj.initial_ub)),
      new Attribute("%Initial Gap", this.f2(this.gap(obj.initial_lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("Initial Time", this.f2(obj.initial_time)),
      new Attribute("Root LB", this.f2(obj.root_lb)),
      new Attribute("Root UB", this.f2(obj.root_ub)),
      new Attribute("Root Price LB", this.f2(obj.root_price_lb)),
      new Attribute("%Root Gap", this.f2(this.gap(obj.root_lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("%Root Price Gap", this.f2(this.gap(obj.root_price_lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("Root Time", this.f2(obj.root_time)),
      new Attribute("Root Price Time", this.f2(obj.root_last_lbl.time)),
      new Attribute("Root Exact Labelings", obj.root_exact_labelings),
      new Attribute("Root Total Labelings", obj.root_total_labelings),
      new Attribute("Final LB", this.f2(obj.lb)),
      new Attribute("Final UB", this.f2(obj.ub)),
      new Attribute("%Final Gap", this.f2(this.gap(obj.lb, obj.ub)), "", "100.0*|LB-UB|/|LB|"),
      new Attribute("CGs iters", obj.dna_iters+1),
      new Attribute("DNA iters", obj.dna_iters),
      new Attribute("LP time", this.f2(obj.lp_time)),
      new Attribute("Pricing time", this.f2(obj.pricing_time)),
      new Attribute("Fixing time", this.f2(obj.fix_time)),
      new Attribute("DNA time", this.f2(obj.dna_time)),
      new Attribute("Iterative Fixing time", this.f2(obj.iterative_fix_time)),
      new Attribute("%LP time", this.f2(100*obj.lp_time/obj.time)),
      new Attribute("%Pricing time", this.f2(100*obj.pricing_time/obj.time)),
      new Attribute("%Fixing time", this.f2(100*obj.fix_time/obj.time)),
      new Attribute("%CG time", this.f2(100*(obj.lp_time + obj.pricing_time + obj.fix_time)/obj.time)),
      new Attribute("%DNA time", this.f2(100*obj.dna_time/obj.time)),
      new Attribute("%Iterative Fixing time", this.f2(100*obj.iterative_fix_time/obj.time))
    ];
  }

  detail_view_rows(obj, view_section) {

    view_section.add_table_row(
      ["Time", "Status", "Final LB", "Final UB", "%Final Gap", "DNAs"],
      [obj.time, obj.status, obj.lb, obj.ub, this.gap(obj.lb, obj.ub), obj.dna_iters]
    );

    view_section.add_table_row(
      ["Initial LB", "Initial UB", "%Initial Gap", "Initial Time"],
      [obj.initial_lb, obj.initial_ub, this.gap(obj.initial_lb, obj.initial_ub), obj.initial_time]
    );

    view_section.add_table_row(
      ["Root Price LB", "Root LB", "Root UB", "%Root Gap", "Root Time"],
      [obj.root_price_lb, obj.root_lb, obj.root_ub, this.gap(obj.root_lb, obj.root_ub), obj.root_time]
    );
    
    view_section.add_table_row(
      ["LP time", "Pricing time", "Fixing time", "DNA time", "Iterative Fixing Time"],
      [obj.lp_time, obj.pricing_time, obj.fix_time, obj.dna_time, obj.iterative_fix_time]
    );
        
    if(obj.subproblems.length > 0) {
        view_section.add_label_row("Subproblems", " ");
        view_section.add_slider_row(
          obj.subproblems,
          (iter, prb, prb_section) => {
            var name = "Main"
            if(iter > 0)
                name = "Subproblem UB = " + this.f2(prb.start_ub);
            prb_section.add_html_row(`<span class="slider_header">#${iter+1} ${name}</span>`);
            kd.get_parser(prb.kd_type)?.detail_view_rows(prb, prb_section);
          }
        );
    }
  }
}
kd.add_parser("tspd_solver", new TSPDSolverParser());
