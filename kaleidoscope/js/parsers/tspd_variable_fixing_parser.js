
class TSPDVariableFixingParser extends Parser
{
  get_attributes(obj)
  {
    let attrs = [
        new Attribute("Total Arcs", this.f2(obj.total_arcs)),
        new Attribute("Labeling Time", this.f2(obj.lbl_pre_time))
    ];

    for (let i = 0; i < obj.gap.length; i++) { 
        attrs.push(new Attribute("Time-" + obj.gap[i], this.f2(obj.time[i])));
        attrs.push(new Attribute("%Time-" + obj.gap[i], this.f2(100*obj.time[i]/obj.lbl_pre_time)));
        attrs.push(new Attribute("Labeling-" + obj.gap[i] + " Time", this.f2(obj.lbl_post_time[i])));
        attrs.push(new Attribute("%Labeling-" + obj.gap[i] + " Time", this.f2(100*obj.lbl_post_time[i]/obj.lbl_pre_time)));
        attrs.push(new Attribute("Arcs-" + obj.gap[i], this.f2(obj.arcs[i])));
        attrs.push(new Attribute("%Arcs-" + obj.gap[i], this.f2(100*obj.arcs[i]/obj.total_arcs)));
    }
    
    return attrs;
  }

  detail_view_rows(obj, view_section) {
    view_section.add_label_row("Duals", this.jsonify(obj.duals));
    
    obj.gap.forEach(function(f, idx, a) {
        view_section.add_table_row(
          ["Gap", "Time", "Arcs"],
          [f, obj.time[idx], obj.arcs[idx]],
        );
    });
  }
}
kd.add_parser("tspd_variable_fixing", new TSPDVariableFixingParser());
