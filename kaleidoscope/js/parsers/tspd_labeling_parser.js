class TSPDLabelingParser extends Parser
{
  get_attributes(obj)
  {
    return [
      new Attribute("Time", this.f2(obj.time)),
      new Attribute("Forward Time", this.f2(obj.forward.time)),
      new Attribute("Backward Time", this.f2(obj.backward.time)),
      new Attribute("Merge point", this.f2(obj.forward.next_length)),
      new Attribute("Enumerated", this.f2(obj.forward.enumerated_count + obj.backward.enumerated_count)),
      new Attribute("B-Dominated", this.f2(obj.forward.b_dominated_count + obj.backward.b_dominated_count)),
      new Attribute("R-Dominated", this.f2(obj.forward.r_dominated_count + obj.backward.r_dominated_count)),
      new Attribute("Processed", this.f2(obj.forward.processed_count + obj.backward.processed_count)),
      new Attribute("Enumeration time", this.f2(obj.forward.enumeration_time + obj.backward.enumeration_time)),
      new Attribute("B-Domination time", this.f2(obj.forward.b_dominated_time + obj.backward.b_dominated_time)),
      new Attribute("R-Domination time", this.f2(obj.forward.r_dominated_time + obj.backward.r_dominated_time)),
      new Attribute("Merge time", this.f2(obj.merge_time))
    ];
  }

  detail_view_rows(obj, view_section) {

    view_section.add_table_row(
      ["Time", "Description", "LB"],
      [obj.time, obj.description, obj.lb],
    );

    const add_bar = (data, title, value) => {
      if (value && value !== 0) {
        data.x.push(title);
        data.y.push(value);
      }
    };

    // #Labels plot.
    var data_forward = {x:[], y:[], name:"FW"};
    var data_backward = {x:[], y:[], name:"BW"};
    add_bar(data_forward, "Enumerated", obj.forward?.enumerated_count);
    add_bar(data_forward, "B-Dominated", obj.forward?.b_dominated_count);
    add_bar(data_forward, "R-Dominated", obj.forward?.r_dominated_count);
    add_bar(data_forward, "Processed", obj.forward?.processed_count);
    add_bar(data_forward, "Bounded", obj.forward?.bounded_count);
    add_bar(data_backward, "Enumerated", obj.backward?.enumerated_count);
    add_bar(data_backward, "B-Dominated", obj.backward?.b_dominated_count);
    add_bar(data_backward, "R-Dominated", obj.backward?.r_dominated_count);
    add_bar(data_backward, "Processed", obj.backward?.processed_count);
    add_bar(data_backward, "Bounded", obj.backward?.bounded_count);
    const labels_plot = bar_plot("#Labels by stage", [data_forward, data_backward], {height:300, width:300});

    // Time plot.
    data_forward = {x:[], y:[], name: "FW"};
    data_backward = {x:[], y:[], name:"BW"};
    var data_merge = {x:[], y:[], name:"MG"};
    add_bar(data_forward, "Enumeration", obj.forward?.enumeration_time);
    add_bar(data_forward, "B-Domination", obj.forward?.b_domination_time);
    add_bar(data_forward, "R-Domination", obj.forward?.r_domination_time);
    add_bar(data_forward, "Bounding", obj.forward?.bounding_time);
    add_bar(data_backward, "Enumeration", obj.backward?.enumeration_time);
    add_bar(data_backward, "B-Domination", obj.backward?.b_domination_time);
    add_bar(data_backward, "R-Domination", obj.backward?.r_domination_time);
    add_bar(data_backward, "Bounding", obj.backward?.bounding_time);
    add_bar(data_merge, "Merge", obj.merge_time);
    const time_plot = bar_plot("Time by stage", [data_forward, data_backward, data_merge], {height:300, width:300});

    // Count by length plot.
    data_forward = {x:[], y:[], name: "FW"};
    data_backward = {x:[], y:[], name:"BW"};
    if (obj.forward?.count_by_length) {
      for (let i = 0; i < obj.forward.count_by_length.length; ++i) {
        add_bar(data_forward, i, obj.forward.count_by_length[i]);
      }
    }
    if (obj.backward?.count_by_length) {
      for (let i = 0; i < obj.backward.count_by_length.length; ++i) {
        add_bar(data_backward, i, obj.backward.count_by_length[i]);
      }
    }
    const length_plot = bar_plot("#Labels by length", [data_forward, data_backward], {height:300, width:300});

    view_section.add_flex_row([labels_plot, time_plot, length_plot]);
  }
}
kd.add_parser("tspd_labeling", new TSPDLabelingParser());
