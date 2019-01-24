class PCCMAlgorithm(object):
  """The labeling algorithm to solve the FRVCP."""

  def __init__(self, instance, init_soc, nodes, adjacency_list,
    node_local_id_dep, node_local_id_arr, max_slope
  ):
    self.instance = instance
    self.init_soc = init_soc
    self.nodes = nodes
    self.adjacency_list = adjacency_list
    self.node_local_id_dep = node_local_id_dep
    self.node_local_id_arr = node_local_id_arr
    self.max_slope = max_slope

  def run_multiobj_shortest_path_algo(self, dominance, stop_at_first):
    #TODO
    return