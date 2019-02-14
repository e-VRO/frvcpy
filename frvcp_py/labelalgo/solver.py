from frvcp_py.labelalgo.core import FRVCPInstance,Node
from frvcp_py.labelalgo.algorithm import PCCMAlgorithm
from typing import List, Any, Tuple

class Solver(object):
  def __init__(self, instance_filename: str, route: List[int], init_soc: float):
    self.instance = FRVCPInstance(instance_filename, init_soc)
    self.route = route

  def solve(self) -> Tuple[float, List[Any]]:
    max_detour_charge_time = self._compute_max_avail_time_detour_charge() # float
    
    # below lists (until nodes_gpr) are of length (num_nodes_in_G)
    min_soc_at_departure = self._compute_min_soc_at_departure() #List[float]
    
    max_allowed_soc_at_arrival = self._compute_max_soc_at_arrival() #list[float]
    
    # length of direct connect is actually len(route)-1
    possible_direct_connect = self._compute_possible_direct_connect(min_soc_at_departure,max_allowed_soc_at_arrival) #list[bool]
    
    possible_cs_connect = self._compute_possible_cs_connections(min_soc_at_departure, max_allowed_soc_at_arrival) #list[list[list[bool]]]
    
    possible_cs_detour = self._compute_possible_cs_detour(max_detour_charge_time,False) #list[list[bool]]
    
    possible_cs_link = self._compute_possible_cs_link(max_detour_charge_time) #list[list[list[bool]]]
    
    node_local_id_dep = 0 #int
    
    node_local_id_arr = len(self.route)-1 #int
    
    # starting here, lists are of length len(num_nodes_in_GPrime)
    nodes_gpr = self._build_gpr_nodes()#list[Node]
    
    adjacencies = self._compute_adjacencies(nodes_gpr, possible_direct_connect,
      possible_cs_connect, possible_cs_detour, possible_cs_link) #list[list[int]]
    
    # all below lists are of length len(nodes_gpr)
    (
      min_travel_time_after_node,
      min_energy_consumed_after_node,
      min_travel_charge_time_after_node,
      latest_departure_time,
      min_energy_at_departure,
      max_energy_at_departure
    ) = self._compute_bounds(nodes_gpr) #list[list[float]]

    # initialize algorithm
    label_algo = PCCMAlgorithm(
      self.instance,
      self.instance.init_soc,
      nodes_gpr, 
      adjacencies,
      node_local_id_dep,
      node_local_id_arr,
      self.instance.max_slope,
      min_energy_consumed_after_node,
      min_travel_time_after_node,
      min_travel_charge_time_after_node,
      max_energy_at_departure,
      latest_departure_time,
      min_energy_at_departure)

    # run algorithm    
    label_algo.run_multiobj_shortest_path_algo(True,True)
    
    # return results
    return (label_algo.get_objective_value(),label_algo.get_optimized_route())

  def _compute_max_avail_time_detour_charge(self) -> float:
    """Max amount of time we have to detour and charge (in consideration
    of the time duration).
    """
    return self.instance.t_max - \
      sum([self.instance.time_matrix[self.route[i]][self.route[i+1]]
        for i in range(len(self.route)-1)]) - \
      sum([self.instance.process_times[stop] for stop in self.route])

  def _compute_min_soc_at_departure(self) -> List[float]:
    """Minimum allowable energy with which we can depart each node."""
    return [self.instance.get_min_energy_to_cs(node.node_id) for node in self.instance.nodes_g]

  def _compute_max_soc_at_arrival(self) -> List[float]:
    """Max allowable charge at arrival to any node.
    Shape is self.instance.n_nodes_g x 1:
    [i] = max allowable charge at arrival to the ith node
    """
    return [self.instance.max_q for _ in range(self.instance.n_nodes_g)]

  def _compute_possible_direct_connect(self,
    min_soc_at_departure: List[float],
    max_soc_at_arrival: List[float]
  ) -> List[bool]:
    """For each stop in the route, can we go directly to the next stop?
    Output is a list of length len(self.route)-1:
    [i] = can we go from stop i in the route to stop i+1 directly
    """
    return [
      (max_soc_at_arrival[self.route[i]] - \
        self.instance.energy_matrix[self.route[i]][self.route[i+1]] >= min_soc_at_departure[i+1]) \
      for i in range(len(self.route)-1)]

  def _compute_possible_cs_connections(self,
    min_soc_at_departure: List[float], 
    max_soc_at_arrival: List[float]
  ) -> List[List[List[bool]]]:
    """From each stop in the route, can we detour to a charging station, then
    from that charging station to the next node?
    
    Shape of returned list: (len(self.route)-1) x (num_css) x 2
    [i][k][0] = can we go from stop i in the route to the kth CS?
    [i][k][1] = can we go from the kth cs to the (i+1)th stop in the route?
    """
    result = [[[None,None] for cs in range(self.instance.n_cs)] for i in range(len(self.route)-1)]
    for i in range(len(self.route)-1):
      init_loc = self.route[i]
      next_loc = self.route[i+1]
      for i_cs in range(self.instance.n_cs):
        cs_id = self.instance.cs_details[i_cs]['node_id']
        result[i][i_cs][0] = max_soc_at_arrival[init_loc] - \
          self.instance.energy_matrix[init_loc][cs_id] >= 0 # ge 0, bc we can charge at the CS
        result[i][i_cs][1] = self.instance.max_q - \
          self.instance.energy_matrix[cs_id][next_loc] >= min_soc_at_departure[next_loc]
    return result

  def _compute_possible_cs_detour(self,
    max_detour_charge_time: float,
    only_one: bool=False
  ) -> List[List[bool]]:
    """Is it possible to go to visit each CS between stops in the route?
    Shape: len(self.route)-1 x num_cs:
    [i][k] = can we visit the kth CS between stops i and i+1 in the route?
    """
    result = [[False for i_cs in range(self.instance.n_cs)] for i in range(len(self.route)-1)]
    for i in range(len(self.route)-1):
      s1 = self.route[i]
      s2 = self.route[i+1]
      for i_cs in range(self.instance.n_cs):
        cs = self.instance.cs_details[i_cs]["node_id"]
        detour_time = self.instance.time_matrix[s1][cs] + \
          self.instance.time_matrix[cs][s2] - \
          self.instance.time_matrix[s1][s2]
        if only_one: # take into account the minimum charge time as well
          detour_time += self.instance.get_charging_time( \
            self.instance.n_nodes_g[cs], \
            0, \
            self.instance.energy_matrix[s1][cs] + \
              self.instance.energy_matrix[cs][s2] - \
              self.instance.energy_matrix[s1][s2]
            )
        if detour_time <= max_detour_charge_time:
          result[i][i_cs] = True
    return result

  def _compute_possible_cs_link(self, 
    max_detour_charge_time: float
  ) -> List[List[List[bool]]]:
    """Can two CSs be connected between stops in the route?
    Shape is: len(self.route)-1 x numcss x numcss
    [i][j][k] = can we connect the jth CS to the kth CS between stops i and i+1 in the route?
    """
    result = [[[False for k2 in range(self.instance.n_cs)] for k1 in range(self.instance.n_cs)] for i in range(len(self.route) -1)]
    for i in range(len(self.route) -1):
      curr_stop = self.route[i]
      next_stop = self.route[i+1]
      for k1 in range(self.instance.n_cs):
        cs1 = self.instance.cs_details[k1]["node_id"]
        for k2 in range(self.instance.n_cs):
          if k1 == k2:
            continue
          cs2 = self.instance.cs_details[k2]["node_id"]
          if (
            (self.instance.energy_matrix[cs1][cs2] <= self.instance.max_q) and
            (self.instance.time_matrix[curr_stop][cs1] + \
              self.instance.time_matrix[cs1][cs2] + \
              self.instance.time_matrix[cs2][next_stop] - \
              self.instance.time_matrix[curr_stop][next_stop] <= max_detour_charge_time)
          ):
            result[i][k1][k2] = True
    return result

  def _build_gpr_nodes(self) -> List[Node]:
    """Build the list of nodes that define the graph G'."""
    return ([self.instance.nodes_g[self.route[i]] for i in range(len(self.route))] +
      ((self.instance.get_cs_nodes())*(len(self.route)-1)))

  def _compute_adjacencies(self, 
    nodes_gpr: List[Node],
    possible_direct_connect: List[bool],
    possible_cs_connect: List[List[List[bool]]],
    possible_cs_detour: List[List[bool]],
    possible_cs_link: List[List[List[bool]]]
  ) -> List[List[int]]:
    """Compute the adjacency list for the nodes in G prime."""
    adjs = [[] for _ in nodes_gpr]
    # add direct connections and one-off detours
    for i in range(len(self.route)-1):
      if possible_direct_connect[i]:
        adjs[i].append(i+1)
      for j in range(len(self.route)+i*self.instance.n_cs, len(self.route)+(i+1)*self.instance.n_cs):
        i_cs = (j - len(self.route)) % self.instance.n_cs
        if possible_cs_detour[i][i_cs]:
          if possible_cs_connect[i][i_cs][0]:
            adjs[i].append(j)
          if possible_cs_connect[i][i_cs][1]:
            adjs[j].append(i+1)
    # add intra-cs links
    for i in range(len(self.route)-1):
      b = len(self.route)+i*self.instance.n_cs
      e = len(self.route)+(i+1)*self.instance.n_cs
      for j1 in range(b,e):
        i_cs1 = (j1 - len(self.route)) % self.instance.n_cs
        for j2 in range(b,e):
          i_cs2 = (j2 - len(self.route)) % self.instance.n_cs
          if possible_cs_link[i][i_cs1][i_cs2]:
            adjs[j1].append(j2)

    return adjs

  def _compute_bounds(self, nodes: List[Node]) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
    """Produces a 6-tuple of bounds that get used in the labeling algorithm:
     1. min_travel_time_after_node: minimum time left in the route after the vehicle departs
     2. min_energy_consumed_after_node: minimum amount of energy required to traverse the rest of the route
     3. min_travel_charge_time_after_node: minimum time left in the route if it were to travel it directly
          and consume any additional required energy without detouring at the fastest rate possible
     4. latest_departure_time: latest time at which the vehicle can depart
     5. min_energy_at_departure: min allowable energy at departure
     6. max_energy_at_departure: max allowable energy at departure
    """
    min_travel_time_after_node = [None for _ in range(len(nodes))]
    min_energy_consumed_after_node = [None for _ in range(len(nodes))]
    min_travel_charge_time_after_node = [None for _ in range(len(nodes))]
    latest_departure_time = [None for _ in range(len(nodes))]
    min_energy_at_departure = [None for _ in range(len(nodes))]
    max_energy_at_departure = [None for _ in range(len(nodes))]
    
    # region traveling and charging after departing nodes
    # initialize trackers
    energy = 0
    time = 0
    time_charge = 0
    next_id = self.route[-1]

    # set entries for last stop in route
    min_travel_time_after_node[len(self.route)-1] = time
    min_energy_consumed_after_node[len(self.route)-1] = energy
    min_travel_charge_time_after_node[len(self.route)-1] = time_charge

    # set entries for all others
    for i in range(len(self.route)-2,-1,-1):
      for j in range(len(self.route)+i*self.instance.n_cs,len(self.route)+(i+1)*self.instance.n_cs):
        curr_id = nodes[j].node_id
        min_travel_time_after_node[j] = time + self.instance.time_matrix[curr_id][next_id] + self.instance.process_times[next_id]
        min_energy_consumed_after_node[j] = energy + self.instance.energy_matrix[curr_id][next_id]
        min_travel_charge_time_after_node[j] = time_charge + (self.instance.time_matrix[curr_id][next_id] +
          self.instance.process_times[next_id] + self.instance.energy_matrix[curr_id][next_id] / self.instance.max_slope)
      curr_id = self.route[i]
      time += self.instance.time_matrix[curr_id][next_id] + self.instance.process_times[next_id]
      energy += self.instance.energy_matrix[curr_id][next_id]
      time_charge += (self.instance.time_matrix[curr_id][next_id] + self.instance.process_times[next_id] +
        self.instance.energy_matrix[curr_id][next_id] / self.instance.max_slope)
      min_travel_time_after_node[i] = time
      min_energy_consumed_after_node[i] = energy
      min_travel_charge_time_after_node[i] = time_charge
      next_id = curr_id

    # endregion

    # region bounds on time and charge when departing nodes
    for i in range(len(nodes)):
      curr_id = nodes[i].node_id
      min_energy_to_charge = min_energy_consumed_after_node[i] - self.instance.max_q
      latest_departure_time[i] = self.instance.t_max - min_travel_time_after_node[i]
      if min_energy_to_charge > 0:
        latest_departure_time[i] -= min_energy_to_charge/self.instance.max_slope
      min_energy_at_departure[i] = self.instance.get_min_energy_to_cs(curr_id)
      max_energy_at_departure[i] = self.instance.max_q
      # endregion
    
    return (
      min_travel_time_after_node,
      min_energy_consumed_after_node,
      min_travel_charge_time_after_node,
      latest_departure_time,
      min_energy_at_departure,
      max_energy_at_departure
    )
