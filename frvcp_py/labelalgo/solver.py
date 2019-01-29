from frvcp_py.labelalgo.core import FRVCPInstance,Node
from frvcp_py.labelalgo.algorithm import PCCMAlgorithm
from typing import List, Any, Tuple

class Solver(object):
  def __init__(self, instance_filename: str, route: List[int]):
    self.instance = FRVCPInstance(instance_filename)
    self.route = route

  def solve(self) -> Tuple[float, List[Any]]:
    max_detour_charge_time = self._compute_max_avail_time_detour_charge() # float
    
    min_soc_at_departure = self._compute_min_soc_at_departure() #List[float]
    
    max_soc_at_arrival = self._compute_max_soc_at_arrival() #list[float]
    
    # TODO in our version for the RP-AEV, in "can go direct" method, check for overlap of completion time and release time, bc if they don't, then we have to idle somewhere in between (or we could allow go direct, but this would not be exactly in line with problem (would be a relaxation), so would be a looser bound)
    possible_direct_connect = self._compute_possible_direct_connect(min_soc_at_departure,max_soc_at_arrival) #list[bool]
    
    possible_cs_connect = self._compute_possible_cs_connections(min_soc_at_departure, max_soc_at_arrival) #list[list[list[bool]]]
    
    possible_cs_detour = self._compute_possible_cs_detour(max_detour_charge_time,False) #list[list[bool]]
    
    possible_cs_link = self._compute_possible_cs_link(max_detour_charge_time) #list[list[list[bool]]]
    
    node_local_id_dep = 0 #int
    
    node_local_id_arr = len(self.route)-1 #int
    
    nodes_gpr = self._build_gpr_nodes()#list[Node]
    print(nodes_gpr)
    print(this) # will break the script
    
    adjacencies = self._compute_adjacencies(nodes_gpr, possible_direct_connect,
      possible_cs_connect, possible_cs_detour, possible_cs_link) #list[list[int]]
    
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
    # TODO
    return 'todo'

  def _compute_min_soc_at_departure(self) -> List[float]:
    # TODO
    return 'todo'

  def _compute_max_soc_at_arrival(self) -> List[float]:
    # TODO
    return 'todo'

  def _compute_possible_direct_connect(self,
    min_soc_at_departure: List[float],
    max_soc_at_arrival: List[float]
  ) -> List[bool]:
    # TODO
    return 'todo'

  def _compute_possible_cs_connections(self,
    min_soc_at_departure: List[float], 
    max_soc_at_arrival: List[float]
  ) -> List[List[List[bool]]]:
    # TODO
    return 'todo'

  def _compute_possible_cs_detour(self,
    max_detour_charge_time: float,
    only_one: bool=False
  ) -> List[List[bool]]:
    # TODO
    return 'todo'

  def _compute_possible_cs_link(self, 
    max_detour_charge_time: float
  ) -> List[List[List[bool]]]:
    # TODO
    return 'todo'

  def _build_gpr_nodes(self) -> List[Node]:
    return ([self.instance.nodes_g[self.route[i]] for i in range(len(self.route))] +
      ((self.instance.get_cs_nodes())*(len(self.route)-1)))

  def _compute_adjacencies(self, 
    nodes_gpr: List[Node],
    possible_direct_connect: List[bool],
    possible_cs_connect: List[List[List[bool]]],
    possible_cs_detour: List[List[bool]],
    possible_cs_link: List[List[List[bool]]]
  ) -> List[List[int]]:
    # TODO
    return 'todo'

  def _compute_bounds(self, nodes: List[Node]) -> Tuple[List[float], List[float], List[float], List[float], List[float], List[float]]:
    # TODO
    return 'todo'