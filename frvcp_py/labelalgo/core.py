from typing import List, Any
from enum import Enum
class NodeType(Enum):
  DEPOT = 0,
  CUSTOMER = 1,
  CHARGING_STATION = 2

class Node(object):
  def __init__(self, node_id: int, name: str, type: NodeType):
    """Defines a node for the graph underlying an FRVCP."""
    self.node_id = node_id
    self.name = name
    self.type = type

  def __str__(self):
    return f'({self.node_id}; {self.type})'

class HeapE(object):
  """Represents an element in the labeling algorithm's heap.
  Since the data type used for heap nodes in the labeling
  algorithm is just int, this object is largely unnecessary.
  We keep it here to maintain consistency"""
  def __init__(self, data: Any):
    self.data = data

from queue import PriorityQueue
import heapq
class PseudoFibonacciHeap(PriorityQueue):
  """Defines a priority queue whose keys can be updated.
  This mimics the Fibonacci heap object used in the original labeling
  algorithm, allowing for increase-/decrease-key functionality.
  However, the underlying implementation is not actually a Fibonacci 
  heap, so it lacks the nice theoretical advantages.
  """
  def __init__(self):
    self._pq = []                         # list of entries arranged in a heap
    self._entry_finder = {}               # mapping of tasks to entries
    self._REMOVED = '<removed-task>'      # placeholder for a removed task
    import itertools
    self._counter = itertools.count()     # unique sequence count

  def __bool__(self):
    return len(self._pq) > 0
  
  def add_task(self, task: Any, priority: float=0):
      'Add a new task or update the priority of an existing task'
      if task in self._entry_finder:
          self.remove_task(task)
      count = next(self._counter)
      entry = [priority, count, task]
      self._entry_finder[task] = entry
      heapq.heappush(self._pq, entry)

  def remove_task(self, task: Any):
      'Mark an existing task as REMOVED.  Raise KeyError if not found.'
      entry = self._entry_finder.pop(task)
      entry[-1] = self._REMOVED

  def pop_task(self) -> Any:
      'Remove and return the lowest priority task. Raise KeyError if empty.'
      while self._pq:
          priority, count, task = heapq.heappop(self._pq)
          if task is not self._REMOVED:
              del self._entry_finder[task]
              return task
      raise KeyError('pop from an empty priority queue')

class PCCMLabel(object):
  """Class defining a label for the labeling algorithm of
  Froger (2018) for the fixed-route vehicle charging problem.
  """

  def __init__ (self, node_id_for_label: int, key_time: float, trip_time: float, 
    last_visited_cs: int, soc_arr_to_last_cs: float, energy_consumed_since_last_cs: float, 
    supporting_pts:List[List[float]], slope: List[float], time_last_arc: float,
    energy_last_arc: float, parent, y_intercept: List[float]=None
  ):
    self.node_id_for_label = node_id_for_label
    self.key_time = key_time
    self.trip_time = trip_time
    self.last_visited_cs = last_visited_cs
    self.soc_arr_to_last_cs = soc_arr_to_last_cs
    self.energy_consumed_since_last_cs = energy_consumed_since_last_cs
    self.supporting_pts = supporting_pts
    self.slope = slope
    self.time_last_arc = time_last_arc
    self.energy_last_arc = energy_last_arc
    self.parent = parent
    self.y_intercept = self._compute_y_intercept() if y_intercept is None else y_intercept

  def _compute_y_intercept(self) -> List[float]:
    if self.slope is None:
      return None
    else:
      return [(self.supporting_pts[1][b]-self.slope[b]*self.supporting_pts[0][b])
        for b in range(len(self.slope))]

  def dominates(self, other) -> bool:
    # drive time dominance
    if self.trip_time > other.trip_time:
      return False
    
    n_pts = len(self.supporting_pts[0])
    n_pts_other = len(other.supporting_pts[0])
    
    # energy dominance (larger reachable SOC)
    if self.supporting_pts[1][-1] < other.supporting_pts[1][-1]:
      return False
    
    # SOC dominance
    # 1) supp pts of this SOC function
    for k in range(n_pts):
      soc_other = other.getSOCDichotomic(self.supporting_pts[0][k])
      if self.supporting_pts[1][k] < soc_other:
        return False

    # 2) supp pts of the other SOC function
    for k in range(n_pts_other):
      soc = self.get_soc_dichotomic(other.supporting_pts[0][k])
      if soc < other.supporting_pts[1][k]:
        return False

    return True

  def get_soc_dichotomic(self, time: float) -> float:
    if time < self.trip_time:
      return -float('inf')
		
    n_pts = len(self.supporting_pts)
    if time >= self.supporting_pts[0][-1]:
      return self.supporting_pts[1][-1]
    
    low = 0
    high = n_pts-1
    while (low + 1 < high):
      mid = (low + high) // 2
      if self.supporting_pts[0][mid] < time:
        low = mid
      else: # self.supporting_pts[0][mid] >= time
        high = mid
    
    return self.slope[low] * time + self.y_intercept[low]

  def get_first_supp_pt_soc(self) -> float:
    return self.supporting_pts[1][0]
  
  def get_last_supp_pt_soc(self) -> float:
    return self.supporting_pts[1][-1]
    
  def get_num_supp_pts(self) -> int:
    """Returns the number of supporting points."""
    return len(self.supporting_pts[0])
  
  def get_path(self) -> List[int]:
    path = []
    curr_parent = self
    stop = False
    while not stop:
      path.append(curr_parent.node_id_for_label)
      curr_parent = curr_parent.parent
      if curr_parent is None:
        stop = True
    return path
  
  def get_path_from_last_customer (self) -> List[int]:
    """Provides a list of the node IDs that the vehicle has 
    visited since the last time it either a) visited a customer,
    b) visited a depot, or c) visited the node at which it
    currently resides.

    I think. Getting closer to the implementation should answer this.
    """
    if self.last_visited_cs is None:
      return []
    
    path = []
    curr_parent = self
    curr_prev_cs = curr_parent.last_visited_cs

    stop = False
    while not stop:
      next_prev_cs = curr_parent.last_visited_cs
      path.append(curr_parent.node_id_for_label)
      curr_parent = curr_parent.parent
      curr_prev_cs = curr_parent.last_visited_cs
      if curr_parent is None or curr_prev_cs is None or curr_prev_cs == next_prev_cs:
        stop = True
    
    return path
  
  def get_charging_amounts(self) -> List[float]:

    if self.last_visited_cs is None:
      return [] # no visits to CS

    # charge amount at last visited CS
    charge_amts = [(self.energy_consumed_since_last_cs + 
      self.get_first_supp_pt_soc - 
      self.soc_arr_to_last_cs)]
      
    # computation of other charge amounts (if any)
    curr_label = self
    while True:
      s_last_vis_cs = curr_label.last_visited_cs

      stop = False
      while not stop:
        charge_reqd = (curr_label.energy_consumed_since_last_cs + 
          curr_label.get_first_supp_pt_soc - 
          curr_label.soc_arr_to_last_cs)
        curr_label = curr_label.parent
        if curr_label.last_visited_cs != s_last_vis_cs:
          stop = True

      if curr_label.last_visited_cs is None:
        break
        
      # compute charging amount
      charge_amts.append(charge_reqd)
    
    return charge_amts

  def __str__(self):
    s = f"---- Label for node {self.node_id_for_label}\n"
    s += (f"keyTime = {self.key_time}\t tripTime = {self.trip_time}\n")
    s += (f"timeLastArc = {self.time_last_arc}\t energyLastArc = {self.energy_last_arc}\n")
    s += (f"lastVisitedCS = {self.last_visited_cs}\t")
    s += (f"socAtArrLastCS = {self.soc_arr_to_last_cs}\n")
    s += (f"energyConsumedSinceLastCS = {self.energy_consumed_since_last_cs}\n")
    s += ("Supporting points \n")
    s += str(self.supporting_pts[0])+"\n"
    s += str(self.supporting_pts[1])+"\n"
    if self.slope is not None:
      s += "Slope\n"
      s += str(self.slope)+"\n"
      s += "Intercept\n"
      s += str(self.y_intercept)+"\n"
    s += "Path\n"
    s += str(self.get_path())
    return s

  # region comparable methods
  def compare_to(self, other) -> int:
    if self.key_time < other.key_time:
      return -1
    elif self.key_time > other.key_time:
      return 1
    
    else:
      diff = other.supporting_pts[1][0]-self.supporting_pts[1][0]
      if diff > 0.0:
        return 1
      elif diff < 0.0:
        return -1
      else:
        return 0
  
  def __eq__(self, other) -> bool:
    return self.compare_to(other) == 0

  def __ne__(self, other) -> bool:
    return self.compare_to(other) != 0

  def __lt__(self, other) -> bool:
    return self.compare_to(other) < 0

  def __le__(self, other) -> bool:
    return self.compare_to(other) <= 0

  def __gt__(self, other) -> bool:
    return self.compare_to(other) > 0

  def __ge__(self, other) -> bool:
    return self.compare_to(other) >= 0

  # endregion

import json
class FRVCPInstance(object):
  def __init__(self, instance_filename: str):
    with open(instance_filename) as f:
      instance_json = json.load(f)
      self._store_instance_parameters(instance_json)
    return
  
  def _store_instance_parameters(self, instance):
    self.energy_matrix = instance["energy_matrix"] # [i][j] are indices in g, not gprime
    self.time_matrix = instance["time_matrix"]
    self.process_times = instance["process_times"]
    # number of nodes in the underlying graph G
    self.n_nodes_g = len(self.process_times)
    self.max_q = instance["max_q"]
    self.init_soc = instance["init_soc"]
    self.t_max = instance["t_max"]
    # keys are cs types, values are dicts that map "time" or "charge" to corresponding arrays of floats
    self.cs_bkpt_info = instance["breakpoints_by_type"]
    self.cs_bkpt_info = {int(k):v for k,v in self.cs_bkpt_info.items()}
    # list of objs with attrs 'node_id' and 'type'
    self.cs_details = instance["css"]
    # number of charging stations
    self.n_cs = len(self.cs_details)
    # keys are cs types, values are 2d arrays of cs breakpoints, formatted
    # similarly to how they are used in the labeling algorithm:
    # [0][:] are time bkpts, [1][:] are charge bkpts
    self.type_to_supp_pts = self._make_type_to_supp_pts()
    # keys are cs types, values are arrays of slopes and y-intercepts
    self.type_to_slopes = self._make_type_to_slopes()
    self.type_to_yints = self._make_type_to_yintercepts()
    # max slope across all charging stations
    self.max_slope = self._compute_max_slope()
    # id is the CS's index in G
    self.cs_id_to_type = self._make_cs_id_to_type_map()
    # list of nodes for the vertices in G
    self.nodes_g = self._make_nodes()

  def get_min_energy_to_cs(self, node_id: int) -> float:
    # TODO in implementation for RP-AEV, in if condition, also check that that cs's slope is > 0 (since idle locs will also be classified as CSs)
    return 0.0 if node_id in self.cs_id_to_type \
      else min([self.energy_matrix[node_id][cs_id] for cs_id in self.cs_id_to_type])
  
  def get_cs_nodes(self) -> List[Node]:
    return [node for node in self.nodes_g if node.type == NodeType.CHARGING_STATION]
  
  def _make_nodes(self) -> List[Node]:
    return [
      Node(i, f'node-{i}',
        (NodeType.CHARGING_STATION if i in self.cs_id_to_type else NodeType.CUSTOMER))
      for i in range(len(self.energy_matrix)) ]
  
  def _make_type_to_supp_pts(self):
    return {cs_type:[pts["time"],pts["charge"]] for cs_type,pts in self.cs_bkpt_info.items()}
  
  def _make_type_to_slopes(self):
    return {
      cs_type:[
        ((arr[1][i+1] - arr[1][i]) / (arr[0][i+1] - arr[0][i]))
        for i in range(len(arr[0])-1)]
      for cs_type,arr in self.type_to_supp_pts.items()}
  
  def _make_type_to_yintercepts(self):
    return {
      cs_type:[ # b = y-mx
        (self.type_to_supp_pts[cs_type][1][i] - 
          self.type_to_slopes[cs_type][i] * self.type_to_supp_pts[cs_type][0][i])
        for i in range(len(arr[0])-1)]
      for cs_type,arr in self.type_to_supp_pts.items()}
  
  def _make_cs_id_to_type_map(self):
    return {cs["node_id"]:cs["type"] for cs in self.cs_details}
  
  def _compute_max_slope(self):
    return max([slopes[0] for slopes in self.type_to_slopes.values()])
  
  def is_cs_faster(self, node1: Node, node2: Node) -> bool:
    return (self.type_to_slopes[self.cs_id_to_type[node1.node_id]][0] > 
      self.type_to_slopes[self.cs_id_to_type[node2.node_id]][0])
  
  def get_supporting_points(self, node: Node) -> List[List[float]]:
    return self.type_to_supp_pts[self.cs_id_to_type[node.node_id]]

  def _get_cf_segment_idx(self, cs_type, value, axis) -> int:
    """Axis is 1 for charge and 0 for time."""
    idx=0
    while not (self.type_to_supp_pts[cs_type][axis][idx] <= value and 
      value < self.type_to_supp_pts[cs_type][axis][idx+1]
    ):
      idx += 1
      # for the last segment, check its upper limit
      if idx == len(self.type_to_supp_pts[cs_type][axis])-1:
        # if it's equal to the upper limit, return the index for the last segment
        if value == self.type_to_supp_pts[cs_type][axis][idx]:
          return idx-1
        # otherwise, it was a bad request
        else:
          raise ValueError(f'Request out of bounds for segment index. Value passed: {value}')
    return idx
  
  def get_slope(self, node: Node, soc: float=None):
    # if node passed but no soc, return slopes of node's charging function
    if soc is None:
      return self.type_to_slopes[self.cs_id_to_type[node.node_id]]
    # otherwise, return the slope of the segment on which the soc lies
    else:
      return self.type_to_slopes[self.cs_id_to_type[node.node_id]][
        self._get_cf_segment_idx(self.cs_id_to_type[node.node_id],soc,1)]

  def get_charging_time(self,
    node: Node,
    starting_energy: float,
    acquired_energy: float
  ):
    return self.get_time_to_charge_from_zero(node,starting_energy+acquired_energy) - \
      self.get_time_to_charge_from_zero(node,starting_energy)
  
  def get_time_to_charge_from_zero(self, node: Node, soc: float):
    seg_idx = self._get_cf_segment_idx(self.cs_id_to_type[node.node_id],soc,1)
    return (soc - self.type_to_yints[self.cs_id_to_type[node.node_id]][seg_idx]) / \
      self.type_to_slopes[self.cs_id_to_type[node.node_id]][seg_idx]