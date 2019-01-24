from enum import Enum
class NodeType(Enum):
  DEPOT = 0,
  CUSTOMER = 1,
  CHARGING_STATION = 2

class Node(object):
  def __init__(self, node_id, name, type):
    """Defines a node for the graph underlying an FRVCP."""
    self.node_id = node_id
    self.name = name
    self.type = type

  def __str__(self):
    return f'({self.node_id}; {self.type})'

class PCCMLabel(object):
  """Class defining a label for the labeling algorithm of
  Froger (2018) for the fixed-route vehicle charging problem.
  """

  def __init__ (self, node_id_for_label, key_time, trip_time, 
    last_visited_cs, soc_arr_to_last_cs, energy_consumed_since_last_cs, 
    supporting_pts, slope, time_last_arc, energy_last_arc, parent, y_intercept=None
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

  def _compute_y_intercept(self):
    if self.slope is None:
      return None
    else:
      return [(self.supporting_pts[1][b]-self.slope[b]*self.supporting_pts[0][b])
        for b in range(len(self.slope))]

  def dominates(self,other):
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

  def get_soc_dichotomic(self, time):
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

  def get_first_supp_pt_soc (self):
    return self.supporting_pts[1][0]
  
  def get_last_supp_pt_soc (self):
    return self.supporting_pts[1][-1]
    
  def get_num_supp_pts (self):
    return len(self.supporting_pts[0])
  
  def get_path (self):
    path = []
    curr_parent = self
    stop = False
    while not stop:
      path.append(curr_parent.node_id_for_label)
      curr_parent = curr_parent.parent
      if curr_parent is None:
        stop = True
    return path
  
  def get_path_from_last_customer (self):
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
  
  def get_charging_amounts(self):

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
  def compare_to (self,other):
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
  
  def __eq__(self, other):
    return self.compare_to(other) == 0

  def __ne__(self, other):
    return self.compare_to(other) != 0

  def __lt__(self, other):
    return self.compare_to(other) < 0

  def __le__(self, other):
    return self.compare_to(other) <= 0

  def __gt__(self, other):
    return self.compare_to(other) > 0

  def __ge__(self, other):
    return self.compare_to(other) >= 0

  # endregion

class FRVCPInstance(object):
  #TODO
  def __init__(self):
    # more TODO
    return

class PCCMAlgorithm(object):
  """The labeling algorithm to solve the FRVCP."""
  # TODO move to separate class
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