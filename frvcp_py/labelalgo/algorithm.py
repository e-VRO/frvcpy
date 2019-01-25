from frvcp_py.labelalgo.core import *
class PCCMAlgorithm(object):
  """The Froger, et al. (2018) labeling algorithm to solve the FRVCP."""

  def __init__(self, instance, init_soc, nodes, adjacency_list,
    node_local_id_dep, node_local_id_arr, max_slope
  ):
    self.instance = instance # the instance
    self.init_soc = init_soc # initial charge
    self.nodes = nodes # array containing nodes of graph (indices are local IDs)
    self.n_nodes = len(nodes) # number of nodes
    self.adjacency_list = adjacency_list # adjacency lists (containing node IDs) defining arcs of the graph
    self.node_local_id_dep = node_local_id_dep # ID of the departure (origin) node
    self.node_local_id_arr = node_local_id_arr # ID of the arrival (destination) node
    self.max_slope = max_slope # max slope in any CS's charging function
    self.min_energy_consumed_after_node = [] # minimum energy consumed after departing each node
    self.min_travel_time_after_node = [] # minimum travel time after departing a node
    # minimum travel time after departing a node plus 
    # the minimum time required to charge to the energy level needed to reach the destination node after this node
    self.min_travel_charge_time_after_node = []
    # upper bound on SOC of EV upon departing each node (using local ID)
    # (according to future course of the route)
    self.max_energy_at_departure = []
    # upper bound on the time at which the vehicle can depart a node (using local ID)
    self.latest_departure_time = []
    # lower bound on the SOC the EV can have as it departs each node (using local ID)
    self.min_energy_at_departure = []


  def run_multiobj_shortest_path_algo(self, dominance, stop_at_first):
    
    # is there an unset label associated with each node (using ID) in the updatable priority queue
    self.in_heap = [False for _ in self.nodes]
    
    # key associated with each node (using local ID) (array of doubles)
    self.key = [None for _ in self.nodes]

    from queue import PriorityQueue
    # priority queues of unset labels for each node
    # ("for CS only labels at departure" -- meaning it represents keytime when departing the CS?)
    self.unset_labels = [PriorityQueue() for _ in self.nodes]
    
    # list of set (nondominated) labels for each node
    # ("for CS only labels at departure" -- meaning it represents keytime when departing the CS?)
    self.set_labels = [[] for _ in self.nodes]
    
    # for each node (by local id), the node in the updatable queue associated with the label currently in the heap
    # TODO BY LOCAL ID??
    self.heap_elements = [None for _ in self.nodes]
    
    # stores best unset labels for each node (tasks are integers or labels?)
    self.heap = PseudoFibonacciHeap()

    # build first label
    first_label = self._build_first_label()
    self.heap_elements[self.node_local_id_dep] = HeapE(self.node_local_id_dep)
    self.key[self.node_local_id_dep] = first_label.key_time
    self.heap.add_task(self.heap_elements[self.node_local_id_dep], first_label.key_time)
    self.in_heap[self.node_local_id_dep] = True
    self.unset_labels[self.node_local_id_dep].put(first_label)

    while(self.heap):
      # heap element containing the local ID of the node whose key time is smallest
      min_node_f = self.heap.pop_task()
      min_node_local_id = min_node_f.data
      min_node_id = self.nodes[min_node_local_id].node_id # node id of the aforementioned node
      self.in_heap[min_node_local_id] = False
      self.heap_elements[min_node_local_id] = None
      self.key[min_node_local_id] = None

      # Return smallest (wrt key) unset label associated with current node
      label_to_set = self.unset_labels[min_node_local_id].get()

      # compute supporting points of label
      label_to_set = self._compute_supporting_points(label_to_set)
      # TODO HERE line 182 in notepad++
      ...

    return

  def _compute_supporting_points(self, label: PCCMLabel) -> PCCMLabel:
    """Provides a new label that is identical to the argument, but with
    the supporting points, slopes, and y-intercepts updated.
    """
    if label.time_last_arc == 0 and label.energy_last_arc == 0:
      return label
    
    local_id = label.node_id_for_label
    supp_pts = label.supporting_pts
    n_pts = label.get_num_supp_pts()
    new_supp_pts_temp = [[None for _ in range(n_pts)] for _ in range(2)]

    # last supporting point index to consider
    last_k = -1
    # need to compute first supporting point
    compute_first_point = False
    for k in range(n_pts):
      new_supp_pts_temp[0][k] = supp_pts[0][k] + label.time_last_arc
      new_supp_pts_temp[1][k] = supp_pts[1][k] + label.energy_last_arc

      # stop at first supp pt with:
      # more SOC than needed or
      # the one that leads to violation of route duration limit
      if (last_k == -1 and 
        (new_supp_pts_temp[1][k] > self.max_energy_at_departure[local_id] or
          new_supp_pts_temp[0][k] > self.latest_departure_time[local_id])
      ):
        last_k = k
        break
      
    
    # if we don't need to restrict final supp pt index
    if last_k == -1:
      last_k = n_pts
    
    # determine first supp pt to consider
    first_k = 0
    while (first_k < last_k and 
      new_supp_pts_temp[1][first_k] < self.min_energy_at_departure[local_id]
    ):
      first_k += 1
    
    # if pts are on a single segment, just consider one valid point
    if first_k == last_k:
      if first_k == 0 or first_k == n_pts:
        # no feasible supporting pts, which shouldn't happen
        raise ValueError(f'No feasible supporting points for label\n{label}')
      else:
        # first and last supp pt may be on single segment between two pts
        compute_first_point = True
    
    # need to compute first point if first_k != 0
    if first_k != 0:
      compute_first_point = True
      first_k -= 1
    # increment one since pts will be considered from first_k to last_k-1
    if last_k != n_pts:
      last_k += 1
    
    # make new supporting points
    if not compute_first_point:
      # all supp pts (up to those with index >= last_k) are feasible
      new_supp_pts = [[new_supp_pts_temp[i][k] for k in range(last_k)] for i in range(2)]
      
      # if label not reduced to singleton
      if label.slope is not None:
        n_pts_rmd = n_pts - last_k # number of removed points from the soc function
        n_seg_initial = len(label.slope) # number of segments previously
        n_seg_now = n_seg_initial - n_pts_rmd # number of segments now
        
        # if number of segments is >0, compute slope and y-intercept
        if n_seg_now > 0:
          slope_now = [label.slope[k] for k in range(n_seg_now)]
          y_int_now = [(label.y_intercept[k]-label.energy_last_arc-
            label.time_last_arc*label.slope[k]) for k in range (n_seg_now)]
    
    # need to compute first supp pt on seg [first_k, first_k+1]
    else:
      n_pts_now = last_k - first_k
      new_supp_pts = [[None for k in range(n_pts_now)] for _ in range(2)]
      # compute slope and y-intercept (which must be shifted) of current segment
      slope = label.slope[first_k]
      y_int = label.y_intercept[first_k] - label.energy_last_arc - label.time_last_arc * label.slope[first_k]
      trip_time = (self.min_energy_at_departure[local_id] - y_int)/slope

      new_supp_pts[0][0] = trip_time
      new_supp_pts[1][0] = self.min_energy_at_departure[local_id]
      for k in range(first_k+1,last_k):
        new_supp_pts[0][k-first_k] = new_supp_pts_temp[0][k]
        new_supp_pts[1][k-first_k] = new_supp_pts_temp[1][k]
      # if num segments in label is > 0, compute slope and y-intercept
      if n_pts_now > 0:
        slope_now = [label.slope[k] for k in range(first_k,last_k-1)]
        y_int_now = [(label.y_intercept[k] - label.energy_last_arc - label.time_last_arc*label.slope[k]) for k in range(first_k,last_k-1)]
    
    # construction complete. return new label
    return PCCMLabel(local_id, label.key_time, label.trip_time, label.last_visited_cs,
      label.soc_arr_to_last_cs, label.energy_consumed_since_last_cs, new_supp_pts,
      slope_now, 0, 0, label.parent, y_int_now)

  def _build_first_label(self) -> PCCMLabel:
    """Constructs the initial label to kick off the labeling algorithm."""
    supp_pts = [[0],[self.init_soc]]
    energyToEnd = self.min_energy_consumed_after_node[self.node_local_id_dep]
    if self.init_soc >= energyToEnd:
      key_time = self.min_travel_time_after_node[self.node_local_id_dep]
    else:
      key_time = self.min_travel_charge_time_after_node[self.node_local_id_dep]
    return PCCMLabel(self.node_local_id_dep, key_time, 0, None, self.init_soc, 
      0, supp_pts, None, 0, 0, None)