"""Module defining the class responsible for implementing the labeling
algorithm for the FRVCP from Froger et al. (2019).
"""

from typing import List, Tuple

from frvcpy import core
# from frvcpy.core import Node, NodeType, FrvcpInstance, PseudoFibonacciHeap, HeapElement, NodeLabel


class FrvcpAlgo():
    """The Froger, et al. (2019) labeling algorithm to solve the FRVCP.
    """

    def __init__(self,
                 instance: core.FrvcpInstance,
                 init_soc: float,
                 nodes_gpr: List[core.Node],
                 adjacency_list: List[List[int]],
                 node_local_id_dep: int, node_local_id_arr: int, max_slope: float,
                 min_energy_consumed_after_node: List[float],
                 min_travel_time_after_node: List[float],
                 min_travel_charge_time_after_node: List[float],
                 max_energy_at_departure: List[float],
                 latest_departure_time: List[float],
                 min_energy_at_departure: List[float]
                 ):
        self.instance = instance  # the instance
        self.init_soc = init_soc  # initial charge
        # array containing nodes of graph g prime (indices are local IDs)
        self.nodes_gpr = nodes_gpr
        self.n_nodes = len(nodes_gpr)  # number of nodes
        # adjacency lists (containing node IDs) defining arcs of the graph
        self.adjacency_list = adjacency_list
        # ID of the departure (origin) node
        self.node_local_id_dep = node_local_id_dep
        # ID of the arrival (destination) node
        self.node_local_id_arr = node_local_id_arr
        self.max_slope = max_slope  # max slope in any CS's charging function
        # minimum energy consumed after departing each node
        self.min_energy_consumed_after_node = min_energy_consumed_after_node
        # minimum travel time after departing a node
        self.min_travel_time_after_node = min_travel_time_after_node
        # minimum travel time after departing a node plus
        # the minimum time required to charge to the energy level needed to
        # reach the destination node after this node
        self.min_travel_charge_time_after_node = min_travel_charge_time_after_node
        # upper bound on SOC of EV upon departing each node (using local ID)
        # (according to future course of the route)
        self.max_energy_at_departure = max_energy_at_departure
        # upper bound on the time at which the vehicle can depart a node (using local ID)
        self.latest_departure_time = latest_departure_time
        # lower bound on the SOC the EV can have as it departs each node (using local ID)
        self.min_energy_at_departure = min_energy_at_departure

        # Attributes related to algorithm execution
        # is there an unset label associated with each node (using ID)
        # in the updatable priority queue
        self.in_heap = None
        # key associated with each node (using local ID) (array of doubles)
        self.key = None
        # priority queues of unset labels for each node
        # ("for CS only labels at departure" -- meaning it represents
        # keytime when departing the CS?)
        self.unset_labels = None
        # list of set (nondominated) labels for each node
        # ("for CS only labels at departure" -- meaning it represents
        # keytime when departing the CS?)
        self.set_labels = None
        # for each node (by local id), the node in the updatable queue associated
        # with the label currently in the heap
        self.heap_elements = None
        # stores best unset labels for each node (tasks are integers or labels?)
        self.heap = None

    def run_algo(self):
        """Execute the labeling algorithm"""

        # is there an unset label associated with each node (using ID) in the updatable priority queue
        self.in_heap = [False for _ in self.nodes_gpr]

        # key associated with each node (using local ID) (array of doubles)
        self.key = [None for _ in self.nodes_gpr]

        # priority queues of unset labels for each node
        # ("for CS only labels at departure" -- meaning it represents keytime when departing the CS?)
        self.unset_labels = [core.PseudoFibonacciHeap() for _ in self.nodes_gpr]

        # list of set (nondominated) labels for each node
        # ("for CS only labels at departure" -- meaning it represents keytime when departing the CS?)
        self.set_labels = [[] for _ in self.nodes_gpr]

        # for each node (by local id), the node in the updatable queue associated with the label currently in the heap
        self.heap_elements = [None for _ in self.nodes_gpr]

        # stores best unset labels for each node (tasks are integers or labels?)
        self.heap = core.PseudoFibonacciHeap()

        # build first label
        first_label = self._build_first_label()
        self.heap_elements[self.node_local_id_dep] = core.HeapElement(
            self.node_local_id_dep)
        self.key[self.node_local_id_dep] = first_label.get_key()
        self.heap.add_task(
            self.heap_elements[self.node_local_id_dep], first_label.get_key())
        self.in_heap[self.node_local_id_dep] = True
        self.unset_labels[self.node_local_id_dep].add_task(
            first_label, first_label.get_key())

        while self.heap:

            # heap element containing the local ID of the node whose key time is smallest
            min_node_f = self.heap.pop_task()
            min_node_local_id = min_node_f.data
            # node id of the aforementioned node
            min_node_id = self.nodes_gpr[min_node_local_id].node_id
            self.in_heap[min_node_local_id] = False
            self.heap_elements[min_node_local_id] = None
            self.key[min_node_local_id] = None

            # Return smallest (wrt key) unset label associated with current node
            label_to_set = self.unset_labels[min_node_local_id].pop_task()

            # compute supporting points of label
            label_to_set = self._compute_supporting_points(label_to_set)
            # check if label dominated by previously set label
            if self._is_dominated(label_to_set, min_node_local_id):
                self._insert_new_node_in_heap(min_node_local_id)
                continue

            # Currently selected label is not dominated

            # if label is a CS, we build the list of labels according to the supp pts
            # of the SOC function
            if (self.nodes_gpr[min_node_local_id].type == core.NodeType.CHARGING_STATION and
                    label_to_set.last_visited_cs != min_node_local_id):
                new_labels = self._build_label_list(label_to_set)
                for new_label in new_labels:
                    self.unset_labels[min_node_local_id].add_task(
                        new_label, new_label.get_key())
                self._insert_new_node_in_heap(min_node_local_id)
                continue

            # if current node is the destination node
            if min_node_local_id == self.node_local_id_arr:
                self.set_labels[min_node_local_id].append(label_to_set)
                break

            # mark current label as set
            self.set_labels[min_node_local_id].append(label_to_set)

            # extend current label
            n_adj_nodes = len(self.adjacency_list[min_node_local_id])
            for k in range(n_adj_nodes):
                next_node_local_id = self.adjacency_list[min_node_local_id][k]
                next_node_id = self.nodes_gpr[next_node_local_id].node_id
                # skip the node if the label cannot be extended
                if not self._can_be_extended_to(label_to_set, next_node_local_id):
                    continue

                # build the new label
                new_label = self._relax_arc(
                    label_to_set,
                    next_node_local_id,
                    self.instance.energy_matrix[min_node_id][next_node_id],
                    self.instance.process_times[next_node_id] + self.instance.time_matrix[min_node_id][next_node_id])

                # if new label exists, modify key associated with node
                if new_label is not None:
                    self.unset_labels[next_node_local_id].add_task(
                        new_label, new_label.get_key())
                    # if we already have something in the heap for the next node
                    if self.in_heap[next_node_local_id]:
                        # if the new label is better than the last one at that node
                        if new_label.get_key() < self.key[next_node_local_id]:
                            # update the key time for it in the heap
                            self.heap.add_task(
                                self.heap_elements[next_node_local_id], new_label.get_key())
                            # and in the self.key reference
                            self.key[next_node_local_id] = new_label.get_key()
                    # if this is the first label for the node
                    else:
                        # add it to the heap
                        self.heap_elements[next_node_local_id] = core.HeapElement(
                            next_node_local_id)
                        self.heap.add_task(
                            self.heap_elements[next_node_local_id], new_label.get_key())
                        self.in_heap[next_node_local_id] = True
                        self.key[next_node_local_id] = new_label.get_key()

            # add min node to the heap
            self._insert_new_node_in_heap(min_node_local_id)

    def _can_be_extended_to(self, curr_label: core.NodeLabel, next_node_local_id: int) -> bool:
        """Can we extend curr_label to node given by next_node_local_id"""

        next_node = self.nodes_gpr[next_node_local_id]

        # Check is for charging stations only
        if next_node.type == core.NodeType.CHARGING_STATION:
            # while scanning nodes of the route backward, we need to encounter
            # the depot, a customer, or a faster charging station before
            # scanning the same charging station
            parents = curr_label.get_path_from_last_customer()
            parent_node = None
            cs_nodes = []
            check = stop = False
            for i in range(len(parents)-1, -1, -1):
                parent_node = self.nodes_gpr[parents[i]]
                if parent_node == next_node:
                    check = True
                    # has not visited a faster CS after first visit
                    stop = True
                    break
                if parent_node.type == core.NodeType.CHARGING_STATION:
                    cs_nodes.append(parent_node)
                else:  # customer or depot
                    stop = True

                if stop:
                    break

            if check:
                for cs_parent_node in cs_nodes:
                    if self.instance.is_cs_faster(cs_parent_node, next_node):
                        # we have visited a faster CS after the first visit to the CS
                        return True
                return False

            return True

        return True

    def _relax_arc(self, curr_label: core.NodeLabel, next_node_local_id: int,
                   energy_arc: float, time_arc: float
                   ) -> core.NodeLabel:
        """Returns the label built by the extension of curr_label to the
        node given by next_node_local_id."""

        max_q = self.instance.max_q

        # going to modify the following
        new_e_consumed_since_last_cs = None
        new_soc_at_last_cs = None
        new_last_visited = None

        # modifications if curr_label is a charging station
        if self.nodes_gpr[curr_label.node_id_for_label].type == core.NodeType.CHARGING_STATION:
            new_e_consumed_since_last_cs = energy_arc
            new_last_visited = curr_label.node_id_for_label
            new_soc_at_last_cs = curr_label.get_first_supp_pt_soc()
        # modifications if cust or depot
        else:
            new_e_consumed_since_last_cs = curr_label.energy_consumed_since_last_cs + energy_arc
            new_last_visited = curr_label.last_visited_cs
            new_soc_at_last_cs = curr_label.soc_arr_to_last_cs

        # compute max soc reachable at next node
        max_soc_at_next = curr_label.get_last_supp_pt_soc() - energy_arc
        # if it's not enough, then we don't extend
        if max_soc_at_next < self.min_energy_at_departure[next_node_local_id]:
            return None

        # compute min soc reachable or needed at next node
        min_soc_at_next = max(self.min_energy_at_departure[next_node_local_id],
                              new_soc_at_last_cs - new_e_consumed_since_last_cs)

        # determine if we need to charge at the last visited CS to reach
        # next node with sufficient SOC
        e_to_charge_at_last_cs = max(0,
                                     self.min_energy_at_departure[next_node_local_id] +
                                     new_e_consumed_since_last_cs -
                                     new_soc_at_last_cs)
        trip_time = curr_label.trip_time + time_arc
        min_time = trip_time

        # if we need to charge some energy at last visited CS
        if e_to_charge_at_last_cs > 0:
            # if there is no prev CS, we can't charge, so it's impossible to extend
            if new_last_visited is None:
                return None

            # if it requires more charge than can be acquired, it's impossible to extend
            if new_soc_at_last_cs + e_to_charge_at_last_cs > max_q:
                return None
            # compute time needed to retroactively charge
            cs_node = self.nodes_gpr[new_last_visited]
            charging_time = self.instance.get_charging_time(cs_node,
                                                            new_soc_at_last_cs, e_to_charge_at_last_cs)
            min_time += charging_time

        # if min time of departure is larger than allowed, it's impossible to extend
        if min_time > self.latest_departure_time[next_node_local_id]:
            return None

        # compute key time
        key_time = None
        # if we don't need to charge after the next node
        if min_soc_at_next > self.min_energy_consumed_after_node[next_node_local_id]:
            key_time = min_time + \
                self.min_travel_time_after_node[next_node_local_id]
        # if we will need to charge
        else:
            key_time = (min_time + self.min_travel_charge_time_after_node[next_node_local_id] -
                        min_soc_at_next/self.max_slope)

        # return a new label for the relaxation to the new node
        return core.NodeLabel(next_node_local_id, key_time, trip_time, new_last_visited,
                         new_soc_at_last_cs, new_e_consumed_since_last_cs, curr_label.supporting_pts,
                         curr_label.slope, time_arc, energy_arc, curr_label, y_intercept=curr_label.y_intercept)

    def _build_label_list(self, curr_label: core.NodeLabel) -> List[core.NodeLabel]:
        """Builds list of labels to extend from the curr_label. Specifically,
        creates one new label for each supporting point of the current SOC
        function in order to explore the possibility of switching over to the
        new CS at that point.
        """

        label_list = []
        curr_local_id = curr_label.node_id_for_label
        curr_node = self.nodes_gpr[curr_local_id]

        # we only split into new labels at charging stations
        if curr_node.type == core.NodeType.CHARGING_STATION:

            # charging function details
            cs_supp_pts = self.instance.get_cf_breakpoints(
                curr_node)  # CS's charging func breakpoints
            # slopes of the CS's charging func segments
            cs_slope = self.instance.get_slope(curr_node)
            # num breakpoints (n bpts -> (n-1) segments)
            cs_n_pts = len(cs_supp_pts[0])

            # current SOC function
            lbl_supp_pts = curr_label.supporting_pts
            lbl_n_pts = curr_label.get_num_supp_pts()

            # iterate over points in the label's SOC function
            for k in range(lbl_n_pts):
                trip_time = lbl_supp_pts[0][k]
                soc_at_cs = lbl_supp_pts[1][k]

                # don't switch if it leads to a time-infeasible path
                if trip_time > self.latest_departure_time[curr_local_id]:
                    continue

                # don't switch if energy is sufficient to finish the route
                if soc_at_cs > self.max_energy_at_departure[curr_local_id]:
                    continue

                # switch only if the new slope is better than the current
                if k < lbl_n_pts - 1:
                    curr_slope = curr_label.slope[k]
                    new_slope = self.instance.get_slope(
                        node=curr_node, soc=max(0, soc_at_cs))
                    if curr_slope >= new_slope:
                        continue

                # don't switch if soc when departing from last CS was sufficient to finish route
                if (curr_label.last_visited_cs is not None and
                        soc_at_cs + curr_label.energy_consumed_since_last_cs >
                        self.max_energy_at_departure[curr_label.last_visited_cs]):
                    continue

                # passed all checks. Will switch to new CS
                # Compute first break pt of charging function above the SOC with which we arrived
                first_k = 0
                if soc_at_cs < 0:
                    raise ValueError("Supporting point has negative energy")
                while first_k < cs_n_pts and cs_supp_pts[1][first_k] <= soc_at_cs:
                    first_k += 1

                e_to_end = self.min_energy_consumed_after_node[curr_local_id]
                # the ones from first_k to the end, plus the arrival point
                n_pts_new = cs_n_pts - first_k + 1
                supp_pts_new = [
                    [None for k in range(n_pts_new)] for _ in range(2)]
                # if there is more than one supp pt, compute slopes of segments, otherwise, it's None
                slope_new = [cs_slope[l-1]
                             for l in range(first_k, cs_n_pts)] if n_pts_new > 1 else None
                supp_pts_new[0][0] = trip_time
                supp_pts_new[1][0] = soc_at_cs

                # compute time to charge
                shift_time = self.instance.get_time_to_charge_from_zero(
                    curr_node, soc_at_cs)
                for supp_pt_idx in range(first_k, cs_n_pts):
                    supp_pts_new[0][supp_pt_idx-first_k+1] = trip_time + \
                        cs_supp_pts[0][supp_pt_idx]-shift_time
                    supp_pts_new[1][supp_pt_idx-first_k +
                                    1] = cs_supp_pts[1][supp_pt_idx]

                # compute key
                if soc_at_cs > e_to_end:
                    key_time = trip_time + \
                        self.min_travel_time_after_node[curr_local_id]
                else:
                    key_time = trip_time + \
                        self.min_travel_charge_time_after_node[curr_local_id] - \
                        soc_at_cs/self.max_slope

                # make new label
                label_list.append(core.NodeLabel(curr_local_id, key_time, trip_time, curr_local_id,
                                            curr_label.soc_arr_to_last_cs, curr_label.energy_consumed_since_last_cs,
                                            supp_pts_new, slope_new, 0, 0, curr_label.parent)
                                  )

        # not a CS. just return the current label
        else:
            label_list.append(curr_label)

        return label_list

    def _insert_new_node_in_heap(self, local_node_id: int):
        # if current node has unset label, insert this node in the heap with a new key
        if self.unset_labels[local_node_id]:
            new_key = self.unset_labels[local_node_id].peek().get_key()
            self.heap_elements[local_node_id] = core.HeapElement(local_node_id)
            self.heap.add_task(self.heap_elements[local_node_id], new_key)
            self.in_heap[local_node_id] = True
            self.key[local_node_id] = new_key

    def _is_dominated(self, label: core.NodeLabel, min_node_local_id: int) -> bool:
        """Is label dominated by any of the set labels at min_node_local_id"""

        for other in self.set_labels[min_node_local_id]:
            if other.dominates(label):
                return True
        return False

    def _compute_supporting_points(self, label: core.NodeLabel) -> core.NodeLabel:
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
            new_supp_pts_temp[1][k] = supp_pts[1][k] - label.energy_last_arc

            # stop at first supp pt with:
            # more SOC than needed or
            # the one that leads to violation of route duration limit
            if (last_k == -1 and
                    (new_supp_pts_temp[1][k] > self.max_energy_at_departure[local_id] or
                     new_supp_pts_temp[0][k] > self.latest_departure_time[local_id])):
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
            if first_k in (0, n_pts):
                # no feasible supporting pts, which shouldn't happen
                raise ValueError(
                    f'No feasible supporting points for label\n{label}')
            # first and last supp pt may be on single segment between two pts
            compute_first_point = True

        # need to compute first point if first_k != 0
        if first_k != 0:
            compute_first_point = True
            first_k -= 1
        # increment one since pts will be considered from first_k to last_k-1
        if last_k != n_pts:
            last_k += 1

        # instantiate things we'll set for the next label
        slope_now = None
        y_int_now = None
        new_supp_pts = None
        # make new supporting points
        if not compute_first_point:
            # all supp pts (up to those with index >= last_k) are feasible
            new_supp_pts = [[new_supp_pts_temp[i][k]
                             for k in range(last_k)] for i in range(2)]

            # if label not reduced to singleton
            if label.slope is not None:
                n_pts_rmd = n_pts - last_k  # number of removed points from the soc function
                # number of segments previously
                n_seg_initial = len(label.slope)
                n_seg_now = n_seg_initial - n_pts_rmd  # number of segments now

                # if number of segments is >0, compute slope and y-intercept
                if n_seg_now > 0:
                    slope_now = [label.slope[k] for k in range(n_seg_now)]
                    y_int_now = [(label.y_intercept[k]-label.energy_last_arc -
                                  label.time_last_arc*label.slope[k]) for k in range(n_seg_now)]

        # need to compute first supp pt on seg [first_k, first_k+1]
        else:
            n_pts_now = last_k - first_k
            new_supp_pts = [[None for k in range(n_pts_now)] for _ in range(2)]
            # compute slope and y-intercept (which must be shifted) of current segment
            slope = label.slope[first_k]
            y_int = label.y_intercept[first_k] - label.energy_last_arc - \
                label.time_last_arc * label.slope[first_k]
            trip_time = (self.min_energy_at_departure[local_id] - y_int)/slope

            new_supp_pts[0][0] = trip_time
            new_supp_pts[1][0] = self.min_energy_at_departure[local_id]
            for k in range(first_k+1, last_k):
                new_supp_pts[0][k-first_k] = new_supp_pts_temp[0][k]
                new_supp_pts[1][k-first_k] = new_supp_pts_temp[1][k]
            # if num segments in label is > 0, compute slope and y-intercept
            if n_pts_now > 0:
                slope_now = [label.slope[k] for k in range(first_k, last_k-1)]
                y_int_now = [(label.y_intercept[k] - label.energy_last_arc -
                              label.time_last_arc*label.slope[k]) for k in range(first_k, last_k-1)]

        # construction complete. return new label
        return core.NodeLabel(local_id, label.key_time, label.trip_time, label.last_visited_cs,
                         label.soc_arr_to_last_cs, label.energy_consumed_since_last_cs, new_supp_pts,
                         slope_now, 0, 0, label.parent, y_int_now)

    def _build_first_label(self) -> core.NodeLabel:
        """Constructs the initial label to kick off the labeling algorithm."""

        supp_pts = [[0], [self.init_soc]]
        energy_to_end = self.min_energy_consumed_after_node[self.node_local_id_dep]
        if self.init_soc >= energy_to_end:
            key_time = self.min_travel_time_after_node[self.node_local_id_dep]
        else:
            key_time = self.min_travel_charge_time_after_node[self.node_local_id_dep]
        return core.NodeLabel(self.node_local_id_dep, key_time, 0, None, self.init_soc,
                         0, supp_pts, None, 0, 0, None)

    def get_optimized_route(self) -> List[Tuple]:
        """Returns the optimal route found by the algorithm if one exists; None otherwise.
        If returned, the optimal route is a list of 2-tuples whose first entry is the node
        ID and second entry is the amount to charge at that node.
        """

        if not self.set_labels[self.node_local_id_arr]:
            return None

        route = []
        label = self.set_labels[self.node_local_id_arr][0]
        path = label.get_path()
        charge_amts = label.get_charging_amounts()
        nodes_path = [self.nodes_gpr[k] for k in path]
        charging_index = 0
        for node in nodes_path:
            if node.type == core.NodeType.CHARGING_STATION:
                route.insert(0, (node.node_id, charge_amts[charging_index]))
                charging_index += 1
            else:
                route.insert(0, (node.node_id, None))
        return route

    def get_objective_value(self) -> float:
        """Returns the key time for the first set label at the destination node
        if a label exists; infinity otherwise.
        """

        if self.set_labels[self.node_local_id_arr]:
            return self.set_labels[self.node_local_id_arr][0].key_time

        return float('inf')

    def solution_found(self) -> bool:
        """Returns True if the destination node has at least one set label;
        False otherwise.
        """

        return len(self.set_labels[self.node_local_id_arr]) > 0
