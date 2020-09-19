"""This module defines objects used in the execution of
the labeling algorithm from algorithm.py.
"""

from enum import Enum
import heapq
import itertools
import json
from typing import List, Any, Tuple


class NodeType(Enum):
    """Nodes are either the depot (0), a customer (1), or a charging station (2)."""

    DEPOT = 0
    CUSTOMER = 1
    CHARGING_STATION = 2


class Node():
    """Defines a node for the graph underlying an FRVCP.
    
    Attributes:
        node_id: The (int) ID of the node in the instance
        name: A name (str) for the node
        type: the core.NodeType of the node
    """

    def __init__(self, node_id: int, name: str, node_type: NodeType):

        self.node_id = node_id
        self.name = name
        self.type = node_type

    def __str__(self):
        
        return f'({self.node_id}; {self.type})'


class HeapElement():
    """Represents an element in the labeling algorithm's heap.

    Since the data type used for heap nodes in the labeling
    algorithm is just int, this object is largely unnecessary.
    We keep it here to maintain consistency.
    """

    def __init__(self, data: Any):
        self.data = data


class PseudoFibonacciHeap():
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
        self._counter = itertools.count()     # unique sequence count

    def __bool__(self):
        return self.peek() is not None

    def add_task(self, task: Any, priority: Tuple[float, float] = 0):
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
            _, _, task = heapq.heappop(self._pq)
            if task is not self._REMOVED:
                del self._entry_finder[task]
                return task
        raise KeyError('pop from an empty priority queue')

    def peek(self) -> Any:
        """Returns the lowest priority task without removing (popping) it.
        Returns None if empty.
        """

        i = 0
        while i < len(self._pq):
            if self._pq[i][2] is not self._REMOVED:
                return self._pq[i][2]

            i += 1
        return None


class NodeLabel():
    """Class defining a label for the labeling algorithm of
    Froger (2018) for the fixed-route vehicle charging problem.
    """

    def __init__(self, node_id_for_label: int, key_time: float, trip_time: float,
                 last_visited_cs: int, soc_arr_to_last_cs: float,
                 energy_consumed_since_last_cs: float,
                 supporting_pts: List[List[float]], slope: List[float],
                 time_last_arc: float,
                 energy_last_arc: float, parent, y_intercept: List[float] = None
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
        self.y_intercept = self._compute_y_intercept(
        ) if y_intercept is None else y_intercept
        self.n_pts = len(self.supporting_pts[0])

    def get_key(self):
        """Returns the key associated with the label."""

        return (self.key_time,
                float('inf') if self.supporting_pts[1][0] == 0 else 1/self.supporting_pts[1][0])

    def _compute_y_intercept(self) -> List[float]:
        """Provides slopes for the segments in self.supporting_pts"""

        if self.slope is None:
            return None

        return [(self.supporting_pts[1][b]-self.slope[b]*self.supporting_pts[0][b])
                for b in range(len(self.slope))]

    def dominates(self, other) -> bool:
        """Does this label dominate other"""

        # does the other label have a lesser trip_time?
        if self.trip_time > other.trip_time:
            return False

        # energy dominance (larger reachable SOC)
        if self.supporting_pts[1][-1] < other.supporting_pts[1][-1]:
            return False

        # SOC dominance
        # 1) supp pts of this SOC function
        for k in range(self.n_pts):
            soc_other = other.get_soc(self.supporting_pts[0][k])
            if self.supporting_pts[1][k] < soc_other:
                return False

        # 2) supp pts of the other SOC function
        for k in range(other.n_pts):
            soc = self.get_soc(other.supporting_pts[0][k])
            if soc < other.supporting_pts[1][k]:
                return False

        return True

    def get_soc(self, time: float) -> float:
        """Given a time, what is the corresponding SOC from
        the label's supporting points."""

        if time < self.trip_time:
            return -float('inf')

        if time >= self.supporting_pts[0][-1]:
            return self.supporting_pts[1][-1]

        low = 0
        high = self.n_pts-1
        while low + 1 < high:
            mid = (low + high) // 2
            if self.supporting_pts[0][mid] < time:
                low = mid
            else:  # self.supporting_pts[0][mid] >= time
                high = mid

        return self.slope[low] * time + self.y_intercept[low]

    def get_first_supp_pt_soc(self) -> float:
        """Get the min charge over all supporting points."""

        return self.supporting_pts[1][0]

    def get_last_supp_pt_soc(self) -> float:
        """Get the max charge over all supporting points."""

        return self.supporting_pts[1][-1]

    def get_num_supp_pts(self) -> int:
        """Returns the number of supporting points."""

        return len(self.supporting_pts[0])

    def get_path(self) -> List[int]:
        """Get the path from this label backwards to the origin."""

        path = []
        curr_parent = self
        while curr_parent is not None:
            path.append(curr_parent.node_id_for_label)
            curr_parent = curr_parent.parent
        return path

    def get_path_from_last_customer(self) -> List[int]:
        """Provides a list of the node IDs that the vehicle has
        visited since the last time it either a) visited a customer,
        b) visited a depot, or c) visited the node at which it
        currently resides.
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
        """For the path traveled to get to this label, the
        amounts that it has recharged along the way.

        Returns these amounts as a list of floats.
        """

        if self.last_visited_cs is None:
            return []  # no visits to CS

        # charge amount at last visited CS
        charge_amts = [(self.energy_consumed_since_last_cs +
                        self.get_first_supp_pt_soc() -
                        self.soc_arr_to_last_cs)]

        # computation of other charge amounts (if any)
        curr_label = self
        while True:
            s_last_vis_cs = curr_label.last_visited_cs

            stop = False
            while not stop:
                charge_reqd = (curr_label.energy_consumed_since_last_cs +
                               curr_label.get_first_supp_pt_soc() -
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
        str_parts = [
            f"---- Label for node {self.node_id_for_label}",
            f"keyTime = {self.key_time}",
            f"tripTime = {self.trip_time}",
            f"timeLastArc = {self.time_last_arc}",
            f"energyLastArc = {self.energy_last_arc}",
            f"lastVisitedCS = {self.last_visited_cs}",
            f"socAtArrLastCS = {self.soc_arr_to_last_cs}",
            f"energyConsumedSinceLastCS = {self.energy_consumed_since_last_cs}",
            "Supporting points",
            str(self.supporting_pts[0]),
            str(self.supporting_pts[1])
        ]
        if self.slope is not None:
            str_parts.extend([
                "Slope",
                str(self.slope),
                "Intercept",
                str(self.y_intercept)])
        str_parts.extend([
            "Path",
            str(self.get_path())])
        return '\n'.join(str_parts)

    # region comparable methods
    def __hash__(self):
        return hash((self.node_id_for_label, self.key_time, self.trip_time,
                     None if self.parent is None else self.parent.node_id_for_label,
                     self.supporting_pts[0][0], self.supporting_pts[1][0]))

    def compare_to(self, other) -> int:
        """Returns -1 if self is a "better" label than other, 1 if vice versa, 0 if the same"""

        if self.key_time < other.key_time:
            return -1
        if self.key_time > other.key_time:
            return 1

        diff = other.supporting_pts[1][0]-self.supporting_pts[1][0]
        if diff > 0.0:
            return 1
        if diff < 0.0:
            return -1
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


class FrvcpInstance():
    """An frvcpy-compliant problem instance.

    Notes:
        1. The triangle inequality must hold for both the energy and time
            matrices.
        2. It is generally suggested that nodes in the instance be ordered
            such that a depot comes first, then customers, then CSs.

    Attributes:
        energy_matrix: Square matrix (List of Lists of numbers) indicating the
            energy required to travel between nodes in the instance
        time_matrix: Square matrix (List of Lists of numbers) indicating the
            time required to travel between nodes in the instance
        process_times: List of numbers indicating the processing time at each
            node (assumed to be 0 for all nodes if not specified)
        n_nodes_g: Integer indicating the number of nodes in the instance
        max_q: The maximum energy that can be stored in the EV's battery
        t_max: The maximum duration of a route (taken to be 6e9 if not
            specified)
        cs_bkpt_info: Dictionary with a key for each CS type/technology. Values
            are the breakpoints defining the (convex) piecewise linear charging
            function, given by two Lists of numbers: "time" and "charge"
        cs_details: List of dicts, each of which contains a field for 'node_id'
            and 'cs_type'
        n_cs: Number of CSs in the instance
        type_to_supp_pts: Dictionary mapping CS types to the breakpoints
            defining their charging functions
        type_to_slopes: Dictionary mapping CS types to the slopes of the
            piecewise linear segments defining their charging functions
        type_to_yints: Dictionary mapping CS types to the y-intercepts of
            the piecewise linear segments defining their charging functions
        max_slope: The maximum charging rate across all CS types in the instance
        cs_id_to_type: Dictionary mapping (CS) node IDs to their CS type
        nodes_g: List of core.Node objects for the nodes in the instance

    """

    def __init__(self, instance, check_tri: bool = False):
        """Instantiates an FRVCP problem instance.
        
        Args:
            instance: either the filename (str) of a compatible problem instance
                or a dictionary containing the required info.
            check_tri: whether to verify that the triangle inequality holds for
                the instance's energy and time matrices.

        """
        # if string, assumed to be filename
        if isinstance(instance, str):
            with open(instance) as instance_file:
                instance = json.load(instance_file)

        self._store_instance_parameters(instance, check_tri)

    def _store_instance_parameters(self, instance, check_tri: bool = False):
        # [i][j] are indices in g, not gprime
        self.energy_matrix = instance["energy_matrix"]
        self.time_matrix = instance["time_matrix"]
        if check_tri:
            if not self._triangle_inequality_holds(self.energy_matrix):
                raise ValueError("The triangle inequality does not hold for the instance's energy_matrix.")
            if not self._triangle_inequality_holds(self.time_matrix):
                raise ValueError("The triangle inequality does not hold for the instance's time_matrix.")

        self.process_times = instance["process_times"] if "process_times" in instance else [
            0 for _ in self.energy_matrix]
        # number of nodes in the underlying graph G
        self.n_nodes_g = len(self.process_times)
        self.max_q = instance["max_q"]
        # come onnnn, 6 billion!
        self.t_max = instance["t_max"] if "t_max" in instance else 6e9
        # list of dicts with charging function breakpoints
        cs_bkpt_info_list = instance["breakpoints_by_type"]
        # use the cs type as the key
        self.cs_bkpt_info = {v['cs_type']: v for v in cs_bkpt_info_list}
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

    def _triangle_inequality_holds(self, matrix: List[List[float]]) -> bool:
        """Verifies that the triangle inequality holds for matrix."""

        # Ensure that the matrix is square
        assert all(len(row) == len(matrix) for row in matrix), "Matrix must be square."

        node_idxs = range(len(matrix))
        
        for i in node_idxs:
            for j in node_idxs:
                if any(matrix[i][k] + matrix[k][j] < matrix[i][j] for k in node_idxs):
                    return False
        
        return True
    
    def get_min_energy_to_cs(self, node_id: int) -> float:
        """Calculates the amount of energy needed to get from node node_id to
        the nearest CS.
        """

        return 0.0 if node_id in self.cs_id_to_type \
            else min([self.energy_matrix[node_id][cs_id] for cs_id in self.cs_id_to_type])

    def get_cs_nodes(self) -> List[Node]:
        """Returns the list of CS nodes."""

        return [node for node in self.nodes_g if node.type == NodeType.CHARGING_STATION]

    def _make_nodes(self) -> List[Node]:
        """Returns a list of Nodes, where each node corresponds to a location (as given
        by the length of the energy matrix). Nodes are listed as customers unless they
        are specifically noted as being CSs in the instance.
        """

        return [
            Node(i, f'node-{i}',
                 (NodeType.CHARGING_STATION if i in self.cs_id_to_type else NodeType.CUSTOMER))
            for i in range(len(self.energy_matrix))]

    def _make_type_to_supp_pts(self):
        """Returns a dictionary with a key for each CS type, where the values are
        the breakpoints defining the charging function for that CS type.
        """

        return {cs_type: [pts["time"], pts["charge"]] for cs_type, pts in self.cs_bkpt_info.items()}

    def _make_type_to_slopes(self):
        """Returns a dictionary with a key for each CS type, where the values are
        the slopes of the linear segments of the charging function for that CS type.
        """

        return {
            cs_type: [
                ((arr[1][i+1] - arr[1][i]) / (arr[0][i+1] - arr[0][i]))
                for i in range(len(arr[0])-1)]
            for cs_type, arr in self.type_to_supp_pts.items()}

    def _make_type_to_yintercepts(self):
        """Returns a dictionary with a key for each CS type, where the values are
        the y-intercepts of the linear segments defining the charging function for that CS type.
        """

        return {
            cs_type: [  # b = y-mx
                (self.type_to_supp_pts[cs_type][1][i] -
                 self.type_to_slopes[cs_type][i] * self.type_to_supp_pts[cs_type][0][i])
                for i in range(len(arr[0])-1)]
            for cs_type, arr in self.type_to_supp_pts.items()}

    def _make_cs_id_to_type_map(self):
        """Returns a dictionary with a key for each CS node ID whose value is
        the CS type of that node.
        """

        return {cs["node_id"]: cs["cs_type"] for cs in self.cs_details}

    def _compute_max_slope(self):
        """Returns the maximum charging rate across all CSs"""

        return max([slopes[0] for slopes in self.type_to_slopes.values()])

    def is_cs_faster(self, node1: Node, node2: Node) -> bool:
        """True if node1 is a faster CS type than node2, False otherwise"""

        return (self.type_to_slopes[self.cs_id_to_type[node1.node_id]][0] >
                self.type_to_slopes[self.cs_id_to_type[node2.node_id]][0])

    def get_cf_breakpoints(self, cs_node: Node) -> List[List[float]]:
        """Get the breakpoints in the charging function of `node`'s CS type"""

        return self.type_to_supp_pts[self.cs_id_to_type[cs_node.node_id]]

    def _get_cf_segment_idx(self, cs_type, value, axis) -> int:
        """Axis is 1 for charge and 0 for time."""

        idx = 0
        while not (self.type_to_supp_pts[cs_type][axis][idx] <=
                   value < self.type_to_supp_pts[cs_type][axis][idx+1]):

            idx += 1
            # for the last segment, check its upper limit
            if idx == len(self.type_to_supp_pts[cs_type][axis])-1:
                # if it's equal to the upper limit, return the index for the last segment
                if value == self.type_to_supp_pts[cs_type][axis][idx]:
                    return idx-1
                # otherwise, it was a bad request
                raise ValueError(
                    f'Request out of bounds for segment index. Value passed: {value}')
        return idx

    def get_slope(self, node: Node, soc: float = None):
        """Returns the charging rate at energy level `soc` at `node`.
        If `soc` is None, returns the charging function for `node`
        """

        # if node passed but no soc, return slopes of node's charging function
        if soc is None:
            return self.type_to_slopes[self.cs_id_to_type[node.node_id]]

        # otherwise, return the slope of the segment on which the soc lies
        return self.type_to_slopes[self.cs_id_to_type[node.node_id]][
            self._get_cf_segment_idx(self.cs_id_to_type[node.node_id], soc, 1)]

    def get_charging_time(self,
                          node: Node,
                          starting_energy: float,
                          acquired_energy: float
                          ):
        """How long does it take to charge from starting_energy to acquired_energy at node"""

        return self.get_time_to_charge_from_zero(node, starting_energy+acquired_energy) - \
            self.get_time_to_charge_from_zero(node, starting_energy)

    def get_time_to_charge_from_zero(self, node: Node, soc: float):
        """How long does it take to charge level `soc` at node.
        node should be a CS
        """

        seg_idx = self._get_cf_segment_idx(
            self.cs_id_to_type[node.node_id], soc, 1)
        return (soc - self.type_to_yints[self.cs_id_to_type[node.node_id]][seg_idx]) / \
            self.type_to_slopes[self.cs_id_to_type[node.node_id]][seg_idx]
