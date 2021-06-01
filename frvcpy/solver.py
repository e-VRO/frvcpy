"""A module for solving FRVCPs.

This module defines the Solver class for the FRVCP and
offers a main function for execution of the solver from
the command line.

Classes:
    Solver: the class users interact with to solve an FRVCP
"""

import argparse
import copy
import os
import sys
from typing import List, Any, Tuple

import xmltodict

from frvcpy import algorithm
from frvcpy import core
from frvcpy import translator


class Solver():
    """A Solver object for the FRVCP.

    The solution of an FRVCP happens through the Solver class.
    The problem instance, route to be solved for, and the EV's
    initial charge are provided to the constructor. Then the
    solution is generated as follows.

        Typical usage example:
            frvcp_solver = Solver("my-instance.json", route, q_init)
            duration, feasible_route = frvcp_solver.solve()
            frvcp_solver.write_solution("my-solution.xml", "my-instance")

    Attributes:
        instance: the core.FrvcpInstance for the FRVCP to be solved
        solution: output from execution of the labeling algorithm, if
            available (see solve()'s docstring for more info on the algorithm's
            output)
    """

    def __init__(self, instance, route: List[int], q_init: float, multi_insert: bool = True, check_tri: bool = False):
        """Initiates a Solver for the FRVCP.

        Args:
            instance: either the filename (str) of a problem instance or a
                dictionary containing the required info. Typically the problem
                instance is already in a compliant JSON file. However, if a
                filename ending in ".xml" is passed, it is assumed to be a
                VRP-REP instance, and it will be translated.
            route: A list of integers representing the fixed route to be
                traveled by the vehicle
            q_init: The energy with which the EV begins the route
            multi_insert: A boolean specifying whether the EV is allowed to
                visit multiple CSs between stops in the route
            check_tri: A boolean specifying whether to verify that the triangle
                inequality holds for the instance's time and energy matrices.
                Default is False, since this adds significant computation time.

        """
        # if passed an XML file, attempt to translate it
        if isinstance(instance, (str)) and instance[-4:] == ".xml":
            print("INFO: Passed instance is an XML file. " +
                  "Assuming it is a VRP-REP instance and attempting to translate it...")
            instance = translator.translate(instance)
            print("INFO: Instance translated.")

        self.instance = core.FrvcpInstance(instance, check_tri)
        self._q_init = q_init
        self._route = route
        self._multi_insert = multi_insert

        self.solution = None

    def solve(self) -> Tuple[float, List[Any]]:
        """Solve the FRVCP defined by the instance, route, and intial energy
        provided to the constructor.

        Returns:
            A tuple (obj,feas_route), where obj (float) is the duration of the
            energy-feasible route (if it exists; inf otherwise) and feas_route
            is a list of stops s_i defining the energy-feasible route. Stops
            s_i=(node_i,charge_amt_i) are themselves tuples consisting of the
            node ID (int) of the stop and the amount (float) to charge there (None if
            node_i is not a CS).
        """

        max_detour_charge_time = self._compute_max_avail_time_detour_charge()  # float

        # below lists (until nodes_gpr) are of length (num_nodes_in_G)
        # List[float]
        min_soc_at_departure = self._compute_min_soc_at_departure()

        # list[float]
        max_allowed_soc_at_arrival = self._compute_max_soc_at_arrival()

        # length of direct connect is actually len(route)-1
        possible_direct_connect = self._compute_possible_direct_connect(
            min_soc_at_departure, max_allowed_soc_at_arrival)  # list[bool]

        possible_cs_connect = self._compute_possible_cs_connections(
            min_soc_at_departure, max_allowed_soc_at_arrival)  # list[list[list[bool]]]

        possible_cs_detour = self._compute_possible_cs_detour(
            max_detour_charge_time, not self._multi_insert)  # list[list[bool]]

        possible_cs_link = self._compute_possible_cs_link(
            max_detour_charge_time, not self._multi_insert)  # list[list[list[bool]]]

        node_local_id_dep = 0  # int

        node_local_id_arr = len(self._route)-1  # int

        # starting here, lists are of length len(num_nodes_in_GPrime)
        nodes_gpr = self._build_gpr_nodes()  # list[core.Node]

        adjacencies = self._compute_adjacencies(nodes_gpr,
                                                possible_direct_connect,
                                                possible_cs_connect,
                                                possible_cs_detour,
                                                possible_cs_link)  # list[list[int]]

        # all below lists are of length len(nodes_gpr)
        (
            min_travel_time_after_node,
            min_energy_consumed_after_node,
            min_travel_charge_time_after_node,
            latest_departure_time,
            min_energy_at_departure,
            max_energy_at_departure
        ) = self._compute_bounds(nodes_gpr)  # list[list[float]]

        # initialize algorithm
        label_algo = algorithm.FrvcpAlgo(
            self.instance,
            self._q_init,
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
        label_algo.run_algo()

        # store solution
        self.solution = label_algo.get_objective_value(), label_algo.get_optimized_route()

        # return results
        return copy.deepcopy(self.solution)

    def _direct_route_travel_time(self):
        return sum([self.instance.time_matrix[i][j] for i,j in zip(self._route[:-1],self._route[1:])])
    
    def write_solution(self, filename: str, instance_name: str) -> None:
        """Writes the current available solution to file.

        Args:
            filename: The name of the file to which the solution should be written
            instance_name: The name of the instance that the solution corresponds to

        """

        if self.solution is None:
            raise ValueError(
                "Trying to get solution for problem that has not yet been solved.")

        if self.solution[1] is None:
            raise ValueError("No feasible solution was found.")

        route_nodes = self._get_xml_unparseable_route()

        output_sol = {
            "solution": {
                "@instance": instance_name,
                "route": {
                    "@id": 0,
                    "@initialcharge": self._q_init,
                    "node": route_nodes
                }
            }
        }

        with open(filename, 'w') as sol_file:
            sol_file.write(xmltodict.unparse(output_sol, pretty=True))

    def _get_xml_unparseable_route(self) -> List:
        """Returns an XML-unparseable route.

        For the VRP-REP solution format.

        Returns:
            A list of dicts with ID keys for stops and charge keys for charge
                amounts

        """

        if self.solution is None:
            raise ValueError(
                "Trying to get solution for problem that has not yet been solved.")

        result = []
        for stop in self.solution[1]:
            # if stop is not a CS
            if stop[1] is None:
                result.append({
                    '@id': stop[0]
                })
            # stop is a CS, needs a charge key
            else:
                result.append({
                    '@id': stop[0],
                    'charge': stop[1]
                })

        return result

    def _direct_route_without_recharge(self) -> bool:
        """Returns True if the EV can travel the route directly without recharging."""

        return self._q_init >= sum(
            [self.instance.energy_matrix[i][j] for i, j in zip(self._route[:-1], self._route[1:])])

    def _compute_max_avail_time_detour_charge(self) -> float:
        """Max amount of time we have to detour and charge (in consideration
        of the time duration).
        """

        return self.instance.t_max - \
            sum([self.instance.time_matrix[self._route[i]][self._route[i+1]]
                 for i in range(len(self._route)-1)]) - \
            sum([self.instance.process_times[stop] for stop in self._route])

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
            (max_soc_at_arrival[self._route[i]] -
             self.instance.energy_matrix[self._route[i]][self._route[i+1]] >=
             min_soc_at_departure[self._route[i+1]])
            for i in range(len(self._route)-1)]

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

        result = [[[None, None] for cs in range(
            self.instance.n_cs)] for i in range(len(self._route)-1)]
        for i in range(len(self._route)-1):
            init_loc = self._route[i]
            next_loc = self._route[i+1]
            for i_cs in range(self.instance.n_cs):

                cs_id = self.instance.cs_details[i_cs]['node_id']

                # check we can arrive with nonzero energy (can recharge at CS if reqd)
                result[i][i_cs][0] = (max_soc_at_arrival[init_loc] -
                                      self.instance.energy_matrix[init_loc][cs_id] >= 0)

                result[i][i_cs][1] = (self.instance.max_q -
                                      self.instance.energy_matrix[cs_id][next_loc] >=
                                      min_soc_at_departure[next_loc])
        return result

    def _compute_possible_cs_detour(self,
                                    max_detour_charge_time: float,
                                    only_one: bool = False
                                    ) -> List[List[bool]]:
        """Is it possible to go to visit each CS between stops in the route?
        Shape: len(self.route)-1 x num_cs:
        [i][k] = can we visit the kth CS between stops i and i+1 in the route?
        """

        result = [[False for i_cs in range(self.instance.n_cs)]
                  for i in range(len(self._route)-1)]
        for i in range(len(self._route)-1):
            stop = self._route[i]
            next_stop = self._route[i+1]
            for i_cs in range(self.instance.n_cs):
                cs = self.instance.cs_details[i_cs]["node_id"]
                detour_time = self.instance.time_matrix[stop][cs] + \
                    self.instance.time_matrix[cs][next_stop] - \
                    self.instance.time_matrix[stop][next_stop]
                if only_one:  # take into account the minimum charge time as well
                    min_charge_amt = self.instance.energy_matrix[stop][cs] + \
                        self.instance.energy_matrix[cs][next_stop] - \
                        self.instance.energy_matrix[stop][next_stop]
                    if min_charge_amt > self.instance.max_q:
                        detour_time = float('inf')
                    else:
                        detour_time += self.instance.get_charging_time(
                            self.instance.nodes_g[cs], 0, min_charge_amt)
                if detour_time <= max_detour_charge_time:
                    result[i][i_cs] = True
        return result

    def _compute_possible_cs_link(self,
                                  max_detour_charge_time: float,
                                  only_one=False
                                  ) -> List[List[List[bool]]]:
        """Can two CSs be connected between stops in the route?
        Shape is: len(self.route)-1 x numcss x numcss
        [i][j][k] = can we connect the jth CS to the kth CS between stops i and i+1 in the route?
        """

        result = [[[False for cs2_idx in range(self.instance.n_cs)] for cs1_idx in range(
            self.instance.n_cs)] for i in range(len(self._route)-1)]

        # if we can only insert one CS between stops, then no connections are possible
        if only_one:
            return result

        # otherwise, compute connections
        for i in range(len(self._route) - 1):
            curr_stop = self._route[i]
            next_stop = self._route[i+1]
            for cs1_idx in range(self.instance.n_cs):
                cs1 = self.instance.cs_details[cs1_idx]["node_id"]
                for cs2_idx in range(self.instance.n_cs):
                    if cs1_idx == cs2_idx:
                        continue
                    cs2 = self.instance.cs_details[cs2_idx]["node_id"]
                    if (
                            (self.instance.energy_matrix[cs1][cs2] <= self.instance.max_q) and
                            (self.instance.time_matrix[curr_stop][cs1] +
                             self.instance.time_matrix[cs1][cs2] +
                             self.instance.time_matrix[cs2][next_stop] -
                             self.instance.time_matrix[curr_stop][next_stop] <= max_detour_charge_time)
                    ):
                        result[i][cs1_idx][cs2_idx] = True
        return result

    def _build_gpr_nodes(self) -> List[core.Node]:
        """Build the list of nodes that define the graph G'."""

        return ([self.instance.nodes_g[self._route[i]] for i in range(len(self._route))] +
                ((self.instance.get_cs_nodes())*(len(self._route)-1)))

    def _compute_adjacencies(self,
                             nodes_gpr: List[core.Node],
                             possible_direct_connect: List[bool],
                             possible_cs_connect: List[List[List[bool]]],
                             possible_cs_detour: List[List[bool]],
                             possible_cs_link: List[List[List[bool]]]
                             ) -> List[List[int]]:
        """Compute the adjacency list for the nodes in G prime."""
        adjs = [[] for _ in nodes_gpr]
        # add direct connections and one-off detours
        for i in range(len(self._route)-1):
            if possible_direct_connect[i]:
                adjs[i].append(i+1)
            for j in range(len(self._route)+i*self.instance.n_cs,
                           len(self._route)+(i+1)*self.instance.n_cs):
                i_cs = (j - len(self._route)) % self.instance.n_cs
                if possible_cs_detour[i][i_cs]:
                    if possible_cs_connect[i][i_cs][0]:
                        adjs[i].append(j)
                    if possible_cs_connect[i][i_cs][1]:
                        adjs[j].append(i+1)
        # add intra-cs links
        for i in range(len(self._route)-1):
            begin_idx = len(self._route)+i*self.instance.n_cs
            end_idx = len(self._route)+(i+1)*self.instance.n_cs
            for mid_idx1 in range(begin_idx, end_idx):
                i_cs1 = (mid_idx1 - len(self._route)) % self.instance.n_cs
                for mid_idx2 in range(begin_idx, end_idx):
                    i_cs2 = (mid_idx2 - len(self._route)) % self.instance.n_cs
                    if possible_cs_link[i][i_cs1][i_cs2]:
                        adjs[mid_idx1].append(mid_idx2)

        return adjs

    def _compute_bounds(self, nodes: List[core.Node]) -> Tuple[
            List[float], List[float], List[float], List[float], List[float], List[float]]:
        """Produces a 6-tuple of bounds that get used in the labeling algorithm:
         1. min_travel_time_after_node: minimum time left in the route after the vehicle departs
         2. min_energy_consumed_after_node: minimum amount of energy required to
              traverse the rest of the route
         3. min_travel_charge_time_after_node: minimum time left in the route if it
              were to travel it directly and consume any additional required energy
              without detouring at the fastest rate possible
         4. latest_departure_time: latest time at which the vehicle can depart
         5. min_energy_at_departure: min allowable energy at departure
         6. max_energy_at_departure: max energy at departure (either Q or min needed to reach end of route)
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
        next_id = self._route[-1]

        # set entries for last stop in route
        min_travel_time_after_node[len(self._route)-1] = time
        min_energy_consumed_after_node[len(self._route)-1] = energy
        min_travel_charge_time_after_node[len(self._route)-1] = time_charge

        # set entries for all others
        for i in range(len(self._route)-2, -1, -1):
            for j in range(len(self._route)+i*self.instance.n_cs,
                           len(self._route)+(i+1)*self.instance.n_cs):
                curr_id = nodes[j].node_id
                min_travel_time_after_node[j] = time + \
                    self.instance.time_matrix[curr_id][next_id] + \
                    self.instance.process_times[next_id]
                min_energy_consumed_after_node[j] = energy + \
                    self.instance.energy_matrix[curr_id][next_id]
                min_travel_charge_time_after_node[j] = (time_charge +
                                                        (self.instance.time_matrix[curr_id][next_id] +
                                                         self.instance.process_times[next_id] +
                                                         self.instance.energy_matrix[curr_id][next_id] /
                                                         self.instance.max_slope))
            curr_id = self._route[i]
            time += self.instance.time_matrix[curr_id][next_id] + \
                self.instance.process_times[next_id]
            energy += self.instance.energy_matrix[curr_id][next_id]
            time_charge += (self.instance.time_matrix[curr_id][next_id] +
                            self.instance.process_times[next_id] +
                            self.instance.energy_matrix[curr_id][next_id] /
                            self.instance.max_slope)
            min_travel_time_after_node[i] = time
            min_energy_consumed_after_node[i] = energy
            min_travel_charge_time_after_node[i] = time_charge
            next_id = curr_id

        # endregion

        # region bounds on time and charge when departing nodes
        for i in range(len(nodes)):
            curr_id = nodes[i].node_id
            min_energy_to_charge = min_energy_consumed_after_node[i] - \
                self.instance.max_q
            latest_departure_time[i] = self.instance.t_max - \
                min_travel_time_after_node[i]
            if min_energy_to_charge > 0:
                latest_departure_time[i] -= min_energy_to_charge / \
                    self.instance.max_slope
            min_energy_at_departure[i] = self.instance.get_min_energy_to_cs(
                curr_id)
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


def main():
    """Solves an FRVCP."""

    parser = argparse.ArgumentParser(description="Solves an FRVCP")
    # grab the optional arguments to re-append after the required named arguments
    optional = parser._action_groups.pop()
    required = parser.add_argument_group('required arguments')

    required.add_argument(
        '-i',
        '--instance',
        type=str,
        required=True,
        help='Filename for the frvcpy-compatible problem instance')
    required.add_argument(
        '-r',
        '--route',
        type=str,
        required=True,
        help='Comma-separated list of node IDs defining the route to be made energy-feasible')
    required.add_argument(
        '-q',
        '--qinit',
        type=float,
        required=True,
        help='The initial energy of the EV traveling the route')

    num_insert_parser = parser.add_mutually_exclusive_group(required=False)
    num_insert_parser.add_argument('--multi', dest='multi_insert', action='store_true',
                                   help='Allow multiple CSs to be inserted between stops (default)')
    num_insert_parser.add_argument('--one', dest='multi_insert', action='store_false',
                                   help='Allow only one CS to be inserted between stops')
    parser.set_defaults(multi_insert=True)

    optional.add_argument(
        "-c",
        "--check-tri",
        action="store_true",
        help="Check that the triangle inequality holds for the passed instance")
    optional.add_argument(
        "-w",
        "--write",
        action="store_true",
        help="Write output to file (specify name with -o NAME)")
    optional.add_argument(
        "-o",
        "--output",
        type=str,
        help="Name of file to which to write solution (with -w flag). Default")

    parser._action_groups.append(optional)  # re-append the optional arguments

    args = parser.parse_args()

    # if an output file was specified, then writing results to file
    if args.output:
        args.write = True

    # solve FRVCP
    route = [int(stop) for stop in args.route.split(',')]
    frvcp_solver = Solver(args.instance, route, args.qinit, args.multi_insert, args.check_tri)
    duration, feas_route = frvcp_solver.solve()

    if args.write:

        instance_wo_ext = os.path.splitext(args.instance)[0]

        # default filename if none provided
        if args.output is None:
            route_substr = ''.join(str(stop) for stop in route[:4])
            q_substr = int(args.qinit)
            args.output = f"{instance_wo_ext}-r{route_substr}-q{q_substr}.xml"

        # infer instance name from args
        instance_name = os.path.split(instance_wo_ext)[1]

        frvcp_solver.write_solution(args.output, instance_name=instance_name)
        print(f"INFO: Solution written to file: {args.output}")

    print(f"Duration: {duration:.4}")
    print(f"Energy-feasible route:\n{feas_route}")

    sys.exit(0)


if __name__ == "__main__":
    main()
