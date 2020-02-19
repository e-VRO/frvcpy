"""A translator to make FRVCP instances from VRP-REP instances.

This module provides support for instance translation from the VRP-REP format
to a format compliant with frvcpy.

Functions:
    translate: responsible for translating an instance

Typical usage:
    # return a dict to pass to Solver:
    translate("vrprep-instance.xml")
    # writes translated instance to file
    translate("vrprep-instance.xml", "my-new-frvcp-instance.json")

More on the VRP-REP instance format: http://www.vrp-rep.org/
More on the frvcpy instance format: the schema in the instances directory of
    frvcpy's homepage: github.com/e-VRO/frvcpy
"""

import argparse
import json
import math
from typing import Any, Optional
import sys

import xmltodict


def _dist(i_node, j_node, dist_type) -> float:
    """Returns the distance between two nodes.

    dist_type is either 'manhattan' or 'euclidean'
    """

    if dist_type == 'manhattan':
        return (abs(float(i_node['cx']) - float(j_node['cx'])) +
                abs(float(i_node['cy']) - float(j_node['cy'])))

    # otherwise assume euclidean
    return math.sqrt(
        (float(i_node['cx']) - float(j_node['cx']))**2 +
        (float(i_node['cy']) - float(j_node['cy']))**2)


def _t(i_node, j_node, speed, dist_type) -> float:
    """Returns the travel time between two nodes.

    dist_type is either 'manhattan' or 'euclidean'
    """

    return _dist(i_node, j_node, dist_type)/speed


def _e(i_node, j_node, consump_rate, dist_type) -> float:
    """Returns the energy required to travel between two nodes.

    dist_type is either 'manhattan' or 'euclidean'
    """

    return _dist(i_node, j_node, dist_type)*consump_rate


def _get_type_to_speed(cfs):
    """Given a list of charging functions, returns an object whose keys
    are the CS types and values are speed rank.

    Speed rank is a CS type's (0-indexed) position in the ordered list of fastest CS types.
    """

    # compute max charge rates by type
    result = [{
        'cs_type': cf['@cs_type'],
        'max_rate': (
            (float(cf['breakpoint'][1]['battery_level'])-float(cf['breakpoint'][0]['battery_level'])) /
            (float(cf['breakpoint'][1]['charging_time']) -
             float(cf['breakpoint'][0]['charging_time']))
        )} for cf in cfs]

    # assign each type its speed rank (lowest = fastest --> highest = slowest)
    result = sorted(result, key=lambda x: x['max_rate'], reverse=True)
    for i, entry in enumerate(result):
        entry.update({'speed_rank': i})

    # return dict type:speed_rank
    return {cf['cs_type']: cf['speed_rank'] for cf in result}


def _get_precision_type(network_el):
    """Given a network element from a VRP-REP instance, returns its precision type:
    floor, ceil, or decimals. If no such precision type is present, returns None.
    """

    if 'decimals' in network_el:
        return 'decimals'
    if 'floor' in network_el:
        return 'floor'
    if 'ceil' in network_el:
        return 'ceil'
    return None


def _warn_unused_els(instance_el):
    """Warn about unused information from the instance."""

    unused_els = ['resources', 'drivers']
    for unused_el in unused_els:
        if unused_el in instance_el:
            print(f"WARNING: Ignoring \'{unused_el}\' element.")


def _check_reqd_ev_info(ev_el):
    """Check the EV element for required information."""

    reqd_in_profile = ['speed_factor', 'custom']
    for req in reqd_in_profile:
        if req not in ev_el:
            raise KeyError('Instance missing required information: '+
                           f'{req} not found in the EV\'s vehicle_profile.')

    reqd_in_custom = ['consumption_rate',
                      'battery_capacity', 'charging_functions']
    for req in reqd_in_custom:
        if req not in ev_el['custom']:
            raise KeyError('Instance missing required information: '+
                           f'{req} not found in the EV\'s \'custom\' element.')

    # also check that there are actually charging functions given
    if 'function' not in ev_el['custom']['charging_functions']:
        raise KeyError('Instance missing required information: '+
                       f'no function found in EV\'s \'charging_functions\' element.')


def _get_process_times(process_times, requests):
    """Read in service_time values from requests."""

    # loop over requests, see if they have service times. if so, set
    for req in requests:
        if 'service_time' in req:
            process_times[int(req['@node'])] = float(req['service_time'])
    return process_times


def translate(
        from_filename,
        to_filename: str = None,
        v_type: Any = None,
        depot_charging: bool = True) -> Optional[dict]:
    """Translate a VRP-REP instance into an instance compatible with frvcpy.

    Args:
        from_filename: filename (str) of the instance to be translated
        to_filename: filename (str) of the translated instance to be written. If
            not specified, the function returns the instance in a Python dictionary.
        v_type: the value of the `type` attribute of the vehicle_profile from the
            instance to be used (what vehicle type to solve the FRVCP for)
        depot_charging: whether the EV is allowed to recharge at the depot
            (True by default)

    Returns:
        If to_filename is specified, writes the translated instance to file and
            returns None. Otherwise (to_filename not specified) returns the
            necessary instance parameters in a dict.
    """

    with open(from_filename) as f_xml:
        doc = xmltodict.parse(f_xml.read())

    instance_xml = doc["instance"]
    network = instance_xml["network"]
    nodes = network["nodes"]["node"]
    requests = instance_xml["requests"]['request']
    ev = instance_xml["fleet"]["vehicle_profile"]

    # getting the right vehicle from the instance
    if isinstance(ev, list):  # multiple vehicle profiles found
        if v_type is None:  # no vehicle type specified; just take the first one, but warn the user
            print("INFO: Multiple vehicle_profiles found, but no type specified. "+
                  f"Using first type listed in the instance ({ev[0]['@type']}).")
            ev = ev[0]
        else:  # a vehicle type was specified; get it
            good_evs = [v for v in ev if v['@type'] == v_type]
            if len(good_evs) >= 2:
                raise ValueError(f"Multiple vehicle profiles of type {v_type} found")
            if len(good_evs) <= 0:
                raise ValueError(f"No vehicle profile found with specified type {v_type}")
            ev = good_evs[0]
    elif v_type is not None:
        if ev['@type'] != v_type:
            raise ValueError(f"EV type (ev['@type']) does not match specified type ({v_type})")

    # Warn user about ignored instance sections
    _warn_unused_els(instance_xml)

    dist_type = None
    if 'euclidean' in network:
        dist_type = 'euclidean'
    elif 'manhattan' in network:
        dist_type = 'manhattan'

    if dist_type is None:
        print("WARNING: An unrecognized (or no) distance type was listed in the instance. "+
              "Assuming Euclidean distance calculations. ('euclidean' and 'manhattan' are "+
              "supported; for all others, manual instance translation necessary.")
        dist_type = 'euclidean'

    precision_type = _get_precision_type(network)
    if precision_type is not None:
        print('WARNING: Using default Python precision. '+
              f'Precision type \'{precision_type}\' ignored.')

    # vehicle info
    _check_reqd_ev_info(ev)
    speed = float(ev['speed_factor'])
    consump_rate = float(ev['custom']['consumption_rate'])
    max_q = float(ev['custom']['battery_capacity'])
    cfs = ev['custom']['charging_functions']['function']
    if not isinstance(cfs, (list,)):  # promote individual CFs to a list
        cfs = [cfs]
    type_to_speed = _get_type_to_speed(cfs)
    # Optional max route duration
    max_t = float(ev['max_travel_time']) if ('max_travel_time' in ev) else None

    # append depot's CS to nodes
    if depot_charging:
        fastest = [t for t, r in type_to_speed.items() if r == 0][0]
        nodes.append({
            '@id': str(len(nodes)),
            'cx': nodes[0]['cx'],
            'cy': nodes[0]['cy'],
            '@type': '2',
            'custom': {'cs_type': fastest}})
        print(f"INFO: Depot assumed to be a CS with the instance's fastest "+
              "charging type (fastest found was \'{fastest}\').")

    # CSs
    css = [node for node in nodes if node['@type'] == '2']

    # Warn if other node types found
    unrec_nodetypes = {node['@type'] for node in nodes if node['@type'] not in ['0', '1', '2']}
    if unrec_nodetypes:
        print(f"WARNING: Unrecognized node types found: {unrec_nodetypes}")

    # request info
    print(f"INFO: Only nodes' service_time is preserved from requests. All other info ignored.")
    process_times = [0 for _ in nodes]
    process_times = _get_process_times(process_times, requests)
    req_nodes = [int(r['@node']) for r in requests]
    if len(req_nodes) != len(set(req_nodes)):
        print("WARNING: Nodes can only have one request. Ignoring duplicate requests.\n\t" +
              "For nodes with multiple requests, please add additional (dummy) nodes to the instance.")

    # instantiate output
    instance = {}

    # populate output
    # max charge and starting charge
    instance["max_q"] = max_q
    # max duration
    if max_t is not None:
        instance["t_max"] = max_t
    # store CSs
    instance["css"] = [{'node_id': int(
        cs['@id']), 'cs_type':type_to_speed[cs['custom']['cs_type']]} for cs in css]
    # process times are zero
    instance["process_times"] = process_times
    # breakpoints
    instance["breakpoints_by_type"] = [
        {
            "cs_type": type_to_speed[cfs[k]['@cs_type']],
            "time":[float(bpt['charging_time']) for bpt in cfs[k]['breakpoint']],
            "charge":[float(bpt['battery_level']) for bpt in cfs[k]['breakpoint']]
        } for k in range(len(cfs))
    ]
    # energy and time matrices
    instance["energy_matrix"] = [
        [_e(i, j, consump_rate, dist_type) for j in nodes] for i in nodes]
    instance["time_matrix"] = [
        [_t(i, j, speed, dist_type) for j in nodes] for i in nodes]

    if to_filename is not None:
        with open(to_filename, 'w') as instance_file:
            json.dump(instance, instance_file)
        return None

    return instance


def main():
    """Translates a VRP-REP instance for use with frvcpy."""

    parser = argparse.ArgumentParser(
        description="A translator for VRP-REP instances to make them compatible with frvcpy")
    parser.add_argument('from_filename', type=str,
                        help='Filename for the VRP-REP instance to translate')
    parser.add_argument(
        'to_filename', type=str, help='Filename for the new frvcpy instance to be created')
    parser.add_argument(
        "-v",
        "--vtype",
        type=str,
        default=None,
        help=('Which vehicle_profile to use (by its "type" attribute); '+
              'defaults to the first vehicle_profile in the instance'))
    # parse boolean for whether or not to allow charging at the depot
    depot_cs_parser = parser.add_mutually_exclusive_group(required=False)
    depot_cs_parser.add_argument('-d', '--depotcs', dest='depot_cs', action='store_true',
                                 help='Allow vehicles to charge at the depot (default)')
    depot_cs_parser.add_argument('--no-depotcs', dest='depot_cs',
                                 action='store_false', help='Disallow charging at the depot')
    parser.set_defaults(depot_cs=True)

    args = parser.parse_args()

    print(f"Preparing to translate instance {args.from_filename}...")
    translate(from_filename=args.from_filename, to_filename=args.to_filename,
              v_type=args.vtype, depot_charging=args.depot_cs)
    print(f"Translated instance file written to {args.to_filename}")

    sys.exit(0)


if __name__ == "__main__":
    main()
