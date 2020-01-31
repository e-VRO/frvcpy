import argparse
import json
import math
import sys

import xmltodict

# TODO accommodate other CS types
# (read in the types from the instance, number them according to speed, then use that instead of this custom key)
CS_INT = {
  'fast':0,
  'moderate':1,
  'normal':1,
  'slow':2
}

# define energy and time functions
def _dist(i_node,j_node) -> float:
  return math.sqrt((float(i_node['cx']) - float(j_node['cx']))**2 + (float(i_node['cy']) - float(j_node['cy']))**2)

def _t(i_node, j_node, speed) -> float:
  return _dist(i_node,j_node)/speed

def _e(i_node,j_node,consump_rate) -> float:
  return _dist(i_node,j_node)*consump_rate

def _get_fastest_cs_type(cfs):
  """Given a list of charging functions, returns the type of the fastest charging station in the instance."""
  # if there's only one type of CS, then that's what it is
  if not isinstance(cfs,(list,)):
    return cfs['@cs_type']

  # otherwise, get the fastest of the types that are present in the instance
  else:
    return min(CS_INT, key=(lambda k: float('inf') if (k not in [cf['@cs_type'] for cf in cfs]) else CS_INT[k]))

def _get_precision_type(network_el):
  """Given a network element from a VRP-REP instance, returns its precision type:
  floor, ceil, or decimals. If no such precision type is present, returns None.
  """
  if 'decimals' in network_el:
    return 'decimals'
  elif 'floor' in network_el:
    return 'floor'
  elif 'ceil' in network_el:
    return 'ceil'
  else:
    return None

def _warn_unused_els(instance_el):
  unused_els = ['resources', 'drivers']
  for unused_el in unused_els:
    if unused_el in instance_el:
      print(f"WARNING: Ignoring \'{unused_el}\' element.")

def _check_reqd_ev_info(ev_el):
  """Check the EV element for required information."""
  
  reqd_in_profile = ['speed_factor','custom']
  for req in reqd_in_profile:
    if req not in ev_el:
      raise KeyError(f'Instance missing required information: {req} not found in the EV\'s vehicle_profile.')

  reqd_in_custom = ['consumption_rate', 'battery_capacity', 'charging_functions']
  for req in reqd_in_custom:
    if req not in ev_el['custom']:
      raise KeyError(f'Instance missing required information: {req} not found in the EV\'s \'custom\' element.')

  # also check that there are actually charging functions given
  if 'function' not in ev_el['custom']['charging_functions']:
    raise KeyError(f'Instance missing required information: no function found in EV\'s \'charging_functions\' element.')

  return

def _get_process_times(process_times, requests):
  """Read in service_time values from requests."""
  # loop over requests, see if they have service times. if so, set
  for req in requests:
    if 'service_time' in req:
      process_times[int(req['@node'])] = float(req['service_time'])
  return process_times

# primary method; converts an old (xml) instance file into a new (json) one
def translate(from_filename, to_filename=None):
  """Translate a VRP-REP instance into an instance compatible with frvcpy.
  
  If to_filename is not specified, returns the instance in a Python dictionary.
  Otherwise, writes the instance in JSON format to the destination specified by
  to_filename and returns None.
  """

  with open(from_filename) as f_xml:
    doc = xmltodict.parse(f_xml.read())

  instance_xml = doc["instance"]
  network = instance_xml["network"]
  nodes = network["nodes"]["node"]
  requests = instance_xml["requests"]['request']
  ev = instance_xml["fleet"]["vehicle_profile"]
  
  if isinstance(ev,list):
    print(f"WARNING: Using first vehicle profile from instance (instance contains {len(ev)}).")
    ev = ev[0]
    # TODO add optional CLI arg for which vehicle to grab from the instance
  
  # Warn user about ignored instance sections
  _warn_unused_els(instance_xml)
  
  if 'euclidean' not in network:
    print("WARNING: Using Euclidean distance calculations. To use non-Euclidean distances, please manually translate the instance.")
    # TODO add support for Manhattan distances
  
  precision_type = _get_precision_type(network)
  if precision_type is not None:
    print(f'WARNING: Using default Python precision. Precision type \'{precision_type}\' ignored.')
    # TODO add support for different precision types


  # vehicle info
  _check_reqd_ev_info(ev)
  speed = float(ev['speed_factor'])
  consump_rate = float(ev['custom']['consumption_rate'])
  max_q = float(ev['custom']['battery_capacity'])
  cfs = ev['custom']['charging_functions']['function']
  # Optional max route duration
  max_t = float(ev['max_travel_time']) if ('max_travel_time' in ev) else None

  # append depot's CS to nodes
  fastest = _get_fastest_cs_type(cfs)
  nodes.append({'@id':str(len(nodes)),'cx':nodes[0]['cx'],'cy':nodes[0]['cy'],'@type':'2','custom':{'cs_type':fastest}})
  print(f"INFO: Depot assumed to be a CS with the instance's fastest charging type (fastest found was \'{fastest}\').")

  # CSs
  css = [node for node in nodes if node['@type'] == '2']
  # TODO instruct user that CSs are assumed to be nodes of type 2 

  # request info
  print(f"INFO: Only nodes' service_time is preserved from requests. All other info ignored.")
  process_times = [0 for _ in nodes]
  process_times = _get_process_times(process_times, requests)
  req_nodes = [int(r['@node']) for r in requests]
  if len(req_nodes) != len(set(req_nodes)):
    print("WARNING: Nodes can only have one request. Ignoring duplicate requests.\n\t"+
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
  instance["css"] = [{'node_id':int(cs['@id']), 'type':CS_INT[cs['custom']['cs_type']]} for cs in css]
  # process times are zero
  instance["process_times"] = process_times
  # breakpoints
  if isinstance(cfs,(list,)): # if we have a list of charging functions
    instance["breakpoints_by_type"] = {
      str(CS_INT[cfs[k]['@cs_type']]):{
        "time":[float(bpt['charging_time']) for bpt in cfs[k]['breakpoint']],
        "charge":[float(bpt['battery_level']) for bpt in cfs[k]['breakpoint']]
      } for k in range(len(cfs))
    }
  else: # if we just have one
    instance["breakpoints_by_type"] = {
      str(CS_INT[cfs['@cs_type']]):{
        "time":[float(bpt['charging_time']) for bpt in cfs['breakpoint']],
        "charge":[float(bpt['battery_level']) for bpt in cfs['breakpoint']]
      }
    }
  # energy and time matrices
  instance["energy_matrix"] = [[_e(i,j,consump_rate) for j in nodes] for i in nodes]
  instance["time_matrix"] = [[_t(i,j,speed) for j in nodes] for i in nodes]

  if to_filename is not None:
    with open(to_filename, 'w') as fp:
        json.dump(instance, fp)
    return
  else:
    return instance


def main():
  """Translates a VRP-REP instance for use with frvcpy."""

  parser = argparse.ArgumentParser(description="A translator for VRP-REP instances to make them compatible with frvcpy")
  parser.add_argument('from_file', help='Filename for the VRP-REP instance to translate')
  parser.add_argument('to_file', help='Filename for the new frvcpy instance to be created')

  args = parser.parse_args()

  print(f"Preparing to translate instance {args.from_file}...")
  translate(args.from_file, args.to_file)
  print(f"Translated instance file written to {args.to_file}")

  sys.exit(0)

if __name__ == "__main__":
  main()
