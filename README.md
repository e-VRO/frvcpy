# frvcpy: An Open-Source Solver for the FRVCP

### Fast optimal solutions to rich FRVCPs

## What is an FRVCP?

Given an elctric vehicle (EV) that's been assigned some sequence of locations to visit (a _fixed route_), the __fixed route vehicle charging problem__ (FRVCP) is the problem of finding the optimal insertion of recharging operations into the route that minimize the time required for the EV to traverse that route in an energy-feasible manner.

## Installation

In a virtual environment with Python 3.6+, frvcpy can be installed via

```bash
pip install frvcpy
```

### Testing the installation

```python
import frvcpy.test
frvcpy.test.runAll()
```

Or from the command line:

```bash
frvcpy-test
```

## Using frvcpy

With a compatible instance file ([see the schema](https://github.com/e-VRO/frvcpy/blob/master/instances/frvcpy-instance.schema.json)), solve the FRVCP from a Python script:

```python
from frvcpy import solver

route = [0,40,12,33,38,16,0]        # route to make energy feasible
q_init = 16000                      # vehicle's initial energy level

# using an existing instance from file
frvcp_solver = solver.Solver("instances/frvcpy-instance.json", route, q_init)

# run the algorithm
duration, feas_route = frvcp_solver.solve()

# write solution to file
frvcp_solver.write_solution("my-solution.xml", instance_name="frvcpy-instance")

print(f"Duration: {duration:.4}")
# Duration: 7.339

print(f"Energy-feasible route:\n{feas_route}")
# Energy-feasible route:
# [(0, None), (40, None), (12, None), (33, None), (48, 6673.379615520617), (38, None), (16, None), (0, None)]
```

Or from the command line:

```bash
frvcpy --instance=instances/frvcpy-instance.json --route=0,40,12,33,38,16,0 --qinit=16000 --output=my-solution.xml
# Duration: 7.339
# Energy-feasible route:
# [(0, None), (40, None), (12, None), (33, None), (48, 6673.379615520617), (38, None), (16, None), (0, None)]
```

Solutions are written in the [VRP-REP](http://www.vrp-rep.org/) format for easy importing and visualization with the [VRP-REP Mapper](https://vrp-rep.github.io/mapper/) (_formal solution specification available [here](http://www.vrp-rep.org/resources.html)_).

_Note: Example problem instances are available in the `instances` directory on the [project's homepage](https://github.com/e-VRO/frvcpy/). For easy access to the example files, we recommend cloning the repository._

## Instance Translation

Instance translation is available for some E-VRP instances formatted according to the VRP-REP specification (_available [here](http://www.vrp-rep.org/resources.html)_).

Translation can be done with the Python API via

```python
from frvcpy import translator

# Option 1) write the translated instance to file
translator.translate("instances/vrprep-instance.xml", to_filename="instances/my-new-instance.json")

# Option 2) make instance object to be passed directly to the solver
frvcp_instance = translator.translate("instances/vrprep-instance.xml")
```

Or with the command line:

```bash
# from CLI, only option is to write translated instance to file
frvcpy-translate instances/vrprep-instance.xml instances/my-new-instance.json
```

_Note: If an instance ending in ".xml" is passed to the solver, it is assumed to be a VRP-REP instance, and the solver will automatically attempt to translate it._

### Translation requirements for VRP-REP instances

frvcpy's translator assumes VRP-REP instances are formatted similarly to the [Montoya et al. (2017) instances](http://vrp-rep.org/datasets/item/2016-0020.html):

- CSs are identified as `<node>` elements having attribute `type="2"`
- Charging stations nodes have a `<custom>` child element which contains a `cs_type` element
- For each unique CS type `t` appearing in those `cs_type` elements, there exists a charging `function` element with attribute `cs_type=t`
- These `function` elements are part of a `charging_functions` element in a `vehicle_profile`'s `custom` element
- The depot has node ID 0, the N customers have IDs {1, ..., N}, and the CSs have IDs {N+1, ..., N+C}

A good example of such an instance is the [example VRP-REP instance in the repository](https://github.com/e-VRO/frvcpy/blob/master/instances/vrprep-instance.xml).

Here is a smaller example meeting these requirements:

```xml
<?xml version="1.0" encoding="UTF-8"?>
<instance>
  <network>
    <nodes>
      <node id="0" type="0">
        <cx>74.83</cx>
        <cy>51.85</cy>
      </node>
      <node id="1" type="1">
        <cx>68.77</cx>
        <cy>75.69</cy>
      </node>
      <node id="11" type="2">
        <cx>57.0</cx>
        <cy>57.04</cy>
        <custom>
          <cs_type>fast</cs_type>
        </custom>
      </node>
    </nodes>
    <euclidean/>
    <decimals>14</decimals>
  </network>
  <fleet>
    <vehicle_profile type="0">
      <departure_node>0</departure_node>
      <arrival_node>0</arrival_node>
      <speed_factor>25.0</speed_factor>
      <custom>
        <consumption_rate>0.125</consumption_rate>
        <battery_capacity>16.0</battery_capacity>
        <charging_functions>
          <function cs_type="fast">
            <breakpoint>
              <battery_level>0.0</battery_level>
              <charging_time>0.0</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>13.6</battery_level>
              <charging_time>0.317</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>15.2</battery_level>
              <charging_time>0.383</charging_time>
            </breakpoint>
            <breakpoint>
              <battery_level>16.0</battery_level>
              <charging_time>0.517</charging_time>
            </breakpoint>
          </function>
        </charging_functions>
      </custom>
    </vehicle_profile>
  </fleet>
  <requests>
    <request id="1" node="1">
      <service_time>0.5</service_time>
    </request>
  </requests>
</instance>
```

## The Solver

To solve FRVCPs, frvcpy implements the labeling algorithm from Froger et al. (2019), providing optimal solutions in low runtime. The algorithm accommodates realistic problem features such as nonlinear charging functions, heterogeneous charging station technologies, and multiple CS visits between stops.

## Additional information

For more information about the algorithm used in the solver, see [Froger et al. (2019)](https://www.sciencedirect.com/science/article/abs/pii/S0305054818303253).

A write-up of this package is available on HAL [here](https://hal.archives-ouvertes.fr/hal-02496381).
