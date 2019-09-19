# navigation-baselines
Baselines for navigation


[![Build Status](https://travis-ci.com/jacobjohnson-bc/navigation-baselines.svg?branch=master)](https://travis-ci.com/jacobjohnson-bc/navigation-baselines)


## Installing
To install the package use pip:
```
pip install .
```

## Run Experiments

To run the planner for a particular robot and environment you can use `scripts/run_planner.py**`. The script takes the following argumnet:
- robot [ `tri`,`diff`] : The robot for model.
- env [`mini`,`turn`,`map`] : The environment that needs to be tested.
- iterations : The number of iterations for the SST planner.
- graph : A flag that will enable display of graph construction.
- render : A flag that will enable the rendering of the environment if a path exists.

Egs:
```
python scripts/run_planner.py tri mini --iterations=100000
python scripts/run_planner.py diff map --iterations=1e6 --seed=5 --graph
python scripts/run_planner.py diff turn --render
```

## Custom Experiments

You can run this planner on an object of class of subclass of `bc_gym_planning_env.envs.base.env.PlanEnv`. 

You can use the following script to create an object of class `sparse_rrt.systems.system_interface.Isystem` for an object `env`.

```python
from sst_wrapper.envs.bc_gym_wrapper import bc_gym_wrapper
from sst_wrapper.envs.gym_sst_wrapper import bc_sst_wrapper

system = bc_sst_wrapper(bc_gym_wrapper(env))
```
`system` can now be used to run `sparse_rrt.planners.SST`. Look at [sparse-rrt](https://github.com/olegsinyavskiy/sparse_rrt) for more info.
