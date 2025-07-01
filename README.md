# From Theory to Practice: Engineering Approximation Algorithms for Dynamic Orientation

To cite:
```
@inproceedings{DynamicOrientationAdaptive,
  author       = {Ernestine Grossmann and Ivor van der Hoog and Henrik Reinstädtler and Eva Rotenberg and Christian Schulz  and Juliette Vlieghe},
  title        = {From Theory to Practice: Engineering Approximation Algorithms for Dynamic Orientation},
  booktitle    = {33rd Annual European Symposium on Algorithms (ESA 2025)},
  editor       = {Anne Benoit and Haim Kaplan and Sebastian Wild and Grzegorz Herman},
  year         = {2025},
  series       = {Leibniz International Proceedings in Informatics (LIPIcs)},
  volume       = {351},
  publisher    = {Schloss Dagstuhl--Leibniz-Zentrum für Informatik},
  address      = {Warsaw, Poland},
  doi          = {10.4230/LIPIcs.ESA.2025.63},
  eventdate    = {September 15--17, 2025}
}

```

The main directory for this project is in `app/algorithms/dynamic/edge_orientation`
## How to build

Have gcc>=12 installed and use bazel (to install we recommend https://github.com/bazelbuild/bazelisk/releases install by placing the bazelisk executable in a location of your path as bazel file)

Compile:

```
bazel build -c opt app --define logging=enabled # last is optionally, -c dbg for debugging symbols.
```

## Running

```sh
./bazel-bin/app/app --command_textproto 'command:"run" hypergraph {    file_path: "<path to seq file>"    format: "seq" } config {algorithm_configs{algorithm_name:"cchhqrs" data_structure:"dynamic_graph_inmemory" double_params:{key:"lambda" value:0.1} int64_params{key:"b" value:10}} }' --undefok=v --v=6 --seed 1234 # replace cchhqrs with packed_cchhqrs for the new implementation.
```

### Available Algorithms

Field `data_structure: "dynamic_graph_inmemory"` is always the same. You can change the algorithm by following values.

| algorithm_name | Description | Parameters |
| ------ | ----- | --------- |
| "limited_bfs" | The `BFS20` algorithm by Borowitz et al. |  int64_params{key: "bfs_depth" value: 20} (default 20) |
| "strong_bfs" | The `StrongDynOpt` algorithm by Großmann et al. |  None |
| "improved_bfs" | The `ImprovedDynOpt` algorithm by Großmann et al. |  None |
| "cchhqrs" | The `Fractional` algorithm from our paper | double_params:{key:"lambda" value:0.1} int64_params{key:"b" value:10} int64_params{key:"theta" value:0} |
| "packed_cchhqrs" | The `PackedFractional` algorithm from our paper | double_params:{key:"lambda" value:0.1} int64_params{key:"b" value:10} int64_params{key:"theta" value:0} |
| "packed_cchhqrs_list" | The `PackedFractional, list` algorithm from our paper | double_params:{key:"lambda" value:0.1} int64_params{key:"b" value:10} int64_params{key:"theta" value:0} |

## Running experiments

To run experiments compile the `runner:fork_runner` target.

```
bazel build -c opt runner:fork_runner
```

Run the experiments by:

```sh
./bazel-bin/runner/fork_runner --experiment_path <absolute path to experiment folder containing experiment.textproto>  --random_order 1 --max_alloc_memory 120000  --max_alloc_memory_per_process 56000 --cycles_to_queue_new 1 --seed 1234 --root_path=<path to folder containing the  extracted graphs folder> --timeout <optional timeout, for example one day: 86400> --sleep_main 1 --retry_count 1
```

## Visualizing

To generate the vis folder execute the following command:

Use spack to install dependencies (python3.12)
```
cd code
spack env activate .
spack install
```

```
spack env activate .
bazel run -c opt tools/plot:plot_cc /absolutepath/to/folder/containing/experiments /absolute/path/to/visualisation.textproto
```
