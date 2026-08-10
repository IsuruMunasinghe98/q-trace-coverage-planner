# Benchmark datasets

Each non-empty line is a Python literal with the form:

```text
[binary_grid, start_cell, destination_cell]
```

`1` denotes a traversable cell and `0` denotes an obstacle. Coordinates use
zero-based `[row, column]` indexing.

The files follow the ordering used by the research notebook:

1. low-obstacle maps;
2. cluttered maps;
3. narrow-passage maps;
4. clustered-obstacle maps (named `irregular` in the original notebook).

Within every category, records are ordered by grid resolution: 5, 10, 20, 35,
and 50. The optimization set contains two maps per category and resolution
(40 maps). The evaluation set contains five maps per category and resolution
(100 maps). The two datasets are disjoint.
