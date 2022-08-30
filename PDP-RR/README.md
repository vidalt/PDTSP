# veenstra2017

TSPPD solver based on Veenstra et al. 2017 paper

### Usage:

```
    veenstra --instance=<filename> [--fast] [--local-search=<ratio>] [--seed=<seed>] [--time-limit=[seconds]] [--proportiona-time-limit]
```

example:

```
veenstra
--instance=instance/pdp/RBO00/Class1/EIL51C.PDT --fast
--local-search=0.1 --seed=42 --proportional-time-limit
```

where:

--instance=[filename]: filename containing TSPPD instance.

--fast: enable fast relocate method

--local-search=[ratio]: enable local search procedure with the given probality ratio.

--seed=[seed]: set pseudo-random generator seed

--proportional-time-limit: set time limit to the size of instance in seconds.

--time-limit=[seconds]: set time limit to the given time in seconds.
