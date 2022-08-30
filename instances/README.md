# Instance formats

## RBO00/Dumitrescu/PDP-X
***
```
11                         # Number of total locations (depot + customers)
     1  454   42           # Depot location: Index | X | Y
     2  336  835  0    7   # Customers:      Index | X | Y | Type | Pair
     3    2  565  0    8   # .
     4  990  188  0    9   # .
     5  366  750  0   10   #    Where:  
     6  573  351  0   11   #           X:    X coord of the location
     7   64  133  1    2   #           Y:    Y coord of the location
     8  154  951  1    3   #           Type: 0 indicates a pickup node.
     9  217  585  1    4   #                 1 indicates a delivery node.
    10  140  807  1    5   #           Pair: Index of the P or D pair.
    11  211  622  1    6   # .
-999                       # End Of Instance
```

Note:
* Distances should be rounded to nearest integral value.


## Grubhub
***

```
GRUBHUB: grubhub-02-1   # Name of the current instance
DIMENSION: 5            # Number of total locations (depot + customers)
0 1050 1123  746  456   #
0    0  228  912 1095   # Edges matrix with explicit cost from i to j.
0  228    0  847 1162   #   Where i=0 is the depot and P-D pairs are defined 
0  912  847    0  667   #   for each following consecutive two lines,
0 1095 1162  667    0   #   so i=1-2 and i=3-4 are P-D pairs.
```

Note:
* Original Grubhub instances files can be obtained [here](https://github.com/grubhub/tsppdlib).