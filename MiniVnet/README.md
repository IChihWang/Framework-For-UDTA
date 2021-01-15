# Framework-For-UDTA (MiniVnet)
A general framework and simulation for ultra-dynamic traffic assignment

## Introduction
To use MiniVnet, you need to import
> from miniVnet import MINIVNET

## Manual for MiniVnet
### Compilation is needed after setting up the network
> compile()

### Setup the network
1. **Add an intersection**  
    `name` is the ID of the intersection. `num_lane` is the number of lanes in each direction.  
    > addIntersection(name, num_lane)

1. **Add an sink**  
    `name` is the ID of the sink (source and destination). `num_lane` is the number of lanes in the sink.  
    > addSink(name, num_lane)
    
1. **Connect two components**  
    `component_1`, `component_2` are the two components. `idx_1` is the index of `component_1`, which is desired to connect to `idx_2` of `component_2`.
    Components can be either sinks or intersections.
    For sinks, the index should be 0.
    For intersections, the index can be {0..3}.  
    > connect(component_1, idx_1, component_2, idx_2)
    
1. **Create grid network**  
    Create grid network with given `N` and the number of lane `num_lane`. (compiled automatically)  
    > createGridNetwork(N, num_lane)
    
### Run the network
