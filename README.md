# Adative-TSC
This study primarily concentrates on enhancing the feedback gains employed in task-space control of legged robots, specifically for the floating-base task. The selection of optimal feedback gains directly correlates with improved tracking performance in locomotion. This paper presents a novel approach that involves the utilization of a well-suited template model and sensitivity analysis to calculate state-dependent feedback gains for the floating-base task. The effectiveness of the proposed feedback gains is demonstrated through extensive simulations conducted in MuJoCo, as well as real-world experiments conducted on the Aliengo robot, showcasing remarkable enhancements in tracking performance.

# Build Package

create `build` folder and then use CMake Tools to build this package

```
cd ${source folder} & mkdir build
cd build
cmake ..
build -j
```



