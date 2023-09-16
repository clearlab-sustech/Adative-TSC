# Adative-TSC (ROS2)
![log](https://user-images.githubusercontent.com/38805251/268448999-a8dab68a-11d3-4140-a132-593469565648.png)

This study primarily concentrates on enhancing the feedback gains employed in task-space control of legged robots, specifically for the floating-base task. The selection of optimal feedback gains directly correlates with improved tracking performance in locomotion. This paper presents a novel approach that involves the utilization of a well-suited template model and sensitivity analysis to calculate state-dependent feedback gains for the floating-base task. The effectiveness of the proposed feedback gains is demonstrated through extensive simulations conducted in MuJoCo, as well as real-world experiments conducted on the Aliengo robot, showcasing remarkable enhancements in tracking performance.



# Build Package
create `build` folder and then use CMake Tools to build this package

```bash
cd ${source folder} & bash build.sh
echo "source ${source folder}/install/setup.bash" >> ~/.bashrc
```



# Run Package

## Simulation
The simulator is based on a open-source platform [MuJoCo](https://mujoco.org/). Compared to other physics engine, it has great advantages of accurancy and computational efficiency. one can start the simulator by
```bash
ros2 launch sim sim_launch.py 
```
And then one can use `ros2 topic list` and `ros2 topic echo ${topic_name}` to check the topics
```bash
/parameter_events
/rosout
/simulation/actuators_cmds
/simulation/imu_data
/simulation/joint_states
/simulation/odom
/simulation/touch_sensor
```
One can also choose own robot to simulate by modifying the following two lines in `src/sim/launch/sim_launch.py`
```python
xml_file_name = "aliengo/aliengo.xml"
xml_file = os.path.join(get_package_share_path("asserts"), xml_file_name)
```





## Estimator
Coding Reference [Cheetah-Software](https://github.com/mit-biomimetics/Cheetah-Software). In this package, the estimator is currently suitable for the flat-ground case where assume the foot tourching the ground has zero height and the contact detection is replaced by predefined gait timing. The futural work can be extended to learning-based contact detection and position-free control framework where only accurate velocity estimation is required in controller level.


## Controller





# package development

Download the package MuJoCo to the folder `src/third_parties/mujocodl` and then delete the `main.cc` in  the folder `src/third_parties/mujocodl/mujoco/simulate`. Besides, copy the library file `src/third_parties/mujocodl/mujoco/lib/libmujoco.so.2.3.7` to /usr/lib and use the follolwing commands to make links
``` bash
sudo ln -s /usr/lib/libmujoco.so.2.3.7 /usr/lib/libmujoco.so
```


# data recording
One can use the following commands to record the all datas
``` bash
ros2 bag record --all
```
and replay by
``` bash
ros2 bag play ${bag_name}
```

