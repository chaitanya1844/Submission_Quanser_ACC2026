# Submission_Quanser_ACC2026

```bash
sudo docker run --rm -it --network host --name virtual-qcar2 quanser/virtual-qcar2 bash
```

```bash
python3 /home/qcar2_scripts/python/Base_Scenarios_Python/Setup_Real_Scenario_Interleaved.py
```

```bash
source install/setup.bash
```

```bash
ros2 launch qcar2_nodes qcar2_cartographer_virtual_launch.py
```

```bash
ros2 launch qcar2_autonomy autonomy_planner_launch.py
```

