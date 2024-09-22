# Reconstruye el paquete
cd ~/ros2_ws
colcon build --packages-select solar_panel_sim
source install/setup.bash

# Lanza la simulación
ros2 launch solar_panel_sim solar_panel_sim.launch.py
