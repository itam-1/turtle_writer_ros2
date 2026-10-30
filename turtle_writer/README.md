
Program w ROS 2 sterujący żółwiem w symulatorze `turtlesim`. Rysuje w górnej części okna napis "LRT", a poniżej inicjały "MD".

Opis działania:
- Sterowanie odbywa się na podstawie bieżącej pozycji z tematu `/turtle1/pose`.
- Program nie używa timerów ani funkcji `sleep()` do odmierzania czasu (brak sterowania czasowego).
- Przejazdy między literami odbywają się z wyłączonym pisakiem (serwis `/turtle1/set_pen`).

Uruchomienie

1. Kompilacja paczki:
cd ~/ros2_ws
colcon build --packages-select turtle_writer

2. W pierwszym terminalu (symulator):
ros2 run turtlesim turtlesim_node

3. W drugim terminalu (skrypt sterujący):
source ~/ros2_ws/install/setup.bash
ros2 run turtle_writer writer_node
