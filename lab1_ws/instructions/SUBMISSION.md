# Lab 1: Intro to ROS 2

## Written Questions

### Q1: During this assignment, you've probably ran these two following commands at some point: ```source /opt/ros/foxy/setup.bash``` and ```source install/local_setup.bash```. Functionally what is the difference between the two?

Answer:
The command source /opt/ros/foxy/setup.bash is used to set up the environment for ROS 2 Foxy, including environment variables, paths to ROS tools, and other necessary settings to use ROS 2 functionalities system-wide. It ensures that the ROS core packages and tools are accessible in your terminal session.

On the other hand, source install/local_setup.bash is used after building your ROS 2 workspace with colcon build. This script sets up the environment for the specific ROS 2 packages you've developed and built in your workspace. It ensures that your local packages and any modifications you've made are accessible, overriding the general ROS 2 settings where applicable.


### Q2: What does the ```queue_size``` argument control when creating a subscriber or a publisher? How does different ```queue_size``` affect how messages are handled?

Answer:
The queue_size argument in a ROS publisher or subscriber controls the size of the message queue. If messages arrive faster than they can be processed, queue_size determines how many messages will be buffered before starting to drop the oldest messages.

A smaller queue_size can lead to more frequent message drops if the incoming rate is high, which might be appropriate in real-time systems where only the most current data is relevant. A larger queue_size can accommodate bursts of messages without loss but at the cost of increased memory usage and potentially processing older data if the processing cannot keep up with the incoming rate.


### Q3: Do you have to call ```colcon build``` again after you've changed a launch file in your package? (Hint: consider two cases: calling ```ros2 launch``` in the directory where the launch file is, and calling it when the launch file is installed with the package.)

Answer:
You do not necessarily need to run colcon build again after changing a launch file if you are directly running the launch file from the source directory using ros2 launch. The changes will be recognized immediately because ros2 launch reads the launch file directly from the file system.

However, if you are running the launch file from an installation (i.e., you are using the launch files installed in your build's install directory), you need to rebuild your workspace with colcon build to update the installed launch files. Only then will the changes to the launch file take effect when you use the launch file from the installation directory. This is crucial when deploying or distributing software where the launch files need to be consistent with the source files.
