
Run simlaunch in rosstd.
This runs a simulation of the robot and in turn simulates all of the topics etc to be visible in rqt reconfigure
Run rosrun rqt_reconfigure rqt_reconfigure in zebros/src
This launches a menu that allows you to dynamically reconfigure topics.

cd in zebros/src/controller_node
and run "roslaunch controller_node arg_launch.launch steering_option:=True" [in order to dynamically reconfigure steering related motors properties]
or arg "speed_option:=True"  [in order to dynamically reconfigure speed related motor properties]
[or use both args in the same line, this enables you to reconfigure the properties of the steering and speed motors]

look back at the rqt_reconfigure menu, a bar titled "reconfigure_server" should appear
adjusting the values on this slider will affect the properties of the related motor values

Remember to rosstd in each terminal
