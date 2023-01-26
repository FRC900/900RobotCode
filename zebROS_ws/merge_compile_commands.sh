#!/usr/bin/env bash
rm -f ~/2023RobotCode/zebROS_ws/build/compile_commands.json
printf '[' > ~/2023RobotCode/zebROS_ws/compile_commands.json
find ~/2023RobotCode/zebROS_ws/build -type f -name 'compile_commands.json' -exec sh -c "cat {} | tail -n+2 | head -n-1 && printf ','" >> compile_commands.json \;
sed -i '$s/.$//' ~/2023RobotCode/zebROS_ws/compile_commands.json
sed -i 's/.2023RobotCode.readonly/2023RobotCode/g' ~/2023RobotCode/zebROS_ws/compile_commands.json
printf '\n]\n' >> ~/2023RobotCode/zebROS_ws/compile_commands.json
mv ~/2023RobotCode/zebROS_ws/compile_commands.json ~/2023RobotCode/zebROS_ws/build 
