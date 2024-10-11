#!/usr/bin/env bash
rm -f ~/900RobotCode/zebROS_ws/build/compile_commands.json
printf '[' > ~/900RobotCode/zebROS_ws/compile_commands.json
find ~/900RobotCode/zebROS_ws/build -type f -name 'compile_commands.json' -exec sh -c "cat {} | tail -n+2 | head -n-1 && printf ','" >> compile_commands.json \;
sed -i '$s/.$//' ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i 's/.900RobotCode.readonly/900RobotCode/g' ~/900RobotCode/zebROS_ws/compile_commands.json
printf '\n]\n' >> ~/900RobotCode/zebROS_ws/compile_commands.json
mv ~/900RobotCode/zebROS_ws/compile_commands.json ~/900RobotCode/zebROS_ws/build 
