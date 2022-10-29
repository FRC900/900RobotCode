#!/usr/bin/env bash
rm -f ~/2022RobotCode/zebROS_ws/build/compile_commands.json
printf '[' > ~/2022RobotCode/zebROS_ws/compile_commands.json
find ~/2022RobotCode/zebROS_ws/build -type f -name 'compile_commands.json' -exec sh -c "cat {} | tail -n+2 | head -n-1 && printf ','" >> compile_commands.json \;
sed -i '$s/.$//' ~/2022RobotCode/zebROS_ws/compile_commands.json
sed -i 's/.2022RobotCode.readonly/2022RobotCode/' ~/2022RobotCode/zebROS_ws/compile_commands.json
printf '\n]\n' >> ~/2022RobotCode/zebROS_ws/compile_commands.json
mv ~/2022RobotCode/zebROS_ws/compile_commands.json ~/2022RobotCode/zebROS_ws/build 
