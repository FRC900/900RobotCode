#!/usr/bin/env bash
rm -f ~/900RobotCode/zebROS_ws/build/compile_commands.json
printf '[' > ~/900RobotCode/zebROS_ws/compile_commands.json
find ~/900RobotCode/zebROS_ws/build -type f -name 'compile_commands.json' -exec sh -c "cat {} | tail -n+2 | head -n-1 && printf ','" >> compile_commands.json \;
sed -i '$s/.$//' ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i 's/.900RobotCode.readonly/900RobotCode/g' ~/900RobotCode/zebROS_ws/compile_commands.json
printf '\n]\n' >> ~/900RobotCode/zebROS_ws/compile_commands.json
# Expand .rsp file command line into the that file's contents : https://unix.stackexchange.com/questions/141387/sed-replace-string-with-file-contents
sed -i "s?--options-file CMakeFiles/gpu_apriltag.dir/includes_CUDA.rsp?$(sed 's/\"//g' /home/ubuntu/900RobotCode/zebROS_ws/build/gpu_apriltag/CMakeFiles/gpu_apriltag.dir/includes_CUDA.rsp)?" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?--options-file CMakeFiles/deeptag.dir/includes_CUDA.rsp?$(sed 's/\"//g' /home/ubuntu/900RobotCode/zebROS_ws/build/deeptag_ros/CMakeFiles/deeptag.dir/includes_CUDA.rsp)?" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?--options-file CMakeFiles/base_trajectory.dir/includes_CUDA.rsp?$(sed 's/\"//g' /home/ubuntu/900RobotCode/zebROS_ws/build/base_trajectory/CMakeFiles/base_trajectory_node.dir/includes_CUDA.rsp)?" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?/usr/local/cuda-11/bin/nvcc?/usr/bin/c++?" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?-forward-unknown-to-host-compiler ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?--expt-relaxed-constexpr ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?\\\\\\\"--generate-code=arch=compute_86,code=\\[compute_86,sm_86\\]\\\\\\\" ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?-x cu ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?-rdc=true ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
sed -i "s?Xcompiler=-fPIC ??g" ~/900RobotCode/zebROS_ws/compile_commands.json
mv ~/900RobotCode/zebROS_ws/compile_commands.json ~/900RobotCode/zebROS_ws/build 
