node {

    stage('Preparation') { 
      // Get some code from a GitHub repository
      checkout changelog: true, poll: false, scm: [
          $class: 'GitSCM', 
          branches: [[name: '**']], 
          doGenerateSubmoduleConfigurations: false, 
          extensions: [], 
          submoduleCfg: [], 
          userRemoteConfigs: [
                [credentialsId: 'zebra_build_user', url: 'https://github.com/FRC900/2018Offseason.git']
              ]
          ]
   }
   
   try {
       docker.image('frc900/zebros-dev:latest').inside('--user root:root -v ' + env.WORKSPACE + ':/home/ubuntu/2018Offseason -l /bin/bash') { c ->
            
            stage('Build') {
            
                sh '''#!/bin/bash
                    cd /home/ubuntu/2018Offseason
                    git log -n1
                    git submodule update --init --recursive
                    echo \"git submodule init\"
                    ./install_ros_desktop_packages.sh
                    echo \"install_ros_desktop_packages.sh\"
                    cd zebROS_ws
                    wstool update -t src --continue-on-error&& \
                    echo \"wstool update\"
                    source /opt/ros/kinetic/setup.bash
                    echo \"source setup.bash\"
                    catkin_make
                    echo \"catkin make\"
                    source devel/setup.bash
                    timeout -k 30 --preserve-status 60 roslaunch ros_control_boilerplate 2018_main_frcrobot.launch hw_or_sim:=sim output:=screen
                    echo \"Finished Ubuntu compile.\"
                '''
            }
        
        
            stage('Test') {
                sh '''#!/bin/bash
                    cd zebROS_ws
                    wstool update -t src --continue-on-error
                    source /opt/ros/kinetic/setup.bash
                    source devel/setup.bash
                    catkin_make run_tests
                    catkin_test_results build/test_results 
                '''
            }
        }
    } finally {
        sh '''#!/bin/bash
            chown -R jenkins:jenkins .
        '''
        deleteDir()
        junit allowEmptyResults: true, testResults: 'zebROS_ws/build/test_results/**/*.xml'
    }

}