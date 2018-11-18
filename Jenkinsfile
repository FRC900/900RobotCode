node {

    stage('Preparation') { 
      // Get some code from a GitHub repository
        checkout scm
    } // end Preparation stage
   
   // Encapsulated builds in try block to allow execution of unit test publication
   // and workspace cleanup
   try {

       // Both Build and Test stages require the same docker image.
       //
       // User is required to be root in order to run the build properly. 
       // This is because the jenkins user owns the directory when checking out,
       // but we need ubuntu/root to be able to modify those files inside the Docker image.
       // The use of the volume option when running the image is the underlying 
       // cause for why we have to do that. The reason we use a volume image though
       // is so that we can delete the workspace. Since the docker image
       // persists, we want to try to keep it as clean as possible. If we checked out
       // code inside the image, it would stay in there (unless we delete it in the docker image I guess).
       docker.image('frc900/zebros-dev:latest').inside('--user root:root -v ' + env.WORKSPACE + ':/home/ubuntu/2018Offseason -l /bin/bash') { c ->
            
            try {
                stage('Build') {
                
                    sh '''#!/bin/bash
                        cd /home/ubuntu/2018Offseason
                        git log -n1
                        git submodule update --init --recursive
                        ./install_ros_desktop_packages.sh
                        cd zebROS_ws
                        wstool update -t src --continue-on-error
                        source /opt/ros/kinetic/setup.bash
                        catkin_make
                        source devel/setup.bash
                        timeout -k 30 --preserve-status 60 roslaunch ros_control_boilerplate 2018_main_frcrobot.launch hw_or_sim:=sim output:=screen
                    '''
                } // end Build stage
            
            
                stage('Test') {
                    sh '''#!/bin/bash
                        chmod -R 777 .
                        cd zebROS_ws
                        wstool update -t src --continue-on-error
                        source /opt/ros/kinetic/setup.bash
                        source devel/setup.bash
                        catkin_make run_tests
                    '''
                } // end Test stage

                // We want to be able to clean the workspace with deleteDir() or similar option
                // at the end of the build. We cannot delete the directory here since
                // we have to publish the jUnit test reports that catkin so lovingly made for us.
                // We can't change the owner to jenkins in the docker image, because it doesn't
                // know about jenkins. deleteDir() also won't delete root owned directories,
                // which these are because of reasons specified earlier.
                // So we use chmod to change permissions to allow jenkins to delete it.
                // It's okay because even though we give anyone on earth permission to touch
                // these files, jenkins will soon delete them.
                // Reference: https://issues.jenkins-ci.org/browse/JENKINS-24440

            } finally {
                sh '''#!/bin/bash
                    chmod -R 777 .
                '''
            } // end try-finally (always update perms)

        } // end Docker Image
    } // end try
    finally {
        junit testResults: 'zebROS_ws/build/test_results/**/*.xml'
        deleteDir()
    } // end finally

} // end Node