node {

    failed_stage = ''
    test_results = ''
    full_commit = ''
    author = ''

    stage('Preparation') { 
      // Get some code from a GitHub repository
        failed_stage = env.STAGE_NAME
        checkout scm

        full_commit = sh(returnStdout: true, script: "git log -n 1 --pretty=format:'%H'").trim()
        author = sh(returnStdout: true, script: "git log -n 1 --pretty=format:'%an'").trim()

    } // end Preparation stage
   
   // Encapsulate builds in try block to allow unconditional execution of unit test publication
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
            
            // This try-finally block is required to always change permissions
            // inside the docker image to allow Jenkins to finally delete it at the end.
            try {
                stage('Build') {
                    
                    failed_stage = env.STAGE_NAME
                
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
                    failed_stage = env.STAGE_NAME
                    sh '''#!/bin/bash
                        cd zebROS_ws
                        wstool update -t src --continue-on-error
                        source /opt/ros/kinetic/setup.bash
                        source devel/setup.bash
                        catkin_make run_tests
                    '''
                    
                    // This script forces an exit 0 because the catkin test
                    // results will set the exit code to 1 if there are any failing
                    // tests. This has the effect of failing the build, which we don't want.
                    test_results = sh(returnStdout: true, script: '''#!/bin/bash
                            cd zebROS_ws
                            source /opt/ros/kinetic/setup.bash
                            catkin_test_results build/test_results
                            exit 0
                        '''
                    ).trim()
                } // end Test stage

            } finally {

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
                sh "chmod -R 777 ."

            } // end try-finally (always update perms)
        } // end Docker Image
    } // end try
    catch(exc) {

        // Build must be set manually to a failure here because the result is not
        // stored until the end of the pipeline, after we send the notification
        // to Slack. As a result, we have to set it prematurely here.
        // Reference: https://issues.jenkins-ci.org/browse/JENKINS-47403
        currentBuild.result = 'FAILURE'
    }
    finally {
        
        junit allowEmptyResults: true, healthScaleFactor: 1.0, testResults: 'zebROS_ws/build/test_results/**/*.xml'
        deleteDir()
        notifySlack(currentBuild.result, full_commit, author, failed_stage)

    } // end finally
    
} // end Node


def notifySlack(
    String buildStatus = 'STARTED',
    String commit='', 
    String author='', 
    String failed_stage=''
    ) {
    
    // Build status of null means success.
    buildStatus = buildStatus ?: 'SUCCESS'



    def color
    if (buildStatus == 'STARTED') {
        color = '#D4DADF'
    } else if (buildStatus == 'SUCCESS') {
        color = 'good'
    } else if (buildStatus == 'UNSTABLE') {
        color = 'warning'
    } else {
        color = 'danger'
    }


    tokens = "${env.JOB_NAME}".tokenize('/')
    org = tokens[tokens.size()-3]
    repo = tokens[tokens.size()-2]
    branch = tokens[tokens.size()-1]

    

    if (buildStatus != 'FAILURE') {

        // This is what the test results message looks like for parsing purposes.
        //Summary: 208 tests, 0 errors, 0 failures, 0 skipped
        results = "${test_results}".tokenize("\n")
        summary = results[results.size()-2]

        // test_details = ['208 tests', ' 0 errors', ' 0 failures', ' 0 skipped']
        test_details = summary.tokenize(',')

        // Get the errors message
        // Split by space, first element in array is the number we want
        errors = test_details[1].trim()
        errors = errors.split()[0]
        errors = errors.toInteger()
        
        // Do the same for number of failures
        failures = test_details[2].trim()
        failures = fails.split()[0].toInteger()

        if (errors || failures) {
            color = 'warning'
            buildStatus = 'UNSTABLE'
        }
    } // end if build not a failure

    commit_url = "https://github.com/FRC900/${repo}/commit/${commit}"
    repo_slug = "${repo}@${branch}"

    duration = currentBuild.durationString
    duration = duration.reverse().drop(13).reverse() // Remove ' and counting' (13 chars)

    short_commit = commit.take(7) // The first 7 chars are the shorthand for a Git commit
    

    // Start building the notification
    msg = "Build <${env.RUN_DISPLAY_URL}|#${env.BUILD_NUMBER}> (<${commit_url}|${short_commit}>)\n"
    msg = msg + "${repo_slug} by ${author}\n"
    msg = msg + "${buildStatus}"

    if (buildStatus == 'FAILURE') {
        msg = msg + " at stage ${failed_stage}"
    }

    msg = msg + " in ${duration}.\n"
    msg = msg + "Test ${summary}"


    slackSend(
        color: color,
        baseUrl: 'https://frc900.slack.com/services/hooks/jenkins-ci/', 
        message: msg, 
        tokenCredentialId: 'slack-token'
    )

} // end notifySlack()
