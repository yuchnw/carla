#!/usr/bin/env groovy

pipeline {
    agent {
        label "docker"
    }
    options {
        timeout(time: 1, unit: 'HOURS')
        buildDiscarder(logRotator(numToKeepStr: '30', daysToKeepStr: '30'))
        skipDefaultCheckout()
    }
    parameters {
        // See: https://jenkins.io/doc/book/pipeline/syntax/#parameters
    }
    stages {
        stage("Kill off any older (running) builds") {
            steps {
                killOldBuilds()
            }
        }
        stage("Jenkins environment injection") {
            steps {
                script {
                    populateEnv() 
                }
            }
        }
        stage("Checkout") {
            steps {
                checkoutScm()
            }
        }
    }    
    post {
        always {
            echo 'One way or another, I have finished'
            notifyBuild("master")
        }
        fixed {
            echo 'I am healthy again!'
            notifyHealthy("master", "LLaMA-Factory", "eng@plus.ai", "#tech")
        }
        success {
            echo 'I succeeeded!'
        }
        unstable {
            echo 'I am unstable :/'
        }
        aborted {
            echo 'I got aborted :/'
        }
        failure {            
            echo 'I failed :('
            notifyFailure("master", "carla", "eng@plus.ai", "#tech")            
        }
        changed {
            echo 'Things were different before...'
        }
    }
}