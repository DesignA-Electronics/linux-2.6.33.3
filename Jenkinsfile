node {
    stage('SCM') {
        checkout scm
    }

    stage('Build') {
        sh "make ARCH=arm V=1 CROSS_COMPILE=/var/lib/jenkins/workspace/toolchains/arm-2010q1/bin/arm-none-linux-gnueabi- salmon_test_system_defconfig"
        sh "make ARCH=arm V=1 CROSS_COMPILE=/var/lib/jenkins/workspace/toolchains/arm-2010q1/bin/arm-none-linux-gnueabi- uImage zImage modules"
    }

    stage('Archive Artifacts') {
    }
}

