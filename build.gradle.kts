plugins {
    id("us.ihmc.ihmc-build")
}

ihmc {
    group = "us.ihmc"
    version = "0.6.0"
    vcsUrl = "https://bitbucket.ihmc.us/scm/libs/ihmc-ros-control.git"
    openSource = true
    maintainer = "Duncan Calvert"

    configureDependencyResolution()
    configurePublications()
}

mainDependencies {

}
