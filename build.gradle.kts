import edu.wpi.first.deployutils.deploy.artifact.FileTreeArtifact
import edu.wpi.first.gradlerio.deploy.roborio.FRCJavaArtifact
import edu.wpi.first.gradlerio.deploy.roborio.RoboRIO
import edu.wpi.first.toolchain.NativePlatforms

plugins {
    java
    alias(libs.plugins.indra.git)
    alias(libs.plugins.shadow)
    alias(libs.plugins.gradlerio)
    id("com.diffplug.spotless") version "6.25.0"
}

group = "edu.msoe.cybercheese"

if (libs.versions.wpilib.get() != wpi.versions.wpilibVersion.get()) {
    throw RuntimeException("wpilib is busted")
}

if (!wpi.vendor.dependencySet.isEmpty()) {
    throw RuntimeException("vendordeps will not work. do not use them")
}

java {
    sourceCompatibility = JavaVersion.VERSION_17
    targetCompatibility = JavaVersion.VERSION_17
}

repositories {
    mavenCentral()
    maven("https://frcmaven.wpi.edu/artifactory/littletonrobotics-mvn-release/")
    maven("https://frcmaven.wpi.edu/artifactory/sleipnirgroup-mvn-release/")
    maven("https://maven.revrobotics.com/")
    maven("https://maven.photonvision.org/repository/internal")
    maven("https://maven.photonvision.org/repository/snapshots")
    maven("https://maven.reduxrobotics.com/")
    maven("https://maven.ctr-electronics.com/release/")
}

val wpilibJni: Configuration by configurations.creating {
    isCanBeResolved = false

    fun addOriginalDependency(dep: ModuleDependency, native: Boolean, debug: Boolean) {
        val configPrefix = if (native) "native" else "roborio"
        val configSuffix = if (debug) "Debug" else "Release"
        val config = configPrefix + configSuffix

        val modified = dep.copy()

        modified.artifact {
            val platformPart = if (native) NativePlatforms.desktop else NativePlatforms.roborio
            val modePart = if (debug) "debug" else ""

            classifier = platformPart + modePart
            type = "zip"
        }

        project.dependencies.add(config, modified)
    }

    dependencies.all {
        if (this is ModuleDependency) {
            addOriginalDependency(this, native = false, debug = false)
            addOriginalDependency(this, native = false, debug = true)
            addOriginalDependency(this, native = true, debug = false)
            addOriginalDependency(this, native = true, debug = true)
        } else {
            throw RuntimeException("only module dependencies can be used in wpilibJni, got $this")
        }
    }
}

wpi.java.debugJni = false

// The sim GUI is *disabled* by default to support running
// AdvantageKit log replay, which requires that all HAL
// sim extensions be disabled.
wpi.sim.addGui().defaultEnabled = !project.properties.containsKey("forceSimGui")
wpi.sim.addDriverstation()

wpi.java.configureExecutableTasks(tasks.shadowJar.get())
wpi.java.configureTestTasks(tasks.test.get())

// NOTE: this needs to be before we try to use the rio configurations since we gradlerio is dumb and registers them in FRCJavaArtifact
deploy.targets.register<RoboRIO>("roborio") {
    team = project.frc.getTeamOrDefault(8847)
    debug = project.frc.getDebugOrDefault(false)

    with (artifacts) {
        register<FRCJavaArtifact>("frcJava") {
            setJarTask(tasks.shadowJar.get())

            jvmArgs.add("-XX:+UnlockExperimentalVMOptions")
            jvmArgs.add("-XX:GCTimeRatio=5")
            jvmArgs.add("-XX:+UseSerialGC")
            jvmArgs.add("-XX:MaxGCPauseMillis=50")

            // The options below may improve performance, but should only be enabled on the RIO 2
            //
            // final MAX_JAVA_HEAP_SIZE_MB = 100;
            // jvmArgs.add("-Xmx" + MAX_JAVA_HEAP_SIZE_MB + "M")
            // jvmArgs.add("-Xms" + MAX_JAVA_HEAP_SIZE_MB + "M")
            // jvmArgs.add("-XX:+AlwaysPreTouch")
        }
    }
}

dependencies {
    val wpiDeps = listOf(
        "annotationProcessor" to wpi.java.deps.wpilibAnnotations(),
        "implementation" to wpi.java.deps.wpilib(),
        "roborioDebug" to wpi.java.deps.wpilibJniDebug(NativePlatforms.roborio),
        "roborioRelease" to wpi.java.deps.wpilibJniRelease(NativePlatforms.roborio),
        "nativeDebug" to wpi.java.deps.wpilibJniDebug(NativePlatforms.desktop),
        "nativeRelease" to wpi.java.deps.wpilibJniRelease(NativePlatforms.desktop),

        "simulationDebug" to wpi.sim.enableDebug(),
        "simulationRelease" to wpi.sim.enableRelease(),
    )

    for ((config, deps) in wpiDeps) {
        for (dep in deps) {
            add(config, dep)
        }
    }

    val phoenix6Version = "26.1.1"

    implementation("com.ctre.phoenix6:wpiapi-java:$phoenix6Version")
    wpilibJni("com.ctre.phoenix6:api-cpp:$phoenix6Version")
    wpilibJni("com.ctre.phoenix6:tools:$phoenix6Version")

    implementation(libs.jspecify)
    implementation(libs.fastutil)

    implementation(libs.wpilib.commands)

    implementation(libs.reduxlib)
    wpilibJni(libs.reduxlib.fifo)

    implementation(libs.revlib)
    wpilibJni(libs.revlib.driver)
    wpilibJni(libs.revlib.driver.backend)
    wpilibJni(libs.revlib.driver.backend.wpi)

    implementation(libs.urcl)
    wpilibJni(libs.urcl.driver)

    implementation(libs.akit)
    wpilibJni(libs.akit.wpilibio)

    implementation(libs.photonvision)
    implementation(libs.photonvision.targeting)
    wpilibJni(libs.photonvision.targeting.native)

    implementation(libs.choreo)
}

tasks {
    register<JavaExec>("replayWatch") {
        mainClass = "org.littletonrobotics.junction.ReplayWatch"
        classpath = sourceSets.main.get().runtimeClasspath
    }

    withType<JavaCompile> {
        options.compilerArgs.add("-XDstringConcat=inline")
    }

    withType<Jar> {
        manifest.attributes.putAll(mapOf(
            "Main-Class" to "edu.msoe.cybercheese.trinity.Main",

            "Build-Timestamp" to System.currentTimeMillis(),
            "Implementation-Title" to "${project.group}:${project.name}",
            "Implementation-Version" to project.version,
        ))

        indraGit.applyVcsInformationToManifest(manifest)
    }
}

spotless {
    java {
        target(fileTree(".") {
            include("**/*.java")
            exclude("**/build/**", "**/build-*/**")
        })
        
        palantirJavaFormat()
    }
}

tasks.compileJava {
    dependsOn(tasks.spotlessApply)
}