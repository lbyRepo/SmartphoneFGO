plugins {
    id("com.android.application")
    id("org.jetbrains.kotlin.android")
}

val nativeAndroidEnabled = providers.gradleProperty("navgraph.enableNative")
    .map(String::toBoolean)
    .getOrElse(true)

android {
    namespace = "com.example.navgraph"
    compileSdk = 36
    ndkVersion = "29.0.14206865"

    defaultConfig {
        applicationId = "com.example.navgraph"
        minSdk = 30
        targetSdk = 36
        versionCode = 13
        versionName = "0.13.0"
        testInstrumentationRunner =
            "com.example.navgraph.estimator.NativeParityInstrumentation"
        if (nativeAndroidEnabled) {
            ndk {
                abiFilters += "arm64-v8a"
            }
        }
    }

    buildFeatures { buildConfig = true }

    kotlinOptions {
        jvmTarget = "17"
    }

    compileOptions {
        sourceCompatibility = JavaVersion.VERSION_17
        targetCompatibility = JavaVersion.VERSION_17
    }

    if (nativeAndroidEnabled) {
        externalNativeBuild {
            cmake {
                path = rootProject.projectDir.resolve("cpp/CMakeLists.txt")
                version = "3.22.1"
            }
        }
    }
}

dependencies {
    implementation("org.maplibre.gl:android-sdk:13.7.1")
    implementation("com.squareup.okhttp3:okhttp:4.12.0")
    testImplementation(kotlin("test"))
    testImplementation("junit:junit:4.13.2")
}

val nativeParityRequested = gradle.startParameter.taskNames.any {
    it.endsWith("nativeHostParityTest")
}
val hostNativeOutput = File(
    System.getProperty("java.io.tmpdir"),
    "navgraph_native_parity/libnavgraph_estimator.dylib",
)
val buildHostNativeParityLibrary = tasks.register<Exec>("buildHostNativeParityLibrary") {
    group = "verification"
    description = "Builds the host JNI GNSS WLS, INS, and EKF library used by parity tests."
    onlyIf { System.getProperty("os.name").contains("Mac", ignoreCase = true) }
    doFirst { hostNativeOutput.parentFile.mkdirs() }
    val cpp = rootProject.projectDir.resolve("cpp")
    val jdk = File("/Library/Java/JavaVirtualMachines/amazon-corretto-11.jdk/Contents/Home")
    inputs.files(
        cpp.resolve("gnss/gnss.cpp"),
        cpp.resolve("ins/ins.cpp"),
        cpp.resolve("core/frame.cpp"),
        cpp.resolve("ekf/ekf.cpp"),
        cpp.resolve("fgo/fgo.cpp"),
        cpp.resolve("jni/navgraph_jni.cpp"),
        cpp.resolve("include/navgraph/gnss.hpp"),
        cpp.resolve("include/navgraph/ins.hpp"),
        cpp.resolve("include/navgraph/frame.hpp"),
        cpp.resolve("include/navgraph/ekf.hpp"),
        cpp.resolve("include/navgraph/fgo.hpp"),
    )
    outputs.file(hostNativeOutput)
    commandLine(
        "xcrun", "clang++", "-std=c++17", "-Wall", "-Wextra", "-Werror", "-pedantic",
        "-dynamiclib",
        "-I${cpp.resolve("include")}",
        "-I${jdk.resolve("include")}",
        "-I${jdk.resolve("include/darwin")}",
        cpp.resolve("gnss/gnss.cpp"),
        cpp.resolve("ins/ins.cpp"),
        cpp.resolve("core/frame.cpp"),
        cpp.resolve("ekf/ekf.cpp"),
        cpp.resolve("fgo/fgo.cpp"),
        cpp.resolve("jni/navgraph_jni.cpp"),
        "-o", hostNativeOutput,
    )
}

tasks.withType<Test>().configureEach {
    if (nativeParityRequested) {
        dependsOn(buildHostNativeParityLibrary)
        outputs.upToDateWhen { false }
    }
}

tasks.register("nativeHostParityTest") {
    group = "verification"
    description = "Runs deterministic Kotlin/JNI C++ GNSS WLS, INS, and EKF parity."
    dependsOn("testDebugUnitTest")
}

tasks.register("printNativeConfiguration") {
    group = "help"
    description = "Prints the opt-in Android native build configuration."
    doLast {
        println("nativeAndroidEnabled=$nativeAndroidEnabled")
        println("ndkVersion=${android.ndkVersion}")
        println("cmakeVersion=3.22.1")
        println("abi=arm64-v8a")
    }
}
