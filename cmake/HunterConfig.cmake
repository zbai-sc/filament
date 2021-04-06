hunter_config(civetweb VERSION 1.11-p0
             CMAKE_ARGS CIVETWEB_ENABLE_WEBSOCKETS=ON)

hunter_config(SPIRV-Headers VERSION 1.5.1.corrected)

hunter_config(SPIRV-Tools VERSION 2020.1-p0)

hunter_config(spirv-cross 
    URL "https://github.com/KhronosGroup/SPIRV-Cross/archive/2021-01-15.tar.gz"
    SHA1 af14f12634e98ff2e1ee64502b86f2e17d1a02dd)

hunter_config(glslang VERSION 8.13.3743-9eef54b2-p0
              CMAKE_ARGS ENABLE_HLSL=OFF ENABLE_GLSLANG_BINARIES=OFF ENABLE_OPT=OFF BUILD_TESTING=OFF)

hunter_config(astc-encoder VERSION 1.3-a47b80f-p1)

hunter_config(VulkanMemoryAllocator VERSION 2.3.0-p0)

hunter_config(cgltf 
    URL "https://github.com/elisemorysc/cgltf/archive/hunter-1.9.tar.gz"
    SHA1 79017ceb5cfe363ca30c11f18033f7be38ac5ee9)