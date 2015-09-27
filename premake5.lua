solution "corridormap"
    platforms { "x64", "x32" }
    configurations { "release", "debug" }
    language "C++"

    objdir ".build/obj"

    configuration "debug"
        flags "Symbols"

    configuration "release"
        optimize "Speed"
        defines { "NDEBUG" }

    configuration { "debug", "x32" }
        targetdir "bin/x32/debug"

    configuration { "release", "x32" }
        targetdir "bin/x32/release"

    configuration { "debug", "x64" }
        targetdir "bin/x64/debug"

    configuration { "release", "x64" }
        targetdir "bin/x64/release"

    configuration { "vs*" }
        defines { "_CRT_SECURE_NO_WARNINGS" }
        buildoptions { "/wd4456" } -- declaration of '...' hides previous local declaration.
        buildoptions { "/wd4577" } -- 'noexcept' used with no exception handling mode specified; termination on exception is not guaranteed.

    project "clew"
        kind "StaticLib"
        language "C"
        files { "deps/clew/clew.c" }

    project "corridormap-library"
        kind "StaticLib"
        flags { "NoPCH", "NoRTTI", "FatalWarnings", "NoExceptions" }
        warnings "Extra"
        files { "source/**.cpp" }
        defines { "CORRIDORMAP_CONFIG_USE_CLEW" }
        includedirs { "include", "deps/OpenCL", "deps/clew" }

    project "glfw"
        kind "StaticLib"
        language "C"

        files { 
            "example/glfw/src/clipboard.c",
            "example/glfw/src/context.c",
            "example/glfw/src/gamma.c",
            "example/glfw/src/init.c",
            "example/glfw/src/input.c",
            "example/glfw/src/joystick.c",
            "example/glfw/src/monitor.c",
            "example/glfw/src/time.c",
            "example/glfw/src/window.c" }

        includedirs { "example/glfw/include" }
        defines { "_GLFW_USE_OPENGL" }

        configuration { "windows" }
            files { "example/glfw/src/win32_*.c", "example/glfw/src/wgl_*.c" }
            defines { "_CRT_SECURE_NO_WARNINGS", "_GLFW_WIN32", "_GLFW_WGL" }

    project "glew"
        kind "StaticLib"
        language "C"
        files { "example/glew/src/glew.c" }
        defines { "GLEW_STATIC" }
        includedirs { "example/glew/include" }

        configuration { "windows" }
            defines { "_CRT_SECURE_NO_WARNINGS" }

    project "nanovg"
        kind "StaticLib"
        language "C"
        files { "example/nanovg/src/*.c" }
        includedirs { "example/nanovg/src" }

        configuration "vs*"
            -- x64 build stb_image.c: conversion from '__int64' to 'int', possible loss of data
            buildoptions { "/wd4244" }

    project "example-voronoi"
        kind "ConsoleApp"
        flags { "NoPCH", "NoRTTI", "FatalWarnings", "NoExceptions" }
        warnings "Extra"
        includedirs { "include", "deps/OpenCL", "deps/clew", "example/glfw/include", "example/glew/include" }
        files { "example/voronoi.cpp" }
        defines { "GLEW_STATIC" }
        links { "corridormap-library", "glew", "glfw", "clew" }

        configuration { "windows" }
            links { "opengl32" }

    project "example-corridor"
        kind "ConsoleApp"
        flags { "NoPCH", "NoRTTI", "FatalWarnings", "NoExceptions" }
        includedirs { "include", "deps/OpenCL", "deps/clew", "example/glfw/include", "example/glew/include", "example/nanovg/src" }
        files { "example/corridor.cpp", "example/draw.cpp" }
        defines { "GLEW_STATIC" }
        links { "corridormap-library", "glew", "glfw", "clew", "nanovg" }

        configuration { "windows" }
            links { "opengl32" }
