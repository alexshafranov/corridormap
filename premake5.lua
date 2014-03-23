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

    project "corridormap-library"
        kind "StaticLib"
        flags { "NoPCH", "NoRTTI", "FatalWarnings", "NoExceptions" }
        warnings "Extra"
        files { "source/**.cpp" }
        includedirs { "include" }

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

    project "example-voronoi"
        kind "ConsoleApp"
        flags { "NoPCH", "NoRTTI", "FatalWarnings", "NoExceptions" }
        warnings "Extra"
        includedirs { "include" }
        files { "example/voronoi.cpp" }
        links { "corridormap-library", "glfw" }

        configuration { "windows" }
            links { "opengl32" }
