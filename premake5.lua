solution "corridormap"
    platforms { "x64", "x32" }
    configurations { "release", "debug" }
    language "C++"

    flags { "NoPCH", "NoRTTI", "FatalWarnings" }
    warnings "Extra"
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
        flags { "FatalWarnings", "NoExceptions" }
        warnings "Extra"
        files { "source/**.cpp" }
        includedirs { "include" }
