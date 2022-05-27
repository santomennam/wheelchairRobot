#include "paths.h"
#include "whereami.h"

#include <vector>


#ifdef __APPLE__
#include <Availability.h> // for deployment target to support pre-catalina targets without std::fs
#endif
#if ((defined(_MSVC_LANG) && _MSVC_LANG >= 201703L) || (defined(__cplusplus) && __cplusplus >= 201703L)) && defined(__has_include)
#if __has_include(<filesystem>) && (!defined(__MAC_OS_X_VERSION_MIN_REQUIRED) || __MAC_OS_X_VERSION_MIN_REQUIRED >= 101500)
#define GHC_USE_STD_FS
#include <filesystem>
namespace fs = std::filesystem;
#endif
#endif
#ifndef GHC_USE_STD_FS
#include <ghc/filesystem.hpp>
namespace fs = ghc::filesystem;
#endif


std::string Paths::executablePath()
{
    size_t length = static_cast<size_t>(wai_getExecutablePath(nullptr, 0, nullptr));
    std::vector<char> buffer(length+1,'\0');
    wai_getExecutablePath(buffer.data(), static_cast<int>(length), nullptr);
    return std::basic_string<char>(buffer.data(),length);
}

std::string Paths::findAsset(const std::string& filename)
{
    fs::path dir = executablePath();

    fs::path exeFolder = dir.remove_filename();

    std::string dirOnly = dir.parent_path().filename().string();

    if (dirOnly == "MacOS") {
        fs::path resource = fs::weakly_canonical(exeFolder / "../Resources");
        if (fs::exists(resource)) {
            dir = resource;
        }
    }
    else if (fs::exists(exeFolder / "Assets")) {
        dir = exeFolder / "Assets";
    }
    else if (fs::exists(exeFolder / "assets")) {
        dir = exeFolder / "assets";
    }

    dir /= filename;

    if (fs::exists(dir)) {
//#if defined(__cpp_lib_char8_t)
        return dir.string();
//#else
//        return dir.u8string();
//#endif
    }

    return filename;
}
