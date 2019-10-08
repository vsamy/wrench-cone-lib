#pragma once

// Handle portable symbol export.
// Defining manually which symbol should be exported is required
// under Windows whether MinGW or MSVC is used.
//
// The headers then have to be able to work in two different modes:
// - dllexport when one is building the library,
// - dllimport for clients using the library.
//
// On Linux, set the visibility accordingly. If C++ symbol visibility
// is handled by the compiler, see: http://gcc.gnu.org/wiki/Visibility
#if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#define WCL_DLLIMPORT __declspec(dllimport)
#define WCL_DLLEXPORT __declspec(dllexport)
#define WCL_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#if __GNUC__ >= 4
#define WCL_DLLIMPORT __attribute__((visibility("default")))
#define WCL_DLLEXPORT __attribute__((visibility("default")))
#define WCL_DLLLOCAL __attribute__((visibility("hidden")))
#else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#define WCL_DLLIMPORT
#define WCL_DLLEXPORT
#define WCL_DLLLOCAL
#endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef WCL_STATIC
// If one is using the library statically, get rid of
// extra information.
#define WCL_DLLAPI
#define WCL_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#ifdef wcl_EXPORTS
#define WCL_DLLAPI WCL_DLLEXPORT
#else
#define WCL_DLLAPI WCL_DLLIMPORT
#endif // WCL_EXPORTS
#define WCL_LOCAL WCL_DLLLOCAL
#endif // WCL_STATIC
