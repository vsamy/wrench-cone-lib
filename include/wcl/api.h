// wrenh-cone-lib: computes a Contact Wrench Cone using polyhedral presentation
// Copyright (C) 2019 Vincent Samy

// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.

// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.

// You should have received a copy of the GNU General Public License
// along with this program.  If not, see <http://www.gnu.org/licenses/>.

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
