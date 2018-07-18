// Copyright (C) 2008-2014 LAAS-CNRS, JRL AIST-CNRS.
//
// This file is part of jrl-cmakemodules.
// jrl-mathtools is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// jrl-mathtools is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// You should have received a copy of the GNU Lesser General Public License
// along with jrl-mathtools.  If not, see <http://www.gnu.org/licenses/>.

#ifndef HQP_CONFIG_HH
# define HQP_CONFIG_HH

// Package version (header).
# define HQP_VERSION "UNKNOWN-dirty"

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
# if defined _WIN32 || defined __CYGWIN__
// On Microsoft Windows, use dllimport and dllexport to tag symbols.
#  define HQP_DLLIMPORT __declspec(dllimport)
#  define HQP_DLLEXPORT __declspec(dllexport)
#  define HQP_DLLLOCAL
# else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#   define HQP_DLLIMPORT __attribute__ ((visibility("default")))
#   define HQP_DLLEXPORT __attribute__ ((visibility("default")))
#   define HQP_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#   define HQP_DLLIMPORT
#   define HQP_DLLEXPORT
#   define HQP_DLLLOCAL
#  endif // __GNUC__ >= 4
# endif // defined _WIN32 || defined __CYGWIN__

# ifdef HQP_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define HQP_DLLAPI
#  define HQP_LOCAL
# else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef hqp_EXPORTS
#   define HQP_DLLAPI HQP_DLLEXPORT
#  else
#   define HQP_DLLAPI HQP_DLLIMPORT
#  endif // HQP_EXPORTS
#  define HQP_LOCAL HQP_DLLLOCAL
# endif // HQP_STATIC
#endif //! HQP_CONFIG_HH
