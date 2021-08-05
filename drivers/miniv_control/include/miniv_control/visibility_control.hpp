// Copyright 2017 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/* This header must be included by all rclcpp headers which declare symbols
 * which are defined in the rclcpp library. When not building the rclcpp
 * library, i.e. when using the headers in other package's code, the contents
 * of this header change the visibility of certain symbols which the rclcpp
 * library cannot have, but the consuming code must have inorder to link.
 */

#ifndef MINIV_CONTROL__VISIBILITY_CONTROL_HPP_
#define MINIV_CONTROL__VISIBILITY_CONTROL_HPP_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define MINIV_CONTROL_EXPORT __attribute__((dllexport))
#define MINIV_CONTROL_IMPORT __attribute__((dllimport))
#else
#define MINIV_CONTROL_EXPORT __declspec(dllexport)
#define MINIV_CONTROL_IMPORT __declspec(dllimport)
#endif
#ifdef MINIV_CONTROL_BUILDING_DLL
#define MINIV_CONTROL_PUBLIC MINIV_CONTROL_EXPORT
#else
#define MINIV_CONTROL_PUBLIC MINIV_CONTROL_IMPORT
#endif
#define MINIV_CONTROL_PUBLIC_TYPE MINIV_CONTROL_PUBLIC
#define MINIV_CONTROL_LOCAL
#else
#define MINIV_CONTROL_EXPORT __attribute__((visibility("default")))
#define MINIV_CONTROL_IMPORT
#if __GNUC__ >= 4
#define MINIV_CONTROL_PUBLIC __attribute__((visibility("default")))
#define MINIV_CONTROL_LOCAL __attribute__((visibility("hidden")))
#else
#define MINIV_CONTROL_PUBLIC
#define MINIV_CONTROL_LOCAL
#endif
#define MINIV_CONTROL_PUBLIC_TYPE
#endif

#endif  // MINIV_CONTROL__VISIBILITY_CONTROL_HPP_
