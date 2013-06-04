/**
 * Copyright (c) 2013 Christian Kerl <christian.kerl@in.tum.de>
 *
 * Permission is hereby granted, free of charge, to any person obtaining
 * a copy of this software and associated documentation files (the
 * "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish,
 * distribute, sublicense, and/or sell copies of the Software, and to
 * permit persons to whom the Software is furnished to do so, subject to
 * the following conditions:
 *
 * The above copyright notice and this permission notice shall be
 * included in all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,
 * EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
 * NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE
 * LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION
 * WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#ifndef TRAITS_H_
#define TRAITS_H_

#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/DoubleParameter.h>
#include <dynamic_reconfigure/StrParameter.h>

namespace rdynamic_reconfigure
{
// ---------------------------- traits ----------------------------
template<typename Type>
struct TypeTraits
{
};

template<>
struct TypeTraits<int>
{
  static const bool IsIntervalType = true;
  static const char* Name() { return "int"; }
  static int DefaultValue() { return 0; }
  static int DefaultMin() { return std::numeric_limits<int>::min(); }
  static int DefaultMax() { return std::numeric_limits<int>::max(); }
  typedef dynamic_reconfigure::IntParameter NativeType;
};

template<>
struct TypeTraits<double>
{
  static const bool IsIntervalType = true;
  static const char* Name() { return "double"; }
  static double DefaultValue() { return 0.0; }
  static double DefaultMin() { return std::numeric_limits<double>::min(); }
  static double DefaultMax() { return std::numeric_limits<double>::max(); }
  typedef dynamic_reconfigure::DoubleParameter NativeType;
};

template<>
struct TypeTraits<std::string>
{
  static const bool IsIntervalType = false;
  static const char* Name() { return "str"; }
  static std::string DefaultValue() { return ""; }
  static std::string DefaultMin() { return DefaultValue(); }
  static std::string DefaultMax() { return DefaultValue(); }
  typedef dynamic_reconfigure::StrParameter NativeType;
};

template<>
struct TypeTraits<bool>
{
  static const bool IsIntervalType = false;
  static const char* Name() { return "bool"; }
  static bool DefaultValue() { return false; }
  static bool DefaultMin() { return DefaultValue(); }
  static bool DefaultMax() { return DefaultValue(); }
  typedef dynamic_reconfigure::BoolParameter NativeType;
};

} /* namespace rdynamic_reconfigure */
#endif /* TRAITS_H_ */
