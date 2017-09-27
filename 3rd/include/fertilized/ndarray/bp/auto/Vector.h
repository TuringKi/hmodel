// -*- c++ -*-
/*
 * Copyright (c) 2010-2012, Jim Bosch
 * All rights reserved.
 *
 * ndarray is distributed under a simple BSD-like license;
 * see the LICENSE file that should be present in the root
 * of the source distribution, or alternately available at:
 * https://github.com/ndarray/ndarray
 */
// THIS FILE IS MACHINE GENERATED BY SCONS. DO NOT EDIT MANUALLY.
#ifndef NDARRAY_BP_AUTO_Vector_h_INCLUDED
#define NDARRAY_BP_AUTO_Vector_h_INCLUDED

#include "boost/numpy.hpp"
#include "fertilized/ndarray/bp/Vector.h"


namespace boost { namespace python {

template <typename T, int N>
struct to_python_value< ndarray::Vector<T,N> const & > : public detail::builtin_to_python {
    inline PyObject * operator()(ndarray::Vector<T,N> const & x) const {
        object result = ndarray::ToBoostPython< ndarray::Vector<T,N> >::apply(x);
        Py_INCREF(result.ptr());
        return result.ptr();
    }
    inline PyTypeObject const * get_pytype() const {
        return converter::object_manager_traits<
            typename ndarray::ToBoostPython< ndarray::Vector<T,N> >::result_type
        >::get_pytype();
    }
};

template <typename T, int N>
struct to_python_value< ndarray::Vector<T,N> & > : public detail::builtin_to_python {
    inline PyObject * operator()(ndarray::Vector<T,N> & x) const {
        object result = ndarray::ToBoostPython< ndarray::Vector<T,N> >::apply(x);
        Py_INCREF(result.ptr());
        return result.ptr();
    }
    inline PyTypeObject const * get_pytype() const {
        return converter::object_manager_traits<
            typename ndarray::ToBoostPython< ndarray::Vector<T,N> >::result_type
        >::get_pytype();
    }
};

namespace converter {

template <typename T, int N>
struct arg_to_python< ndarray::Vector<T,N> > : public handle<> {
    inline arg_to_python(ndarray::Vector<T,N> const & v) :
        handle<>(to_python_value<ndarray::Vector<T,N> const &>()(v)) {}
};

template <typename T, int N>
struct arg_rvalue_from_python< ndarray::Vector<T,N> const & > {
    typedef ndarray::Vector<T,N> result_type;
    arg_rvalue_from_python(PyObject * p) :
        _converter(boost::python::object(boost::python::handle<>(boost::python::borrowed(p)))) {}
    bool convertible() const { return _converter.convertible(); }
    result_type operator()() const { return _converter(); }
private:
    mutable ndarray::FromBoostPython< ndarray::Vector<T,N> > _converter;
};

template <typename T, int N>
struct arg_rvalue_from_python< ndarray::Vector<T,N> > : public arg_rvalue_from_python< ndarray::Vector<T,N> const &> {
    arg_rvalue_from_python(PyObject * p) : arg_rvalue_from_python< ndarray::Vector<T,N> const & >(p) {}
};

template <typename T, int N>
struct arg_rvalue_from_python< ndarray::Vector<T,N> const > : public arg_rvalue_from_python< ndarray::Vector<T,N> const &> {
    arg_rvalue_from_python(PyObject * p) : arg_rvalue_from_python< ndarray::Vector<T,N> const & >(p) {}
};

template <typename T, int N>
struct extract_rvalue< ndarray::Vector<T,N> > : private noncopyable {
    typedef ndarray::Vector<T,N> result_type;
    extract_rvalue(PyObject * x) : m_converter(x) {}
    bool check() const { return m_converter.convertible(); }
    result_type operator()() const { return m_converter(); }
private:
    arg_rvalue_from_python< result_type const & > m_converter;
};

}}} // namespace boost::python::converter

#endif // !NDARRAY_BP_AUTO_Vector_h_INCLUDED
