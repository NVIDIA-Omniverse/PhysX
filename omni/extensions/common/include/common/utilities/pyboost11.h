/*
 * Copyright (c) 2021, Yung-Yu Chen <yyc@solvcon.net>
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 *   this list of conditions and the following disclaimer.
 * - Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 * - Neither the name of the copyright holder nor the names of its contributors
 *   may be used to endorse or promote products derived from this software
 *   without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

namespace pyboost11
{

// Pybind11 cast by using Boost.Python.
template <typename T>
struct caster
{
    caster(pybind11::handle src) : obj(boost::python::handle<>(boost::python::borrowed(src.ptr()))), ext(obj)
    {
    }

    bool check() const
    {
        return ext.check();
    }

    // From-Python conversion.
    operator T()
    {
        return ext();
    }
    T operator()()
    {
        return ext();
    }

    // To-Python conversion.
    static pybind11::handle to_python(T& src)
    {
        namespace bpy = boost::python;
        return bpy::incref(bpy::object(src).ptr());
    }

    boost::python::object obj;
    boost::python::extract<T> ext;
};

} // end namespace pyboost11

namespace pybind11
{

namespace detail
{

template <typename type>
struct pyboost11_type_caster
{
protected:
    type value;

public:
    template <typename T_, enable_if_t<std::is_same<type, remove_cv_t<T_>>::value, int> = 0>
    static handle cast(T_* src, return_value_policy policy, handle parent)
    {
        if (!src)
            return none().release();
        if (policy == return_value_policy::take_ownership)
        {
            auto h = cast(std::move(*src), policy, parent);
            delete src;
            return h;
        }
        else
        {
            return cast(*src, policy, parent);
        }
    }
    operator type*()
    {
        return &value;
    }
    operator type&()
    {
        return value;
    }
    operator type&&() &&
    {
        return std::move(value);
    }
    template <typename T_>
    using cast_op_type = pybind11::detail::movable_cast_op_type<T_>;

    // Boilerplate.
    bool load(handle src, bool)
    {
        if (!src)
        {
            return false;
        }
        pyboost11::caster<type> ext(src);
        if (!ext.check())
        {
            return false;
        }
        value = ext();
        return true;
    }
    static handle cast(type* src, return_value_policy /* policy */, handle /* parent */)
    {
        return pyboost11::caster<type>::to_python(src);
    }
    static handle cast(type src, return_value_policy /* policy */, handle /* parent */)
    {
        return pyboost11::caster<type>::to_python(src);
    }
};

#define PYBOOST11_TYPE_CASTER(type, py_name)                                                                           \
    template <>                                                                                                        \
    struct type_caster<type> : public pyboost11_type_caster<type>                                                      \
    {                                                                                                                  \
        static constexpr auto name = py_name;                                                                          \
    }

} // end namespace detail

} // end namespace pybind11
