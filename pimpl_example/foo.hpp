#ifndef EXAMPLE_FOO_HPP_INCLUDED_
#define EXAMPLE_FOO_HPP_INCLUDED_

#include <string>

#include "pimpl.hpp"

namespace example {

class foo {
public:
    foo();
    foo(std::string msg);

    void do_a_thing();

private:
    class foo_impl;
    pimpl::pimpl_ptr<foo_impl> impl_;
};

}

#endif
