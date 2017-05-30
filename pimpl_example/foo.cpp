#include "foo.hpp"

#include <iostream>

namespace example {

struct foo::foo_impl {
    foo_impl(std::string msg) :
    msg_(std::move(msg))
    {}

    void do_a_thing() {
     std::cout << msg_ << std::endl;
    }

private:
    std::string msg_;
};

foo::foo() :
foo("default message")
{}

foo::foo(std::string msg) :
impl_(std::move(msg))
{}

void foo::do_a_thing() {
    impl_->do_a_thing();
}

}

PIMPL_PTR_MANAGE(example::foo::foo_impl);
