#include "foo.hpp"

static_assert(sizeof(example::foo) == sizeof(void*), "no extra space cost");

int main() {
    example::foo f;
    f.do_a_thing();

    example::foo f2{ "testing" };
    f2.do_a_thing();
}
