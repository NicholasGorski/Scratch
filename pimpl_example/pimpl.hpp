#ifndef PIMPL_PIMPL_HPP_INCLUDED_
#define PIMPL_PIMPL_HPP_INCLUDED_

#include <type_traits>
#include <utility>

#include "detail/pimpl.hpp"

namespace pimpl {

// Used to define the lifetime management functions for a pimpl
// implementation class. Once the class has been made complete,
// use this macro from the global namespace, providing the
// fully-qualified identifier for the implementation class.
#define PIMPL_PTR_MANAGE(impl) PIMPL_PTR_MANAGE_DETAIL(impl)

// Provides a dynamically-allocated pointer to Impl.
//
// This is not movable nor copyable. You would probably want to use
// policies of some sort to do this, as it's not necessarily correct
// to simply move/copy the impl pointer (leaving the source without
// an implementation), nor eagerly allocate and move/copy the impl
// itself via the mananger (adding overhead and providing a possibly
// throwing move constructor). In its current form, it is never empty.
//
// Construction of pimpl_ptr must occur after Impl is made complete.
//
// In the translation unit that Impl is made complete, use the
// PIMPL_PTR_MAMANGE macro in the global namespace to define the
// lifetime management functions for pimpl_ptr.
//
// TODO: it should be relatively straight-forward to make this an
// allocator-aware container for allocating and deallocating the
// raw memory used to store Impl, and modify detail::manager to
// operate on raw buffers ("copy-construct into this buffer", or
// "destroy (but do not deallocate) impl"). This would allow a
// user to provide a statically-sized buffer as the storage, if
// they wish to use pimpl without dynamic allocation (at the cost
// of needing to update the size when it changes/breaking ABI).
template <typename Impl>
class pimpl_ptr {
public:
    static_assert(!std::is_array<Impl>{}, "Cannot impl with an array.");

    typedef detail::manager<Impl> manager_t;

    template <typename... Args>
    pimpl_ptr(Args&&... args) :
    impl_(new Impl(std::forward<Args>(args)...))
    {}

    // TODO: Add copy/move using policies.
    pimpl_ptr(const pimpl_ptr&) = delete;
    pimpl_ptr& operator=(const pimpl_ptr&) = delete;

    ~pimpl_ptr() noexcept {
        manager_t::destroy(impl_);
    }

    Impl* get() noexcept {
        return impl_;
    }

    const Impl* get() const noexcept {
        return impl_;
    }

    Impl& operator*() {
        return *get();
    }

    const Impl& operator*() const {
        return *get();
    }

    Impl* operator->() {
        return get();
    }

    const Impl* operator->() const {
        return get();
    }

private:
    Impl* impl_;
};

}  // pimpl

#endif
