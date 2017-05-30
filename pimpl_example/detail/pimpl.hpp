#ifndef PIMPL_DETAIL_PIMPL_HPP_INCLUDED_
#define PIMPL_DETAIL_PIMPL_HPP_INCLUDED_

namespace pimpl {

namespace detail {

// This declares, but does not define, the necessary functions
// for managing pimpl lifetimes. When implicit instantiation
// occurs, no definitions will exist; this allows us to defer
// the definition until the point after Impl is made complete.
template <typename Impl>
struct manager {
    static void destroy(Impl* const ptr) noexcept;

    // E.g., to support copying you could add:
    // static Impl* copy(const Impl& other);
};

// Actually defines the functions of the manager template,
// then explicitly instantiates them for impl (now complete).
//
// WARNING: It is very important that is *not* a specialization of
// manager, or the behavior would be undefined (specializations must
// be declared prior to any implicit instantiation that occurs).
// Rather, this provides a consistent definition of the non-specialized
// manager class at all locations (giving well-defined behavior),
// and then ensures a definition for Impl is instantiated.
#define PIMPL_PTR_MANAGE_DETAIL(impl)                                           \
        template <typename Impl>                                                \
        void pimpl::detail::manager<Impl>::destroy(Impl* const ptr) noexcept {  \
            static_assert(sizeof(impl) > 0, "Impl must be complete.");          \
            delete ptr;                                                         \
        }                                                                       \
                                                                                \
        template class pimpl::detail::manager<impl>;

}  // detail

}  // pimpl

#endif
