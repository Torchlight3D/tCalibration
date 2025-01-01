#pragma once

#define Q_DECLARE_PIMPL(Class) \
    Q_DISABLE_COPY(Class)      \
    Q_DECLARE_PRIVATE(Class)   \
    const QScopedPointer<Class##Private> d_ptr;

#define Q_DEFINE_PIMPL(Class)      \
    Q_DISABLE_COPY(Class##Private) \
    Q_DECLARE_PUBLIC(Class)        \
    Class* const q_ptr;

namespace tl {

} // namespace tl
