#pragma once

#define Q_DECLARE_PIMPL(Class) \
    Q_DISABLE_COPY(Class)      \
    Q_DECLARE_PRIVATE(Class)   \
    const QScopedPointer<Class##Private> d_ptr;

#define Q_DEFINE_PIMPL(Class)      \
    Q_DISABLE_COPY(Class##Private) \
    Q_DECLARE_PUBLIC(Class)        \
    Class* const q_ptr;

// Make translations in private class go to parent's scope
#define AX_Q_DECLARE_PUBLIC(Class)                                            \
    Q_DECLARE_PUBLIC(Class)                                                   \
    inline static auto Tr(const char* s, const char* c = nullptr, int n = -1) \
    {                                                                         \
        return Class::tr(s, c, n);                                            \
    }

#define AX_Q_DEFINE_PIMPL(Class)   \
    Q_DISABLE_COPY(Class##Private) \
    AX_Q_DECLARE_PUBLIC(Class)     \
    Class* const q_ptr;

class QLayout;

namespace tl {

void initializeResources();

void clearLayout(QLayout* layout, bool deleteWidget = true);

// TODO: Add a QDialog wrapper template

} // namespace tl
