#pragma once

namespace thoht {

#define DISABLE_COPY(Class)       \
    Class(const Class&) = delete; \
    Class& operator=(const Class&) = delete;

#define DISABLE_COPY_MOVE(Class) \
    DISABLE_COPY(Class)          \
    Class(Class&&) = delete;     \
    Class& operator=(Class&&) = delete;

#define UNUSED(x) ((void)(x))

} // namespace thoht
