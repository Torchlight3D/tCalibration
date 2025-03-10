#pragma once

#include <memory>
#include <string>
#include <unordered_map>

namespace tl {

std::string demangle(const char* name);

// TODO:
// 1. The registered key is a full class name with namespace. Need some
// post-processing if neccessary.
// 2. Multi-level inheritance is not supported.
template <class Base, class... Args>
class Factory
{
public:
    template <class T>
    struct Registry : Base
    {
        friend T;

        static bool registered;

    private:
        Registry() : Base(Key{}) { (void)registered; }
    };

    template <class... T>
    static std::unique_ptr<Base> create(const std::string& name, T&&... args)
    {
        const auto& creators = creatrorsMap();
        if (const auto found = creators.find(name); found == creators.cend()) {
            return nullptr;
        }
        else {
            return found->second(std::forward<T>(args)...);
        }
    }

    friend Base;

private:
    // Use PassKey pattern to prevent any class from directly inheriting the
    // Base class
    class Key
    {
        Key() {}

        template <class T>
        friend struct Registry;
    };

    Factory() = default;

    using BaseCreator = std::add_pointer_t<std::unique_ptr<Base>(Args...)>;
    static auto& creatrorsMap()
    {
        static std::unordered_map<std::string, BaseCreator> creators;
        return creators;
    }
};

template <class Base, class... Args>
template <class T>
bool Factory<Base, Args...>::Registry<T>::registered = []() {
    const auto name = demangle(typeid(T).name());
    Factory::creatrorsMap()[name] = [](Args... args) -> std::unique_ptr<Base> {
        return std::make_unique<T>(std::forward<Args>(args)...);
    };

    return true;
}();

} // namespace tl
