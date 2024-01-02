#pragma once

namespace thoht {

class NoMask
{
public:
    enum
    {
        DesignVariableDimension = 0
    };

    NoMask() {}
    ~NoMask() {}

    template <typename K>
    bool isValid(const K& /* k */) const
    {
        return true;
    }

    bool isSet() const { return false; }
};

} // namespace thoht
