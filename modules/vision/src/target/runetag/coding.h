#pragma once

#include <memory>
#include <vector>

namespace tl {
namespace runetag {

class Coding
{
public:
    Coding();
    ~Coding();

    void init();

    long get_index(const std::vector<long>& code_vec);
    void align(std::vector<long>& code_vec, long& index, long& rotation);
    int decode(std::vector<long>& code_vec);
    long generate(std::vector<long>& code, long index);

    static std::vector<bool> unpack(const std::vector<long> code);
    static std::vector<long> pack(const std::vector<bool> code);

private:
    class Impl;
    const std::unique_ptr<Impl> d;
};

} // namespace runetag
} // namespace tl
