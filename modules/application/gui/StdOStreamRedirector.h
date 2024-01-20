#pragma once

#include <streambuf>

namespace tl {

template <class _Char = char, class _Traits = std::char_traits<_Char>>
class StdOStreamRedirector_ : public std::basic_streambuf<_Char, _Traits>
{
    using char_type = _Char;
    using traits_type = _Traits;
    using int_type = typename traits_type::int_type;
    using callback_type = std::add_pointer_t<void(
        const _Char*, std::streamsize count, void* data)>;

public:
    StdOStreamRedirector_(std::ostream& stream, callback_type callback,
                          void* data)
        : stream_(stream), callback_(callback), data_(data)
    {
        buf_ = stream_.rdbuf(this);
    };

    ~StdOStreamRedirector_() { stream_.rdbuf(buf_); }

    std::streamsize xsputn(const char_type* str, std::streamsize count) override
    {
        callback_(str, count, data_);
        return count;
    }

    int_type overflow(int_type c = traits_type::eof()) override
    {
        char_type ch = traits_type::to_char_type(c);
        callback_(&ch, 1, data_);
        return traits_type::not_eof(c);
    }

private:
    std::basic_ostream<char_type, traits_type>& stream_;
    std::streambuf* buf_;
    callback_type callback_;
    void* data_;
};

using StdOStreamRedirector =
    StdOStreamRedirector_<char, std::char_traits<char>>;

} // namespace tl
