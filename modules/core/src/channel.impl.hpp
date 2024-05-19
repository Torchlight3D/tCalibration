#pragma once

#include "channel.h"

namespace tl {

template <typename T>
constexpr Channel<T>::Channel(size_type capacity) : capacity_{capacity}
{
}

template <typename T>
Channel<typename std::decay_t<T>>& operator<<(
    Channel<typename std::decay_t<T>>& channel, T&& element)
{
    if (channel.closed()) {
        throw ClosedChannelError{"Cannot write on closed channel"};
    }

    {
        std::unique_lock<std::mutex> lock{channel.mtx_};
        channel.waitBeforeWrite(lock);

        channel.queue_.push(std::forward<T>(element));
        ++channel.size_;
    }

    channel.condition_.notify_one();

    return channel;
}

template <typename T>
Channel<T>& operator>>(Channel<T>& channel, T& element)
{
    if (channel.closed() && channel.empty()) {
        return channel;
    }

    {
        std::unique_lock<std::mutex> lock{channel.mtx_};
        channel.waitBeforeRead(lock);

        if (!channel.empty()) {
            element = std::move(channel.queue_.front());
            channel.queue_.pop();
            --channel.size_;
        }
    }

    channel.condition_.notify_one();

    return channel;
}

template <typename T>
constexpr typename Channel<T>::size_type Channel<T>::size() const noexcept
{
    return size_;
}

template <typename T>
constexpr bool Channel<T>::empty() const noexcept
{
    return size_ == 0;
}

template <typename T>
void Channel<T>::close() noexcept
{
    closed_.store(true);
    {
        std::unique_lock<std::mutex> lock{mtx_};
        closed_.store(true);
    }
    condition_.notify_all();
}

template <typename T>
void Channel<T>::resume()
{
    closed_.store(false);
    condition_.notify_all();
}

template <typename T>
bool Channel<T>::closed() const noexcept
{
    return closed_.load();
}

template <typename T>
blocking_iterator<Channel<T>> Channel<T>::begin() noexcept
{
    return blocking_iterator<Channel<T>>{*this};
}

template <typename T>
blocking_iterator<Channel<T>> Channel<T>::end() noexcept
{
    return blocking_iterator<Channel<T>>{*this};
}

template <typename T>
void Channel<T>::waitBeforeRead(std::unique_lock<std::mutex>& lock)
{
    condition_.wait(lock, [this]() { return !empty() || closed(); });
}

template <typename T>
void Channel<T>::waitBeforeWrite(std::unique_lock<std::mutex>& lock)
{
    if (capacity_ > 0 && size_ == capacity_) {
        condition_.wait(lock, [this]() { return size_ < capacity_; });
    }
}

} // namespace tl
