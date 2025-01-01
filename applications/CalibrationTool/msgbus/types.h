#pragma once

#include <QByteArray>
#include <QDataStream>

namespace tl {
namespace msgbus {

inline constexpr int kDefaultPort{9001};
inline constexpr int kPageSize{1000};

struct Message
{
    inline static constexpr uint8_t kHead{0xa5};
    inline static constexpr uint8_t kTail{0x5a};
    inline static constexpr size_t kMTU{1024};

    const uint8_t _head; // Placeholder, help parsing
    const uint8_t _tail; // Placeholder, help parsing
    uint8_t sender;
    uint8_t receiver;
    uint8_t cmd;
    uint8_t cmdSub;
    uint8_t lenLow;
    uint8_t lenHigh;
    uint8_t seqLow;  // Placeholder, won't change
    uint8_t seqHigh; // Placeholder, won't change

    Message();
    virtual ~Message();
};

struct Request : Message
{
    Request(uint8_t sender, uint8_t receiver);
    virtual ~Request() = default;

    Request& setCommand(uint8_t cmd, uint8_t cmdSub);

    QByteArray toDatagram() const;

    virtual QByteArray bodyBytes() const = 0;
};

struct SysCmdRequest final : Request
{
    std::string data;

    using Request::Request;

    QByteArray bodyBytes() const override;
};

struct FileControlRequest final : Request
{
    uint32_t mode;
    std::string path;

    using Request::Request;

    QByteArray bodyBytes() const override;
};

struct FileDataRequest final : Request
{
    uint32_t seek;
    uint32_t size;
    std::string data;

    using Request::Request;

    QByteArray bodyBytes() const override;
};

struct Response : Message
{
    virtual ~Response() = default;

    bool setFromDatagram(const QByteArray& datagram);

    virtual bool finished() const = 0;

    virtual bool parse(QDataStream& stream, int dataLen) = 0;
};

struct StringResponse final : Response
{
    std::string data{};

    bool finished() const override { return true; }

    bool parse(QDataStream& stream, int len) override;
};

struct SysCmdResponse final : Response
{
    int ret;
    std::string output;

    bool finished() const override { return true; }

    bool parse(QDataStream& stream, int len) override;
};

struct FileControlResponse final : Response
{
    int32_t err;
    std::string data;

    bool finished() const override { return true; }

    bool parse(QDataStream& stream, int len) override;
};

struct FileDataResponse final : Response
{
    int32_t err;
    uint32_t seek;
    uint32_t size;
    std::string data;

    bool finished() const override { return err == -1; }

    bool parse(QDataStream& stream, int len) override;
};

} // namespace msgbus
} // namespace tl
