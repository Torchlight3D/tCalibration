#include "types.h"

#include <QIODevice>

namespace tl {
namespace msgbus {

Message::Message() : _head(kHead), _tail(kTail) {}

Message::~Message() = default;

Request::Request(uint8_t _sender, uint8_t _receiver) : Message()
{
    sender = _sender;
    receiver = _receiver;
    lenLow = 0;
    lenHigh = 0;
    seqLow = 0;
    seqHigh = 0;
}

Request& Request::setCommand(uint8_t _cmd, uint8_t _cmdSub)
{
    cmd = _cmd;
    cmdSub = _cmdSub;
    return *this;
}

QByteArray Request::toDatagram() const
{
    // FIXME:
    // 1. These fields in class are not changed
    // 2. Method to get high and low bits is weird???
    const auto _seqLow = seqLow & 0xff;
    const auto _seqHigh = seqHigh >> 8;

    const auto body = bodyBytes();
    const auto _lenLow = body.size() & 0xff;
    const auto _lenHigh = body.size() >> 8;

    // clang-format off
    const char header[]{
        static_cast<char>(_head),
        static_cast<char>(_tail),
        static_cast<char>(sender),
        static_cast<char>(receiver),
        static_cast<char>(cmd),
        static_cast<char>(cmdSub),
        static_cast<char>(_lenLow),
        static_cast<char>(_lenHigh),
        static_cast<char>(_seqLow),
        static_cast<char>(_seqHigh)
    };
    // clang-format on

    QByteArray msg{header, std::size(header)};
    msg.append(body);

    auto crc16 = [](QByteArrayView bytes) -> int {
        int table[256]{0};
        for (auto i{0}; i < 256; ++i) {
            auto temp = i;
            for (auto j{0}; j < 8; ++j) {
                if (temp & 1) {
                    temp = ((temp >> 1) ^ 0xa001);
                }
                else {
                    temp >>= 1;
                }
            }
            table[i] = temp;
        }

        auto crc = 0xffff;
        for (const auto& byte : bytes) {
            crc = (crc >> 8) ^ table[(crc ^ byte) & 0xff];
        }

        return (((crc & 0xff) << 8) + (crc >> 8));
    };

    const auto checksum = crc16(msg);

    msg.append(checksum >> 8);
    msg.append(checksum & 0xff);

    return msg;
}

QByteArray SysCmdRequest::bodyBytes() const
{
    return QByteArray::fromStdString(data);
}

QByteArray FileControlRequest::bodyBytes() const
{
    QByteArray bytes;
    QDataStream stream{&bytes, QIODevice::WriteOnly};
    stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    stream << mode;
    stream.writeRawData(path.data(), path.length());

    return bytes;
}

QByteArray FileDataRequest::bodyBytes() const
{
    QByteArray bytes;
    QDataStream stream{&bytes, QIODevice::WriteOnly};
    stream.setByteOrder(QDataStream::ByteOrder::LittleEndian);

    stream << seek << size;
    stream.writeRawData(data.data(), data.length());

    return bytes;
}

bool Response::setFromDatagram(const QByteArray& datagram)
{
    const auto start = datagram.indexOf(kHead);
    if (start == -1) {
        return false;
    }

    QDataStream stream{datagram};

    stream.skipRawData(start);
    stream.skipRawData(1);
    stream.skipRawData(1);
    // clang-format off
    stream >> sender >> receiver
           >> cmd >> cmdSub
           >> lenLow >> lenHigh
           >> seqLow >> seqHigh;
    // clang-format on

    const auto len = (lenHigh << 8) | lenLow;

    return parse(stream, len);
}

bool StringResponse::parse(QDataStream& stream, int len)
{
    uint8_t strLen;
    stream >> strLen;

    QByteArray _data;
    _data.resize(strLen);
    stream.readRawData(_data.data(), strLen);
    data = _data.data();

    return true;
}

bool SysCmdResponse::parse(QDataStream& stream, int len)
{
    stream >> ret;

    constexpr auto offset = sizeof(ret);

    const auto strLen = len - offset;

    QByteArray _data;
    _data.resize(strLen);
    if (const auto bytesRead = stream.readRawData(_data.data(), strLen);
        bytesRead == -1) {
        return false;
    }
    else if (bytesRead != strLen) {
        _data.resize(strLen);
    }

    output = _data.toStdString();

    return true;
}

bool FileControlResponse::parse(QDataStream& stream, int len)
{
    stream >> err;

    const auto strLen = len - sizeof(err);

    QByteArray _data;
    _data.resize(strLen);
    if (const auto bytesRead = stream.readRawData(_data.data(), strLen);
        bytesRead == -1) {
        return false;
    }
    else if (bytesRead != strLen) {
        _data.resize(strLen);
    }

    data = _data.toStdString();

    return true;
}

bool FileDataResponse::parse(QDataStream& stream, int len)
{
    stream >> err >> seek >> size;

    constexpr auto offset = sizeof(err) + sizeof(seek) + sizeof(size);

    const auto strLen = len - offset;

    QByteArray _data;
    _data.resize(strLen);
    if (const auto bytesRead = stream.readRawData(_data.data(), strLen);
        bytesRead == -1) {
        return false;
    }
    else if (bytesRead != strLen) {
        _data.resize(strLen);
    }

    data = _data.toStdString();

    return true;
}

} // namespace msgbus
} // namespace tl
