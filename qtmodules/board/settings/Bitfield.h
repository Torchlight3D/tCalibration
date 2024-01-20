#pragma once

#include <QList>
#include <QString>

namespace tl {

class Bitfield
{
public:
    Bitfield();
    ~Bitfield();

    void setBitLogic(int bitNumber, bool oneLogic);
    QList<bool>& bitLogics();

    void setBitDescription(int bitNumber, const QString& description);
    QStringList& bitDescriptions();

    void setModified(bool modified);
    bool modified() const;

protected:
    QList<bool> mBitLogics;
    QStringList mBitDescriptions;

    bool mModified;
};

} // namespace tl
