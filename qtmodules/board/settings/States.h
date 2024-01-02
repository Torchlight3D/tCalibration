#pragma once

#include <QMap>
#include <QString>

#include "ColorSettings.h"

namespace thoht {

class States
{
public:
    States();
    ~States();

    void clearStates();
    void addState(bool active, qlonglong value, const QString& text,
                  const ColorSettings& settings);

    QString text(qlonglong value);
    ColorSettings colorSettings(qlonglong value, bool& colorIsSet);

    bool active();

    QMap<qlonglong, ColorSettings> statesColor() const;
    QMap<qlonglong, QString> statesText() const;
    QMap<qlonglong, bool> statesActive() const;

    void setModified(bool modified);
    bool modified() const;

protected:
    QMap<qlonglong, ColorSettings> mStatesColor;
    QMap<qlonglong, QString> mStatesText;
    QMap<qlonglong, bool> mStatesActive;
    bool mModified;
};

} // namespace thoht
