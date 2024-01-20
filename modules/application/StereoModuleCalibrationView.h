#pragma once

#include <QWidget>
#include "gui/guiutils.h"

class QAbstractButton;

namespace tl {

class StereoModuleCalibrationViewPrivate;
class StereoModuleCalibrationView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleCalibrationView(QWidget *parent = nullptr);
    ~StereoModuleCalibrationView();

    bool setFromJson(const QJsonObject &jo);

    void setDevMode(bool on);

signals:
    // For async call
    void requestShowMessage(const QString &msg);
    void requestAppendMessage(const QString &msg);
    void requestShowImage(const QImage &image, int index);
    void requestShowStereoImages(const QImage &left, const QImage &right,
                                 int index);

private:
    Q_DECLARE_PIMPL(StereoModuleCalibrationView)
};

} // namespace tl
