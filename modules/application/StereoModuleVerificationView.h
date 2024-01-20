#pragma once

#include <QWidget>
#include "gui/guiutils.h"

#include <tCalibration/StereoCameraVerification>

namespace tl {

class StereoModuleVerificationViewPrivate;
class StereoModuleVerificationView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleVerificationView(QWidget *parent = nullptr);
    ~StereoModuleVerificationView();

    bool setFromJson(const QJsonObject &jo);

    void setDevMode(bool on);

signals:
    // For async call
    void requestShowImage(const QImage &img);
    void requestShowStereoImages(const QImage &left, const QImage &right);
    void requestShowMessage(const QString &msg);
    void requestUpdateIndicator(bool pass);
    void requestShowSummary(const StereoCameraVerification::Summary &summary);

private slots:
    void startVerify();

private:
    Q_DECLARE_PIMPL(StereoModuleVerificationView)
};

} // namespace tl
