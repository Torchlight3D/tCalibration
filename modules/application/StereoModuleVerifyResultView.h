#pragma once

#include <QWidget>
#include "gui/guiutils.h"
#include <tCalibration/StereoCameraVerification>

namespace tl {

class StereoModuleVerifyResultViewPrivate;
class StereoModuleVerifyResultView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleVerifyResultView(QWidget *parent = nullptr);
    ~StereoModuleVerifyResultView();

    // Assume all the results share one reference
    void setReference(const StereoCameraVerification::Reference &ref);
    void showSummary(const StereoCameraVerification::Summary &summary);

    void clearReport();

    /// Related text
    using VerifyItem = StereoCameraVerification::Summary::Item;
    static QString verifyItemName(VerifyItem item);
    static QString verifyItemValue(
        const StereoCameraVerification::Summary &summary, VerifyItem item);
    static QString verifyItemReference(
        const StereoCameraVerification::Reference &ref, VerifyItem item);

    enum class VerifyStatus
    {
        Failed,
        Passed,
        NA,

    };
    static QString verifyStatusName(VerifyStatus status);
    static VerifyStatus verifyItemStatus(
        const StereoCameraVerification::Summary &summary, VerifyItem item);
    inline static QString verifyItemStatusName(
        const StereoCameraVerification::Summary &summary, VerifyItem item)
    {
        return verifyStatusName(verifyItemStatus(summary, item));
    }

    // In case we need a text-based report
    static QString verifySummaryToReport(
        const StereoCameraVerification::Reference &ref,
        const StereoCameraVerification::Summary &summary);

private:
    Q_DECLARE_PIMPL(StereoModuleVerifyResultView)
};
} // namespace tl
