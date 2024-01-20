#pragma once

#include <QWidget>

#include <tCore/EnumUtils>
#include "gui/guiutils.h"
#include "StereoModuleTask.h"

namespace tl {

class StereoModuleCalibResultViewPrivate;
class StereoModuleCalibResultView : public QWidget
{
    Q_OBJECT

public:
    explicit StereoModuleCalibResultView(QWidget *parent = nullptr);
    ~StereoModuleCalibResultView();

    // Assume all the results share one reference
    void setReference(const StereoModuleTask::ResultReference &ref);

    // Incremental interface
    void append(const StereoModuleTask::ResultSummary &summary);

    // Batch interface
    void showSummaries(
        const std::vector<StereoModuleTask::ResultSummary> &summaries);

    void clearReport();

    void showPlaceholder(bool show = true);
    inline void hidePlaceholder() { showPlaceholder(false); }

    /// Related text
    using VerifyItem = StereoModuleTask::ResultSummary::Item;
    static QString verifyItemName(VerifyItem item);
    static QString verifyItemValue(
        const StereoModuleTask::ResultSummary &summary, VerifyItem item);
    static QString verifyItemReference(
        const StereoModuleTask::ResultReference &ref, VerifyItem item);

    enum class VerifyStatus
    {
        Failed,
        Passed,
        NA,
    };
    static QString verifyStatusName(VerifyStatus status);
    static VerifyStatus verifyItemStatus(
        const StereoModuleTask::ResultSummary &summary, VerifyItem item);
    inline static QString verifyItemStatusName(
        const StereoModuleTask::ResultSummary &summary, VerifyItem item)
    {
        return verifyStatusName(verifyItemStatus(summary, item));
    }

    // In case we need a text-based report
    static QString verifySummaryToReport(
        const StereoModuleTask::ResultReference &ref,
        const StereoModuleTask::ResultSummary &summary);

private:
    Q_DECLARE_PIMPL(StereoModuleCalibResultView)
};

} // namespace tl
