#pragma once

#include <QWidget>

#include "gui/qtcoreutils.h"
#include "ReportSummary.h"

namespace tl {

class ReportViewPrivate;
class ReportView : public QWidget
{
    Q_OBJECT

public:
    explicit ReportView(QWidget *parent = nullptr);
    ~ReportView();

    void showSummaries(const std::vector<ReportSummary> &summaries);
    void clear();

    void showPlaceholder(bool show = true);
    inline void hidePlaceholder() { showPlaceholder(false); }

private:
    Q_DECLARE_PIMPL(ReportView)
};

} // namespace tl
