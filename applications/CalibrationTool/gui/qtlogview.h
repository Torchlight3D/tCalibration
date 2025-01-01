#pragma once

#include <QWidget>

#include "qtcoreutils.h"

namespace tl {

class QtLogViewToolBar;

enum class AutoScrollPolicy
{
    AutoScrollPolicyDisabled =
        0, // Never scroll to the bottom, leave the scrollbar where it was.
    AutoScrollPolicyEnabled =
        1, // Always scroll to the bottom when new rows are inserted.
    AutoScrollPolicyEnabledIfBottom =
        2, // Scroll to the bottom only if the scrollbar was at the bottom
           // before inserting the new ones.
};

class QtLogViewPrivate;
class QtLogView : public QWidget
{
    Q_OBJECT

public:
    explicit QtLogView(QWidget* parent = nullptr);
    ~QtLogView();

    void clear();

    void registerToolbar(QtLogViewToolBar* toolbar);
    void removeToolbar(QtLogViewToolBar* toolbar);

    size_t itemsCount() const;

    void setMaxEntries(size_t maxEntries);
    size_t maxEntries() const;

    void setForeground(std::string_view category, std::optional<QColor> brush);
    std::optional<QColor> foreground(std::string_view category) const;

    void setBackground(std::string_view category, std::optional<QBrush> brush);
    std::optional<QBrush> background(std::string_view category) const;

    void setLoggerFont(std::string_view category, std::optional<QFont> font);
    std::optional<QFont> loggerFont(std::string_view category) const;

    void setAutoScrollPolicy(AutoScrollPolicy policy);

    void sinkQtMessage(QtMsgType type, const QMessageLogContext& context,
                       const QString& msg);

private slots:
    void filterData(const QString& text, bool isRegularExpression,
                    bool caseSensitive);
    void updateAutoScrollPolicy(int index);

private:
    Q_DECLARE_PIMPL(QtLogView)
};

} // namespace tl
