#pragma once

#include <QDialog>
#include <QWidget>

#include "gui/guiutils.h"

namespace tl {

class CalibBoardConfigsWidgetPrivate;
class CalibBoardConfigsWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CalibBoardConfigsWidget(QWidget* parent = nullptr);
    ~CalibBoardConfigsWidget();

    // TODO:

private:
    Q_DECLARE_PIMPL(CalibBoardConfigsWidget)
};

// A Dialog wrapper
class CalibBoardConfigsDialog : public QDialog
{
    Q_OBJECT

public:
    explicit CalibBoardConfigsDialog(QWidget* parent = nullptr);

    // Don't take ownership
    CalibBoardConfigsWidget* const widget() const;

private:
    CalibBoardConfigsWidget* const w;
};

} // namespace tl
