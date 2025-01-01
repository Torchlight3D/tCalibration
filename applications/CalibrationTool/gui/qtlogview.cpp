#include "qtlogview.h"

#include <deque>

#include <QAction>
#include <QCheckBox>
#include <QComboBox>
#include <QCompleter>
#include <QDateTime>
#include <QDialog>
#include <QDialogButtonBox>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QLineEdit>
#include <QMenu>
#include <QScrollBar>
#include <QSettings>
#include <QSortFilterProxyModel>
#include <QStringListModel>
#include <QToolBar>
#include <QTreeView>

namespace tl {

namespace {

//    glog:                 INFO < WARNING <    ERROR < FATAL
//  QDebug:         Debug < Info < Warning < Critical < Fatal
// [value]:       Debug < Warning < Critical < Fatal < Info
//  spdlog: trace < debug < info <    warn <      err < critical < off

constexpr std::array<const char*, 6> kLogLevelIcons{
    ":/icons/log/trace.png", ":/icons/log/debug.png",
    ":/icons/log/info.png",  ":/icons/log/warn.png",
    ":/icons/log/error.png", ":/icons/log/critical.png"};

inline constexpr int glogLevelIndex(int level) { return level + 2; }

inline constexpr int QtMsgTypeIndex(QtMsgType type)
{
    constexpr std::array kMap{1, 3, 4, 5, 2};
    return kMap.at(type);
}

} // namespace

class QtLogViewToolBar : public QToolBar
{
    Q_OBJECT

public:
    struct FilteringSettings
    {
        QString text;             // The text to filter by.
        bool isRegularExpression; // Whether the text is a regular expression.
        bool isCaseSensitive;     // Whether the filtering is case sensitive.
    };

    QtLogViewToolBar(QWidget* parent = nullptr);
    ~QtLogViewToolBar();

    QLineEdit* filter();
    QAction* caseSensitive();
    QAction* regex();
    QAction* clearHistory();
    QAction* style();
    QComboBox* autoScrollPolicy();

    FilteringSettings filteringSettings() const;
    void checkInputValidity();
    void clearCompleterHistory();

signals:
    void styleChangeRequested();
    void filterChanged();
    void autoScrollPolicyChanged(int index);

private:
    void loadCompleterHistory();
    void saveCompleterHistory();

private:
    QLineEdit* _filterWidget;
    QAction* _caseAction;
    QAction* _regexAction;
    QAction* _clearHistory;
    QAction* _styleAction;
    QComboBox* _autoScrollPolicy;
    QStringListModel* _completerData;
    QCompleter* _completer;
};

QtLogViewToolBar::QtLogViewToolBar(QWidget* parent)
    : QToolBar(parent),
      _filterWidget(new QLineEdit(this)),
      _clearHistory(new QAction("Clear History", this)),
      _autoScrollPolicy(new QComboBox(this)),
      _completerData(new QStringListModel(this)),
      _completer(new QCompleter(_completerData, this))
{
    addWidget(_filterWidget);

    _caseAction = addAction("Aa");
    _caseAction->setCheckable(true);

    _regexAction = addAction(".*");
    _regexAction->setCheckable(true);

    _styleAction = addAction("Set style");

    _autoScrollPolicy->addItems(
        {tr("Manual Scroll"), tr("Scroll To Bottom"), tr("Smart Scroll")});
    addWidget(_autoScrollPolicy);

    _filterWidget->setPlaceholderText("Filter");

    _completer->setCaseSensitivity(Qt::CaseInsensitive);
    _completer->setCompletionMode(QCompleter::PopupCompletion);
    _filterWidget->setCompleter(_completer);

    connect(_filterWidget, &QLineEdit::textChanged, this,
            &QtLogViewToolBar::filterChanged);
    connect(_filterWidget, &QLineEdit::editingFinished, this, [this]() {
        QString text = _filterWidget->text();
        if (text.isEmpty() || _completerData->stringList().contains(text))
            return;

        if (_completerData->insertRow(_completerData->rowCount())) {
            QModelIndex index =
                _completerData->index(_completerData->rowCount() - 1, 0);
            _completerData->setData(index, text);
        }
        saveCompleterHistory();
    });
    connect(_caseAction, &QAction::toggled, this,
            &QtLogViewToolBar::filterChanged);
    connect(_regexAction, &QAction::toggled, this,
            &QtLogViewToolBar::filterChanged);
    connect(_styleAction, &QAction::triggered, this,
            [this]() { emit styleChangeRequested(); });
    connect(_autoScrollPolicy,
            QOverload<int>::of(&QComboBox::currentIndexChanged), this,
            &QtLogViewToolBar::autoScrollPolicyChanged);
    connect(this, &QtLogViewToolBar::filterChanged, this,
            &QtLogViewToolBar::checkInputValidity);
    connect(_clearHistory, &QAction::triggered, this,
            &QtLogViewToolBar::clearCompleterHistory);
    loadCompleterHistory();
}

QtLogViewToolBar::~QtLogViewToolBar() = default;

QLineEdit* QtLogViewToolBar::filter() { return _filterWidget; }

QAction* QtLogViewToolBar::caseSensitive() { return _caseAction; }

QAction* QtLogViewToolBar::regex() { return _regexAction; }

QAction* QtLogViewToolBar::clearHistory() { return _clearHistory; }

QAction* QtLogViewToolBar::style() { return _styleAction; }

QComboBox* QtLogViewToolBar::autoScrollPolicy() { return _autoScrollPolicy; }

QtLogViewToolBar::FilteringSettings QtLogViewToolBar::filteringSettings() const
{
    return {_filterWidget->text(), _regexAction->isChecked(),
            _caseAction->isChecked()};
}

void QtLogViewToolBar::checkInputValidity()
{
    FilteringSettings settings = filteringSettings();

    if (!settings.isRegularExpression) {
        // everything is ok, the input text is valid
        _filterWidget->setPalette(QWidget::palette());
        _filterWidget->setToolTip("");
        return;
    }

    QRegularExpression regex{settings.text};
    if (regex.isValid()) {
        _filterWidget->setPalette(QWidget::palette());
        _filterWidget->setToolTip("");
        return;
    }

    QPalette palette = _filterWidget->palette();
    palette.setColor(QPalette::Text, Qt::red);
    _filterWidget->setPalette(palette);
    _filterWidget->setToolTip(regex.errorString());
}

void QtLogViewToolBar::clearCompleterHistory()
{
    _completerData->setStringList({});
    saveCompleterHistory();
}

void QtLogViewToolBar::loadCompleterHistory()
{
    _completerData->setStringList(
        QSettings{"settings.ini", QSettings::IniFormat}
            .value("logview_completer_history")
            .toStringList());
}

void QtLogViewToolBar::saveCompleterHistory()
{
    QSettings{"settings.ini", QSettings::IniFormat}.setValue(
        "logview_completer_history", _completerData->stringList());
}

class QtLogViewModel : public QAbstractListModel
{
    Q_OBJECT

public:
    struct Entry
    {
        std::chrono::duration<double> time;
        int level;
        std::string message;
        std::string category;
    };

    enum class Column
    {
        Level = 0,
        Category,
        Datetime,
        Message,

        Count
    };

    using QAbstractListModel::QAbstractListModel;

    void addEntry(Entry entry);
    void clear();

    void setMaxEntries(size_t max);
    size_t maxEntries() const;

    void setForeground(std::string_view category, std::optional<QColor> color);
    std::optional<QColor> foreground(std::string_view category) const;

    void setBackground(std::string_view category, std::optional<QBrush> brush);
    std::optional<QBrush> background(std::string_view category) const;

    void setLoggerFont(std::string_view category, std::optional<QFont> font);
    std::optional<QFont> loggerFont(std::string_view category) const;

#pragma region QAbstractListModel
    int rowCount(const QModelIndex& parent = {}) const override;
    int columnCount(const QModelIndex& parent = {}) const override;
    QVariant data(const QModelIndex& index,
                  int role = Qt::DisplayRole) const override;
    QVariant headerData(int section, Qt::Orientation orientation,
                        int role = Qt::DisplayRole) const override;
#pragma endregion

private:
    static const QString& sectionName(int section)
    {
        static const std::array kNames{tr("Level"), tr("Category"),
                                       tr("Datetime"), tr("Message")};
        return kNames[section];
    }

    static const QString& logLevelName(int level)
    {
        static const std::array kNames{
            tr("Trace"), tr("Debug"),    tr("Info"), tr("Warning"),
            tr("Error"), tr("Critical"), tr("Off")};
        return kNames[level];
    }

private:
    std::deque<Entry> m_entries;
    size_t _maxEntries{2000};
    std::map<std::string, QBrush> m_backgrounds;
    std::map<std::string, QColor> m_foregrounds;
    std::map<std::string, QFont> m_fonts;
};

void QtLogViewModel::addEntry(Entry entry)
{
    if (m_entries.size() == _maxEntries) {
        beginRemoveRows(QModelIndex(), 0, 0);
        m_entries.pop_front();
        endRemoveRows();
    }

    beginInsertRows(QModelIndex(), rowCount(), rowCount());
    m_entries.push_back(std::move(entry));
    endInsertRows();
}

void QtLogViewModel::setMaxEntries(size_t maxEntries)
{
    _maxEntries = maxEntries;
    // Incase the new maximum is below the current amount of items.
    if (m_entries.size() > _maxEntries) {
        std::size_t offset = m_entries.size() - _maxEntries;

        beginRemoveRows(QModelIndex(), 0, offset - 1);
        m_entries.erase(m_entries.begin(), m_entries.begin() + offset);
        endRemoveRows();
    }
}

size_t QtLogViewModel::maxEntries() const { return _maxEntries; }

void QtLogViewModel::clear()
{
    beginResetModel();
    m_entries.clear();
    endResetModel();
}

int QtLogViewModel::rowCount(const QModelIndex& parent) const
{
    return static_cast<int>(m_entries.size());
}

int QtLogViewModel::columnCount(const QModelIndex& parent) const
{
    return static_cast<int>(Column::Count);
}

QVariant QtLogViewModel::data(const QModelIndex& index, int role) const
{
    if (!index.isValid() || index.row() >= m_entries.size()) {
        return {};
    }

    const auto& item = m_entries[index.row()];
    const auto& category = item.category;

    switch (role) {
        case Qt::DisplayRole: {
            switch (static_cast<Column>(index.column())) {
                case Column::Level:
                    return logLevelName(item.level);
                case Column::Category:
                    return QString::fromStdString(category);
                case Column::Datetime:
                    return QDateTime::fromMSecsSinceEpoch(
                        std::chrono::duration_cast<std::chrono::milliseconds>(
                            item.time)
                            .count());
                case Column::Message:
                    return QString::fromStdString(item.message);
                default:
                    break;
            }
            break;
        }
        case Qt::DecorationRole: {
            if (index.column() == 0) {
                if (item.level >= 0 && item.level < kLogLevelIcons.size()) {
                    return QIcon{kLogLevelIcons[item.level]};
                }
            }
            break;
        }
        case Qt::BackgroundRole: {
            if (m_backgrounds.contains(category)) {
                return m_backgrounds.at(category);
            }
            break;
        }
        case Qt::ForegroundRole: {
            if (m_foregrounds.contains(category)) {
                return m_foregrounds.at(category);
            }
            break;
        }
        case Qt::FontRole: {
            if (m_fonts.contains(category)) {
                return m_fonts.at(category);
            }
            break;
        }
        default:
            break;
    }

    return {};
}

QVariant QtLogViewModel::headerData(int section, Qt::Orientation orientation,
                                    int role) const
{
    if (role == Qt::DisplayRole && orientation == Qt::Horizontal) {
        return sectionName(section);
    }

    return {};
}

void QtLogViewModel::setForeground(std::string_view category,
                                   std::optional<QColor> color)
{
    const int lastRow = std::max(0, rowCount() - 1);
    const int lastColumn = std::max(0, columnCount() - 1);

    if (color.has_value()) {
        m_foregrounds[std::string(category)] = color.value();
        emit dataChanged(index(0), index(lastRow, lastColumn),
                         {Qt::ForegroundRole});
    }
    else if (m_foregrounds.contains(std::string(category))) {
        m_foregrounds.erase(std::string(category));
        emit dataChanged(index(0), index(lastRow, lastColumn),
                         {Qt::ForegroundRole});
    }
}

std::optional<QColor> QtLogViewModel::foreground(
    std::string_view category) const
{
    if (m_foregrounds.contains(std::string(category))) {
        return m_foregrounds.at(std::string(category));
    }

    return {};
}

void QtLogViewModel::setBackground(std::string_view category,
                                   std::optional<QBrush> brush)
{
    const int lastRow = std::max(0, rowCount() - 1);
    const int lastColumn = std::max(0, columnCount() - 1);
    if (brush.has_value()) {
        m_backgrounds[std::string(category)] = brush.value();
        emit dataChanged(index(0), index(lastRow, lastColumn),
                         {Qt::BackgroundRole});
    }
    else if (m_backgrounds.contains(std::string(category))) {
        m_backgrounds.erase(std::string(category));
        emit dataChanged(index(0), index(lastRow, lastColumn),
                         {Qt::BackgroundRole});
    }
}

std::optional<QBrush> QtLogViewModel::background(
    std::string_view category) const
{
    if (m_backgrounds.contains(std::string(category))) {
        return m_backgrounds.at(std::string(category));
    }

    return {};
}

void QtLogViewModel::setLoggerFont(std::string_view category,
                                   std::optional<QFont> font)
{
    const int lastRow = std::max(0, rowCount() - 1);
    const int lastColumn = std::max(0, columnCount() - 1);
    if (font.has_value()) {
        m_fonts[std::string(category)] = font.value();
        emit dataChanged(index(0), index(lastRow, lastColumn), {Qt::FontRole});
    }
    else if (m_fonts.contains(std::string(category))) {
        m_fonts.erase(std::string(category));
        emit dataChanged(index(0), index(lastRow, lastColumn), {Qt::FontRole});
    }
}

std::optional<QFont> QtLogViewModel::loggerFont(std::string_view category) const
{
    if (m_fonts.contains(std::string(category))) {
        return m_fonts.at(std::string(category));
    }

    return {};
}

class QtLogViewProxyModel : public QSortFilterProxyModel
{
    Q_OBJECT

public:
    explicit QtLogViewProxyModel(QObject* parent = nullptr);
};

QtLogViewProxyModel::QtLogViewProxyModel(QObject* parent)
    : QSortFilterProxyModel(parent)
{
    setFilterKeyColumn(-1);
}

class QtLogViewSettingDialog : public QDialog
{
    Q_OBJECT

public:
    explicit QtLogViewSettingDialog(QWidget* parent = nullptr);

    struct Style
    {
        std::string loggerName;
        std::optional<QColor> backgroundColor;
        std::optional<QColor> textColor;
        bool fontBold;
    };

    Style result() const;
    void setModel(const QtLogViewModel* model);

private:
    Style _result;
    const QtLogViewModel* _model;
};

QtLogViewSettingDialog::QtLogViewSettingDialog(QWidget* parent)
    : QDialog(parent)
{
    auto categoryEdit = new QLineEdit(this);
    categoryEdit->setPlaceholderText(tr("Category"));

    auto backgroundColorEdit = new QLineEdit(this);
    backgroundColorEdit->setPlaceholderText(tr("Background color"));

    auto textColorEdit = new QLineEdit(this);
    textColorEdit->setPlaceholderText(tr("Text color"));

    auto checkBoxBold = new QCheckBox(tr("Bold"), this);

    auto buttonBox = new QDialogButtonBox(
        QDialogButtonBox::Ok | QDialogButtonBox::Cancel, this);

    auto layout = new QVBoxLayout(this);
    layout->addWidget(categoryEdit);
    layout->addWidget(backgroundColorEdit);
    layout->addWidget(textColorEdit);
    layout->addWidget(checkBoxBold);
    layout->addWidget(buttonBox);

    connect(categoryEdit, &QLineEdit::textChanged, this,
            [this, backgroundColorEdit, textColorEdit,
             checkBoxBold](const QString& name) {
                const auto namestdstr = name.toStdString();
                auto bg = _model->background(namestdstr);
                auto fg = _model->foreground(namestdstr);
                auto font = _model->loggerFont(namestdstr);

                backgroundColorEdit->setText(bg ? bg.value().color().name()
                                                : QString{});
                textColorEdit->setText(fg ? fg.value().name() : QString{});
                checkBoxBold->setChecked(font && font->bold());
            });

    connect(buttonBox, &QDialogButtonBox::accepted, this,
            [this, categoryEdit, backgroundColorEdit, textColorEdit,
             checkBoxBold]() {
                if (!categoryEdit->text().isEmpty()) {
                    reject();
                }

                _result.loggerName = categoryEdit->text().toStdString();

                if (!backgroundColorEdit->text().isEmpty())
                    _result.backgroundColor =
                        QColor(backgroundColorEdit->text());
                else
                    _result.backgroundColor = std::nullopt;

                if (!textColorEdit->text().isEmpty())
                    _result.textColor = QColor(textColorEdit->text());
                else
                    _result.textColor = std::nullopt;

                _result.fontBold = checkBoxBold->isChecked();

                accept();
            });

    connect(buttonBox, &QDialogButtonBox::rejected, this,
            &QtLogViewSettingDialog::reject);
}

QtLogViewSettingDialog::Style QtLogViewSettingDialog::result() const
{
    return _result;
}

void QtLogViewSettingDialog::setModel(const QtLogViewModel* model)
{
    _model = model;
}

class QtLogViewPrivate
{
    Q_DEFINE_PIMPL(QtLogView)

public:
    explicit QtLogViewPrivate(QtLogView* q);

    void init();

public:
    QtLogViewModel* _sourceModel;
    QtLogViewProxyModel* _proxyModel;
    QTreeView* _view;
    QMetaObject::Connection _scrollConnection;
    std::list<QtLogViewToolBar*> _toolbars;
    bool _scrollIsAtBottom;
};

QtLogViewPrivate::QtLogViewPrivate(QtLogView* q)
    : q_ptr(q),
      _sourceModel(new QtLogViewModel(q)),
      _proxyModel(new QtLogViewProxyModel(q)),
      _view(new QTreeView(q))
{
}

void QtLogViewPrivate::init()
{
    Q_Q(QtLogView);
    _view->setModel(_proxyModel);
    _view->setRootIsDecorated(false);
    _proxyModel->setSourceModel(_sourceModel);

    QHeaderView* header = _view->header();
    header->setContextMenuPolicy(Qt::CustomContextMenu);

    auto layout = new QHBoxLayout(q);
    layout->setContentsMargins({});
    layout->addWidget(_view);
}

QtLogView::QtLogView(QWidget* parent)
    : QWidget(parent), d_ptr(new QtLogViewPrivate(this))
{
    Q_D(QtLogView);
    d->init();

    connect(
        d->_view->header(), &QHeaderView::customContextMenuRequested, this,
        [this, d](const QPoint& pos) {
            QMenu menu;
            for (auto i{0}; i < d->_sourceModel->columnCount(); ++i) {
                auto* action = menu.addAction(
                    d->_sourceModel->headerData(i, Qt::Horizontal).toString());
                action->setCheckable(true);
                action->setChecked(!d->_view->header()->isSectionHidden(i));
                action->setData(i);

                connect(
                    action, &QAction::toggled, this, [this, d](bool checked) {
                        if (auto* action = qobject_cast<QAction*>(sender())) {
                            d->_view->header()->setSectionHidden(
                                action->data().toInt(), !checked);
                        }
                    });
            }

            menu.exec(d->_view->header()->mapToGlobal(pos));
        });

    connect(d->_sourceModel, &QAbstractItemModel::rowsAboutToBeInserted, this,
            [d](const QModelIndex& parent, int first, int last) {
                const auto bar = d->_view->verticalScrollBar();
                d->_scrollIsAtBottom = bar && (bar->value() == bar->maximum());
            });
}

QtLogView::~QtLogView() = default;

void QtLogView::clear()
{
    Q_D(QtLogView);
    d->_sourceModel->clear();
}

void QtLogView::registerToolbar(QtLogViewToolBar* toolbar)
{
    Q_D(QtLogView);
    toolbar->setParent(this);
    d->_toolbars.push_back(toolbar);

    QLineEdit* filter = toolbar->filter();
    QAction* regex = toolbar->regex();
    QAction* caseSensitive = toolbar->caseSensitive();
    QAction* style = toolbar->style();
    QComboBox* autoScrollPolicyCombo = toolbar->autoScrollPolicy();

    auto updateFilter = [this, filter, regex, caseSensitive]() {
        filterData(filter->text(), regex->isChecked(),
                   caseSensitive->isChecked());
    };

    connect(filter, &QLineEdit::textChanged, this, updateFilter);
    connect(regex, &QAction::toggled, this, updateFilter);
    connect(caseSensitive, &QAction::toggled, this, updateFilter);
    connect(style, &QAction::triggered, this, [d]() {
        QtLogViewSettingDialog dialog;
        dialog.setModel(d->_sourceModel);
        if (!dialog.exec()) {
            return;
        }

        const auto value = dialog.result();

        d->_sourceModel->setBackground(value.loggerName, value.backgroundColor);
        d->_sourceModel->setForeground(value.loggerName, value.textColor);

        QFont font;
        font.setBold(value.fontBold);
        d->_sourceModel->setLoggerFont(value.loggerName, font);
    });

    connect(autoScrollPolicyCombo, &QComboBox::currentIndexChanged, this,
            &QtLogView::updateAutoScrollPolicy);
}

void QtLogView::removeToolbar(QtLogViewToolBar* toolbar)
{
    Q_D(QtLogView);
    d->_toolbars.erase(
        std::remove(d->_toolbars.begin(), d->_toolbars.end(), toolbar),
        d->_toolbars.end());
}

void QtLogView::filterData(const QString& text, bool isRegularExpression,
                           bool isCaseSensitive)
{
    Q_D(QtLogView);
    d->_proxyModel->setFilterCaseSensitivity(
        isCaseSensitive ? Qt::CaseSensitive : Qt::CaseInsensitive);

    if (isRegularExpression) {
        QRegularExpression regex{text};
        if (!regex.isValid()) {
            return;
        }

        d->_proxyModel->setFilterRegularExpression(text);
    }
    else {
        d->_proxyModel->setFilterFixedString(text);
    }
}

void QtLogView::setAutoScrollPolicy(AutoScrollPolicy policy)
{
    Q_D(QtLogView);
    QObject::disconnect(d->_scrollConnection);

    switch (policy) {
        case AutoScrollPolicy::AutoScrollPolicyEnabled: {
            d->_scrollConnection =
                connect(d->_sourceModel, &QtLogViewModel::rowsInserted,
                        d->_view, &QTreeView::scrollToBottom);
            break;
        }
        case AutoScrollPolicy::AutoScrollPolicyEnabledIfBottom: {
            d->_scrollConnection = connect(
                d->_sourceModel, &QtLogViewModel::rowsInserted, this, [d]() {
                    // We can't check if the scrollbar is at the bottom here
                    // because the new rows are already inserted and the
                    // position of the scrollbar may not be at the bottom of the
                    // widget anymore. That's why the scroll position is checked
                    // before actually adding the rows (AKA in the
                    // rowsAboutToBeInserted signal).
                    if (d->_scrollIsAtBottom) {
                        d->_view->scrollToBottom();
                    }
                });
            break;
        }
        default: {
            // The connection is already disconnected. No need for handling the
            // AutoScrollPolicyDisabled case.
            break;
        }
    }

    for (auto& toolbar : d->_toolbars) {
        if (QComboBox* policyComboBox = toolbar->autoScrollPolicy()) {
            QSignalBlocker blocker{policyComboBox};
            policyComboBox->setCurrentIndex(static_cast<int>(policy));
        }
    }
}

void QtLogView::sinkQtMessage(QtMsgType type, const QMessageLogContext& context,
                              const QString& msg)
{
    Q_D(QtLogView);
    d->_sourceModel->addEntry(
        {.time = std::chrono::system_clock::now().time_since_epoch(),
         .level = QtMsgTypeIndex(type),
         .message = msg.toStdString(),
         .category = context.category});
}

void QtLogView::updateAutoScrollPolicy(int index)
{
    setAutoScrollPolicy(static_cast<AutoScrollPolicy>(index));
}

size_t QtLogView::itemsCount() const
{
    Q_D(const QtLogView);
    return static_cast<std::size_t>(d->_proxyModel->rowCount());
}

void QtLogView::setMaxEntries(size_t maxEntries)
{
    Q_D(QtLogView);
    d->_sourceModel->setMaxEntries(maxEntries);
}

size_t QtLogView::maxEntries() const
{
    Q_D(const QtLogView);
    return d->_sourceModel->maxEntries();
}

void QtLogView::setForeground(std::string_view category,
                              std::optional<QColor> color)
{
    Q_D(QtLogView);
    d->_sourceModel->setForeground(category, color);
}

std::optional<QColor> QtLogView::foreground(std::string_view category) const
{
    Q_D(const QtLogView);
    return d->_sourceModel->foreground(category);
}

void QtLogView::setBackground(std::string_view category,
                              std::optional<QBrush> brush)
{
    Q_D(QtLogView);
    d->_sourceModel->setBackground(category, brush);
}

std::optional<QBrush> QtLogView::background(std::string_view category) const
{
    Q_D(const QtLogView);
    return d->_sourceModel->background(category);
}

void QtLogView::setLoggerFont(std::string_view category,
                              std::optional<QFont> font)
{
    Q_D(QtLogView);
    d->_sourceModel->setLoggerFont(category, font);
}

std::optional<QFont> QtLogView::loggerFont(std::string_view category) const
{
    Q_D(const QtLogView);
    return d->_sourceModel->loggerFont(category);
}

} // namespace tl

#include "moc_qtlogview.cpp"
#include "qtlogview.moc"
