#include "MainWindow.h"

#include <QApplication>
#include <QBoxLayout>
#include <QCloseEvent>
#include <QDialogButtonBox>
#include <QFrame>
#include <QHeaderView>
#include <QLabel>
#include <QLineEdit>
#include <QMessageBox>
#include <QMutex>
#include <QPushButton>
#include <QSettings>
#include <QSortFilterProxyModel>
#include <QStackedWidget>
#include <QToolButton>
#include <QTreeView>

#include "gui/qtanimation.h"
#include "gui/qtjsontreemodel.h"
#include "gui/qtlogview.h"
#include "EthernetAdaptorHelper.h"
#include "GlobalConfigs.h"
#include "MonoCameraCalibrationView.h"
#include "MonoCameraVerificationView.h"
#include "StereoCameraCalibrationView.h"
#include "StereoCameraVerificationView.h"
#include "StereoModuleCalibrationView.h"
#include "StereoModuleVerificationView.h"
#include "CalibDataReplayView.h"
#include "CalibDataStatisticsView.h"
#include "version.h"

namespace tl {

using namespace Qt::Literals::StringLiterals;

namespace prop {
constexpr char kMenu[]{"menu"};
} // namespace prop

namespace {
constexpr char kDefaultSettingFilename[]{"settings.ini"};

// NOTE: qInstallMessageHandler() only accepts function pointer (or lambda
// without captured variables), so these variables need to live in global scope.
QtMessageHandler kDefaultHandler{nullptr}; // Not owned
QtLogView* kLogView{nullptr};              // Owned
QMutex kLogViewMtx;
} // namespace

// TODO: Change name. A very naive LoginDialog with a very general name.
class LoginDialog : public QDialog
{
    Q_OBJECT

public:
    explicit LoginDialog(QWidget* parent = nullptr);

private:
    void setupUi();
};

LoginDialog::LoginDialog(QWidget* parent) : QDialog(parent)
{
    setupUi();
    setWindowTitle(tr("User Login"));
}

void LoginDialog::setupUi()
{
    auto editUsername = new QLineEdit(this);
    editUsername->setInputMethodHints(Qt::ImhLatinOnly);

    auto labelUsername = new QLabel(tr("Username"), this);
    labelUsername->setBuddy(editUsername);

    auto editPassword = new QLineEdit(this);
    editPassword->setEchoMode(QLineEdit::Password);

    auto labelPassword = new QLabel(tr("Password"), this);
    labelPassword->setBuddy(editPassword);

    auto buttons = new QDialogButtonBox(this);
    buttons->setCenterButtons(true);
    buttons->addButton(QDialogButtonBox::Ok);
    buttons->addButton(QDialogButtonBox::Cancel);
    buttons->button(QDialogButtonBox::Ok)->setText(tr("Login"));

    connect(buttons->button(QDialogButtonBox::Cancel),
            &QAbstractButton::clicked, this, &QDialog::reject);
    connect(buttons->button(QDialogButtonBox::Ok), &QAbstractButton::clicked,
            this, [=, this]() {
                constexpr char kUsername[]{"dev"};
                constexpr char kPassword[]{"tl43"};
                if (editUsername->text() != kUsername ||
                    editPassword->text() != kPassword) {
                    [[maybe_unused]] const auto _ = QMessageBox::warning(
                        this, tr("Login Error"),
                        tr("Invalid username or password."));
                    return;
                }

                accept();
            });

    auto* layout = new QGridLayout(this);
    layout->addWidget(labelUsername, 0, 0);
    layout->addWidget(editUsername, 0, 1);
    layout->addWidget(labelPassword, 1, 0);
    layout->addWidget(editPassword, 1, 1);
    layout->addWidget(buttons, 2, 0, 1, 2);
}

class ConfigsEditor : public QDialog
{
    Q_OBJECT

public:
    explicit ConfigsEditor(const QString& filename, QWidget* parent = nullptr)
        : QDialog(parent)
    {
        auto searchInput = new QLineEdit(this);
        searchInput->setPlaceholderText(tr("Search here..."));

        auto view = new QTreeView{this};
        view->header()->setSectionResizeMode(QHeaderView::Stretch);
        view->setAlternatingRowColors(true);
        auto model = new QtJsonTreeModel(view);
        model->loadFromFile(filename);
        auto proxy = new QSortFilterProxyModel(view);
        proxy->setFilterKeyColumn(-1);
        proxy->setSourceModel(model);
        view->setModel(proxy);

        auto layout = new QVBoxLayout(this);
        layout->setContentsMargins({});
        layout->addWidget(searchInput);
        layout->addWidget(view);

        // connect(searchInput, &QLineEdit::textChanged, this,
        //         [proxy](const QString& input) {
        //             // Support fixed string for now
        //             proxy->setFilterWildcard(input);
        //         });
    }
};

///------- MainWindowPrivate starts from here
class MainWindowPrivate
{
    Q_DECLARE_TR_FUNCTIONS(tl::MainWindow)
    Q_DEFINE_PIMPL(MainWindow)

public:
    explicit MainWindowPrivate(MainWindow* q);
    ~MainWindowPrivate();

    void init();
    void setupUi();
    void loadSettings();
    void saveSettings() const;

    /// GUI
    void updateUiByDevMode(bool on);
    void updateToolTitle(const QString& title);
    void setCurrentPage(MainWindow::Page page);

    /// Actions

public:
    QLabel* m_mainToolbarTitle;
    QStackedWidget* m_pages;
    std::map<MainWindow::Page, ToolView*> m_toolViews;
    bool m_devMode{false};
};

MainWindowPrivate::MainWindowPrivate(MainWindow* q) : q_ptr(q) {}

MainWindowPrivate::~MainWindowPrivate()
{
    // Maybe delete ui here
}

void MainWindowPrivate::init()
{
    Q_Q(MainWindow);
    q->setWindowTitle(
        tr("Sensor Calibration Tool (%1)").arg(APP_VERSION_STRING));
    q->setWindowIcon(QIcon{":/logo/logo.png"});

    static std::once_flag kInitLogViewFlag;
    std::call_once(kInitLogViewFlag, [q]() {
        kLogView = new QtLogView(q);
        kLogView->setAutoScrollPolicy(
            AutoScrollPolicy::AutoScrollPolicyEnabledIfBottom);
    });

    /// Title bar
    auto* mainToolbar = new QFrame(q);
    mainToolbar->setProperty("toolbar", true);
    mainToolbar->setFrameShape(QFrame::StyledPanel);

    auto* mainToolbarMenuBtn = new QToolButton(mainToolbar);
    mainToolbarMenuBtn->setIcon(QIcon(":/icons/th-list.png"));

    m_mainToolbarTitle = new QLabel(mainToolbar);
    auto titleFont = q->font();
    titleFont.setPointSize(17);
    titleFont.setBold(true);
    m_mainToolbarTitle->setFont(titleFont);

    auto* mainToolbarLayout = new QHBoxLayout(mainToolbar);
    mainToolbarLayout->setContentsMargins({});
    mainToolbarLayout->setSpacing(10);
    mainToolbarLayout->addWidget(mainToolbarMenuBtn);
    mainToolbarLayout->addWidget(m_mainToolbarTitle);

    /// Tool views
    auto monoCalibView = new MonoCameraCalibrationView(q);
    monoCalibView->setFromJson(GlobalConfigs::instance().monoCalibration());
    auto monoVerifyView = new MonoCameraVerificationView(q);
    monoVerifyView->setFromJson(GlobalConfigs::instance().monoVerification());
    auto stereoCalibV2View = new StereoCameraCalibrationView(q);
    stereoCalibV2View->setFromJson(
        GlobalConfigs::instance().stereoCalibration());
    auto stereoVerifyV2View = new StereoCameraVerificationView(q);
    stereoVerifyV2View->setFromJson(
        GlobalConfigs::instance().stereoVerification());
    auto stereoCalibV1View = new StereoModuleCalibrationView(q);
    auto stereoVerifyV1View = new StereoModuleVerificationView(q);
    stereoVerifyV1View->setFromJson(
        GlobalConfigs::instance().stereoVerificationV1());
    auto calib_data_replay_view = new CalibDataReplayView(q);
    calib_data_replay_view->setFromJson(
        GlobalConfigs::instance().stereoCalibration());
    calib_data_replay_view->setVerifFromJson(
        GlobalConfigs::instance().stereoVerification());

    auto calib_data_statistics_view = new CalibDataStatisticsView(q);
    calib_data_statistics_view->setFromJson(
        GlobalConfigs::instance().stereoCalibration());
    calib_data_statistics_view->setVerifFromJson(
        GlobalConfigs::instance().stereoVerification());

    m_toolViews.insert({MainWindow::Page::MonoCalibration, monoCalibView});
    m_toolViews.insert({MainWindow::Page::MonoVerification, monoVerifyView});
    m_toolViews.insert(
        {MainWindow::Page::StereoCalibrationV2, stereoCalibV2View});
    m_toolViews.insert(
        {MainWindow::Page::StereoVerificationV2, stereoVerifyV2View});
    m_toolViews.insert(
        {MainWindow::Page::StereoCalibrationV1, stereoCalibV1View});
    m_toolViews.insert(
        {MainWindow::Page::StereoVerificationV1, stereoVerifyV1View});
    m_toolViews.insert(
        {MainWindow::Page::CalibDataReplay, calib_data_replay_view});
    m_toolViews.insert(
        {MainWindow::Page::CalibDataStatistics, calib_data_statistics_view});

    m_pages = new QStackedWidget(q);
    m_pages->addWidget(monoCalibView);
    m_pages->addWidget(monoVerifyView);
    m_pages->addWidget(stereoCalibV2View);
    m_pages->addWidget(stereoVerifyV2View);
    m_pages->addWidget(stereoCalibV1View);
    m_pages->addWidget(stereoVerifyV1View);
    m_pages->addWidget(calib_data_replay_view);
    m_pages->addWidget(calib_data_statistics_view);

    auto* mainLayout = new QVBoxLayout(q);
    mainLayout->setContentsMargins({});
    mainLayout->setSpacing(0);
    mainLayout->addWidget(mainToolbar);
    mainLayout->addWidget(m_pages);
    mainLayout->addWidget(kLogView);
    mainLayout->setStretchFactor(mainToolbar, 0);
    mainLayout->setStretchFactor(m_pages, 8);
    mainLayout->setStretchFactor(kLogView, 2);

    /// Side menu
    constexpr int kMenuWidth{300};

    auto* menu = new QFrame(q);
    menu->setProperty(prop::kMenu, true);
    menu->setFixedWidth(kMenuWidth);

    auto* menuToolbar = new QFrame(menu);
    menuToolbar->setFrameShape(QFrame::StyledPanel);
    auto* menuToolbarBackBtn = new QToolButton(menuToolbar);
    menuToolbarBackBtn->setIcon(QIcon{":/icons/to-left.png"});
    auto* menuToolbarTitle = new QLabel(menuToolbar);
    menuToolbarTitle->setFont(titleFont);
    menuToolbarTitle->setText(tr("Menu"));

    auto* menuToolbarLayout = new QHBoxLayout(menuToolbar);
    menuToolbarLayout->setContentsMargins({});
    menuToolbarLayout->setSpacing(10);
    menuToolbarLayout->addWidget(menuToolbarBackBtn);
    menuToolbarLayout->addWidget(menuToolbarTitle);

    auto* menuLayout = new QVBoxLayout(menu);
    menuLayout->setContentsMargins({});
    menuLayout->setSpacing(0);
    menuLayout->addWidget(menuToolbar);

    // Add views navigator
    for (const auto& [page, view] : m_toolViews) {
        auto* viewBtn = new QPushButton(view->windowTitle(), menu);
        viewBtn->setProperty(prop::kMenu, true);
        viewBtn->setToolTip(view->windowTitle());
        q->connect(viewBtn, &QAbstractButton::clicked, q, [page, menu, this] {
            Animation::sideSlideOut(menu, Animation::LeftSide);
            setCurrentPage(page);
        });
        q->connect(view, &QWidget::windowTitleChanged, viewBtn,
                   [viewBtn](const QString& title) {
                       viewBtn->setText(title);
                       viewBtn->setToolTip(title);
                   });
        menuLayout->addWidget(viewBtn);
    }

    auto* menuBtnEthernetAdaptorHelper =
        new QPushButton(tr("Ethernet Adaptor Helper"), menu);
    menuBtnEthernetAdaptorHelper->setProperty(prop::kMenu, true);

    auto menuBtnConfigEdit = new QPushButton(tr("Configs Editor"), menu);
    menuBtnConfigEdit->setProperty(prop::kMenu, true);

    auto* menuBtnExit = new QPushButton(tr("Exit"), menu);
    menuBtnExit->setProperty(prop::kMenu, true);

    menuLayout->addWidget(menuBtnEthernetAdaptorHelper);
    menuLayout->addWidget(menuBtnConfigEdit);
    menuLayout->addWidget(menuBtnExit);
    menuLayout->addStretch();

    menu->hide();

    q->connect(mainToolbarMenuBtn, &QAbstractButton::clicked, q, [=, this] {
        if (!m_devMode) {
            LoginDialog dialog{q};
            if (dialog.exec() == QDialog::Rejected) {
                return;
            }

            m_devMode = true;
            updateUiByDevMode(m_devMode);
        }

        Animation::sideSlideIn(menu, Animation::LeftSide);
    });
    q->connect(menuToolbarBackBtn, &QAbstractButton::clicked, q,
               [=] { Animation::sideSlideOut(menu, Animation::LeftSide); });
    q->connect(menuBtnEthernetAdaptorHelper, &QAbstractButton::clicked, q,
               [q]() {
                   EthernetAdaptorHelper helper{q};
                   [[maybe_unused]] const auto _ = helper.exec();
               });
    q->connect(menuBtnConfigEdit, &QAbstractButton::clicked, q, [q]() {
        ConfigsEditor editor{"configs.tt", q};
        editor.exec();
    });
    q->connect(menuBtnExit, &QAbstractButton::clicked, q, &QApplication::quit);

    // clang-format off
    q->setStyleSheet(u"QAbstractButton { "
                        "outline: none; "
                      "}"
                      "QToolButton { "
                        "border: none; "
                        "min-width: 40px; "
                        "min-height: 40px; "
                        "icon-size: 24px; "
                      "}"
                     "QPushButton[menu=true] { "
                       "text-align: left; "
                       "background-color: white; "
                       "border: none; "
                       "border-bottom: 1px solid "
                       "palette(dark); "
                       "padding: 8px; "
                      "}"
                      "QFrame[toolbar=true] { "
                        "background-color: #66C966; "
                      "}"
                      "QFrame[menu=false] { "
                        "background-color: palette(window); "
                      "}"
                      "QFrame[menu=true] { "
                        "background-color: white; "
                        "border: none; "
                        "border-right: 1px solid palette(dark); "
                      "}"
                      "QFrame[notifyArea=true] { "
                        "background-color: white; "
                      "}"
                      "QFrame[notifyMessage=true] { "
                        "background-color: #232323; "
                        "color: white; "
                      "}"
                      "QLabel[notifyMessage=true] { "
                        "background-color: #232323; "
                        "color: white; "
                      "}"_s);
    // clang-format on
}

void MainWindowPrivate::setupUi()
{
    // ...
}

void MainWindowPrivate::loadSettings()
{
    Q_Q(MainWindow);

    QSettings settings{kDefaultSettingFilename, QSettings::IniFormat};

    settings.beginGroup("MainWindow");
    q->restoreGeometry(settings.value("geometry").toByteArray());
    settings.endGroup();

    for (const auto& [_, view] : m_toolViews) {
        view->restoreSettings(settings);
    }
}

void MainWindowPrivate::saveSettings() const
{
    Q_Q(const MainWindow);

    QSettings settings{kDefaultSettingFilename, QSettings::IniFormat};

    settings.beginGroup("MainWindow");
    settings.setValue("geometry", q->saveGeometry());
    settings.endGroup();

    for (const auto& [_, view] : m_toolViews) {
        view->saveSettings(settings);
    }
}

void MainWindowPrivate::updateUiByDevMode(bool on)
{
    for (const auto& [_, view] : m_toolViews) {
        view->setDevMode(on);
    }
}

void MainWindowPrivate::updateToolTitle(const QString& title)
{
    m_mainToolbarTitle->setText(title);
}

void MainWindowPrivate::setCurrentPage(MainWindow::Page page)
{
    if (!m_toolViews.contains(page)) {
        return;
    }

    const auto& view = m_toolViews.at(page);
    m_pages->setCurrentIndex(m_pages->indexOf(view));
    updateToolTitle(view->windowTitle());
}

///------- MainWindow starts from here
MainWindow::MainWindow(QWidget* parent)
    : QWidget(parent), d_ptr(new MainWindowPrivate(this))
{
    Q_D(MainWindow);
    d->init();

    std::once_flag kInitQtMessageHandlerFlag;
    std::call_once(kInitQtMessageHandlerFlag, []() {
        kDefaultHandler = qInstallMessageHandler(
            [](QtMsgType type, const QMessageLogContext& context,
               const QString& msg) {
                if (!kLogView) {
                    return;
                }

                {
                    QMutexLocker locker{&kLogViewMtx};
                    kLogView->sinkQtMessage(type, context, msg);
                }

                if (kDefaultHandler) {
                    kDefaultHandler(type, context, msg);
                }
            });
    });

    d->loadSettings();
}

MainWindow::~MainWindow()
{
    // Restore Qt Message handler, so that LogView can be safely destroyed.
    [[maybe_unused]] const auto _ = qInstallMessageHandler(0);
}

void MainWindow::setHomePage(Page home)
{
    Q_D(MainWindow);
    d->setCurrentPage(home);
}

void MainWindow::setDevMode(bool on)
{
    Q_D(MainWindow);
    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

bool MainWindow::isDevMode() const
{
    Q_D(const MainWindow);
    return d->m_devMode;
}

void MainWindow::setFromConfigs(const Configs& configs)
{
    // ...
}

void MainWindow::closeEvent(QCloseEvent* event)
{
    Q_D(MainWindow);
    d->saveSettings();

    event->accept();
}

} // namespace tl

#include "MainWindow.moc"
#include "moc_MainWindow.cpp"
