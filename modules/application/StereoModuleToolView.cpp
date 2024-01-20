#include "StereoModuleToolView.h"

#include <QApplication>
#include <QBoxLayout>
#include <QFrame>
#include <QLabel>
#include <QLineEdit>
#include <QPushButton>
#include <QStackedWidget>
#include <QToolButton>
#include <QVariant>

#include "gui/Animation.h"
#include "gui/qstringutils.h"
#include "EthernetAdaptorHelper.h"
#include "StereoModuleCalibrationView.h"
#include "StereoModuleImageQualityView.h"
#include "StereoModuleVerificationView.h"

namespace tl {

namespace prop {
constexpr char kMenu[]{"menu"};
constexpr char kPageIndex[]{"pageIndex"};
} // namespace prop

///------- StereoModuleToolViewPrivate starts from here
class StereoModuleToolViewPrivate
{
    AX_Q_DEFINE_PIMPL(StereoModuleToolView)

public:
    explicit StereoModuleToolViewPrivate(StereoModuleToolView* q);
    ~StereoModuleToolViewPrivate();

    void init();
    void setupUi();

    /// GUI
    void updateUiByDevMode(bool on);
    void updateToolTitle(const QString& title);
    void setCurrentPage(StereoModuleToolView::Page page);

    /// Actions

public:
    QLabel* m_mainToolbarTitle;
    QStackedWidget* m_pages;
    StereoModuleImageQualityView* m_qualityView;
    StereoModuleCalibrationView* m_calibView;
    StereoModuleVerificationView* m_verifyView;
    bool m_devMode{false};
};

StereoModuleToolViewPrivate::StereoModuleToolViewPrivate(
    StereoModuleToolView* q)
    : q_ptr(q)
{
}

StereoModuleToolViewPrivate::~StereoModuleToolViewPrivate()
{
    // Maybe delete ui here
}

void StereoModuleToolViewPrivate::init()
{
    Q_Q(StereoModuleToolView);
    q->setWindowTitle(StereoModuleToolView::tr("Calibration Tool"));
    q->setWindowIcon(QIcon{":/logo/logo-256.png"});

    auto* mainToolbar = new QFrame(q);
    mainToolbar->setProperty("toolbar", true);
    mainToolbar->setFrameShape(QFrame::StyledPanel);

    auto* mainToolbarMenuBtn = new QToolButton(mainToolbar);
    mainToolbarMenuBtn->setIcon(QIcon(":/icons/th-list.svg"));

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

    m_pages = new QStackedWidget(q);
    m_qualityView = new StereoModuleImageQualityView(q);
    m_pages->addWidget(m_qualityView);
    m_calibView = new StereoModuleCalibrationView(q);
    m_pages->addWidget(m_calibView);
    m_verifyView = new StereoModuleVerificationView(q);
    m_pages->addWidget(m_verifyView);

    auto* mainLayout = new QVBoxLayout(q);
    mainLayout->setContentsMargins({});
    mainLayout->setSpacing(0);
    mainLayout->addWidget(mainToolbar);
    mainLayout->addWidget(m_pages);

    // Menu
    constexpr int kMenuWidth{300};

    auto* menu = new QFrame(q);
    menu->setProperty(prop::kMenu, true);
    menu->setFixedWidth(kMenuWidth);

    auto* menuToolbar = new QFrame(menu);
    menuToolbar->setFrameShape(QFrame::StyledPanel);
    auto* menuToolbarBackBtn = new QToolButton(menuToolbar);
    menuToolbarBackBtn->setIcon(QIcon(":/icons/chevron-left.svg"));
    auto* menuToolbarTitle = new QLabel(menuToolbar);
    menuToolbarTitle->setFont(titleFont);
    menuToolbarTitle->setText(StereoModuleToolView::tr("Menu"));

    auto* menuToolbarLayout = new QHBoxLayout(menuToolbar);
    menuToolbarLayout->setContentsMargins({});
    menuToolbarLayout->setSpacing(10);
    menuToolbarLayout->addWidget(menuToolbarBackBtn);
    menuToolbarLayout->addWidget(menuToolbarTitle);

    auto* menuBtnQuality = new QPushButton(m_qualityView->windowTitle(), menu);
    menuBtnQuality->setProperty(prop::kMenu, true);
    auto* menuBtnCalib = new QPushButton(m_calibView->windowTitle(), menu);
    menuBtnCalib->setProperty(prop::kMenu, true);
    auto* menuBtnVerify = new QPushButton(m_verifyView->windowTitle(), menu);
    menuBtnVerify->setProperty(prop::kMenu, true);
    auto* menuBtnEthernetAdaptorHelper = new QPushButton(
        StereoModuleToolView::tr("Ethernet Adaptor Helper"), menu);
    menuBtnEthernetAdaptorHelper->setProperty(prop::kMenu, true);
    auto* menuBtnExit = new QPushButton(StereoModuleToolView::tr("Exit"), menu);
    menuBtnExit->setProperty(prop::kMenu, true);

    auto* menuLayout = new QVBoxLayout(menu);
    menuLayout->setContentsMargins({});
    menuLayout->setSpacing(0);
    menuLayout->addWidget(menuToolbar);
    menuLayout->addWidget(menuBtnQuality);
    menuLayout->addWidget(menuBtnCalib);
    menuLayout->addWidget(menuBtnVerify);
    menuLayout->addWidget(menuBtnEthernetAdaptorHelper);
    menuLayout->addWidget(menuBtnExit);
    menuLayout->addStretch();

    menu->hide();

    // User login
    auto* auth = new QFrame(q);
    auth->setProperty(prop::kMenu, false);
    auth->setFrameShape(QFrame::StyledPanel);
    auth->setFixedWidth(kMenuWidth);
    auto* authUserName = new QLineEdit(auth);
    authUserName->setPlaceholderText(StereoModuleToolView::tr("User Name"));
    auto* authPassword = new QLineEdit(auth);
    authPassword->setPlaceholderText(StereoModuleToolView::tr("Password"));
    authPassword->setEchoMode(QLineEdit::Password);
    auto* authLoginBtn =
        new QPushButton(StereoModuleToolView::tr("Login"), auth);
    auto* authCancelBtn =
        new QPushButton(StereoModuleToolView::tr("Cancel"), auth);

    auto* authButtonsLayout = new QHBoxLayout;
    authButtonsLayout->addWidget(authLoginBtn);
    authButtonsLayout->addWidget(authCancelBtn);

    auto* authLayout = new QVBoxLayout(auth);
    authLayout->addWidget(authUserName);
    authLayout->addWidget(authPassword);
    authLayout->addLayout(authButtonsLayout);

    auth->hide();

    QObject::connect(mainToolbarMenuBtn, &QToolButton::clicked, q, [=, this] {
        if (!m_devMode) {
            Animation::sideSlideIn(auth, Animation::TopSide);
            authUserName->setFocus();
            return;
        }

        Animation::sideSlideIn(menu, Animation::LeftSide);
    });
    QObject::connect(menuToolbarBackBtn, &QToolButton::clicked, q, [=] {
        Animation::sideSlideOut(menu, Animation::LeftSide);
    });
    QObject::connect(authLoginBtn, &QPushButton::clicked, q, [=, this] {
        Animation::sideSlideOut(auth, Animation::TopSide);
        const auto userName = authUserName->text();
        const auto password = authPassword->text();

        constexpr char kUserName[]{"dev"};
        constexpr char kPassword[]{"tl43"};
        if (userName != kUserName || password != kPassword) {
            qDebug() << "Invalid username or password";
            return;
        }

        m_devMode = true;
        updateUiByDevMode(m_devMode);
    });
    QObject::connect(authCancelBtn, &QPushButton::clicked, q, [=] {
        Animation::sideSlideOut(auth, Animation::TopSide);
    });

    // TODO: Use dynamic property to store page
    QObject::connect(menuBtnQuality, &QPushButton::clicked, q, [=, this] {
        Animation::sideSlideOut(menu, Animation::LeftSide);
        setCurrentPage(StereoModuleToolView::Page::ImageQuality);
    });
    QObject::connect(menuBtnCalib, &QPushButton::clicked, q, [=, this] {
        Animation::sideSlideOut(menu, Animation::LeftSide);
        setCurrentPage(StereoModuleToolView::Page::Calibration);
    });
    QObject::connect(menuBtnVerify, &QPushButton::clicked, q, [=, this] {
        Animation::sideSlideOut(menu, Animation::LeftSide);
        setCurrentPage(StereoModuleToolView::Page::Verification);
    });

    QObject::connect(menuBtnEthernetAdaptorHelper, &QPushButton::clicked, q,
                     [q]() {
                         EthernetAdaptorHelper helper{q};
                         helper.exec();
                     });

    QObject::connect(menuBtnExit, &QPushButton::clicked, q,
                     &QApplication::quit);

    q->setStyleSheet(u"QAbstractButton { outline: none; }"
                     "QToolButton { border: none; min-width: "
                     "40px; min-height: 40px; "
                     "icon-size: 24px; }"
                     "QPushButton[menu=true] { text-align: left; "
                     "background-color: white; "
                     "border: none; border-bottom: 1px solid "
                     "palette(dark); padding: 8px; }"
                     "QFrame[toolbar=true] { background-color: "
                     "#66C966; }"
                     "QFrame[menu=false] { background-color: "
                     "palette(window); }"
                     "QFrame[menu=true] { background-color: "
                     "white; border: none; "
                     "border-right: 1px solid palette(dark); }"
                     "QFrame[notifyArea=true] { background-color: "
                     "white; }"
                     "QFrame[notifyMessage=true] { "
                     "background-color: #232323; color: white; "
                     "}"
                     "QLabel[notifyMessage=true] { "
                     "background-color: #232323; color: white; "
                     "}"_s);
}

void StereoModuleToolViewPrivate::updateUiByDevMode(bool on)
{
    m_qualityView->setDevMode(on);
    m_calibView->setDevMode(on);
    m_verifyView->setDevMode(on);
}

void StereoModuleToolViewPrivate::updateToolTitle(const QString& title)
{
    m_mainToolbarTitle->setText(title);
}

void StereoModuleToolViewPrivate::setCurrentPage(
    StereoModuleToolView::Page page)
{
    using Page = StereoModuleToolView::Page;

    switch (page) {
        case Page::ImageQuality:
            m_pages->setCurrentIndex(m_pages->indexOf(m_qualityView));
            updateToolTitle(m_qualityView->windowTitle());
            break;
        case Page::Calibration:
            m_pages->setCurrentIndex(m_pages->indexOf(m_calibView));
            updateToolTitle(m_calibView->windowTitle());
            break;
        case Page::Verification:
            m_pages->setCurrentIndex(m_pages->indexOf(m_verifyView));
            updateToolTitle(m_verifyView->windowTitle());
            break;
        default:
            break;
    }
}

///------- StereoModuleToolView starts from here
StereoModuleToolView::StereoModuleToolView(QWidget* parent)
    : QWidget(parent), d_ptr(new StereoModuleToolViewPrivate(this))
{
    d_ptr->init();
}

StereoModuleToolView::~StereoModuleToolView() = default;

void StereoModuleToolView::setHomePage(Page home)
{
    Q_D(StereoModuleToolView);
    d->setCurrentPage(home);
}

void StereoModuleToolView::setDevMode(bool on)
{
    Q_D(StereoModuleToolView);
    d->m_devMode = on;
    d->updateUiByDevMode(on);
}

bool StereoModuleToolView::isDevMode() const
{
    Q_D(const StereoModuleToolView);
    return d->m_devMode;
}

} // namespace tl

#include "moc_StereoModuleToolView.cpp"
