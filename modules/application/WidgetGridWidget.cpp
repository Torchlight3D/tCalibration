#include "WidgetGridWidget.h"

#include <QApplication>
#include <QEvent>
#include <QGridLayout>
#include <QLabel>
#include <QMenu>

#include "gui/ImageView.h"

namespace thoht {

///------- VideoBox starts from here
class VideoBox : public QObject
{
    Q_OBJECT

public:
    explicit VideoBox(QObject *parent = nullptr);

    void showImage(const QImage &image, int index);

public slots:
    void setVideoType(const QString &videoType);
    void setLayout(QGridLayout *layout);
    void setWidgets(QWidgetList widgets);

    void setMenuFlag(const QString &menuFlag);
    void setActionFlag(const QString &actionFlag);

    void setTypes(const QMap<int, QStringList> &types);

    void initMenu(QMenu *menu, const QList<bool> &enable);

    void show_video(int type, int index);
    void show_video();

    void showAllViews();
    void hideAllViews();

    void change_video_normal(int index, int flag);
    void change_video_custom(int index, int type);

    void change_video_6(const QList<int> &indexs);
    void change_video_8(const QList<int> &indexs);
    void change_video_13(const QList<int> &indexs);

    void change_video_1(int index);
    void change_video_4(int index);
    void change_video_6(int index);
    void change_video_8(int index);
    void change_video_9(int index);

signals:
    void changeVideo(int type, const QString &videoType, bool videoMax);

private:
    void addMenu(QMenu *menu, int type);

private:
    QGridLayout *m_gridLayout{nullptr}; // not own
    QWidgetList m_widgets;              // not own

    int m_videoCount;
    QString m_videoType;

    QString m_menuFlag;
    QString m_actionFlag;

    QMap<int, QStringList> m_types;
};

VideoBox::VideoBox(QObject *parent) : QObject(parent)
{
    m_gridLayout = nullptr;
    m_videoCount = 64;
    m_videoType = "1_16";

    m_menuFlag = "Layout";
    m_actionFlag = "Channel";

    m_types.insert(4, QStringList{"1_4", "5_8", "9_12", "13_16", "17_20",
                                  "21_24", "25_28", "29_32", "33_36"});
    m_types.insert(
        6, QStringList{"1_6", "7_12", "13_18", "19_24", "25_30", "31_36"});
    m_types.insert(8, QStringList{"1_8", "9_16", "17_24", "25_32", "33_40",
                                  "41_48", "49_57", "57_64"});
    m_types.insert(9, QStringList{"1_9", "9_17", "18_26", "27_35", "36_42",
                                  "43_50", "51_59"});
}

void VideoBox::showImage(const QImage &image, int index)
{
    if (index >= m_widgets.size()) {
        return;
    }

    qobject_cast<ImageView *>(m_widgets[index])->showImage(image);
}

void VideoBox::addMenu(QMenu *menu, int type)
{
    // Root menu
    const auto name =
        QString("Switch to %1:%2").arg(QString::number(type), m_menuFlag);
    // Possible layouts for this type
    QStringList flags = m_types.value(type);

    // Add sub-menu if more than 1 flag exist.
    QMenu *menuSub;
    if (flags.count() > 1) {
        menuSub = menu->addMenu(name);
    }
    else {
        menuSub = menu;
    }

    for (const auto &flag : flags) {
        QStringList list = flag.split("_");
        QString start = list.at(0);
        QString end = list.at(1);

        // Flag text
        auto text = QString("%1%2-%1%3").arg(m_actionFlag, start, end);
        if (flags.count() == 1) {
            text = name;
        }

        auto *action =
            menuSub->addAction(text, this, qOverload<>(&VideoBox::show_video));
        // Use QObject dynamic property
        action->setProperty("index", start);
        action->setProperty("type", type);
        action->setProperty("flag", flag);
    }
}

void VideoBox::setVideoType(const QString &videoType)
{
    m_videoType = videoType;
}

void VideoBox::setLayout(QGridLayout *gridLayout) { m_gridLayout = gridLayout; }

void VideoBox::setWidgets(QWidgetList widgets)
{
    m_widgets = widgets;
    m_videoCount = widgets.count();
}

void VideoBox::setMenuFlag(const QString &menuFlag) { m_menuFlag = menuFlag; }

void VideoBox::setActionFlag(const QString &actionFlag)
{
    m_actionFlag = actionFlag;
}

void VideoBox::setTypes(const QMap<int, QStringList> &types)
{
    m_types = types;
}

void VideoBox::initMenu(QMenu *menu, const QList<bool> &enable)
{
    if (enable.count() < 9) {
        return;
    }

    if (enable.at(0)) {
        addMenu(menu, 4);
    }
    if (enable.at(1)) {
        addMenu(menu, 6);
    }
    if (enable.at(2)) {
        addMenu(menu, 8);
    }
    if (enable.at(3)) {
        addMenu(menu, 9);
    }
    if (enable.at(4)) {
        addMenu(menu, 13);
    }
    if (enable.at(5)) {
        addMenu(menu, 16);
    }
    if (enable.at(6)) {
        addMenu(menu, 25);
    }
    if (enable.at(7)) {
        addMenu(menu, 36);
    }
    if (enable.at(8)) {
        addMenu(menu, 64);
    }
}

void VideoBox::show_video(int type, int index)
{
    switch (type) {
        case 1:
            change_video_1(index);
            break;
        case 4:
            change_video_4(index);
            break;
        case 6:
            change_video_6(index);
            break;
        case 8:
            change_video_8(index);
            break;
        case 9:
            change_video_9(index);
            break;
        default:
            break;
    }
}

void VideoBox::show_video()
{
    auto *action = qobject_cast<QAction *>(sender());
    // Retrieve dynamic properties
    int index = action->property("index").toInt() - 1;
    int type = action->property("type").toInt();
    QString videoType = action->property("flag").toString();

    if (m_videoType != videoType) {
        m_videoType = videoType;
        show_video(type, index);
    }
}

void VideoBox::showAllViews()
{
    int type = 1;
    if (m_videoType.startsWith("0_")) {
        int index = m_videoType.split("_").last().toInt() - 1;
        change_video_1(index);
        emit changeVideo(type, m_videoType, true);
    }
    else {
        int index = m_videoType.split("_").first().toInt() - 1;
        auto iter = m_types.begin();
        while (iter != m_types.end()) {
            QStringList flags = iter.value();
            if (flags.contains(m_videoType)) {
                type = iter.key();
                show_video(type, index);
                break;
            }
            iter++;
        }
    }
}

void VideoBox::hideAllViews()
{
    for (int i{0}; i < m_videoCount; ++i) {
        auto w = m_widgets.at(i);
        m_gridLayout->removeWidget(w);
        w->setVisible(false);
    }
}

// For square layout, it's easy to calculate the positions.
void VideoBox::change_video_normal(int index, int flag)
{
    hideAllViews();
    int size = 0;
    int row = 0;
    int column = 0;

    for (int i = 0; i < m_videoCount; ++i) {
        if (i >= index) {
            m_gridLayout->addWidget(m_widgets.at(i), row, column);
            m_widgets.at(i)->setVisible(true);

            size++;
            column++;
            if (column == flag) {
                row++;
                column = 0;
            }
        }

        if (size == (flag * flag)) {
            break;
        }
    }
}

void VideoBox::change_video_custom(int index, int type)
{
    QList<int> indexs;
    for (int i = index; i < (index + type); ++i) {
        indexs << i;
    }

    if (type == 6) {
        change_video_6(indexs);
    }
    else if (type == 8) {
        change_video_8(indexs);
    }
    else if (type == 13) {
        change_video_13(indexs);
    }
}

void VideoBox::change_video_6(const QList<int> &indices)
{
    if (indices.count() < 6) {
        return;
    }

    hideAllViews();

    m_gridLayout->addWidget(m_widgets[indices[0]], 0, 0, 2, 2);
    m_gridLayout->addWidget(m_widgets[indices[1]], 0, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[2]], 1, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[3]], 2, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[4]], 2, 1, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[5]], 2, 0, 1, 1);

    for (const auto &i : indices) {
        m_widgets.at(i)->setVisible(true);
    }
}

void VideoBox::change_video_8(const QList<int> &indices)
{
    if (indices.count() < 8) {
        return;
    }

    hideAllViews();

    m_gridLayout->addWidget(m_widgets[indices[0]], 0, 0, 3, 3);
    m_gridLayout->addWidget(m_widgets[indices[1]], 0, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[2]], 1, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[3]], 2, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[4]], 3, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[5]], 3, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[6]], 3, 1, 1, 1);
    m_gridLayout->addWidget(m_widgets[indices[7]], 3, 0, 1, 1);

    for (const auto &i : indices) {
        m_widgets.at(i)->setVisible(true);
    }
}

void VideoBox::change_video_13(const QList<int> &indices)
{
    if (indices.count() < 13) {
        return;
    }

    hideAllViews();

    m_gridLayout->addWidget(m_widgets.at(indices.at(0)), 0, 0, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(1)), 0, 1, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(2)), 0, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(3)), 0, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(4)), 1, 0, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(5)), 2, 0, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(6)), 1, 1, 2, 2);
    m_gridLayout->addWidget(m_widgets.at(indices.at(7)), 1, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(8)), 2, 3, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(9)), 3, 0, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(10)), 3, 1, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(11)), 3, 2, 1, 1);
    m_gridLayout->addWidget(m_widgets.at(indices.at(12)), 3, 3, 1, 1);

    for (const auto &i : indices) {
        m_widgets.at(i)->setVisible(true);
    }
}

void VideoBox::change_video_1(int index)
{
    hideAllViews();
    auto w = m_widgets.at(index);
    m_gridLayout->addWidget(w, 0, 0);
    w->setVisible(true);

    emit changeVideo(1, m_videoType, false);
}

void VideoBox::change_video_4(int index)
{
    change_video_normal(index, 2);
    emit changeVideo(2, m_videoType, false);
}

void VideoBox::change_video_6(int index)
{
    change_video_custom(index, 6);
    emit changeVideo(6, m_videoType, false);
}

void VideoBox::change_video_8(int index)
{
    change_video_custom(index, 8);
    emit changeVideo(8, m_videoType, false);
}

void VideoBox::change_video_9(int index)
{
    change_video_normal(index, 3);
    emit changeVideo(9, m_videoType, false);
}

///------- WidgetGridWidgetPrivate starts from here
class WidgetGridWidgetPrivate
{
    Q_DEFINE_PIMPL(WidgetGridWidget)

public:
    explicit WidgetGridWidgetPrivate(WidgetGridWidget *q);

    void init();

    void initForm();
    void doAction();

    void changeVideo(int type, const QString &videoType, bool videoMax);

    void setupMenu(QMenu &menu) const;

public:
    VideoBox *m_box;
    QGridLayout *m_gridLayout; // own
    bool m_max;
};

WidgetGridWidgetPrivate::WidgetGridWidgetPrivate(WidgetGridWidget *q) : q_ptr(q)
{
}

void WidgetGridWidgetPrivate::init()
{
    Q_Q(WidgetGridWidget);
    q->setContextMenuPolicy(Qt::CustomContextMenu);

    m_gridLayout = new QGridLayout(q);
    m_gridLayout->setContentsMargins(9, 9, 9, 9);
    m_gridLayout->setSpacing(6);

    initForm();
}

void WidgetGridWidgetPrivate::initForm()
{
    Q_Q(WidgetGridWidget);
    m_max = false;
    q->installEventFilter(q);

    QWidgetList widgets;
    for (int i = 0; i < 12; ++i) {
        auto *imageView = new ImageView;
        imageView->installEventFilter(q);
        imageView->setFrameShape(QLabel::Box);
        imageView->setAlignment(Qt::AlignCenter);
        widgets << imageView;
    }

    m_box = new VideoBox(q);
    m_box->setVideoType("1_4");
    m_box->setLayout(m_gridLayout);
    m_box->setWidgets(widgets);
    m_box->setMenuFlag("Layout");
    m_box->setActionFlag("Monitor");
    m_box->showAllViews();

    //    q->connect(m_box, &VideoBox::changeVideo, this,
    //               &WidgetGridWidgetPrivate::changeVideo);
}

void WidgetGridWidgetPrivate::changeVideo(int type, const QString &videoType,
                                          bool videoMax)
{
    qDebug() << QString("Main menu：%1  Sub-menu：%2")
                    .arg(QString::number(type), videoType);
}

void WidgetGridWidgetPrivate::setupMenu(QMenu &menu) const
{
    Q_Q(const WidgetGridWidget);
    menu.addAction("Switch to full screen", q,
                   [q]() { qDebug() << "Full screen."; });
    menu.addAction("Switch to loop mode", q,
                   [q]() { qDebug() << "Full screen."; });
    menu.addSeparator();

    QList<bool> enable;
    enable << true << true << true << true << true << true << true << true
           << true;
    m_box->initMenu(&menu, enable);
}

///------- WidgetGridWidget starts from here
WidgetGridWidget::WidgetGridWidget(QWidget *parent)
    : QWidget(parent), d_ptr(new WidgetGridWidgetPrivate(this))
{
    Q_D(WidgetGridWidget);
    d->init();

    connect(this, &QWidget::customContextMenuRequested, this,
            [this, d](const QPoint &pos) {
                QMenu menu;
                d->setupMenu(menu);
                menu.exec(mapToGlobal(pos));
            });
}

WidgetGridWidget::~WidgetGridWidget() = default;

WidgetGridWidget::GridType WidgetGridWidget::gridType() const
{
    // TODO
    return SixView;
}

void WidgetGridWidget::setGridType(GridType type)
{
    // TODO
}

int WidgetGridWidget::viewCount() const
{
    // TODO
    return 0;
}

void WidgetGridWidget::setViewCount() const
{
    // TODO
}

void WidgetGridWidget::showImage(const QImage &image, int index)
{
    Q_D(WidgetGridWidget);
    d->m_box->showImage(image, index);
}

bool WidgetGridWidget::eventFilter(QObject *watched, QEvent *event)
{
    Q_D(WidgetGridWidget);
    // Double click to maximize and restore
    if (event->type() == QEvent::MouseButtonDblClick) {
        auto *widget = qobject_cast<QWidget *>(watched);
        if (widget == this) {
            return true;
        }

        if (!d->m_max) {
            d->m_max = true;
            d->m_box->hideAllViews();
            d->m_gridLayout->addWidget(widget, 0, 0);
            widget->setVisible(true);
        }
        else {
            d->m_max = false;
            d->m_box->showAllViews();
        }

        return true;
    }

    return QWidget::eventFilter(watched, event);
}

} // namespace thoht

#include "WidgetGridWidget.moc"
#include "moc_WidgetGridWidget.cpp"
