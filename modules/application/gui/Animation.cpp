#include "Animation.h"

#include <QEvent>
#include <QMap>
#include <QPainter>
#include <QPropertyAnimation>
#include <QTimeLine>
#include <QWidget>

namespace tl {

///------- AbstractAnimator starts from here
class AbstractAnimator : public QObject
{
    Q_OBJECT

public:
    using QObject::QObject;

    virtual int animationDuration() const = 0;

    virtual void animateForward() = 0;
    virtual void animateBackward() = 0;

protected:
    void setAnimatedForward()
    {
        m_isAnimated = true;
        m_isAnimatedForward = true;
    }
    void setAnimatedBackward()
    {
        m_isAnimated = true;
        m_isAnimatedForward = false;
    }
    void setAnimatedStopped() { m_isAnimated = false; }

    bool isAnimated() const { return m_isAnimated; }

    bool isAnimatedForward() const { return m_isAnimatedForward; }
    bool isAnimatedBackward() const { return !isAnimatedForward(); }

private:
    bool m_isAnimated = false;
    bool m_isAnimatedForward = true;
};

///------- SideSlideAnimator starts from here
// This is a private helper
class SideSlideDecorator : public QWidget
{
    Q_OBJECT
    Q_PROPERTY(QPoint slidePos READ slidePos WRITE setSlidePos)

public:
    explicit SideSlideDecorator(QWidget* parent);

    void grabSlideWidget(QWidget* slideWidget);

    void grabParent();

    void decorate(bool dark);

    QPoint slidePos() const;
    Q_SLOT void setSlidePos(const QPoint& pos);

signals:
    void clicked();

protected:
    void paintEvent(QPaintEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;

private:
    QPoint m_slidePos;
    QPixmap m_slideWidgetPixmap;
    QTimeLine m_timeline;
    QPixmap m_backgroundPixmap;
    QColor m_decorationColor;
};

SideSlideDecorator::SideSlideDecorator(QWidget* _parent) : QWidget(_parent)
{
    resize(maximumSize());

    m_timeline.setDuration(260);
    m_timeline.setUpdateInterval(40);
    m_timeline.setEasingCurve(QEasingCurve::OutQuad);
    m_timeline.setStartFrame(0);
    m_timeline.setEndFrame(10000);

    m_decorationColor = QColor(0, 0, 0, 0);

    connect(&m_timeline, &QTimeLine::frameChanged, this, [this](int value) {
        m_decorationColor = QColor(0, 0, 0, value / 100);
        update();
    });
}

void SideSlideDecorator::grabSlideWidget(QWidget* slideWidget)
{
    m_slideWidgetPixmap = slideWidget->grab();
}

void SideSlideDecorator::grabParent()
{
#if defined(Q_OS_ANDROID) || defined(Q_OS_IOS)
    //
    // В андройде Qt не умеет рисовать прозрачные виджеты
    // https://bugreports.qt.io/browse/QTBUG-43635 поэтому сохранем картинку с
    // изображением подложки
    //
    m_backgroundPixmap = parentWidget()->grab();
#endif
}

void SideSlideDecorator::decorate(bool dark)
{
    if (m_timeline.state() == QTimeLine::Running) {
        m_timeline.stop();
    }

    m_timeline.setDirection(dark ? QTimeLine::Forward : QTimeLine::Backward);
    m_timeline.start();
}

QPoint SideSlideDecorator::slidePos() const { return m_slidePos; }

void SideSlideDecorator::setSlidePos(const QPoint& pos)
{
    if (m_slidePos != pos) {
        m_slidePos = pos;
        update();
    }
}

void SideSlideDecorator::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.drawPixmap(0, 0, m_backgroundPixmap);
    painter.fillRect(rect(), m_decorationColor);
    painter.drawPixmap(m_slidePos, m_slideWidgetPixmap);

    QWidget::paintEvent(event);
}

void SideSlideDecorator::mousePressEvent(QMouseEvent* event)
{
    emit clicked();

    QWidget::mousePressEvent(event);
}

class SideSlideAnimator : public AbstractAnimator
{
    Q_OBJECT

public:
    explicit SideSlideAnimator(QWidget* widgetForSlide);

    int animationDuration() const override;

    void animateForward() override;
    void animateBackward() override;

    void setApplicationSide(Animation::ApplicationSide side);

    void setDecorateBackground(bool decorate);

    void slideIn();
    void slideOut();

protected:
    bool eventFilter(QObject* object, QEvent* event) override;

private:
    QWidget* widgetForSlide() const;

private:
    Animation::ApplicationSide m_side;
    SideSlideDecorator* m_decorator = nullptr;
    QPropertyAnimation* m_animation = nullptr;
    bool m_decorateBackground;
};

SideSlideAnimator::SideSlideAnimator(QWidget* _widgetForSlide)
    : AbstractAnimator(_widgetForSlide),
      m_decorateBackground(true),
      m_decorator(new SideSlideDecorator(_widgetForSlide->parentWidget())),
      m_animation(new QPropertyAnimation(m_decorator, "slidePos"))
{
    Q_ASSERT(_widgetForSlide);
    _widgetForSlide->parentWidget()->installEventFilter(this);

    m_animation->setEasingCurve(QEasingCurve::InQuad);
    m_animation->setDuration(260);

    m_decorator->hide();

    connect(m_animation, &QPropertyAnimation::finished, this, [this] {
        setAnimatedStopped();
        if (isAnimatedForward()) {
            widgetForSlide()->move(m_decorator->slidePos());
        }
        else {
            m_decorator->hide();
        }
    });

    connect(m_decorator, &SideSlideDecorator::clicked, this,
            &SideSlideAnimator::slideOut);
}

void SideSlideAnimator::setApplicationSide(Animation::ApplicationSide side)
{
    if (m_side != side) {
        m_side = side;
    }
}

void SideSlideAnimator::setDecorateBackground(bool decorate)
{
    if (m_decorateBackground != decorate) {
        m_decorateBackground = decorate;
    }
}

int SideSlideAnimator::animationDuration() const
{
    return m_animation->duration();
}

void SideSlideAnimator::animateForward() { slideIn(); }

void SideSlideAnimator::animateBackward() { slideOut(); }

void SideSlideAnimator::slideIn()
{
    if (isAnimated() && isAnimatedForward()) {
        return;
    }

    setAnimatedForward();

    widgetForSlide()->lower();
    widgetForSlide()->move(-widgetForSlide()->width(),
                           -widgetForSlide()->height());
    widgetForSlide()->show();
    widgetForSlide()->resize(widgetForSlide()->sizeHint());

    QWidget* topWidget = widgetForSlide()->parentWidget();
    while (topWidget->parentWidget()) {
        topWidget = topWidget->parentWidget();
    }

    // TODO
    QSize finalSize = widgetForSlide()->size();
    QPoint startPosition, finalPosition;
    switch (m_side) {
        case Animation::LeftSide: {
            finalSize.setHeight(topWidget->height());
            startPosition = QPoint(-finalSize.width(), 0);
            finalPosition = QPoint(0, 0);
            break;
        }

        case Animation::TopSide: {
            finalSize.setWidth(topWidget->width());
            startPosition = QPoint(0, -finalSize.height());
            finalPosition = QPoint(0, 0);
            break;
        }

        case Animation::RightSide: {
            finalSize.setHeight(topWidget->height());
            startPosition = QPoint(topWidget->width(), 0);
            finalPosition = QPoint(topWidget->width() - finalSize.width(), 0);
            break;
        }

        case Animation::BottomSide: {
            finalSize.setWidth(topWidget->width());
            startPosition = QPoint(0, topWidget->height());
            finalPosition = QPoint(0, topWidget->height() - finalSize.height());
            break;
        }
    }

    if (m_decorateBackground) {
        m_decorator->setParent(topWidget);
        m_decorator->move(0, 0);
        m_decorator->grabParent();
        m_decorator->show();
        m_decorator->raise();
    }

    widgetForSlide()->move(startPosition);
    widgetForSlide()->resize(finalSize);
    widgetForSlide()->raise();

    m_decorator->grabSlideWidget(widgetForSlide());

    if (m_animation->state() == QPropertyAnimation::Running) {
        m_animation->pause();
        m_animation->setDirection(QPropertyAnimation::Backward);
        m_animation->resume();
    }
    else {
        m_animation->setEasingCurve(QEasingCurve::OutQuart);
        m_animation->setDirection(QPropertyAnimation::Forward);
        m_animation->setStartValue(startPosition);
        m_animation->setEndValue(finalPosition);
        m_animation->start();
    }

    if (m_decorateBackground) {
        m_decorator->decorate(true);
    }
}

void SideSlideAnimator::slideOut()
{
    if (isAnimated() && isAnimatedBackward()) {
        return;
    }

    setAnimatedBackward();

    if (widgetForSlide()->isVisible()) {
        QWidget* topWidget = widgetForSlide()->parentWidget();
        while (topWidget->parentWidget()) {
            topWidget = topWidget->parentWidget();
        }

        widgetForSlide()->hide();

        const QSize finalSize = widgetForSlide()->size();
        QPoint startPosition = widgetForSlide()->pos(), finalPosition;
        switch (m_side) {
            case Animation::LeftSide: {
                startPosition = QPoint(0, 0);
                finalPosition = QPoint(-finalSize.width(), 0);
                break;
            }

            case Animation::TopSide: {
                startPosition = QPoint(0, 0);
                finalPosition = QPoint(0, -finalSize.height());
                break;
            }

            case Animation::RightSide: {
                startPosition =
                    QPoint(topWidget->width() - finalSize.width(), 0);
                finalPosition = QPoint(topWidget->width(), 0);
                break;
            }

            case Animation::BottomSide: {
                startPosition =
                    QPoint(0, topWidget->height() - finalSize.height());
                finalPosition = QPoint(0, topWidget->height());
                break;
            }
        }

        if (m_animation->state() == QPropertyAnimation::Running) {
            m_animation->pause();
            m_animation->setDirection(QPropertyAnimation::Backward);
            m_animation->resume();
        }
        else {
            m_animation->setEasingCurve(QEasingCurve::InQuart);
            m_animation->setDirection(QPropertyAnimation::Forward);
            m_animation->setStartValue(startPosition);
            m_animation->setEndValue(finalPosition);
            m_animation->start();
        }

        if (m_decorateBackground) {
            m_decorator->decorate(false);
        }
    }
}

bool SideSlideAnimator::eventFilter(QObject* object, QEvent* event)
{
    if (object == widgetForSlide()->parentWidget() &&
        event->type() == QEvent::Resize) {
        QWidget* widgetForSlideParent = widgetForSlide()->parentWidget();
        switch (m_side) {
            case Animation::RightSide: {
                widgetForSlide()->move(
                    widgetForSlideParent->width() - widgetForSlide()->width(),
                    0);
            }
            case Animation::LeftSide: {
                widgetForSlide()->resize(widgetForSlide()->width(),
                                         widgetForSlideParent->height());
                break;
            }

            case Animation::BottomSide: {
                widgetForSlide()->move(0, widgetForSlideParent->height() -
                                              widgetForSlide()->height());
            }
            case Animation::TopSide: {
                widgetForSlide()->resize(widgetForSlideParent->width(),
                                         widgetForSlide()->height());
                break;
            }
        }

        m_decorator->grabSlideWidget(widgetForSlide());
    }

    return QObject::eventFilter(object, event);
}

QWidget* SideSlideAnimator::widgetForSlide() const
{
    return qobject_cast<QWidget*>(parent());
}

///------- SlideAnimator starts from here
// This is a private helper
class SlideForegroundDecorator : public QWidget
{
    Q_OBJECT

public:
    using QWidget::QWidget;

    void grabParent(const QSize& size);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QPixmap m_foreground;
};

void SlideForegroundDecorator::grabParent(const QSize& size)
{
    resize(size);
    m_foreground = QPixmap(size);
    parentWidget()->render(&m_foreground, QPoint(),
                           QRegion(QRect(QPoint(), size)));
}

void SlideForegroundDecorator::paintEvent(QPaintEvent* event)
{
    QPainter painter(this);
    painter.drawPixmap(0, 0, m_foreground);

    QWidget::paintEvent(event);
}

class SlideAnimator : public AbstractAnimator
{
    Q_OBJECT

public:
    explicit SlideAnimator(QWidget* widgetForSlide);

    int animationDuration() const override;
    void animateForward() override;
    void animateBackward() override;

    void setAnimationDirection(Animation::AnimationDirection direction);

    void setFixBackground(bool fix);
    void setFixStartSize(bool fix);

    void slideIn();
    void slideOut();

protected:
    bool eventFilter(QObject* object, QEvent* event) override;

private:
    bool isWidth() const;

    void fixSize(const QSize& sourceSize, QSize& targetSize) const;
    void fixSizeOfWidgetForSlide(const QSize& sourceSize) const;
    QWidget* widgetForSlide() const;

private:
    Animation::AnimationDirection m_direction;

    bool m_isFixBackground;

    bool m_isFixStartSize;

    QSize m_startMinSize, m_startMaxSize, m_startSize;

    QPropertyAnimation* m_animation;

    SlideForegroundDecorator* m_decorator;
};

SlideAnimator::SlideAnimator(QWidget* _widgetForSlide)
    : AbstractAnimator(_widgetForSlide),
      m_direction(Animation::FromLeftToRight),
      m_isFixBackground(true),
      m_isFixStartSize(false),
      m_animation(new QPropertyAnimation(_widgetForSlide, "maximumWidth")),
      m_decorator(new SlideForegroundDecorator(_widgetForSlide))
{
    Q_ASSERT(_widgetForSlide);

    _widgetForSlide->installEventFilter(this);

    m_animation->setDuration(300);

    m_decorator->hide();

    connect(m_animation, &QPropertyAnimation::valueChanged, this, [this] {
        if (isWidth()) {
            widgetForSlide()->setMinimumWidth(widgetForSlide()->maximumWidth());
        }
        else {
            widgetForSlide()->setMinimumHeight(
                widgetForSlide()->maximumHeight());
        }
    });

    connect(m_animation, &QPropertyAnimation::finished, this, [this] {
        setAnimatedStopped();
        m_decorator->hide();
    });
}

void SlideAnimator::setAnimationDirection(
    Animation::AnimationDirection direction)
{
    if (m_direction != direction) {
        m_direction = direction;
        m_animation->setPropertyName(isWidth() ? "maximumWidth"
                                               : "maximumHeight");
    }
}

void SlideAnimator::setFixBackground(bool fix)
{
    if (m_isFixBackground != fix) {
        m_isFixBackground = fix;
    }
}

void SlideAnimator::setFixStartSize(bool fix)
{
    if (m_isFixStartSize != fix) {
        m_isFixStartSize = fix;
    }
}

int SlideAnimator::animationDuration() const { return m_animation->duration(); }

void SlideAnimator::animateForward() { slideIn(); }

void SlideAnimator::animateBackward() { slideOut(); }

void SlideAnimator::slideIn()
{
    if (!m_startMinSize.isValid()) {
        m_startMinSize = widgetForSlide()->minimumSize();
    }
    if (!m_startMaxSize.isValid()) {
        m_startMaxSize = widgetForSlide()->maximumSize();
    }
    if (!m_startSize.isValid()) {
        m_startSize = widgetForSlide()->sizeHint();
    }

    if (isAnimated() && isAnimatedForward()) {
        return;
    }

    setAnimatedForward();

    if (isWidth()) {
        widgetForSlide()->setMaximumWidth(0);
    }
    else {
        widgetForSlide()->setMaximumHeight(0);
    }
    widgetForSlide()->show();
    const QSize currentSize = widgetForSlide()->size();

    QSize finalSize = currentSize;
    fixSize(m_startSize, finalSize);

    widgetForSlide()->hide();
    fixSizeOfWidgetForSlide(finalSize);
    m_decorator->grabParent(finalSize);
    fixSizeOfWidgetForSlide(currentSize);
    widgetForSlide()->show();

    if (m_isFixBackground) {
        m_decorator->move(0, 0);
        m_decorator->show();
        m_decorator->raise();
    }

    if (m_animation->state() == QPropertyAnimation::Running) {
        m_animation->pause();
        m_animation->setDirection(QPropertyAnimation::Backward);
        m_animation->resume();
    }
    else {
        m_animation->setEasingCurve(QEasingCurve::OutQuart);
        m_animation->setDirection(QPropertyAnimation::Forward);
        m_animation->setStartValue(isWidth() ? widgetForSlide()->width()
                                             : widgetForSlide()->height());
        m_animation->setEndValue(isWidth() ? finalSize.width()
                                           : finalSize.height());
        m_animation->start();
    }
}

void SlideAnimator::slideOut()
{
    if (!m_startMinSize.isValid()) {
        m_startMinSize = widgetForSlide()->minimumSize();
    }
    if (!m_startMaxSize.isValid()) {
        m_startMaxSize = widgetForSlide()->maximumSize();
    }
    if (!m_startSize.isValid() || !m_isFixStartSize) {
        m_startSize = widgetForSlide()->size();
    }

    if (isAnimated() && isAnimatedBackward()) {
        return;
    }

    setAnimatedBackward();

    QSize finalSize = widgetForSlide()->size();
    if (isWidth()) {
        finalSize.setWidth(0);
    }
    else {
        finalSize.setHeight(0);
    }

    m_decorator->grabParent(widgetForSlide()->size());

    if (m_isFixBackground) {
        m_decorator->move(0, 0);
        m_decorator->show();
        m_decorator->raise();
    }

    if (m_animation->state() == QPropertyAnimation::Running) {
        m_animation->pause();
        m_animation->setDirection(QPropertyAnimation::Backward);
        m_animation->resume();
    }
    else {
        m_animation->setEasingCurve(QEasingCurve::InQuart);
        m_animation->setDirection(QPropertyAnimation::Forward);
        m_animation->setStartValue(isWidth() ? widgetForSlide()->width()
                                             : widgetForSlide()->height());
        m_animation->setEndValue(isWidth() ? finalSize.width()
                                           : finalSize.height());
        m_animation->start();
    }
}

bool SlideAnimator::eventFilter(QObject* object, QEvent* event)
{
    if (object == widgetForSlide() && event->type() == QEvent::Resize &&
        m_decorator->isVisible()) {
        switch (m_direction) {
            case Animation::FromLeftToRight: {
                m_decorator->move(
                    widgetForSlide()->width() - m_decorator->width(), 0);
            } break;
            case Animation::FromTopToBottom: {
                m_decorator->move(
                    0, widgetForSlide()->height() - m_decorator->height());

            } break;
            case Animation::FromRightToLeft:
            case Animation::FromBottomToTop:
                break;
            default:
                Q_ASSERT_X(0, Q_FUNC_INFO, "Not setted animation direction");
        }
    }

    return QObject::eventFilter(object, event);
}

bool SlideAnimator::isWidth() const
{
    return m_direction == Animation::FromLeftToRight ||
           m_direction == Animation::FromRightToLeft;
}

void SlideAnimator::fixSize(const QSize& sourceSize, QSize& targetSize) const
{
    if (isWidth()) {
        targetSize.setWidth(sourceSize.width());
    }
    else {
        targetSize.setHeight(sourceSize.height());
    }
}

void SlideAnimator::fixSizeOfWidgetForSlide(const QSize& sourceSize) const
{
    if (isWidth()) {
        widgetForSlide()->setFixedWidth(sourceSize.width());
    }
    else {
        widgetForSlide()->setFixedHeight(sourceSize.height());
    }
}

QWidget* SlideAnimator::widgetForSlide() const
{
    return qobject_cast<QWidget*>(parent());
}

///------- AnimationPrivate starts from here
class AnimationPrivate
{
public:
    enum AnimatorType
    {
        SideSlide,
        Slide,
        Popup
    };

    bool hasAnimator(QWidget* widget, AnimatorType type) const
    {
        if (m_animators.contains(type)) {
            return m_animators.value(type).contains(widget);
        }
        return false;
    }

    AbstractAnimator* animator(QWidget* widget, AnimatorType type) const
    {
        if (m_animators.contains(type)) {
            return m_animators.value(type).value(widget, nullptr);
        }
        return nullptr;
    }

    void saveAnimator(QWidget* widget, AbstractAnimator* animator,
                      AnimatorType type)
    {
        if (!hasAnimator(widget, type)) {
            auto animators = m_animators.value(type);
            animators.insert(widget, animator);
            m_animators.insert(type, animators);
        }
    }

private:
    QMap<AnimatorType, QHash<QWidget*, AbstractAnimator*>> m_animators;
};

int Animation::sideSlide(QWidget* widget, ApplicationSide side,
                         bool decorateBackground, bool in)
{
    constexpr auto kType = AnimationPrivate::SideSlide;

    AbstractAnimator* animator{nullptr};
    if (pimpl()->hasAnimator(widget, kType)) {
        animator = pimpl()->animator(widget, kType);
        if (auto* sideSlideAnimator =
                qobject_cast<SideSlideAnimator*>(animator)) {
            sideSlideAnimator->setApplicationSide(side);
            sideSlideAnimator->setDecorateBackground(decorateBackground);
        }
    }
    else {
        auto* sideSlideAnimator = new SideSlideAnimator(widget);
        sideSlideAnimator->setApplicationSide(side);
        sideSlideAnimator->setDecorateBackground(decorateBackground);
        animator = sideSlideAnimator;

        pimpl()->saveAnimator(widget, animator, kType);
    }

    return runAnimation(animator, in);
}

int Animation::slide(QWidget* widget, AnimationDirection direction,
                     bool fixBackground, bool fixStartSize, bool in)
{
    constexpr auto kType = AnimationPrivate::Slide;

    AbstractAnimator* animator{nullptr};
    if (pimpl()->hasAnimator(widget, kType)) {
        animator = pimpl()->animator(widget, kType);
        if (auto* slideAnimator = qobject_cast<SlideAnimator*>(animator)) {
            slideAnimator->setAnimationDirection(direction);
            slideAnimator->setFixBackground(fixBackground);
            slideAnimator->setFixStartSize(fixStartSize);
        }
    }
    else {
        auto* slideAnimator = new SlideAnimator(widget);
        slideAnimator->setAnimationDirection(direction);
        slideAnimator->setFixBackground(fixBackground);
        slideAnimator->setFixStartSize(fixStartSize);
        animator = slideAnimator;

        pimpl()->saveAnimator(widget, animator, kType);
    }

    return runAnimation(animator, in);
}

int Animation::runAnimation(AbstractAnimator* animator, bool in)
{
    if (in) {
        animator->animateForward();
    }
    else {
        animator->animateBackward();
    }

    return animator->animationDuration();
}

AnimationPrivate* Animation::m_pimpl = nullptr;
AnimationPrivate* Animation::pimpl()
{
    if (!m_pimpl) {
        m_pimpl = new AnimationPrivate;
    }

    return m_pimpl;
}

} // namespace tl

#include "Animation.moc"
