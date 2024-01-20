#pragma once

class QWidget;

namespace tl {

class AbstractAnimator;
class AnimationPrivate;

// Even though all of the public interfaces are static, we need a class to
// manage animations life time.
class Animation
{
public:
    enum ApplicationSide
    {
        LeftSide,
        TopSide,
        RightSide,
        BottomSide
    };

    enum AnimationDirection
    {
        Undefined,

        FromLeftToRight,
        FromTopToBottom,
        FromRightToLeft,
        FromBottomToTop
    };

    static int sideSlide(QWidget* widget, ApplicationSide side = LeftSide,
                         bool decorateBackground = true, bool in = true);
    inline static int sideSlideIn(QWidget* widget,
                                  ApplicationSide side = LeftSide,
                                  bool decorateBackground = true)
    {
        return sideSlide(widget, side, decorateBackground);
    }
    inline static int sideSlideOut(QWidget* widget,
                                   ApplicationSide side = LeftSide,
                                   bool decorateBackground = true)
    {
        return sideSlide(widget, side, decorateBackground, false);
    }

    static int slide(QWidget* widget, AnimationDirection direction,
                     bool fixBackground = true, bool fixStartSize = false,
                     bool in = true);
    inline static int slideIn(QWidget* widget, AnimationDirection direction,
                              bool fixBackground = true,
                              bool fixStartSize = false)
    {
        return slide(widget, direction, fixStartSize, fixStartSize);
    }
    inline static int slideOut(QWidget* widget, AnimationDirection direction,
                               bool fixBackground = true,
                               bool fixStartSize = false)
    {
        return slide(widget, direction, fixStartSize, fixStartSize, false);
    }

private:
    static int runAnimation(AbstractAnimator* animator, bool in);

private:
    static AnimationPrivate* m_pimpl;
    static AnimationPrivate* pimpl();
};

} // namespace tl
