#include "messagelogwidget.h"

#include <QApplication>
#include <QBasicTimer>
#include <QClipboard>
#include <QHash>
#include <QPainter>
#include <QPaintEvent>
#include <QRegularExpression>
#include <QScrollBar>

class MessageHighlighter
{
public:
    struct Style
    {
        QRegularExpression regex;
        QColor background;
        QColor foreground;

        Style() {}
        Style(const QColor& color)
            : background(Qt::transparent), foreground(color)
        {
        }
        Style(const QRegularExpression& regex, const QColor& bg,
              const QColor& fg)
            : regex(regex), background(bg), foreground(fg)
        {
        }
    };

    MessageHighlighter() {}
    ~MessageHighlighter() {}

    bool contains(int id) const { return (id > 0 && id <= styles.size()); }

    int setup(const Style& style)
    {
        styles.push_back(style);
        return styles.size();
    }
    inline int setup(const QRegularExpression& regex, const QColor& background,
                     const QColor& foreground)
    {
        return setup({regex, background, foreground});
    }

    int remove(int id)
    {
        if (id > 0 && id < styles.size()) {
            styles.removeAt(id - 1);
            return 1;
        }
        return 0;
    }

    int highlight(const QString& text) const
    {
        QRegularExpressionMatch match;
        int i = 1;
        for (auto it = styles.begin(); it != styles.end(); ++it, ++i) {
            match = it->regex.match(text);
            if (match.hasMatch())
                return i;
        }
        return 0;
    }

    const Style& style(const int id) const
    {
        static const Style defaultStyle(Qt::black);
        if (id > 0 && id <= styles.size()) {
            return styles[id - 1];
        }
        return defaultStyle;
    }

    void clear() { styles.clear(); }

private:
    QList<Style> styles;
};

class MessageLogWidgetPrivate
{
    Q_DEFINE_PIMPL(MessageLogWidget)

public:
    explicit MessageLogWidgetPrivate(MessageLogWidget* q);

    void updateCache() const;

    void triggerTimer()
    {
        Q_Q(MessageLogWidget);
        if (!timer.isActive()) {
            timer.start(25, q);
        }
    }

    void addPendingLines();
    void enforceHistorySize();
    void updateScrollRanges();

    void updateGeometry();

    QPair<int, int> visibleLines(int top, int bottom)
    {
        return qMakePair(qMax(0, lineByYCoordinate(top)),
                         qMax(0, 1 + lineByYCoordinate(bottom)));
    }

    int lineByYCoordinate(int x) const;

    QPoint scrollOffset() const;

    QRect lineRect(int idx) const
    {
        assert(!cache.dirty);
        return QRect(0, idx * cache.fontMetrics.lineSpacing,
                     cache.dimensions.longestLineLength,
                     cache.fontMetrics.lineSpacing - 1);
    }

    struct LineItem
    {
        QString text;
        unsigned int styleID;
    };

private:
    mutable struct Cache
    {
        enum
        {
            Dimensions = 1,
            FontMetrics = 2,
            All = FontMetrics | Dimensions
        };

        Cache() : dirty(All) {}
        int dirty;

        struct
        {
            int lineSpacing;
            int ascent;
            int averageCharWidth;
            QVector<int> lineWidths;
        } fontMetrics;

        struct
        {
            int indexOfLongestLine;
            int longestLineLength;
        } dimensions;
    } cache;

    MessageHighlighter highlighter;

    QVector<LineItem> lines, pendingLines;

    QRect visibleRect;
    QPair<int, int> linesVisible;

    int historySize;
    int minimumVisibleLines, minimumVisibleColumns;

    bool alternatingRowColors;

    QBasicTimer timer;
};

MessageLogWidgetPrivate::MessageLogWidgetPrivate(MessageLogWidget* q)
    : q_ptr(q),
      historySize(0xFFFFFFFF),
      minimumVisibleLines(1),
      minimumVisibleColumns(1),
      alternatingRowColors(false)
{
    // PENDING(marc) find all the magic flags we need here...
    QWidget* const vp = q->viewport();
    vp->setBackgroundRole(QPalette::Base);
    vp->setAttribute(Qt::WA_StaticContents);
    vp->setAttribute(Qt::WA_NoSystemBackground);
#ifndef QT_NO_CURSOR
    vp->setCursor(Qt::IBeamCursor);
#endif
}

void MessageLogWidgetPrivate::updateCache() const
{
    Q_Q(const MessageLogWidget);
    if (cache.dirty >= Cache::FontMetrics) {
        const auto fm = q->fontMetrics();
        cache.fontMetrics.lineSpacing = fm.lineSpacing();
        cache.fontMetrics.ascent = fm.ascent();
        cache.fontMetrics.averageCharWidth = fm.averageCharWidth();

        auto& lineWidths = cache.fontMetrics.lineWidths;
        lineWidths.clear();
        lineWidths.reserve(lines.size());
        for (const auto& line : lines) {
            lineWidths.push_back(fm.horizontalAdvance(line.text));
        }
    }

    if (cache.dirty >= Cache::Dimensions) {
        const auto& lineWidths = cache.fontMetrics.lineWidths;
        const auto max = std::max_element(lineWidths.begin(), lineWidths.end());
        if (max == lineWidths.end()) {
            cache.dimensions.indexOfLongestLine = -1;
            cache.dimensions.longestLineLength = 0;
        }
        else {
            cache.dimensions.indexOfLongestLine = max - lineWidths.begin();
            cache.dimensions.longestLineLength = *max;
        }
    }

    cache.dirty = false;
}

void MessageLogWidgetPrivate::enforceHistorySize()
{
    const size_t numLimes = lines.size();
    if (numLimes <= historySize)
        return;

    const int remove = numLimes - historySize;
    lines.erase(lines.cbegin(), lines.cbegin() + remove);

    // Can't quickly update the dimensions if the fontMetrics aren't uptodate.
    if (cache.dirty & Cache::FontMetrics) {
        cache.dirty |= Cache::Dimensions;
        return;
    }

    QVector<int>& lw = cache.fontMetrics.lineWidths;

    assert(lw.size() > remove);
    lw.erase(lw.cbegin(), lw.cbegin() + remove);

    if (cache.dirty & Cache::Dimensions)
        return;

    if (cache.dimensions.indexOfLongestLine >= remove) {
        cache.dimensions.indexOfLongestLine -= remove;
    }
    else {
        cache.dirty |= Cache::Dimensions;
    }
}

void MessageLogWidgetPrivate::updateScrollRanges()
{
    updateCache();

    auto setScrollBarProps = [](QScrollBar& sb, int document, int viewport,
                                int singleStep, Qt::Orientation o) {
        const int min = 0;
        const int max = std::max(0, document - viewport);
        const int value = sb.value();
        const bool wasAtEnd = value == sb.maximum();
        sb.setRange(min, max);
        sb.setPageStep(viewport);
        sb.setSingleStep(singleStep);
        sb.setValue(o == Qt::Vertical && wasAtEnd ? sb.maximum() : value);
    };

    Q_Q(MessageLogWidget);
    if (auto* const sb = q->verticalScrollBar()) {
        const int document = lines.size() * cache.fontMetrics.lineSpacing;
        const int viewport = q->viewport()->height();
        const int singleStep = cache.fontMetrics.lineSpacing;
        setScrollBarProps(*sb, document, viewport, singleStep, Qt::Vertical);
    }

    if (auto* const sb = q->horizontalScrollBar()) {
        const int document = cache.dimensions.longestLineLength;
        const int viewport = q->viewport()->width();
        const int singleStep = cache.fontMetrics.lineSpacing;
        setScrollBarProps(*sb, document, viewport, singleStep, Qt::Horizontal);
    }
}

void MessageLogWidgetPrivate::updateGeometry()
{
    Q_Q(MessageLogWidget);
    QPoint p = -scrollOffset();
    QTransform m;
    m.translate(p.x(), p.y());
    visibleRect = m.inverted().mapRect(q->rect());
    linesVisible = visibleLines(visibleRect.top(), visibleRect.bottom());
    assert(linesVisible.first <= linesVisible.second);
}

void MessageLogWidgetPrivate::addPendingLines()
{
    if (pendingLines.empty())
        return;

    Q_Q(MessageLogWidget);
    const unsigned int oldNumLines = lines.size();

    lines += pendingLines;

    // if the cache isn't dirty, we can quickly update it without
    // invalidation:

    if (!cache.dirty) {
        // update fontMetrics:
        const QFontMetrics& fm = q->fontMetrics();
        QVector<int>& lw = cache.fontMetrics.lineWidths;
        lw.reserve(pendingLines.size());
        for (auto it = pendingLines.begin(); it != pendingLines.end(); ++it) {
            lw.push_back(fm.horizontalAdvance(it->text));
        }

        // update dimensions:
        const auto it = std::max_element(lw.cbegin() + oldNumLines, lw.cend());
        if (*it >= cache.dimensions.longestLineLength) {
            cache.dimensions.longestLineLength = *it;
            cache.dimensions.indexOfLongestLine =
                oldNumLines + (it - lw.cbegin());
        }

        cache.fontMetrics.lineSpacing = fm.lineSpacing();
        cache.fontMetrics.ascent = fm.ascent();

        cache.fontMetrics.averageCharWidth = fm.averageCharWidth();
    }

    pendingLines.clear();

    enforceHistorySize();
    updateScrollRanges();
    q->viewport()->update();
}

int MessageLogWidgetPrivate::lineByYCoordinate(int y) const
{
    updateCache();
    if (cache.fontMetrics.lineSpacing == 0)
        return -1;

    const int raw = y / cache.fontMetrics.lineSpacing;
    if (raw < 0)
        return -1;

    if (raw >= lines.size())
        return lines.size() - 1;
    return raw;
}

QPoint MessageLogWidgetPrivate::scrollOffset() const
{
    auto scrollBarOffset = [](const QScrollBar* sb) -> int {
        return sb ? sb->value() : 0;
    };

    Q_Q(const MessageLogWidget);
    return {scrollBarOffset(q->horizontalScrollBar()),
            scrollBarOffset(q->verticalScrollBar())};
}

MessageLogWidget::MessageLogWidget(QWidget* parent)
    : QAbstractScrollArea(parent), d_ptr(new MessageLogWidgetPrivate(this))
{
}

MessageLogWidget::~MessageLogWidget() = default;

void MessageLogWidget::setHistorySize(int hs)
{
    Q_D(MessageLogWidget);
    if (hs == d->historySize) {
        return;
    }

    d->historySize = hs;
    d->enforceHistorySize();
    d->updateScrollRanges();
    viewport()->update();
}

unsigned int MessageLogWidget::historySize() const
{
    Q_D(const MessageLogWidget);
    return d->historySize;
}

QString MessageLogWidget::text() const
{
    Q_D(const MessageLogWidget);
    QString result;
    result.reserve((d->lines.size() + d->pendingLines.size()) *
                   qMin(512, d->cache.dimensions.longestLineLength / 4));

    // Maybe use join()
    for (const auto& line : d->lines) {
        result += line.text;
        result += '\n';
    }
    for (const auto& line : d->pendingLines) {
        result += line.text;
        result += '\n';
    }
    // Erase last '\n'
    result.remove(result.size() - 1, 1);
    return result;
}

void MessageLogWidget::setMinimumVisibleLines(int num)
{
    Q_D(MessageLogWidget);
    if (num == d->minimumVisibleLines) {
        return;
    }

    d->minimumVisibleLines = num;
    updateGeometry();
}

unsigned int MessageLogWidget::minimumVisibleLines() const
{
    Q_D(const MessageLogWidget);
    return d->minimumVisibleLines;
}

void MessageLogWidget::setMinimumVisibleColumns(int num)
{
    Q_D(MessageLogWidget);
    if (num == d->minimumVisibleColumns) {
        return;
    }

    d->minimumVisibleColumns = num;
    updateGeometry();
}

unsigned int MessageLogWidget::minimumVisibleColumns() const
{
    Q_D(const MessageLogWidget);
    return d->minimumVisibleColumns;
}

void MessageLogWidget::setAlternatingRowColors(bool on)
{
    Q_D(MessageLogWidget);
    if (on == d->alternatingRowColors) {
        return;
    }

    d->alternatingRowColors = on;
    update();
}

bool MessageLogWidget::alternatingRowColors() const
{
    Q_D(const MessageLogWidget);
    return d->alternatingRowColors;
}

int MessageLogWidget::setupStyle(const QRegularExpression& regex,
                                 const QColor& background,
                                 const QColor& foreground)
{
    Q_D(MessageLogWidget);
    return d->highlighter.setup(regex, background, foreground);
}

int MessageLogWidget::removeStyle(int id)
{
    Q_D(MessageLogWidget);
    return d->highlighter.remove(id);
}

void MessageLogWidget::clearStyles()
{
    Q_D(MessageLogWidget);
    return d->highlighter.clear();
}

QSize MessageLogWidget::minimumSizeHint() const
{
    Q_D(const MessageLogWidget);
    d->updateCache();
    const QSize base = QAbstractScrollArea::minimumSizeHint();
    const QSize view(
        d->minimumVisibleColumns * d->cache.fontMetrics.averageCharWidth,
        d->minimumVisibleLines * d->cache.fontMetrics.lineSpacing);
    const QSize scrollbars(
        verticalScrollBar() ? verticalScrollBar()->minimumSizeHint().width()
                            : 0,
        horizontalScrollBar()
            ? horizontalScrollBar()->minimumSizeHint().height()
            : 0);
    return base + view + scrollbars;
}

QSize MessageLogWidget::sizeHint() const
{
    Q_D(const MessageLogWidget);
    if (d->minimumVisibleLines > 1 || d->minimumVisibleColumns > 1) {
        return minimumSizeHint();
    }

    return 2 * minimumSizeHint();
}

void MessageLogWidget::clear()
{
    Q_D(MessageLogWidget);
    d->timer.stop();
    d->linesVisible.first = 0;
    d->linesVisible.second = 0;
    d->lines.clear();
    d->pendingLines.clear();
    d->cache.dirty = MessageLogWidgetPrivate::Cache::All;
    viewport()->update();
}

void MessageLogWidget::message(const QString& str)
{
    Q_D(MessageLogWidget);

    MessageLogWidgetPrivate::LineItem li;
    li.text = str;
    li.styleID = d->highlighter.highlight(str);
    d->pendingLines.push_back(li);
    d->triggerTimer();
}

void MessageLogWidget::scrollContentsBy(int dx, int dy)
{
    Q_D(MessageLogWidget);
    d->updateScrollRanges();
    d->updateCache();
    d->updateGeometry();
    QAbstractScrollArea::scrollContentsBy(dx, dy);
}

void MessageLogWidget::paintEvent(QPaintEvent*)
{
    Q_D(MessageLogWidget);

    // d->updateCache();

    QPainter painter(viewport());
    painter.translate(-d->scrollOffset());

    const auto& cache = d->cache;

    painter.setPen(Qt::NoPen);
    painter.setBrush(palette().base());

    if (d->alternatingRowColors) {
        painter.drawRect(d->visibleRect);
        painter.setBrush(palette().alternateBase());
        for (auto i = d->linesVisible.first % 2 ? d->linesVisible.first
                                                : d->linesVisible.first + 1,
                  end = d->linesVisible.second;
             i < end; i += 2) {
            painter.drawRect(d->lineRect(i).adjusted(0, 0, size().width(), 0));
        }
    }
    else {
        painter.drawRect(d->visibleRect);
    }

    // Unused optimization: paint lines by styles to minimise pen changes.
    for (auto i = d->linesVisible.first, end = d->linesVisible.second; i != end;
         ++i) {
        const auto& line = d->lines[i];
        assert(!line.styleID || d->highlighter.contains(line.styleID));

        const auto& style = d->highlighter.style(line.styleID);
        painter.setPen(Qt::NoPen);
        painter.setBrush(style.background);
        painter.drawRect(d->lineRect(i).adjusted(0, 0, size().width(), 0));

        painter.setPen(style.foreground);
        painter.drawText(
            0, i * cache.fontMetrics.lineSpacing + cache.fontMetrics.ascent,
            line.text);
    }
}

void MessageLogWidget::timerEvent(QTimerEvent* e)
{
    Q_D(MessageLogWidget);
    if (e->timerId() == d->timer.timerId()) {
        d->timer.stop();
        d->addPendingLines();
        d->updateGeometry();
    }
    else {
        QAbstractScrollArea::timerEvent(e);
    }
}

void MessageLogWidget::changeEvent(QEvent* e)
{
    Q_D(MessageLogWidget);
    QAbstractScrollArea::changeEvent(e);
    d->cache.dirty |= MessageLogWidgetPrivate::Cache::FontMetrics;
    d->updateCache();
    d->updateGeometry();
    update();
}

void MessageLogWidget::keyPressEvent(QKeyEvent* e)
{
#ifndef QT_NO_CLIPBOARD
    if (e->matches(QKeySequence::Copy)) {
        QGuiApplication::clipboard()->setText(text());
    }
#endif
    QAbstractScrollArea::keyPressEvent(e);
}

void MessageLogWidget::resizeEvent(QResizeEvent* e)
{
    Q_D(MessageLogWidget);
    d->updateScrollRanges();
    d->updateCache();
    d->updateGeometry();
    QAbstractScrollArea::resizeEvent(e);
}
