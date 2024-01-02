#pragma once

#include <qcp/qcustomplot.h>
#include "SelectionRect.h"

namespace thoht {

using ElementIdRect = std::tuple<int, int, int>;

class BoardLayout;
class BoardElement;
class DataManager;
class Project;

class Board : public QCustomPlot
{
    Q_OBJECT

public:
    explicit Board(QWidget *parent = nullptr);
    ~Board() override;

    BoardLayout *dashboardLayout() const;
    QSharedPointer<DataManager> dataManager() const;
    QSharedPointer<Project> project() const;

    bool toggleFullscreen();

    void addElement(BoardElement *element, QPointF pos);
    BoardElement *element(QPointF pos);

    void checkModification();

    void loadHistoricalData();
    void initDataManager();

    static QString elementNameMimeType() { return u"board/element_name"_qs; }
    static QString elementCopyMimeType() { return u"board/elements_copy"_qs; }
    static QString parameterNameMimeType()
    {
        return u"board/parameter_name"_qs;
    }
    static QString propertyNameMimeType() { return u"board/property_name"_qs; }
    static QString alarmConfigMimeType() { return u"board/alarm_config"_qs; }

    bool liveDataRefreshEnabled() const;
    void setLiveDataRefreshEnabled(bool liveDataRefreshEnabled);
    void setUserInteractions(bool userInteractions);
    void setMouseInteractions(bool mouseInteractions);
    void setStatMode(bool statMode);

    double currentTimestamp();
    bool statMode() const;

    QColor randomColor(int index = 0);

    void setReferencePlaybackTime(int referencePlaybackTime);
    void setReferencePlaybackDate(const QDate &referencePlaybackDate);
    void setLastPlaybackTime();

    bool pageModified() const;
    void setPageModified(bool pageModified);

    void setRefreshTimerPeriod(int refreshTimerPeriod);

    bool mouseInteractions() const;

    bool layoutLocked() const;
    void setLayoutLocked(bool layoutLocked);

signals:
    void timeUpdate(QDateTime);
    void mouseLeave(QEvent *e);
    void widgetClosed();
    void dataSourcesLoaded();

public slots:
    void reloadProject();
    void clearPage();
    void loadPage();
    void savePage();
    void checkParameters();
    void startLiveTimer();
    void stopLiveTimer();
    void replotLayer(QString layerName);

    void updateElements(bool forceUpdate = false);
    void updateFinished();
    void partialReplot();

    void updateLive();
    void updateLiveData();

    void updatePlayback(int msecSinceStartOfTheDay, QDate date);
    void updatePlaybackData();
    void resetPlayback();

protected slots:
    void editElement();
    void clearElements();
    void deleteElements();
    void cutElements();
    void copyElements();
    void pasteElements();

protected:
    void dropEvent(QDropEvent *event) override;
    void dragEnterEvent(QDragEnterEvent *event) override;
    void dragMoveEvent(QDragMoveEvent *event) override;
    void dragLeaveEvent(QDragLeaveEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseDoubleClickEvent(QMouseEvent *event) override;
    void leaveEvent(QEvent *event) override;
    void resizeEvent(QResizeEvent *event) override;
    void closeEvent(QCloseEvent *event) override;

    void buildPalette();
    int mColorIndex;

    QTimer *mLiveRefreshTimer;
    int mRefreshTimerPeriod;

    BoardLayout *mDashboardLayout;
    QSharedPointer<DataManager> mDataManager;
    QSharedPointer<Project> mProject;

    QDateTime mReferenceLiveTime;
    int mReferencePlaybackTime;
    QDate mReferencePlaybackDate;

    bool mLayoutLocked;
    bool mPageLoading{false};
    bool mBoardReplotting{false};
    bool mPageModified{false};
    bool mLiveDataRefreshEnabled{true};
    bool mUserInteractions{false};
    bool mStatMode{false};
    bool mMouseInteractions{false};

    bool mFirstReplot;

    QLabel *mEmptyBoardLabel;

    QWidget *mFullScreenParent;
    QWidget *mFullScreenEmbeddedParent;
    int mFullScreenParentLayoutIndex;
    int mFullScreenParentGridRow;
    int mFullScreenParentGridCol;

    QShortcut *mFullscreenLockedShortcut;

    SelectionRect *mSaveRect;

    QPoint mMousePos;
    SelectionRect *mHoverRect;
    SelectionRect::HoverHandle mResizeOrientation;

    QParallelAnimationGroup *mAnimationGroup;

    QRubberBand *mRubberBand;
    QPoint mRubberBandOrigin;

    QVector<QSharedPointer<SelectionRect>> mSelectedRects;
    QVector<int> mSelectedIndexes;

    QMenu *mContextMenu;
    QAction *mEditElementAction;
    QAction *mClearElementsAction;
    QAction *mDeleteElementsAction;
    QAction *mCutElementsAction;
    QAction *mCopyElementsAction;
    QAction *mPasteElementsAction;
};
} // namespace thoht
