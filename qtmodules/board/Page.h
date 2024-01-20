#pragma once

#include <QSharedData>

namespace tl {

class Board;

class Page : public QSharedData
{
public:
    Page(const QString &pageName, const QString &pageDirectory);

    void loadPageInformation();
    void savePageInformation();

    void loadPageElements(Board *board);
    void savePageElements(Board *board);

    void setName(const QString &name);
    QString name() const;

    void setDescription(const QString &description);
    QString description() const;

    void setColumnCount(int count);
    int columnCount() const;

    void setRowCount(int count);
    int rowCount() const;

    void setSingleElementColumnCount(int count);
    int singleElementColumnCount() const;

    void setSingleElementRowCount(int count);
    int singleElementRowCount() const;

    void setBackground(const QString &background);
    QString background() const;

    void setPageDirectory(const QString &pageDirectory);
    QString pageDirectory() const;

private:
    QString mSettingsPath;

    QString mName;
    QString mPageDirectory;
    QString mDescription;
    QString mBackground;
    int mColumnCount;
    int mRowCount;
    int mSingleElementColumnCount;
    int mSingleElementRowCount;
};

} // namespace tl
