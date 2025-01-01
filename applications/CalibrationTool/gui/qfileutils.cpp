#include "qfileutils.h"

#include <QDir>

namespace tl {
namespace qfile {

bool copyDir(const QString &source, const QString &dest, bool recursively)
{
    // TODO:
    // 1. Not all the directories are allow to copy
    QDir sourceDir{source};
    if (!sourceDir.exists()) {
        qDebug() << "Source directory does not exist:" << source;
        return false;
    }

    QDir destDir{dest};
    if (!destDir.exists()) {
        destDir.mkpath(".");
    }

    for (const auto &item :
         sourceDir.entryInfoList(QDir::NoDotAndDotDot | QDir::AllEntries)) {
        const auto srcItemPath = item.filePath();
        const auto dstItemPath = destDir.filePath(item.fileName());

        if (item.isDir()) {
            if (recursively) {
                if (!copyDir(srcItemPath, dstItemPath, recursively)) {
                    return false;
                }
            }
        }
        else if (item.isFile()) {
            // Copy files
            if (!QFile::copy(srcItemPath, dstItemPath)) {
                qDebug() << "Failed to copy file:" << srcItemPath;
                return false;
            }
        }
    }
    return true;
}

} // namespace qfile
} // namespace tl
