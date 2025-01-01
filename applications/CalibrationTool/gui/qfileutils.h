#pragma once

#include <QString>

namespace tl {
namespace qfile {

bool copyDir(const QString &source, const QString &dest, bool recursively);

}
} // namespace tl
