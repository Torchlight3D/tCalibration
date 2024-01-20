#pragma once

#include "BoardElement.h"

namespace tl {

class BoardElementPrivate
{
    Q_DECLARE_PUBLIC(BoardElement)
    Q_DISABLE_COPY(BoardElementPrivate)
    BoardElementPrivate *const q_ptr;

public:
    explicit BoardElementPrivate(BoardElement *q);

    void init();

public:
};
} // namespace tl
