#pragma once

#include <algorithm>
#include <vector>

#include <AxCore/AxGlobal>

namespace apriltags {

//! A lookup table in 2D for implementing nearest neighbor.
template <class T>
class Gridder
{
    struct Cell
    {
        T* object{nullptr};
        Cell* next{nullptr};

        Cell() = default;
        Cell(const Cell& c) : object(c.object), next(c.next) {}
        Cell& operator=(const Cell& other)
        {
            if (this == &other) {
                return *this;
            }

            object = other.object;
            next = other.next;
            return *this;
        }
        ~Cell() { delete next; }
    };

public:
    Gridder(float x0, float y0, float x1, float y1, float ppCell)
        : x0_(x0),
          y0_(y0),
          x1_(),
          y1_(),
          width_(),
          height_(),
          pixels_per_cell_(ppCell),
          cells_()
    {
        init(x0, y0, x1, y1, ppCell);
    }

    ~Gridder()
    {
        for (size_t i{0}; i < cells_.size(); i++) {
            for (size_t j{0}; j < cells_[i].size(); j++) {
                delete cells_[i][j];
            }
        }
    }

    void add(float x, float y, T* object)
    {
        int ix = (int)((x - x0_) / pixels_per_cell_);
        int iy = (int)((y - y0_) / pixels_per_cell_);

        if (ix >= 0 && iy >= 0 && ix < width_ && iy < height_) {
            auto* c = new Cell;
            c->object = object;
            c->next = cells_[iy][ix];
            cells_[iy][ix] = c;
        }
    }

    class Iterator
    {
    public:
        Iterator(Gridder* grid, float x, float y, float range)
            : _outer(grid),
              _x0(),
              _x1(),
              _y0(),
              _y1(),
              _x(),
              _y(),
              _cell(nullptr)
        {
            init(x, y, range);
        }

        Iterator(const Iterator& it)
            : _outer(it.outer),
              _x0(it.ix0),
              _x1(it.ix1),
              _y0(it.iy0),
              _y1(it.iy1),
              _x(it.ix),
              _y(it.iy),
              _cell(it._cell)
        {
        }

        Iterator& operator=(const Iterator& it)
        {
            _outer = it.outer;
            _x0 = it.ix0;
            _x1 = it.ix1;
            _y0 = it.iy0;
            _y1 = it.iy1;
            _x = it.ix;
            _y = it.iy;
            _cell = it._cell;
        }

        bool hasNext()
        {
            if (!_cell)
                findNext();
            return _cell;
        }

        T& next()
        {
            T* thisObj = _cell->object;
            findNext();
            return *thisObj; // return Segment
        }

    private:
        void init(float x, float y, float range)
        {
            _x0 = (int)((x - range - _outer->x0_) / _outer->pixels_per_cell_);
            _y0 = (int)((y - range - _outer->y0_) / _outer->pixels_per_cell_);

            _x1 = (int)((x + range - _outer->x0_) / _outer->pixels_per_cell_);
            _y1 = (int)((y + range - _outer->y0_) / _outer->pixels_per_cell_);

            _x0 = std::max(0, _x0);
            _x0 = std::min(_outer->width_ - 1, _x0);

            _x1 = std::max(0, _x1);
            _x1 = std::min(_outer->width_ - 1, _x1);

            _y0 = std::max(0, _y0);
            _y0 = std::min(_outer->height_ - 1, _y0);

            _y1 = std::max(0, _y1);
            _y1 = std::min(_outer->height_ - 1, _y1);

            _x = _x0;
            _y = _y0;

            _cell = _outer->cells_[_y][_x];
        }

        void findNext()
        {
            if (_cell) {
                _cell = _cell->next;
            }
            if (_cell) {
                return;
            }

            _x++;
            while (true) {
                if (_x > _x1) {
                    _y++;
                    _x = _x0;
                }
                if (_y > _y1) {
                    break;
                }

                _cell = _outer->cells_[_y][_x];

                if (_cell) {
                    break;
                }
                _x++;
            }
        }

    private:
        Gridder* _outer;
        int _x0, _x1, _y0, _y1;
        int _x, _y;
        Cell* _cell;
    };

    using iterator = Iterator;
    iterator find(float x, float y, float range) { return {this, x, y, range}; }

private:
    DISABLE_COPY(Gridder)

    void init(float x0, float y0, float x1, float y1, float pixels_per_cell)
    {
        width_ = static_cast<int>((x1 - x0) / pixels_per_cell + 1);
        height_ = static_cast<int>((y1 - y0) / pixels_per_cell + 1);

        x1_ = x0 + pixels_per_cell * width_;
        y1_ = y0 + pixels_per_cell * height_;
        cells_ = std::vector<std::vector<Cell*>>(
            height_, std::vector<Cell*>(width_, nullptr));
    }

private:
    float x0_, y0_, x1_, y1_;
    float pixels_per_cell_;
    int width_, height_;
    std::vector<std::vector<Cell*>> cells_;
};

} // namespace apriltags
