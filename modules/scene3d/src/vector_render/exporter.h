#pragma once

#include <QString>
#include <QTextStream>

#include "primitive.h"

// Set of classes for exporting in various formats, like EPS, XFig3.2, SVG.
namespace vrender {

class VRenderParams;

class Exporter
{
public:
    Exporter();
    virtual ~Exporter() {}

    virtual void exportToFile(const QString& filename,
                              const std::vector<PrimitivePtr>&, VRenderParams&);

    void setBoundingBox(float xmin, float ymin, float xmax, float ymax);
    void setClearColor(float r, float g, float b);
    void setClearBackground(bool b);
    void setBlackAndWhite(bool b);

protected:
    virtual void spewPoint(const Point*, QTextStream& out) = 0;
    virtual void spewSegment(const Segment*, QTextStream& out) = 0;
    virtual void spewPolygone(const Polygone*, QTextStream& out) = 0;

    virtual void writeHeader(QTextStream& out) const = 0;
    virtual void writeFooter(QTextStream& out) const = 0;

    float _clearR, _clearG, _clearB;
    float _pointSize;
    float _lineWidth;

    GLfloat _xmin, _xmax, _ymin, _ymax, _zmin, _zmax;

    bool _clearBG, _blackAndWhite;
};

// Exports to encapsulated postscript.
class EPSExporter : public Exporter
{
public:
    EPSExporter();
    virtual ~EPSExporter() {}

protected:
    void spewPoint(const Point*, QTextStream& out) override;
    void spewSegment(const Segment*, QTextStream& out) override;
    void spewPolygone(const Polygone*, QTextStream& out) override;

    void writeHeader(QTextStream& out) const override;
    void writeFooter(QTextStream& out) const override;

private:
    void setColor(QTextStream& out, float, float, float);

    static float last_r;
    static float last_g;
    static float last_b;
};

//  Exports to postscript. The only difference is the filename extension and
// the showpage at the end.
class PSExporter : public EPSExporter
{
public:
    virtual ~PSExporter() {}

protected:
    void writeFooter(QTextStream& out) const override;
};

class FIGExporter : public Exporter
{
public:
    FIGExporter();
    virtual ~FIGExporter() {}

protected:
    void spewPoint(const Point*, QTextStream& out) override;
    void spewSegment(const Segment*, QTextStream& out) override;
    void spewPolygone(const Polygone*, QTextStream& out) override;

    void writeHeader(QTextStream& out) const override;
    void writeFooter(QTextStream& out) const override;

private:
    mutable int _sizeX;
    mutable int _sizeY;
    mutable int _depth;

    int FigCoordX(double) const;
    int FigCoordY(double) const;
    int FigGrayScaleIndex(float red, float green, float blue) const;
};

#ifdef A_FAIRE
class SVGExporter : public Exporter
{
protected:
    void spewPoint(const Point*, QTextStream& out) override;
    void spewSegment(const Segment*, QTextStream& out) override;
    void spewPolygone(const Polygone*, QTextStream& out) override;

    void writeHeader(QTextStream& out) const override;
    void writeFooter(QTextStream& out) const override;
};
#endif

} // namespace vrender
