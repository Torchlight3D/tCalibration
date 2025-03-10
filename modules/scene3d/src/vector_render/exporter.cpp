#include "exporter.h"

#include <QFile>
#include <QMessageBox>

#include "vrender_params.h"

using namespace vrender;

Exporter::Exporter()
{
    _xmin = _xmax = _ymin = _ymax = _zmin = _zmax = 0.0;
    _pointSize = 1;
}

void Exporter::exportToFile(const QString &filename,
                            const std::vector<PrimitivePtr> &primitive_tab,
                            VRenderParams &vparams)
{
    QFile file{filename};
    if (!file.open(QIODevice::WriteOnly | QIODevice::Text)) {
        QMessageBox::warning(nullptr, "Exporter error",
                             QString("Unable to open file %1.").arg(filename));
        return;
    }

    QTextStream out{&file};

    writeHeader(out);

    unsigned int N = primitive_tab.size() / 200 + 1;
    for (size_t i = 0; i < primitive_tab.size(); ++i) {
        auto *p = dynamic_cast<Point *>(primitive_tab[i]);
        auto *s = dynamic_cast<Segment *>(primitive_tab[i]);
        auto *P = dynamic_cast<Polygone *>(primitive_tab[i]);

        if (p)
            spewPoint(p, out);
        if (s)
            spewSegment(s, out);
        if (P)
            spewPolygone(P, out);

        if (i % N == 0)
            vparams.progress(i / (float)primitive_tab.size(),
                             QString("Exporting to file %1").arg(filename));
    }

    writeFooter(out);

    file.close();
}

void Exporter::setBoundingBox(float xmin, float ymin, float xmax, float ymax)
{
    _xmin = xmin;
    _ymin = ymin;
    _xmax = xmax;
    _ymax = ymax;
}

void Exporter::setClearColor(float r, float g, float b)
{
    _clearR = r;
    _clearG = g;
    _clearB = b;
}
void Exporter::setClearBackground(bool b) { _clearBG = b; }
void Exporter::setBlackAndWhite(bool b) { _blackAndWhite = b; }
