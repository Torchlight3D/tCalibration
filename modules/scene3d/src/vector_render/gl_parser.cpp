#include "gl_parser.h"

#include <stdio.h>

#include "vrender_params.h"
#include "primitive.h"

using namespace vrender;

class ParserUtils
{
public:
    static void NormalizeBufferCoordinates(GLint size, GLfloat *buffer,
                                           GLfloat MaxSize, GLfloat &zmin,
                                           GLfloat &zmax);
    
    static PrimitivePtr checkPoint(Point *&P);
    static PrimitivePtr checkSegment(Segment *&P);
    static PrimitivePtr checkPolygon(Polygone *&P);

    static void ComputeBufferBB(GLint size, GLfloat *buffer, GLfloat &xmin,
                                GLfloat &xmax, GLfloat &ymin, GLfloat &ymax,
                                GLfloat &zmin, GLfloat &zmax);

private:
    static void print3DcolorVertex(GLint size, GLint *count, GLfloat *buffer);
    static void debug_printBuffer(GLint size, GLfloat *buffer);

    static void NormalizePrimitiveCoordinates(GLfloat *&loc, GLfloat MaxSize,
                                              GLfloat zmin, GLfloat zmax);
    static void ComputePrimitiveBB(GLfloat *&loc, GLfloat &xmin, GLfloat &xmax,
                                   GLfloat &ymin, GLfloat &ymax, GLfloat &zmin,
                                   GLfloat &zmax);

    static const char *nameOfToken(int token);

    static const double EGALITY_EPS;
};

const double ParserUtils::EGALITY_EPS = 0.00001;

void ParserGL::parseFeedbackBuffer(GLfloat *buffer, int size,
                                   std::vector<PrimitivePtr> &primitive_tab,
                                   VRenderParams &vparams)
{
    int token;
    int nvertices = 0;
    nb_lines = 0;
    nb_polys = 0;
    nb_points = 0;
    nb_degenerated_lines = 0;
    nb_degenerated_polys = 0;
    nb_degenerated_points = 0;

    // pre-treatment of coordinates so as to get something more consistent

    _xmin = FLT_MAX;
    _ymin = FLT_MAX;
    _zmin = FLT_MAX;
    _xmax = -FLT_MAX;
    _ymax = -FLT_MAX;
    _zmax = -FLT_MAX;

    ParserUtils::ComputeBufferBB(size, buffer, _xmin, _xmax, _ymin, _ymax,
                                 _zmin, _zmax);

#ifdef DEBUGEPSRENDER
    printf("Buffer bounding box: %f %f %f %f %f %f\n", xmin, xmax, ymin, ymax,
           zmin, zmax);
#endif
    float Zdepth = std::max(_ymax - _ymin, _xmax - _xmin);
    ParserUtils::NormalizeBufferCoordinates(size, buffer, Zdepth, _zmin, _zmax);

    // now, read buffer
    GLfloat *end = buffer + size;

    GLfloat *loc = buffer;
    int next_step = 0;
    int N = size / 200 + 1;

    while (loc < end) {
        token = int(0.5f + *loc);
        loc++;

        if ((end - loc) / N >= next_step)
            vparams.progress((end - loc) / (float)size,
                             "Parsing feedback buffer."),
                ++next_step;

        switch (token) {
            case GL_LINE_TOKEN:
            case GL_LINE_RESET_TOKEN: {
                auto *S = new Segment(
                    Feedback3DColor(loc),
                    Feedback3DColor(loc + Feedback3DColor::sizeInBuffer()));

                primitive_tab.push_back(ParserUtils::checkSegment(S));

                if (!S)
                    nb_degenerated_lines++;

                nb_lines++;
                loc += 2 * Feedback3DColor::sizeInBuffer();
            } break;

            case GL_POLYGON_TOKEN: {
                nvertices = int(0.5f + *loc);
                loc++;

                std::vector<Feedback3DColor> verts;
                for (int i = 0; i < nvertices; ++i)
                    verts.push_back(Feedback3DColor(loc)),
                        loc += Feedback3DColor::sizeInBuffer();

                Polygone *P = new Polygone(verts);

                primitive_tab.push_back(ParserUtils::checkPolygon(P));

                if (!P)
                    nb_degenerated_polys++;

                nb_polys++;
            } break;

            case GL_POINT_TOKEN: {
                auto *Pt = new Point(Feedback3DColor(loc));

                primitive_tab.push_back(Pt); // ParserUtils::checkPoint(Pt)) ;

                if (!Pt)
                    nb_degenerated_points++;

                nb_points++;
                loc += Feedback3DColor::sizeInBuffer();
            } break;
            default:
                break;
        }
    }
}

// Traitement des cas degeneres. Renvoie false si le polygone est degenere.
// Traitement des cas degeneres. Renvoie false si le segment est degenere.

PrimitivePtr ParserUtils::checkPoint(Point *&P) { return P; }

PrimitivePtr ParserUtils::checkSegment(Segment *&P)
{
    if ((P->vertex(0) - P->vertex(1)).infNorm() < EGALITY_EPS) {
        auto *pp = new Point(P->sommet3DColor(0));
        delete P;
        P = nullptr;

        return checkPoint(pp);
    }

    return P;
}

PrimitivePtr ParserUtils::checkPolygon(Polygone *&P)
{
    if (P->nbVertices() != 3) {
        std::cout << "unexpected case: Polygon with " << P->nbVertices()
                  << " vertices !" << std::endl;
        delete P;
        return nullptr;
    }

    if (P->FlatFactor() < FLAT_POLYGON_EPS) {
        // On ne traite que le cas du triangle plat, vu qu'on est sur d'avoir un
        // triangle

        size_t n = P->nbVertices();

        for (size_t i = 0; i < n; ++i)
            if ((P->vertex(i) - P->vertex((i + 1) % n)).norm() > EGALITY_EPS) {
                Segment *pp = new Segment(P->sommet3DColor((i + 1) % n),
                                          P->sommet3DColor((i + 2) % n));
                delete P;
                P = nullptr;

                return checkSegment(pp);
            }

        Point *pp = new Point(P->sommet3DColor(0));
        delete P;
        P = nullptr;

        return checkPoint(pp);
    }

    // No problem detected.

    return P;
}

/* Write contents of one vertex to stdout. */

void ParserUtils::print3DcolorVertex(GLint size, GLint *count, GLfloat *buffer)
{
    printf("  ");
    for (size_t i = 0; i < Feedback3DColor::sizeInBuffer(); i++) {
        printf("%4.2f ", buffer[size - (*count)]);
        *count = *count - 1;
    }
    printf("\n");
}

void ParserUtils::debug_printBuffer(GLint size, GLfloat *buffer)
{
    GLint count;
    int token, nvertices;

    count = size;
    while (count) {
        token = int(buffer[size - count]);
        count--;
        switch (token) {
            case GL_PASS_THROUGH_TOKEN:
                printf("GL_PASS_THROUGH_TOKEN\n");
                printf("  %4.2f\n", buffer[size - count]);
                count--;
                break;
            case GL_POINT_TOKEN:
                printf("GL_POINT_TOKEN\n");
                print3DcolorVertex(size, &count, buffer);
                break;
            case GL_LINE_TOKEN:
                printf("GL_LINE_TOKEN\n");
                print3DcolorVertex(size, &count, buffer);
                print3DcolorVertex(size, &count, buffer);
                break;
            case GL_LINE_RESET_TOKEN:
                printf("GL_LINE_RESET_TOKEN\n");
                print3DcolorVertex(size, &count, buffer);
                print3DcolorVertex(size, &count, buffer);
                break;
            case GL_POLYGON_TOKEN:
                printf("GL_POLYGON_TOKEN\n");
                nvertices = int(buffer[size - count]);
                count--;
                for (; nvertices > 0; nvertices--)
                    print3DcolorVertex(size, &count, buffer);
        }
    }
}

void ParserUtils::NormalizePrimitiveCoordinates(GLfloat *&loc, GLfloat MaxSize,
                                                GLfloat zmin, GLfloat zmax)
{
    int token;
    int nvertices, i;

    token = int(*loc);
    loc++;
    int size = Feedback3DColor::sizeInBuffer();

    switch (token) {
        case GL_LINE_RESET_TOKEN:
        case GL_LINE_TOKEN: {
            for (i = 0; i < 2; i++)
                (loc + size * i)[2] =
                    ((loc + size * i)[2] - zmin) / (zmax - zmin) * MaxSize;

            loc += 2 * size; /* Each vertex element in the feedback buffer is
                                size GLfloats. */
            break;
        }
        case GL_POLYGON_TOKEN: {
            nvertices = int(*loc);
            loc++;

            for (i = 0; i < nvertices; i++)
                (loc + size * i)[2] =
                    ((loc + size * i)[2] - zmin) / (zmax - zmin) * MaxSize;

            loc += nvertices * size; /* Each vertex element in the feedback
                                        buffer is size GLfloats. */
            break;
        }
        case GL_POINT_TOKEN: {
            loc[2] = (loc[2] - zmin) / (zmax - zmin) * MaxSize;

            loc += size; /* Each vertex element in the feedback buffer is size
                            GLfloats. */
            break;
        }
        default:
            /* XXX Left as an excersie to the reader. */
#ifdef DEBUGEPSRENDER
            printf("%s (%d) not handled yet. Sorry.\n",
                   ParserUtils::nameOfToken(token), token);
#endif
            ;
    }
}

void ParserUtils::ComputePrimitiveBB(GLfloat *&loc, GLfloat &xmin,
                                     GLfloat &xmax, GLfloat &ymin,
                                     GLfloat &ymax, GLfloat &zmin,
                                     GLfloat &zmax)
{
    int token;
    int nvertices, i;

    token = int(*loc);
    loc++;
    int size = Feedback3DColor::sizeInBuffer();

    switch (token) {
        case GL_LINE_RESET_TOKEN:
        case GL_LINE_TOKEN: {
            for (i = 0; i < 2; i++) {
                Feedback3DColor f(loc + size * i);

                if (f.x() < xmin)
                    xmin = GLfloat(f.x());
                if (f.y() < ymin)
                    ymin = GLfloat(f.y());
                if (f.z() < zmin)
                    zmin = GLfloat(f.z());
                if (f.x() > xmax)
                    xmax = GLfloat(f.x());
                if (f.y() > ymax)
                    ymax = GLfloat(f.y());
                if (f.z() > zmax)
                    zmax = GLfloat(f.z());
            }

            loc += 2 * size; /* Each vertex element in the feedback
                                         buffer is size GLfloats. */
            break;
        }
        case GL_POLYGON_TOKEN: {
            nvertices = int(*loc);
            loc++;

            for (i = 0; i < nvertices; i++) {
                Feedback3DColor f(loc + size * i);

                if (f.x() < xmin)
                    xmin = GLfloat(f.x());
                if (f.y() < ymin)
                    ymin = GLfloat(f.y());
                if (f.z() < zmin)
                    zmin = GLfloat(f.z());
                if (f.x() > xmax)
                    xmax = GLfloat(f.x());
                if (f.y() > ymax)
                    ymax = GLfloat(f.y());
                if (f.z() > zmax)
                    zmax = GLfloat(f.z());
            }

            loc += nvertices *
                   size; /* Each vertex element in the
                                     feedback buffer is size GLfloats. */
            break;
        }
        case GL_POINT_TOKEN: {
            Feedback3DColor f(loc);

            if (f.x() < xmin)
                xmin = GLfloat(f.x());
            if (f.y() < ymin)
                ymin = GLfloat(f.y());
            if (f.z() < zmin)
                zmin = GLfloat(f.z());
            if (f.x() > xmax)
                xmax = GLfloat(f.x());
            if (f.y() > ymax)
                ymax = GLfloat(f.y());
            if (f.z() > zmax)
                zmax = GLfloat(f.z());

            loc += size; /* Each vertex element in the feedback
                                    buffer is size GLfloats. */
            break;
        }
        default:
            /* XXX Left as an excersie to the reader. */
#ifdef DEBUGEPSRENDER
            printf("Incomplete implementation.  Unexpected token (%d).\n",
                   token);
#endif
            ;
    }
}

void ParserUtils::NormalizeBufferCoordinates(GLint size, GLfloat *buffer,
                                             GLfloat MaxSize, GLfloat &zmin,
                                             GLfloat &zmax)
{
    GLfloat *loc, *end;

    if (zmax == zmin) {
#ifdef DEBUGEPSRENDER
        printf("Warning: zmin = zmax in NormalizePrimitiveCoordinates\n");
#endif
        return;
    }

    loc = buffer;
    end = buffer + size;

    while (loc < end) NormalizePrimitiveCoordinates(loc, MaxSize, zmin, zmax);

    zmin = 0.0;
    zmax = MaxSize;
}

void ParserUtils::ComputeBufferBB(GLint size, GLfloat *buffer, GLfloat &xmin,
                                  GLfloat &xmax, GLfloat &ymin, GLfloat &ymax,
                                  GLfloat &zmin, GLfloat &zmax)
{
    GLfloat *loc, *end;

    loc = buffer;
    end = buffer + size;

    while (loc < end)
        ComputePrimitiveBB(loc, xmin, xmax, ymin, ymax, zmin, zmax);
}

typedef struct _DepthIndex
{
    GLfloat *ptr;
    GLfloat depth;
} DepthIndex;

const char *ParserUtils::nameOfToken(int token)
{
    switch (token) {
        case GL_PASS_THROUGH_TOKEN:
            return "GL_PASS_THROUGH_TOKEN";
        case GL_POINT_TOKEN:
            return "GL_POINT_TOKEN";
        case GL_LINE_TOKEN:
            return "GL_LINE_TOKEN";
        case GL_POLYGON_TOKEN:
            return "GL_POLYGON_TOKEN";
        case GL_BITMAP_TOKEN:
            return "GL_BITMAP_TOKEN";
        case GL_DRAW_PIXEL_TOKEN:
            return "GL_DRAW_PIXEL_TOKEN";
        case GL_COPY_PIXEL_TOKEN:
            return "GL_COPY_PIXEL_TOKEN";
        case GL_LINE_RESET_TOKEN:
            return "GL_LINE_RESET_TOKEN";
        default:
            return "(Unidentified token)";
    }
}
