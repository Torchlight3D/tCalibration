#include "viewer.h"
#include "ui_ImageInterface.h"

#include <QApplication>
#include <QClipboard>
#include <QCursor>
#include <QFileDialog>
#include <QFileInfo>
#include <QImageWriter>
#include <QInputDialog>
#include <QMap>
#include <QMessageBox>
#include <QProgressDialog>
#include <QScreen>

#ifndef NO_VECTORIAL_RENDER
#include "vector_render/vrender_params.h"
#include "ui_VRenderInterface.h"
#endif

#include "camera.h"

// List of available output file formats, formatted for QFileDialog.
static QString formats;
// Converts QFileDialog resulting format to Qt snapshotFormat.
static QMap<QString, QString> Qtformat;
// Converts Qt snapshotFormat to QFileDialog menu string.
static QMap<QString, QString> FDFormatString;
// Converts snapshotFormat to file extension
static QMap<QString, QString> extension;

const QString &QtOpenGLViewer::snapshotFileName() const
{
    return snapshotFileName_;
}

void QtOpenGLViewer::setSnapshotFileName(const QString &name)
{
    snapshotFileName_ = QFileInfo(name).absoluteFilePath();
}

const QString &QtOpenGLViewer::snapshotFormat() const
{
    return snapshotFormat_;
}

void QtOpenGLViewer::setSnapshotFormat(const QString &format)
{
    snapshotFormat_ = format;
}

int QtOpenGLViewer::snapshotCounter() const { return snapshotCounter_; }

void QtOpenGLViewer::setSnapshotCounter(int counter)
{
    snapshotCounter_ = counter;
}

int QtOpenGLViewer::snapshotQuality() { return snapshotQuality_; }

void QtOpenGLViewer::setSnapshotQuality(int quality)
{
    snapshotQuality_ = quality;
}

bool QtOpenGLViewer::openSnapshotFormatDialog()
{
    QStringList list = formats.split(";;", Qt::SkipEmptyParts);
    int current = list.indexOf(FDFormatString[snapshotFormat()]);

    bool ok{false};
    QString format = QInputDialog::getItem(this, "Snapshot format",
                                           "Select a snapshot format", list,
                                           current, false, &ok);
    if (ok)
        setSnapshotFormat(Qtformat[format]);
    return ok;
}

// Finds all available Qt output formats, so that they can be available in
// saveSnapshot dialog. Initialize snapshotFormat() to the first one.
void QtOpenGLViewer::initializeSnapshotFormats()
{
    QList<QByteArray> list = QImageWriter::supportedImageFormats();
    QStringList formatList;
    for (int i = 0; i < list.size(); ++i) {
        formatList << QString(list.at(i).toUpper());
        //        qWarning("Available image formats: ");
        //        auto it = formatList.begin();
        //        while( it != formatList.end() )
        //  	      qWarning((*it++).);  QT4 change this. qWarning no longer
        //  accepts QString
    }

#ifndef NO_VECTORIAL_RENDER
    // We add the 3 vectorial formats to the list
    formatList += "EPS";
    formatList += "PS";
    formatList += "XFIG";
#endif

    // Check that the interesting formats are available and add them in
    // "formats" Unused formats: XPM XBM PBM PGM
    QStringList QtText, MenuText, Ext;
    QtText += "JPEG";
    MenuText += "JPEG (*.jpg)";
    Ext += "jpg";
    QtText += "PNG";
    MenuText += "PNG (*.png)";
    Ext += "png";
    QtText += "EPS";
    MenuText += "Encapsulated Postscript (*.eps)";
    Ext += "eps";
    QtText += "PS";
    MenuText += "Postscript (*.ps)";
    Ext += "ps";
    QtText += "PPM";
    MenuText += "24bit RGB Bitmap (*.ppm)";
    Ext += "ppm";
    QtText += "BMP";
    MenuText += "Windows Bitmap (*.bmp)";
    Ext += "bmp";
    QtText += "XFIG";
    MenuText += "XFig (*.fig)";
    Ext += "fig";

    auto itText = QtText.begin();
    auto itMenu = MenuText.begin();
    auto itExt = Ext.begin();
    while (itText != QtText.end()) {
        // QMessageBox::information(this, "Snapshot ", "Trying
        // format\n"+(*itText));
        if (formatList.contains((*itText))) {
            // QMessageBox::information(this, "Snapshot ", "Recognized
            // format\n"+(*itText));
            if (formats.isEmpty())
                setSnapshotFormat(*itText);
            else
                formats += ";;";
            formats += (*itMenu);
            Qtformat[(*itMenu)] = (*itText);
            FDFormatString[(*itText)] = (*itMenu);
            extension[(*itText)] = (*itExt);
        }
        // Synchronize parsing
        itText++;
        itMenu++;
        itExt++;
    }
}

// Returns false if the user refused to use the fileName
static bool checkFileName(QString &fileName, QWidget *widget,
                          const QString &snapshotFormat)
{
    if (fileName.isEmpty())
        return false;

    // Check that extension has been provided
    QFileInfo info(fileName);

    if (info.suffix().isEmpty()) {
        // No extension given. Silently add one
        if (fileName.right(1) != ".")
            fileName += ".";
        fileName += extension[snapshotFormat];
        info.setFile(fileName);
    }
    else if (info.suffix() != extension[snapshotFormat]) {
        // Extension is not appropriate. Propose a modification
        QString modifiedName = info.absolutePath() + '/' + info.baseName() +
                               "." + extension[snapshotFormat];
        QFileInfo modifInfo(modifiedName);
        const auto choice = QMessageBox::warning(
            widget, "Wrong extension",
            info.fileName() + " has a wrong extension.\nSave as " +
                modifInfo.fileName() + " instead ?",
            QMessageBox::Yes | QMessageBox::No | QMessageBox::Cancel);
        if (choice == QMessageBox::Cancel) {
            return false;
        }

        if (choice == QMessageBox::Yes) {
            fileName = modifiedName;
            info.setFile(fileName);
        }
    }

    return true;
}

#ifndef NO_VECTORIAL_RENDER
// static void drawVectorial(void* param)
void drawVectorial(void *param) { ((QtOpenGLViewer *)param)->drawVectorial(); }

class ProgressDialog
{
public:
    static void showProgressDialog(QOpenGLWidget *parent);
    static void updateProgress(float progress, const QString &stepString);
    static void hideProgressDialog();

private:
    static QProgressDialog *progressDialog;
};

QProgressDialog *ProgressDialog::progressDialog = nullptr;

void ProgressDialog::showProgressDialog(QOpenGLWidget *parent)
{
    progressDialog = new QProgressDialog(parent);
    progressDialog->setWindowTitle("Image rendering progress");
    progressDialog->setMinimumSize(300, 40);
    progressDialog->setCancelButton(nullptr);
    progressDialog->show();
}

void ProgressDialog::updateProgress(float progress, const QString &stepString)
{
    progressDialog->setValue(int(progress * 100));
    QString message(stepString);
    if (message.length() > 33)
        message = message.left(17) + "..." + message.right(12);
    progressDialog->setLabelText(message);
    progressDialog->update();
    qApp->processEvents();
}

void ProgressDialog::hideProgressDialog()
{
    progressDialog->close();
    delete progressDialog;
    progressDialog = nullptr;
}

// FIXME: weird syntax, but fine...
class VRenderInterface : public QDialog, public Ui::VRenderInterface
{
public:
    VRenderInterface(QWidget *parent) : QDialog(parent) { setupUi(this); }
};

// Pops-up a vectorial output option dialog box and save to fileName
// Returns -1 in case of Cancel, 0 for success and (todo) error code in case of
// problem.
int saveVectorialSnapshot(const QString &fileName, QOpenGLWidget *widget,
                          const QString &snapshotFormat)
{
    static VRenderInterface *VRinterface = nullptr;

    if (!VRinterface) {
        VRinterface = new VRenderInterface(widget);
    }

    // Configure interface according to selected snapshotFormat
    if (snapshotFormat == "XFIG") {
        VRinterface->tightenBBox->setEnabled(false);
        VRinterface->colorBackground->setEnabled(false);
    }
    else {
        VRinterface->tightenBBox->setEnabled(true);
        VRinterface->colorBackground->setEnabled(true);
    }

    if (VRinterface->exec() == QDialog::Rejected)
        return -1;

    vrender::VRenderParams vparams;
    vparams.setFilename(fileName);

    if (snapshotFormat == "EPS")
        vparams.setFormat(vrender::VRenderParams::EPS);
    if (snapshotFormat == "PS")
        vparams.setFormat(vrender::VRenderParams::PS);
    if (snapshotFormat == "XFIG")
        vparams.setFormat(vrender::VRenderParams::XFIG);

    vparams.setOption(vrender::VRenderParams::CullHiddenFaces,
                      !(VRinterface->includeHidden->isChecked()));
    vparams.setOption(vrender::VRenderParams::OptimizeBackFaceCulling,
                      VRinterface->cullBackFaces->isChecked());
    vparams.setOption(vrender::VRenderParams::RenderBlackAndWhite,
                      VRinterface->blackAndWhite->isChecked());
    vparams.setOption(vrender::VRenderParams::AddBackground,
                      VRinterface->colorBackground->isChecked());
    vparams.setOption(vrender::VRenderParams::TightenBoundingBox,
                      VRinterface->tightenBBox->isChecked());

    switch (VRinterface->sortMethod->currentIndex()) {
        case 0:
            vparams.setSortMethod(vrender::VRenderParams::NoSorting);
            break;
        case 1:
            vparams.setSortMethod(vrender::VRenderParams::BSPSort);
            break;
        case 2:
            vparams.setSortMethod(vrender::VRenderParams::TopologicalSort);
            break;
        case 3:
            vparams.setSortMethod(
                vrender::VRenderParams::AdvancedTopologicalSort);
            break;
        default:
            qWarning(
                "VRenderInterface::saveVectorialSnapshot: Unknown SortMethod");
    }

    vparams.setProgressFunction(&ProgressDialog::updateProgress);
    ProgressDialog::showProgressDialog(widget);
    widget->makeCurrent();
    widget->raise();
    vrender::VectorialRender(drawVectorial, (void *)widget, vparams);
    ProgressDialog::hideProgressDialog();
    widget->setCursor(QCursor(Qt::ArrowCursor));

    // Should return vparams.error(), but this is currently not set.
    return 0;
}
#endif // NO_VECTORIAL_RENDER

class ImageInterface : public QDialog, public Ui::ImageInterface
{
public:
    ImageInterface(QWidget *parent) : QDialog(parent) { setupUi(this); }
};

// Pops-up an image settings dialog box and save to fileName.
// Returns false in case of problem.
bool QtOpenGLViewer::saveImageSnapshot(const QString &fileName)
{
    static ImageInterface *imageInterface = nullptr;
    qreal devicePixelRatio = screen()->devicePixelRatio();
    qreal dipWidth = devicePixelRatio * width();
    qreal dipHeight = devicePixelRatio * height();

    if (!imageInterface) {
        imageInterface = new ImageInterface(this);
    }

    imageInterface->imgWidth->setValue(dipWidth);
    imageInterface->imgHeight->setValue(dipHeight);

    imageInterface->imgQuality->setValue(snapshotQuality());

    if (imageInterface->exec() == QDialog::Rejected) {
        return true;
    }

    // Hide closed dialog
    qApp->processEvents();

    setSnapshotQuality(imageInterface->imgQuality->value());

    QColor previousBGColor = backgroundColor();
    if (imageInterface->whiteBackground->isChecked()) {
        setBackgroundColor(Qt::white);
    }

    QSize finalSize(imageInterface->imgWidth->value(),
                    imageInterface->imgHeight->value());

    qreal oversampling = imageInterface->oversampling->value();
    QSize subSize(int(dipWidth / oversampling), int(dipHeight / oversampling));

    qreal aspectRatio = dipWidth / static_cast<qreal>(dipHeight);
    qreal newAspectRatio =
        finalSize.width() / static_cast<qreal>(finalSize.height());

    qreal zNear = camera()->zNear();
    qreal zFar = camera()->zFar();

    qreal xMin, yMin;
    bool expand = imageInterface->expandFrustum->isChecked();
    if (camera()->type() == viewer::Camera::Type::Perspective)
        if ((expand && (newAspectRatio > aspectRatio)) ||
            (!expand && (newAspectRatio < aspectRatio))) {
            yMin = zNear * tan(camera()->fieldOfView() / 2.0);
            xMin = newAspectRatio * yMin;
        }
        else {
            xMin = zNear * tan(camera()->fieldOfView() / 2.0) * aspectRatio;
            yMin = xMin / newAspectRatio;
        }
    else {
        GLdouble width, height;
        camera()->getOrthoWidthHeight(width, height);
        xMin = qreal(width);
        yMin = qreal(height);
        if ((expand && (newAspectRatio > aspectRatio)) ||
            (!expand && (newAspectRatio < aspectRatio)))
            xMin = newAspectRatio * yMin;
        else
            yMin = xMin / newAspectRatio;
    }

    QImage image(finalSize.width(), finalSize.height(), QImage::Format_ARGB32);
    if (image.isNull()) {
        QMessageBox::warning(this, tr("Image saving error"),
                             tr("Unable to create resulting image"),
                             QMessageBox::Ok);
        return false;
    }

    // ProgressDialog disabled since it interfers with the screen grabing
    // mecanism on some platforms. Too bad.
    // ProgressDialog::showProgressDialog(this);

    qreal scaleX = subSize.width() / static_cast<qreal>(finalSize.width());
    qreal scaleY = subSize.height() / static_cast<qreal>(finalSize.height());

    qreal deltaX = 2.0 * xMin * scaleX;
    qreal deltaY = 2.0 * yMin * scaleY;

    int nbX = finalSize.width() / subSize.width();
    int nbY = finalSize.height() / subSize.height();

    // Extra subimage on the right/bottom border(s) if needed
    if (nbX * subSize.width() < finalSize.width())
        nbX++;
    if (nbY * subSize.height() < finalSize.height())
        nbY++;

    makeCurrent();

    // tileRegion_ is used by startScreenCoordinatesSystem to appropriately set
    // the local coordinate system when tiling
    tileRegion_ = new TileRegion();
    qreal tileXMin, tileWidth, tileYMin, tileHeight;
    if ((expand && (newAspectRatio > aspectRatio)) ||
        (!expand && (newAspectRatio < aspectRatio))) {
        qreal tileTotalWidth = newAspectRatio * dipHeight;
        tileXMin = (dipWidth - tileTotalWidth) / 2.0;
        tileWidth = tileTotalWidth * scaleX;
        tileYMin = 0.0;
        tileHeight = dipHeight * scaleY;
        tileRegion_->textScale = 1.0 / scaleY;
    }
    else {
        qreal tileTotalHeight = dipWidth / newAspectRatio;
        tileYMin = (dipHeight - tileTotalHeight) / 2.0;
        tileHeight = tileTotalHeight * scaleY;
        tileXMin = 0.0;
        tileWidth = dipWidth * scaleX;
        tileRegion_->textScale = 1.0 / scaleX;
    }

    int count = 0;
    for (int i = 0; i < nbX; i++)
        for (int j = 0; j < nbY; j++) {
            preDraw();

            // Change projection matrix
            glMatrixMode(GL_PROJECTION);
            glLoadIdentity();
            if (camera()->type() == viewer::Camera::Type::Perspective)
                glFrustum(-xMin + i * deltaX, -xMin + (i + 1) * deltaX,
                          yMin - (j + 1) * deltaY, yMin - j * deltaY, zNear,
                          zFar);
            else
                glOrtho(-xMin + i * deltaX, -xMin + (i + 1) * deltaX,
                        yMin - (j + 1) * deltaY, yMin - j * deltaY, zNear,
                        zFar);
            glMatrixMode(GL_MODELVIEW);

            tileRegion_->xMin = tileXMin + i * tileWidth;
            tileRegion_->xMax = tileXMin + (i + 1) * tileWidth;
            tileRegion_->yMin = tileYMin + j * tileHeight;
            tileRegion_->yMax = tileYMin + (j + 1) * tileHeight;

            draw();
            postDraw();

            // ProgressDialog::hideProgressDialog();
            // qApp->processEvents();

            QImage snapshot = QOpenGLWidget::grabFramebuffer();

            // ProgressDialog::showProgressDialog(this);
            // ProgressDialog::updateProgress(count / (qreal)(nbX*nbY),
            // "Generating image
            // ["+QString::number(count)+"/"+QString::number(nbX*nbY)+"]");
            // qApp->processEvents();

            QImage subImage = snapshot.scaled(subSize, Qt::IgnoreAspectRatio,
                                              Qt::SmoothTransformation);

            // Copy subImage in image
            for (int ii = 0; ii < subSize.width(); ii++) {
                int fi = i * subSize.width() + ii;
                if (fi == image.width())
                    break;

                for (int jj = 0; jj < subSize.height(); jj++) {
                    int fj = j * subSize.height() + jj;
                    if (fj == image.height())
                        break;

                    image.setPixel(fi, fj, subImage.pixel(ii, jj));
                }
            }
            count++;
        }

    bool saveOK = image.save(fileName, snapshotFormat().toLatin1().constData(),
                             snapshotQuality());

    // ProgressDialog::hideProgressDialog();
    // setCursor(QCursor(Qt::ArrowCursor));

    delete tileRegion_;
    tileRegion_ = nullptr;

    if (imageInterface->whiteBackground->isChecked())
        setBackgroundColor(previousBGColor);

    return saveOK;
}

void QtOpenGLViewer::saveSnapshot(bool automatic, bool overwrite)
{
    // Ask for file name
    if (snapshotFileName().isEmpty() || !automatic) {
        QString selectedFormat = FDFormatString[snapshotFormat()];
        auto fileName = QFileDialog::getSaveFileName(
            this, "Choose a file name to save under", snapshotFileName(),
            formats, &selectedFormat,
            overwrite ? QFileDialog::DontConfirmOverwrite
                      : QFlags<QFileDialog::Option>(0));
        setSnapshotFormat(Qtformat[selectedFormat]);

        if (!checkFileName(fileName, this, snapshotFormat())) {
            return;
        }

        setSnapshotFileName(fileName);
    }

    QFileInfo fileInfo(snapshotFileName());

    if (automatic && (snapshotCounter() >= 0)) {
        // In automatic mode, names have a number appended
        const QString baseName = fileInfo.baseName();
        auto count = QString("%1").arg(snapshotCounter_++, 4, 10, QChar('0'));
        auto suffix = fileInfo.suffix();
        if (suffix.isEmpty()) {
            suffix = extension[snapshotFormat()];
        }
        fileInfo.setFile(fileInfo.absolutePath() + '/' + baseName + '-' +
                         count + '.' + suffix);

        if (!overwrite) {
            while (fileInfo.exists()) {
                count =
                    QString("%1").arg(snapshotCounter_++, 4, 10, QChar('0'));
                fileInfo.setFile(fileInfo.absolutePath() + '/' + baseName +
                                 '-' + count + '.' + fileInfo.suffix());
            }
        }
    }

    bool saveOK;
#ifndef NO_VECTORIAL_RENDER
    if ((snapshotFormat() == "EPS") || (snapshotFormat() == "PS") ||
        (snapshotFormat() == "XFIG"))
        // Vectorial snapshot. -1 means cancel, 0 is ok, >0 (should be) an error
        saveOK = (saveVectorialSnapshot(fileInfo.filePath(), this,
                                        snapshotFormat()) <= 0);
    else
#endif
        if (automatic) {
        QImage snapshot = frameBufferSnapshot();
        saveOK = snapshot.save(fileInfo.filePath(),
                               snapshotFormat().toLatin1().constData(),
                               snapshotQuality());
    }
    else {
        saveOK = saveImageSnapshot(fileInfo.filePath());
    }

    if (!saveOK) {
        QMessageBox::warning(
            this, tr("Snapshot problem"),
            "Unable to save snapshot in\n" + fileInfo.filePath());
    }
}

QImage QtOpenGLViewer::frameBufferSnapshot()
{
    // Viewer must be on top of other windows.
    makeCurrent();
    raise();
    // Hack: Qt has problems if the frame buffer is grabbed after QFileDialog is
    // displayed. We grab the frame buffer before, even if it might be not
    // necessary (vectorial rendering). The problem could not be reproduced on a
    // simple example to submit a Qt bug. However, only grabs the
    // backgroundImage in the eponym example. May come from the driver.
    return QOpenGLWidget::grabFramebuffer();
}

void QtOpenGLViewer::saveSnapshot(const QString &fileName, bool overwrite)
{
    const QString previousName = snapshotFileName();
    const int previousCounter = snapshotCounter();
    setSnapshotFileName(fileName);
    setSnapshotCounter(-1);
    saveSnapshot(true, overwrite);
    setSnapshotFileName(previousName);
    setSnapshotCounter(previousCounter);
}

void QtOpenGLViewer::snapshotToClipboard()
{
    QClipboard *cb = QApplication::clipboard();
    cb->setImage(frameBufferSnapshot());
}
