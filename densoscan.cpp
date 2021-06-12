#include <QFile>
#include <QSettings>
#include <QMessageBox>
#include <QStandardPaths>
#include <QFileDialog>
#include <QPainter>
#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#include <sys/stat.h>
#include <fmt/format.h>
#include <png.h>

using namespace cv;
using namespace std;

#include "spline.h"
#include "densoscan.h"
#include "ui_densoscan.h"

#define LMAX        65535.0
#define LUT_SIZE    (LMAX + 1)
typedef unsigned short pixel;

#define TYPE_FRAME 1
#define TYPE_SPACE 0

#define OPT_UNAVAILABLE "Unavailable"

inline QImage  cvMatToQImage( const cv::Mat &inMat )
{
   switch ( inMat.type() )
   {
      // 8-bit, 4 channel
      case CV_8UC4:
      {
         QImage image( inMat.data,
                       inMat.cols, inMat.rows,
                       static_cast<int>(inMat.step),
                       QImage::Format_ARGB32 );

         return image;
      }

      // 8-bit, 3 channel
      case CV_8UC3:
      {
         QImage image( inMat.data,
                       inMat.cols, inMat.rows,
                       static_cast<int>(inMat.step),
                       QImage::Format_RGB888 );

         return image.rgbSwapped();
      }

      // 8-bit, 1 channel
      case CV_8UC1:
      {
         QImage image( inMat.data,
                       inMat.cols, inMat.rows,
                       static_cast<int>(inMat.step),
                       QImage::Format_Grayscale8 );
         return image;
      }

       // 8-bit, 1 channel
       case CV_16UC1:
       {
          QImage image( inMat.data,
                        inMat.cols, inMat.rows,
                        static_cast<int>(inMat.step),
                        QImage::Format_Grayscale16 );
          return image;
       }

      default:
         //qWarning() << "ASM::cvMatToQImage() - cv::Mat image type not handled in switch:" << inMat.type();
         break;
   }

   return QImage();
}

DensoScan* DensoScan::instance = nullptr;

DensoScan::DensoScan(QWidget *parent)
    : QMainWindow(parent)
    , ui(new Ui::DensoScan)
{
    instance = this;

    ui->setupUi(this);

    QSettings settingsProfiles ("denso", "profiles" );
    QFile qFile  ( settingsProfiles.fileName() );
    qFile.open(QIODevice::ReadOnly);
    QByteArray sj = qFile.readAll();
    QJsonDocument doc = QJsonDocument::fromJson( sj );
    profiles.fromJson( doc.object() );
    qFile.close();

    loadProfiles ();
    on_buttonRefresh_clicked();

    QSettings settings("denso", "scan" );
    settings.beginGroup("main");
    ui->comboDevice->setCurrentText( settings.value("device").toString() );
    ui->rollName->setText( settings.value("rollName").toString() );
    ui->folderName->setText( settings.value("folderName").toString() );
    ui->startingIndex->setCurrentText( settings.value("startingIndex").toString() );
    settings.endGroup();

    scanner.setDebug(true);
}

DensoScan::~DensoScan()
{
    delete ui;
}

void DensoScan::loadDeviceSettings()
{
    QSettings settings("denso", "scan" );
    settings.beginGroup( ui->comboDevice->currentText() );
    if ( ui->comboDPI->isEnabled() )
        ui->comboDPI->setCurrentText( settings.value("DPI").toString() );
    if ( ui->comboType->isEnabled() )
        ui->comboType->setCurrentText( settings.value("type").toString() );
    if ( ui->comboProfile->isEnabled() )
        ui->comboProfile->setCurrentText( settings.value("profile").toString() );
    if ( ui->outputType->isEnabled() )
        ui->outputType->setCurrentIndex( settings.value("outputType").toInt() );

    settings.endGroup();
}

void DensoScan::saveDeviceSettings()
{
    QSettings settings("denso", "scan" );
    settings.beginGroup( ui->comboDevice->currentText() );
    if ( ui->comboDPI->isEnabled() )
        settings.setValue( "DPI", ui->comboDPI->currentText() );
    if ( ui->comboType->isEnabled() )
        settings.setValue( "type", ui->comboType->currentText() );
    if ( ui->comboProfile->isEnabled() )
        settings.setValue( "profile", ui->comboProfile->currentText() );
    if ( ui->outputType->isEnabled() )
        settings.setValue( "outputType", ui->outputType->currentIndex() );
    settings.endGroup();
}

void DensoScan::loadProfiles()
{
    ui->comboProfile->clear();
    ui->comboProfile->addItem( "Image raw data" /* , const QVariant &userData = QVariant()) */ );

    for ( int i = 0; i < profiles.size(); i++ )
    {
        QProfile *p = (QProfile *)profiles.at(i);
        ui->comboProfile->addItem( p->name ); // "Negative (Stouffer T2115)" /* , const QVariant &userData = QVariant()) */ );
    }
}

void DensoScan::on_buttonRefresh_clicked()
{
    scanner.close();

    QString current = ui->comboDevice->currentText();
    ui->comboDevice->clear();

    std::vector <ScannerDevice> devices = getDevices ();

    for ( unsigned i = 0; i < devices.size(); i++ )
        ui->comboDevice->addItem( QString ( ( devices[i].vendor + " " + devices[i].model ).c_str() ), QString ( devices[i].name.c_str() ) );

    ui->comboDevice->setCurrentText( current );
}


void DensoScan::on_comboDevice_currentIndexChanged(int index)
{
    if ( index == -1 )
        return;

    QString name = ui->comboDevice->currentData().toString();
    scanner.open( name.toStdString().c_str() );

    updateDeviceOptions();
    drawLogo();
}

void DensoScan::updateDeviceOptions()
{
    std::vector<std::string> types = scanner.getFilmTypes ( );
    if ( types.size () )
    {
        QString current = ui->comboType->currentText();
        ui->comboType->clear();
        for ( unsigned i = 0; i < types.size(); i++ )
            ui->comboType->addItem( QString ( types[i].c_str() ) );

        ui->comboType->setCurrentText( current );
        ui->comboType->setEnabled(true);
    }
    else
    {
        ui->comboType->clear();
        ui->comboType->addItem( OPT_UNAVAILABLE );
        ui->comboType->setCurrentText( OPT_UNAVAILABLE );
        ui->comboType->setEnabled(false);
    }

   std::vector<int> DPIs = scanner.getDPIs ( );
   if ( DPIs.size() )
   {
        QString current = ui->comboDPI->currentText();
        ui->comboDPI->clear();
        for ( unsigned i = 0; i < DPIs.size(); i++ )
            ui->comboDPI->addItem( QString::number( DPIs[i] ) );

        ui->comboDPI->setCurrentText( current );
        ui->comboDPI->setEnabled(true);
    }
    else
    {
        ui->comboDPI->clear();
        ui->comboDPI->addItem( OPT_UNAVAILABLE );
        ui->comboDPI->setCurrentText( OPT_UNAVAILABLE );
        ui->comboDPI->setEnabled(false);
    }

   loadDeviceSettings();
}

void DensoScan::drawLogo()
{
    QImage logo ( QString ( ":/icon/icon.png" ) );
    Size size = scanner.getPreviewSize();
    if ( size.height > 256 )
    {
        if ( size.width > 1000 )
        {
            double scale = 1000.0 / size.width;
            size.width *= scale;
            size.height *= scale;
        }

        QImage preview ( size.height, size.width, QImage::Format_ARGB32 );
        QPainter painter(&preview);
        painter.fillRect( 0, 0, size.height, size.width, QColor( 0, 0, 0 ));

        QImage scaled = logo.scaledToHeight( std::min( size.width, size.height ) * .90, Qt::SmoothTransformation );
        QRect target ( ( size.height - scaled.size().width()) / 2, ( size.width - scaled.size().height()) / 2, scaled.size().width(), scaled.size().height() );

        painter.drawImage( target, scaled );

        QPixmap px = QPixmap::fromImage ( preview );
        ui->preview->setPixmap(px);
    }
}

void DensoScan::drawChart()
{
    return;
    int idx = ui->comboProfile->currentIndex();

    std::vector<double> expected;
    std::vector<double> measured;

    if ( idx )
    {
        expected = profiles.at(idx-1)->expected;
        measured = profiles.at(idx-1)->measured;
    }
    else
        for ( unsigned i = 0; i  <= 100; i++ )
        {
            measured.push_back(i);
            expected.push_back(i);
        }

    QLineSeries *seriesExpected = new QLineSeries();
    QLineSeries *seriesMeasured = new QLineSeries();
    QLineSeries *seriesExpectedD = new QLineSeries();
    QLineSeries *seriesMeasuredD = new QLineSeries();

    seriesExpected->setName( "Expected L" );
    seriesMeasured->setName( "Measured L" );
    seriesExpectedD->setName( "Expected D" );
    seriesMeasuredD->setName( "Measured D" );

    QValueAxis *axisX = new QValueAxis;
    axisX->setTickCount( expected.size() );
    axisX->setLabelFormat("%d");
    axisX->setRange( 1, expected.size() );

    QValueAxis *axisY = new QValueAxis;
    axisY->setTickCount( 11 );
    axisY->setLabelFormat("%d%%");
    axisY->setRange( 0, 100 );

    QValueAxis *axisYD = new QValueAxis;
    axisYD->setTickCount( 11 );
    axisYD->setMinorTickCount ( 1 );
    axisYD->setLabelFormat("%.2f");
    axisYD->setMax( 4 );

    QTableWidgetItem *newItem;
    for ( unsigned int i = 0; i < expected.size(); i++)
    {
        seriesExpected->append(i + 1, expected.at(i) );
        seriesExpectedD->append(i + 1, log10 ( 100 / expected.at(i) ) );
    }

    for ( unsigned int i = 0; i < measured.size(); i++)
    {
        seriesMeasured->append(i + 1, measured.at(i) );
        seriesMeasuredD->append(i + 1, log10 ( 100 / measured.at(i) ) );
    }

    QChart *chart = new QChart();
    //chart->setTheme(QChart::QChart::ChartThemeBlueIcy);
    //chart->legend()->hide();

    chart->addAxis(axisYD, Qt::AlignRight);
    chart->addAxis(axisY, Qt::AlignLeft);
    chart->addAxis(axisX, Qt::AlignBottom);

    chart->addSeries(seriesExpected);
    chart->addSeries(seriesMeasured);
    chart->addSeries(seriesExpectedD);
    chart->addSeries(seriesMeasuredD);

    seriesExpected->attachAxis(axisX);
    seriesExpected->attachAxis(axisY);
    seriesMeasured->attachAxis(axisX);
    seriesMeasured->attachAxis(axisY);
    seriesExpectedD->attachAxis(axisX);
    seriesExpectedD->attachAxis(axisYD);
    seriesMeasuredD->attachAxis(axisX);
    seriesMeasuredD->attachAxis(axisYD);

    //if (chartView)
        //ui->chartLayout->removeWidget( chartView );
//    chartView = new QChartView(chart, ui->label_10);
    chartView->setRenderHint(QPainter::Antialiasing);
    //ui->chartLayout->addWidget(chartView);
}

void DensoScan::enableOptions ( bool enabled  )
{
    ui->outputType->setEnabled(enabled);
    ui->comboDevice->setEnabled(enabled);
    ui->buttonRefresh->setEnabled(enabled);
    ui->brightness->setEnabled(enabled);
    ui->comboProfile->setEnabled(enabled);
    ui->folderName->setEnabled(enabled);
    ui->toolSelectFolder->setEnabled(enabled);
    ui->rollName->setEnabled(enabled);
    ui->comboType->setEnabled(enabled);
    ui->comboDPI->setEnabled(enabled);
    ui->startingIndex->setEnabled(enabled);

    ui->pushScan->setEnabled( enabled );
    ui->pushCancel->setEnabled( !enabled );

    if ( enabled )
    {
        updateDeviceOptions();
        on_outputType_currentIndexChanged( ui->outputType->currentIndex() );
    }
}

class PNGWriter
{
    FILE *fp = nullptr;
    png_structp png_ptr;
    png_infop info_ptr;

public:
    PNGWriter ( const char *filename, int w, int h, int dpi, int depth, int color_type )
    {
        fp = fopen( filename, "wb");
        png_ptr = png_create_write_struct(PNG_LIBPNG_VER_STRING, NULL, NULL, NULL);
        info_ptr = png_create_info_struct(png_ptr);
        setjmp(png_jmpbuf(png_ptr));
        png_init_io(png_ptr, fp);
        png_set_IHDR(png_ptr, info_ptr, w, h,
                     depth, color_type, PNG_INTERLACE_NONE,
                     PNG_COMPRESSION_TYPE_BASE, PNG_FILTER_TYPE_BASE);

        /* Set the physical pixel size (resolution, given in DPI). */
        if (dpi > 0)
        {
            /* one metre = 100 centimetre (cm), and 2.54 cm = 1 inch */
            /* 1 metre is about 40 inches (well, 100/2.54 or 39.37) */
            /* so the number of dots per metre is about 40 times */
            /* larger than the number of dots per inch */
            /* thus DPM = DPI * 100 / 2.54 = DPI * 10000 / 254 */
            int ppm_x, ppm_y; /* pixels per metre */
            ppm_x = (dpi * 39.37) + 1; /* round to nearest */
            ppm_y = ppm_x;
            png_set_pHYs(png_ptr, info_ptr, ppm_x, ppm_y,
                         PNG_RESOLUTION_UNKNOWN);
        }

        png_text title_text;
        title_text.compression = PNG_TEXT_COMPRESSION_NONE;
        title_text.key = "Title";
        title_text.text = "My title";
        png_set_text(png_ptr, info_ptr, &title_text, 1);
        png_write_info(png_ptr, info_ptr);
    }

    ~PNGWriter ()
    {
        close();
    }

    void write ( const unsigned char *buffer )
    {
        png_write_row( png_ptr, buffer );
    }

    void close ()
    {
        if ( fp )
        {
            png_write_end( png_ptr, info_ptr );
            fclose ( fp );
            fp = nullptr;
        }
    }
};

int prv = 0;

void DensoScan::on_pushScan_clicked()
{
/*    Scan preview = imread( "preview-test.png", IMREAD_GRAYSCALE );
    preview.ppmm = 150 / 25.4; // 5.9;// 11.8110;
    onPreviewCompleted( preview ); // */
//scanner.dumpopts ();
    QSettings settings("denso", "scan" );
    settings.beginGroup("main");
    settings.setValue( "device", ui->comboDevice->currentText() );
    settings.setValue( "rollName", ui->rollName->text() );
    settings.setValue( "folderName", ui->folderName->text() );
    settings.setValue( "startingIndex", ui->startingIndex->currentText() );
    settings.endGroup();
    saveDeviceSettings();

    enableOptions ( false );
    drawLogo();

    scanner.preview();
}


void DensoScan::on_pushCancel_clicked()
{
    scanner.cancel();
    ui->pushCancel->setEnabled(false);
    onProgressUpdate( "Aborting...", 100 );
}

void DensoScan::onScanCompleted ()
{
    enableOptions( true );
}

void DensoScan::onError ( const std::string &error )
{
    QMessageBox messageBox;
    messageBox.critical( 0, "Error", error.c_str() );
    messageBox.show();
    scanner.cancel();
    enableOptions( true );
    //messageBox.setFixedSize(500,200);
}

bool file_exist (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void DensoScan::onPreviewCompleted ( const Scan &preview )
{
    if ( preview.empty() )
    {
        ui->pushCancel->setEnabled( false );
        ui->pushScan->setEnabled( true );
        return;
    }

    imwrite("preview.png", preview);
    double ppmm = preview.ppmm;

    cv::Mat output;
    cv::cvtColor( preview, output, cv::COLOR_GRAY2BGR);
    cv::Scalar agfa_red ( 58, 66, 249 );

    rotate ( output, output, ROTATE_90_COUNTERCLOCKWISE);

    vector<Slot> holders = scanner.guessSlots ( preview );
    vector<Rect2d> frames = scanner.guessFrames ( preview, holders );

    filenames.clear();
    int offset = 0;

    QString start = ui->startingIndex->currentText();
    if ( start == "XX" )
        offset = -3;
    else if ( start == "X" )
        offset = -2;
    else if ( start == "00" )
        offset = -1;
    else
        offset = start.toInt();

    std::string name = (ui->folderName->text() + "/" + ui->rollName->text()).toStdString();

    auto framenumber = [] ( int offset )
    {
        if ( offset == -3 )
            return std::string("XX");
        if ( offset == -2 )
            return std::string("X");
        if ( offset == -1 )
            return std::string("00");

        return fmt::format ( "{}", offset );
    };

    double fontSize = ppmm / 2;
    int fontTic = round ( ppmm / 2);
    int rectTic = round ( ppmm / 3);
    for ( unsigned int f = 0; f < frames.size(); f++ )
    {
        while ( file_exist( fmt::format ( "{} ({}).png", name, framenumber(offset) ) ) )
            offset++;

        filenames.push_back( fmt::format ( "{} ({}).png", name, framenumber(offset) ) );

        Rect rect_frame ( frames[f].y * ppmm, output.size().height - ( frames[f].x + frames[f].width ) * ppmm, frames[f].height * ppmm, frames[f].width * ppmm );

        cv::rectangle( output, rect_frame, agfa_red, rectTic );
        int baseline = 0;
        Size ts = getTextSize( framenumber(offset), cv::FONT_HERSHEY_DUPLEX, fontSize, fontTic, &baseline  );
        Point point ( ( rect_frame.br() + rect_frame.tl()) * 0.5 );
        point.x -= ts.width / 2;
        point.y += ts.height / 2;
        putText(output, framenumber(offset), point,
            cv::FONT_HERSHEY_DUPLEX, fontSize, agfa_red, fontTic, cv::LINE_AA );
        offset++;
    }

    imwrite ( "preview-output.png", output );

    QImage imDraw = cvMatToQImage( output );
    QPixmap px = QPixmap::fromImage ( imDraw );
    ui->preview->setPixmap ( px );

    Scanner::OutputMode mode = (Scanner::OutputMode)(ui->outputType->currentIndex());
    string filmType = ui->comboType->currentText().toStdString();
    int DPI = ui->comboDPI->currentText().toInt();
    int profile = ui->comboProfile->currentIndex() - 1;
    int brightness = ui->brightness->value();

    if ( profile > 0 )
        scanner.setProfile( *(profiles.at(profile)) );
    else
        scanner.setProfile( );

    scanner.setMode( mode );
    scanner.setFilmType( filmType );
    scanner.setDPI( DPI );
    scanner.setBrightness( brightness );

    scanner.scan ( frames ); // */
}

void DensoScan::onNewScan ( Scan &image, int frameNumber )
{
    imwrite ( filenames[frameNumber], image );
}

void DensoScan::customEvent(QEvent * event)
{
    if(event->type() == EVENT_SCAN_COMPLETED )
    {
        onScanCompleted ( );
    }
    if(event->type() == EVENT_PREVIEW_COMPLETED )
    {
        onPreviewCompleted ( static_cast<PreviewCompletedEvent *>(event)->preview );
    }
    else if(event->type() == EVENT_PROGRESS_UPDATE )
    {
        onProgressUpdate ( static_cast<ProgressUpdateEvent *>(event)->text, static_cast<ProgressUpdateEvent *>(event)->percent );
    }
    else if(event->type() == EVENT_ERROR )
    {
        onError ( static_cast<ErrorEvent *>(event)->error );
    }
    else if(event->type() == EVENT_NEW_SCAN )
    {
        onNewScan ( static_cast<NewScanEvent *>(event)->image, static_cast<NewScanEvent *>(event)->frameNumber );
    }
}

void DensoScan::onProgressUpdate ( const std::string text, int percent )
{
    ui->progressBar->setFormat( (text + " %p%").c_str() );
    ui->progressBar->setValue(percent);
}

void DScanner::onNewScan ( Scan &image, int frameNumber )
{
    QCoreApplication::postEvent( DensoScan::getInstance(), new NewScanEvent( image, frameNumber ));
}

void DScanner::onScanCompleted ()
{
    QCoreApplication::postEvent( DensoScan::getInstance(), new ScanCompletedEvent());
}

void DScanner::onPreviewCompleted ( const Scan &preview )
{
    QCoreApplication::postEvent( DensoScan::getInstance(), new PreviewCompletedEvent( preview ));
}

void DScanner::onProgressUpdate ( const std::string text, int percent )
{
    QCoreApplication::postEvent( DensoScan::getInstance(), new ProgressUpdateEvent( text, percent ));
}

void DScanner::onError ( const std::string error )
{
    QCoreApplication::postEvent( DensoScan::getInstance(), new ErrorEvent( error ));
}

void DensoScan::on_toolSelectFolder_clicked()
{
    QString folder = ui->folderName->text();
    if ( folder == "" || !file_exist(folder.toStdString()) )
        folder = QStandardPaths::writableLocation(QStandardPaths::PicturesLocation);

    ui->folderName->setText( QFileDialog::getExistingDirectory(this, tr("Open Directory"),
                                                 folder,
                                                 QFileDialog::ShowDirsOnly
                                                 | QFileDialog::DontResolveSymlinks) );
}


void DensoScan::on_outputType_currentIndexChanged(int index)
{
    if ( index == 1 )
    {
        QSettings settings("denso", "scan" );
        settings.beginGroup( ui->comboDevice->currentText() );
        settings.setValue( "profile", ui->comboProfile->currentText() );

        ui->comboProfile->setEnabled(false);
        ui->comboProfile->clear();
        ui->comboProfile->addItem( OPT_UNAVAILABLE );
        ui->comboProfile->setCurrentText( OPT_UNAVAILABLE );
    }
    else
    {
        ui->comboProfile->setEnabled(true);

        loadProfiles();
        QSettings settings("denso", "scan" );
        settings.beginGroup( ui->comboDevice->currentText() );
        ui->comboProfile->setCurrentText( settings.value("profile").toString() );
    }
}


void DensoScan::on_comboProfile_currentIndexChanged(int index)
{
    drawChart();
}

