#include <QFile>
#include <QSettings>
#include <QMessageBox>

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

double get_mode ( vector<double> size, double approx )
{
    std::sort(size.begin(), size.end(), std::less<double>());

    double number = size[0];
    double mode = number;
    int count = 1;
    int countMode = 1;

    for (unsigned int i = 1; i < size.size(); i++)
    {
          if ( size[i] <= number + approx && size[i] >= number - approx )
          { // count occurrences of the current number
             ++count;
            //Average += (NewValue - Average) / NewSampleCount;
             number += ( size[i] - number ) / count;
          }
          else
          { // now this is a different number
                if (count > countMode)
                {
                      countMode = count; // mode is the biggest ocurrences

                      mode = number;
                }
               count = 1; // reset count for the new number
               number = size[i];
      }
    }

    return mode;
}

vector<Rect> Frames ( int nb, cv::Mat input, double ppmm = 5.9 )
{
//    cv::Mat input = cv::imread("/home/andrea/Sviluppo/build-boxes-Desktop-Debug/test-0039.png");
//    cv::Mat input = cv::imread("/home/andrea/Sviluppo/build-boxes-Desktop-Debug/test gamma 1.0007 (copia).png");
    cv::Mat output;
    cv::cvtColor( input, output, cv::COLOR_GRAY2BGR);
    Mat holder = input.clone();

    threshold ( input, input, 250, 255, cv::THRESH_TOZERO_INV );

    Mat blurred;
    GaussianBlur( input, blurred, Size ( 17, 17 ), 0, 0 );
    double min, max;
    minMaxIdx( blurred, &min, &max );
    input -= min;
    input *= ( 255 / (max - min) );

    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, 3 ) * 255.0);
    Mat res = input.clone();
    LUT( res, lookUpTable, input);

    imwrite ( fmt::format ( "frames-gray-{}.png", nb ), input );

    int threshold = 127;
    cv::Mat mask = input > threshold;

    imwrite ( fmt::format ( "frames-mask-{}.png", nb ), mask );

    int w = input.size().width;
    int h = input.size().height;

    bool in = 0;
    int gap_begin = 0;
    int gap_end = 0;

    int mingap = 1 * ppmm;
    int minframe = 10 * ppmm;
    int treshold = 230;
    int v;

    vector<Rect> frames;

    vector <int> begins;
    vector <int> ends;

    for ( int y = 0; y < h; y++ )
    {
        double sum = 0;
        for ( int x = 0; x < w; x ++ )
        {
            v = mask.at<unsigned char>( y, x );
            sum += v;
        }

        double avg = sum / w;

        if ( avg > treshold || y == ( h - 1 ) )
        {
            if ( !in )
            {
                double size = ( y - gap_end )  / ppmm;

                gap_begin = y;
                in = true;

                if ( ( size <= 37 && size >= 34 ) )
                {
                    cv::rectangle( output, Point ( 0, gap_end + 1 ), Point ( w, gap_begin - 1 ), cv::Scalar( 255, 0, 0 ), 4 );

                    begins.push_back( gap_end );
                    ends.push_back( gap_begin );
                }
                else
                {
                    cv::rectangle( output, Point ( 0, gap_end + 1 ), Point ( w, gap_begin - 1 ), cv::Scalar( 0, 0, 255 ), 1 );
                }
            }
        }
        else
        {
            if ( in )
            {
                double size = ( y - gap_begin ) / ppmm;

                if ( size > 1.5 )
                {
                    in = false;
                    gap_end = y;
//                    cv::rectangle( output, Point ( 0, begin ), Point ( w, y ), cv::Scalar( 0, 255, 0 ), 1 );
                }
            }
        }
    }

    int prev = 0;
    int framesize = 36 * ppmm;
    int framedistance = 38 * ppmm;
    unsigned int framemax = 6 - 1;
    int size;
    unsigned int i = 0;


    while ( i < begins.size() )
    {
        prev = 0;
        for ( i = 0; i < begins.size(); i++ )
        {
            size = ends [ i ] - begins [ i ];

            if ( ( begins[i] - prev ) > framedistance )
            {
                begins.insert( begins.begin() + i, begins[i] - framedistance );
                ends.insert( ends.begin() + i, begins[i] + framesize );
                break;
            }

            if ( size != framesize )
            {
                if ( !i )
                    begins[i] += size - framesize;
                else if ( i == framemax )
                    ends[i] -= size - framesize;
                else
                {
                    begins[i] += ( size - framesize ) / 2;
                    ends[i] -= ( size - framesize ) / 2;
                }

                if ( ends[i] > h )
                    ends[i] = h;
            }

            if ( i == ( begins.size() - 1 ) && ( ( h - ends[i] ) > framedistance ) )
            {
                begins.push_back ( begins[i] + framedistance );
                ends.push_back ( begins[i] + framedistance + framesize );
                break;
            }

            prev = ends[i];
        }
    }

    for ( i = 0; i < begins.size(); i++ )
    {
        Rect box;
        box.x = 0;
        box.y = begins[i];
        box.width = w;
        box.height = ends[i] - begins[i];

        double m = mean ( holder(box) )[0];
        if ( m < 240 )
        {
            frames.push_back(box);
            cv::rectangle( output, box, cv::Scalar( 0, 255, 0 ), 1 );
        }
    }

    imwrite ( fmt::format ( "frames-out-{}.png", nb ), output );
//    double framemean = get_mode ( framesize, 0.5 );
//    double spacemean = get_mode ( spacesize, 0.1 );
    return frames;
}

vector <Rect> FramesOld ( cv::Mat input, double ppmm = 5.9 )
{
//    cv::Mat input = cv::imread("/home/andrea/Sviluppo/build-boxes-Desktop-Debug/test-0039.png");
//    cv::Mat input = cv::imread("/home/andrea/Sviluppo/build-boxes-Desktop-Debug/test gamma 1.0007 (copia).png");

    cv::Mat output;
    cv::cvtColor( input, output, cv::COLOR_GRAY2BGR);

    Mat blurred;

    GaussianBlur( input, blurred, Size ( 17, 17 ), 0, 0 );
    double min, max;
    minMaxIdx( blurred, &min, &max );
    input -= min;
    input *= ( 255 / (max - min) );

    Mat lookUpTable(1, 256, CV_8U);
    uchar* p = lookUpTable.ptr();
    for( int i = 0; i < 256; ++i)
        p[i] = saturate_cast<uchar>(pow(i / 255.0, 3 ) * 255.0);
    Mat res = input.clone();
    LUT( res, lookUpTable, input);

    imwrite ( "frames-gray.png", input );

    int threshold = 220;
    cv::Mat mask = input > threshold;

    imwrite ( "frames-mask.png", mask );

    int w = input.size().width;
    int h = input.size().height;

    bool in = 0;
    int begin = 0;
    int end = 0;

    int mingap = 1 * ppmm;
    int minframe = 10 * ppmm;
    int treshold = 240;
    int v;

    vector <Rect> frames;

    vector <double> framesize;
    vector <double> spacesize;

    for ( int y = 0; y < h; y++ )
    {
        double sum = 0;
        for ( int x = 0; x < w; x ++ )
        {
            v = mask.at<unsigned char>( y, x );
            sum += v;
        }

        double avg = sum / w;

//        cout << avg << endl;

        if ( avg > treshold || y == ( h - 1 ) )
        {
            if ( !in )
            {
                if ( ( y - end) > minframe || y == ( h - 1 ) )
                {
                    begin = y;
                    in = true;

                    double size = ( y - end )  / ppmm;
                    if ( size >= 35.5 &&  size <= 35.5 )
                    {
                        framesize.push_back( size );

                        cv::rectangle( output, Point ( 0, end + 1 ), Point ( w, y - 1 ), cv::Scalar( 0, 0,255 ), 1 );
                        cout << "frame: " << size << endl;

                        Rect box;
                        box.x = 0;
                        box.y = end;
                        box.width = w;
                        box.height = y - end;
                        frames.push_back(box);
                    }
                }
            }
        }
        else
        {
            if ( in )
            {
                if ( ( y - begin) > mingap )
                {
                    in = false;
                    end = y;

                    double size = ( y - begin ) / ppmm;
                    spacesize.push_back( size );

                    cout << "Space: " << size << endl;

                    cv::rectangle( output, Point ( 0, begin ), Point ( w, y ), cv::Scalar( 0, 255, 0 ), 1 );
                }
            }
        }
    }

    imwrite ( "frames-out.png", output );

//    for ( unsigned int i = 0; i < beginat.size(); i++ )
//    {
//        if ( type.at(i) == TYPE_FRAME && size.at(i) < minframesize )
//    }

    double framemean = get_mode ( framesize, 0.5 );
    double spacemean = get_mode ( spacesize, 0.1 );
    return frames;
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

    ui->comboProfile->addItem( "Image raw data" /* , const QVariant &userData = QVariant()) */ );

    for ( int i = 0; i < profiles.size(); i++ )
    {
        QProfile *p = (QProfile *)profiles.at(i);
        ui->comboProfile->addItem( p->name ); // "Negative (Stouffer T2115)" /* , const QVariant &userData = QVariant()) */ );
    }

    on_buttonRefresh_clicked();
}

DensoScan::~DensoScan()
{
    delete ui;
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

    if ( scanner.optExist ( SCAN_FILM_TYPE ) )
    {
        std::vector<std::string> domain = scanner.getDomainS ( SCAN_FILM_TYPE );
        QString current = ui->comboType->currentText();
        ui->comboType->clear();
        for ( unsigned i = 0; i < domain.size(); i++ )
            ui->comboType->addItem( QString ( domain[i].c_str() ) );

        ui->comboType->setCurrentText( current );
        ui->comboType->setEnabled(true);
    }
    else
    {
        ui->comboType->clear();
        ui->comboType->setEnabled(true);
    }

    if ( scanner.optExist ( SCAN_DPI ) )
    {
        std::vector<int> domain = scanner.getDomainI ( SCAN_DPI );
        QString current = ui->comboDPI->currentText();
        ui->comboDPI->clear();
        for ( unsigned i = 0; i < domain.size(); i++ )
            ui->comboDPI->addItem( QString::number( domain[i] ) );

        ui->comboDPI->setCurrentText( current );
        ui->comboDPI->setEnabled(true);
    }
    else
    {
        ui->comboDPI->clear();
        ui->comboDPI->setEnabled(true);
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

//bool file_exist (const std::string& name) {
//  struct stat buffer;
//  return (stat (name.c_str(), &buffer) == 0);
//}

//int process ( const char *filename, Profile &profile );

//void DensoScan::scan ( const std::string &name, vector <Box> boxes, Profile *profile )
//{
//    unsigned int nboxes;
//    double ymin = 0, ymax = 0;

//    for ( nboxes = 0; nboxes < boxes.size(); nboxes++ )
//    {
//        if ( !ymin || ymin > boxes[nboxes].ymm )
//            ymin = boxes[nboxes].ymm;

//        if ( ymax < ( boxes[nboxes].ymm + boxes[nboxes].hmm ) )
//            ymax = boxes[nboxes].ymm + boxes[nboxes].hmm;
//    }

//    scanner.set ( SCAN_TLY, ymin - 5 );
//    scanner.set ( SCAN_BRY, ymax + 1 );

//    if ( nboxes )
//    {
//        int width, height;
//        const unsigned char *buffer = scanner.start ( &width, &height );

//        Frame *frames[nboxes];
//        vector <std::string> filenames;
//        vector <std::thread> threads;

//        std::string filename;
//        int fofset = 1;
//        for ( int i = 0; i < nboxes; i++)
//        {
//            filename = fmt::format ( "{}-{}.png", name, i + fofset );
//            while ( file_exist( filename ) )
//                filename = fmt::format ( "{}-{}.png", name, i + ++fofset );

//            frames[i] = new Frame ( filename, scanner.get_lpmm(), boxes[i], profile );
//            filenames.push_back(filename);
//        }

//        int line = ( ymin - 5 ) * scanner.get_lpmm(), len;
//        while ( ( len = scanner.read () ) )
//        {
//            for ( int i = 0; i < nboxes; i++ )
//            {
//                if ( frames[i]->isinto( line ) )
//                    frames[i]->addline( buffer );
//                if ( frames[i]->iscomplete( line ) )
//                {
//                    frames[i]->running = true;
//                    threads.push_back ( std::thread ( [ &frames, i ] () { frames[i]->process(); } ) );
//                }
//            }

//            line++;
//        }

//        for ( unsigned int i = 0; i < threads.size(); i++)
//            threads[i].join();

//        for ( unsigned i = 0; i < nboxes; i++)
//            delete frames[i];
//    }
//}


int prv = 0;

void DensoScan::on_pushScan_clicked()
{
/*    prv++;
        if ( prv > 17 )
            prv = 1;
//    prv = 7;

    Frame f ( fmt::format ( "negative/test-{}-debug-1-scan.png", prv ),
             "negative/out.png", 4800, profiles.at(1) );
    f.process();
    return; // */

/*    prv++;
    if ( prv > 5 )
        prv = 1;

    Mat preview = imread( fmt::format ( "preview-example0{}.png", prv ), IMREAD_GRAYSCALE );
    double ppmm = 11.8110; // 5.9; */

    Scan preview = imread( "preview-test.png", IMREAD_GRAYSCALE );
    preview.ppmm = 150 / 25.4; // 5.9;// 11.8110; //  */
    onPreviewCompleted( preview );

    ui->pushScan->setEnabled( false );
    ui->pushCancel->setEnabled( true );
    scanner.preview();


/*    scanner.set ( SCAN_SOURCE, "Transparency Unit" );
    scanner.set ( "film-type", "Positive Film" );
    scanner.set ( SCAN_MODE, "Gray" );
            if ( ( begins[i]
    scanner.set ( SCAN_DEPTH, 16 );
    scanner.set ( "brightness", 0 );
    scanner.set ( SCAN_PREVIEW, false );
    scanner.set ( SCAN_DPI, 6400 );
    scanner.set ( SCAN_SHARP, 2 );

    curprofile = 0;
    scan ( "positive/parco", boxes );  // */
}


void DensoScan::on_pushCancel_clicked()
{
    scanner.cancel();
}

void DensoScan::onScanCompleted ()
{
    ui->pushCancel->setEnabled( false );
    ui->pushScan->setEnabled( true );
}

void DensoScan::onError ( const std::string &error )
{
    QMessageBox messageBox;
    messageBox.critical( 0, "Error", error.c_str() );
    messageBox.show();
    //messageBox.setFixedSize(500,200);
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

    for ( unsigned int f = 0; f < frames.size(); f++ )
    {
      Rect rect_frame ( frames[f].y * ppmm, output.size().height - ( frames[f].x + frames[f].width ) * ppmm, frames[f].height * ppmm, frames[f].width * ppmm );

        cv::rectangle( output, rect_frame, agfa_red, 2 );
        int baseline = 0;
        Size ts = getTextSize( fmt::format ("{}", f + 1), cv::FONT_HERSHEY_DUPLEX, 3, 3, &baseline  );
        Point point ( ( rect_frame.br() + rect_frame.tl()) * 0.5 );
        point.x -= ts.width / 2;
        point.y += ts.height / 2;
        putText(output, fmt::format ("{}", f + 1), point,
            cv::FONT_HERSHEY_DUPLEX, 3, agfa_red, 3, cv::LINE_AA );

    }

    imwrite ( "preview-output.png", output );

//return;

    scanner.optSet ( SCAN_SOURCE, "Transparency Unit" );
    scanner.optSet ( "film-type", "Negative Film" );
    scanner.optSet ( SCAN_MODE, "Gray" );
    scanner.optSet ( SCAN_DEPTH, 16 );
    scanner.optSet ( SCAN_PREVIEW, false );
    scanner.optSet ( SCAN_DPI, 4800 );
    scanner.optSet ( SCAN_SHARP, 2 );

    scanner_debug = false;

    scanner.scan ( frames ); // */
}

void DensoScan::onNewScan ( Scan &image, int frameNumber )
{
    imwrite ( fmt::format ( "image-{}.png", frameNumber ), image );
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
