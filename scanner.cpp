#include "scanner.h"
#include "frame.h"
#include "spline.h"

#include <fmt/format.h>
#include <png.h>

#include <sys/stat.h>

SANE_Int version_code = 0;
bool scanner_debug = true;

std::vector <ScannerDevice> getDevices ()
{
    SANE_Status ret;

    if ( !version_code )
    {
        ret = sane_init ( &version_code, nullptr );
        if ( ret != SANE_STATUS_GOOD )
            throw std::runtime_error ( string ( "Scanner init: " ) + sane_strstatus( ret ) );
    }

    SANE_Device **device_list = nullptr;

    std::vector <ScannerDevice> devices;

    ret = sane_get_devices ( (const SANE_Device***)&device_list, true );
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( string ( "Get devices: " ) + sane_strstatus( ret ) );

    int dev_index;
    for ( dev_index = 0; device_list[dev_index]; dev_index++ )
    {
        ScannerDevice device;
        device.name = device_list[dev_index]->name;
        device.vendor = device_list[dev_index]->vendor;
        device.model = device_list[dev_index]->model;
        device.type = device_list[dev_index]->type;

        devices.push_back(device);
    }

    return devices;
}

int Scanner::optindex ( const char *optname )
{
    for ( SANE_Int i = 0; i < nOptions; i++ )
        if ( optDescriptions[i]->name && !strcmp ( optDescriptions[i]->name, optname ) )
            return i;

    throw fmt::format ( "option \"{}\" not found", optname);
}

bool Scanner::optExist ( const char *name )
{
    for ( SANE_Int i = 0; i < nOptions; i++ )
        if ( optDescriptions[i]->name && !strcmp ( optDescriptions[i]->name, name ) )
            return true;

    return false;
}

void Scanner::open ( const char *name )
{
    close ();

    SANE_Status ret;

    ret = sane_open ( name, &hDevice );
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Open device {}: {}", name, sane_strstatus( ret ) ) );

    SANE_Int info;
    ret = sane_control_option ( hDevice, 0,SANE_ACTION_GET_VALUE, &nOptions, &info );
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Get device options {}: {}", name, sane_strstatus( ret ) ) );
    optDescriptions = new const SANE_Option_Descriptor *[nOptions];

    int i;
    for ( i = 0; i < nOptions; i++ )
        optDescriptions[i] = sane_get_option_descriptor ( hDevice, i);
}

void Scanner::close ()
{
    if ( hDevice )
        sane_close (hDevice);
    hDevice = nullptr;

    if ( optDescriptions )
        delete optDescriptions;

    optDescriptions = nullptr;
    nOptions = 0;
}

void Scanner::start ( int *maxx, int *maxy )
{
    SANE_Status ret;
    ret = sane_start ( hDevice );
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Start scanning: {}", sane_strstatus( ret ) ) );

    ret = sane_get_parameters ( hDevice, &parameters);
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( sane_strstatus( ret ) );

    pScanBuffer = new SANE_Byte [ parameters.bytes_per_line ];

    if ( maxx )
        *maxx = parameters.pixels_per_line;

    if ( maxy )
        *maxy = parameters.lines;
}

void Scanner::optSet ( const char *name, double value )
{
    SANE_Status ret;
    SANE_Int info;
    SANE_Fixed fix = SANE_FIX(value);
    ret = sane_control_option ( hDevice, optindex(name), SANE_ACTION_SET_VALUE, &fix, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", name, value, sane_strstatus( ret ) ) );
}

void Scanner::optSet ( const char *name, int value )
{
    SANE_Status ret;
    SANE_Int info;
    ret = sane_control_option ( hDevice, optindex(name), SANE_ACTION_SET_VALUE, &value, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", name, value, sane_strstatus( ret ) ) );
}

void Scanner::optSet ( const char *name, const char *value )
{
    SANE_Status ret;
    SANE_Int info;
    ret = sane_control_option ( hDevice, optindex(name), SANE_ACTION_SET_VALUE, (void *)value, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", name, value, sane_strstatus( ret ) ) );
}

double Scanner::optGetMaxD ( const char *name )
{
    return SANE_UNFIX(optGetMaxI ( name ));
}

double Scanner::getPpmm ( )
{
    return optGetI( SCAN_DPI ) / 25.4;
}

int Scanner::optGetMaxI ( const char *name )
{
    int index = optindex(name);
    if ( optDescriptions[index]->constraint_type != SANE_CONSTRAINT_RANGE )
        throw std::runtime_error ( fmt::format ("the \"{}\" parameter does not have a range", name) );

    return optDescriptions[index]->constraint.range->max;
}

double Scanner::optGetD ( const char *name )
{
    return SANE_UNFIX( optGetI ( name ) );
}

int Scanner::optGetI ( const char *name )
{
    SANE_Status ret;

    int value;
    ret = sane_control_option ( hDevice, optindex(name), SANE_ACTION_GET_VALUE, &value, nullptr );
    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option get {}: {}", name, sane_strstatus( ret ) ) );

    return value;
}

int Scanner::getDepth ()
{
    return parameters.depth;
}

int Scanner::getBpp ()
{
    return parameters.bytes_per_line / parameters.pixels_per_line;
}

bool Scanner::isColor ()
{
    return parameters.format == SANE_FRAME_RGB;
}

int Scanner::read ( )
{
    SANE_Status ret;
    SANE_Int len = 0;
    ret = sane_read ( hDevice, pScanBuffer, parameters.bytes_per_line, &len);

    if ( ret == SANE_STATUS_EOF || ret == SANE_STATUS_CANCELLED )
        return 0;

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( sane_strstatus( ret ) );

    return len;
}

void Scanner::cancel ( )
{
    sane_cancel (hDevice);
}



void Scanner::doscan ( int height )
{
    int n = 0;

    time_t lastTime = time(NULL);

    while ( read() )
    {
        for ( unsigned int i = 0; i < frames.size(); i++ )
        {
            if ( frames[i].isinto( nCurrentLine ) )
                frames[i].addline( pScanBuffer );
            if ( frames[i].iscomplete( nCurrentLine ) )
            {
                frames[i].running = true;
                Frame *f = &(frames[i]);
                Scanner *t = this;
                threads.push_back ( std::thread ( [ t, f, i ] () { t->doprocess( *f, i ); } ) );
            }
        }

        n++;

        if ( lastTime != time(NULL) )
        {
            onProgressUpdate ( "Scanning...", (double)n / height * 100 );
            lastTime = time(NULL);
        }

        nCurrentLine++;
    }

    for ( unsigned int i = 0; i < threads.size(); i++)
        threads[i].join();

    frames.clear();
    threads.clear();

    onProgressUpdate ( "Done.", 100 );
    onScanCompleted ();
}

std::vector<std::string> Scanner::getDomainS ( const char *param )
{
    std::vector<std::string>  domain;

    int i = optindex( param );
    if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_STRING_LIST )
        for ( int ns = 0; optDescriptions[i]->constraint.string_list[ns]; ns++ )
            domain.push_back( optDescriptions[i]->constraint.string_list[ns] );

    return domain;
}

std::vector<int> Scanner::getDomainI ( const char *param )
{
    std::vector<int>  domain;

    int i = optindex( param );
    int size = optDescriptions[i]->constraint.word_list[0];
    for ( int nw = 1; nw <= size; nw++ )
        domain.push_back( optDescriptions[i]->constraint.word_list[nw] );

    return domain;
}

void Scanner::dumpopts ()
{
    for ( int i = 0; i < nOptions; i++ )
    {
        if ( optDescriptions[i]->name )
        {
            fmt::print ( stderr, "{}\n{} - {}", optDescriptions[i]->name, optDescriptions[i]->title, optDescriptions[i]->desc );
            if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_RANGE )
                if ( optDescriptions[i]->type == SANE_TYPE_FIXED )
                    fmt::print ( stderr, ": min={}, max={}, quant={}", SANE_UNFIX(optDescriptions[i]->constraint.range->min),
                                                                       SANE_UNFIX(optDescriptions[i]->constraint.range->max),
                                                                       SANE_UNFIX(optDescriptions[i]->constraint.range->quant));
                else
                    fmt::print ( stderr, ": min={}, max={}, quant={}", optDescriptions[i]->constraint.range->min,
                                                                   optDescriptions[i]->constraint.range->max,
                                                                   optDescriptions[i]->constraint.range->quant);

            else if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_STRING_LIST )
            {
                fmt::print ( stderr, ": " );
                for ( int ns = 0; optDescriptions[i]->constraint.string_list[ns]; ns++ )
                {
                    fmt::print ( stderr, "\"{}\"", optDescriptions[i]->constraint.string_list[ns] );
                    if ( optDescriptions[i]->constraint.string_list[ns + 1] )
                        fmt::print ( stderr, ", " );
                }
            }
            else if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_WORD_LIST )
            {
                int size = optDescriptions[i]->constraint.word_list[0];
                fmt::print ( stderr, ": " );
                for ( int nw = 1; nw <= size; nw++ )
                {
                    fmt::print ( stderr, "{}", optDescriptions[i]->constraint.word_list[nw] );
                    if ( nw != size )
                        fmt::print ( stderr, ", " );
                }
            }
        }
        fmt::print ( stderr, "\n" );
    }
}

bool file_exist (const std::string& name) {
  struct stat buffer;
  return (stat (name.c_str(), &buffer) == 0);
}

void Scanner::scan ( std::vector <cv::Rect2d> boxes )
{
    onProgressUpdate ( "Starting...", 0 );

    unsigned int nboxes;
    double ymin = 0, ymax = 0;

    for ( nboxes = 0; nboxes < boxes.size(); nboxes++ )
    {
        if ( !ymin || ymin > boxes[nboxes].y )
            ymin = boxes[nboxes].y;

        if ( ymax < ( boxes[nboxes].y + boxes[nboxes].height ) )
            ymax = boxes[nboxes].y + boxes[nboxes].height;
    }

    optSet ( SCAN_TLY, ymin - 5 );
    optSet ( SCAN_BRY, ymax + 1 );

    if ( nboxes )
    {
        int width, height;

        start ( &width, &height );

        // Frame *frames[nboxes];
/*        vector <std::string> filenames;

        std::string filename;
        int fofset = 1;
        for ( unsigned int i = 0; i < nboxes; i++)
        {
            filename = fmt::format ( "{}-{}.png", name, i + fofset );
            while ( file_exist( filename ) )
                filename = fmt::format ( "{}-{}.png", name, i + ++fofset );

            frames.push_back( Frame ( filename, getPpmm(), boxes[i], profile ) );
            filenames.push_back(filename);
        } */

        nCurrentLine = ( ymin - 5 ) * getPpmm();

        Scanner *s = this;
        new std::thread ( [ s, height ] () { s->doscan(height); } );
    }
    else
        onScanCompleted ();
}

void Scanner::dopreview ( )
{
    onProgressUpdate ( "Starting...", 0 );

    Scan image;

    try {
        optSet ( SCAN_DPI, 150 );
        optSet ( SCAN_SOURCE, "Transparency Unit" );
        optSet ( SCAN_FILM_TYPE, "Negative Film" );
        optSet ( SCAN_MODE, "Gray" );
        optSet ( SCAN_DEPTH, 8 );
        optSet ( SCAN_PREVIEW, true );
        optSet ( "brightness", 0 );
        optSet ( SCAN_SHARP, 0 );

        optSet ( SCAN_TLY, 0 );
        optSet ( SCAN_BRY, optGetMaxI(SCAN_BRY) );

        int width, height;
        start ( &width, &height );

        image = Scan ( height, width, CV_8UC1, getPpmm() );

        int x = 0, y = 0, w = image.size().width;

        time_t lastTime = 0;

        while ( read () )
        {
            for ( x = 0; x < w; x++ )
                image.at<unsigned char>( y, x ) = pScanBuffer [x];
            y++;

            if ( lastTime != time(NULL) )
            {
                onProgressUpdate ( "Preview...", (double)y / height * 100 );
                lastTime = time(NULL);
            }
        }

        onPreviewCompleted( image );
    }
    catch ( std::runtime_error error )
    {
        cancel();
        onError ( error.what() );
        onPreviewCompleted( Scan () );
    }
    onProgressUpdate ( "Done...", 100 );
}

void Scanner::preview ( )
{
    Scanner *s = this;
    new std::thread ( [ s ] () { s->dopreview(); } );
}

bool compare_boxes(const Rect & a, const Rect &b) {
    return a.x >= b.x;
}

vector <Slot> Scanner::guessSlots ( const Scan &preview )
{
    vector <Slot> slots;

    // since your image has compression artifacts, we have to threshold the image
    int threshold = 10;
    cv::Mat mask = preview > threshold;

    // extract contours
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);

    cv::Mat output;
    if ( scanner_debug )
        cv::cvtColor( preview, output, cv::COLOR_GRAY2BGR );


    for( unsigned int i=0; i < contours.size(); ++i)
    {
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

        int area = rotatedRect.boundingRect().area();

        if ( area < ( 24 * 36 * preview.ppmm ) )
            continue;

        // read points and angle
        cv::Point2f rect_points[4];
        rotatedRect.points( rect_points );
        float  angle = rotatedRect.angle; // angle

        slots.push_back( Slot ( rotatedRect.boundingRect(), preview.ppmm, angle ) );

        if ( scanner_debug )
        {
            // draw bounding rect
            cv::rectangle( output, rotatedRect.boundingRect(), cv::Scalar( 255, 0,255 ), 3 );

            // read center of rotated rect
            cv::Point2f center = rotatedRect.center; // center

            // draw rotated rect
            for ( unsigned int j = 0; j < 4; ++j )
                cv::line( preview, rect_points[j], rect_points[( j + 1 ) % 4], cv::Scalar( 0, 255, 0 ));

            // draw center and print text
            std::stringstream ss;   ss << angle; // convert float to string
            cv::circle(output, center, 5, cv::Scalar(0,255,0)); // draw center
            cv::putText(output, ss.str(), center + cv::Point2f(-25,25), cv::FONT_HERSHEY_COMPLEX_SMALL, 1, cv::Scalar(255,0,255)); // print angle
        }
    }

    stable_sort( slots.begin(), slots.end(), compare_boxes );

    if ( scanner_debug )
    {
        imwrite ( "slots-mask.png", mask );
        imwrite ( "slots-output.png", output );
    }

    return slots;
}

void Scanner::guessFrames ( vector<Rect2d> &frames, const Scan &preview, const Slot &holder, Mat &dbgOutput )
{
    double ppmm = preview.ppmm;

    Rect holderRect = holder.px (ppmm);

    holderRect.x = holderRect.x + ( holderRect.width / 2.0 ) - ( holder.frameW * ppmm / 2.0 );
    holderRect.width = holder.frameW * ppmm;

    cv::Mat output;

    if ( scanner_debug )
        output = dbgOutput ( holderRect );

    Mat input;
    preview ( holderRect ).copyTo( input );

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

//    imwrite ( fmt::format ( "frames-gray-{}.png", nb ), input );

    int threshold = 127;
    cv::Mat mask = input > threshold;

//    imwrite ( fmt::format ( "frames-mask-{}.png", nb ), mask );

    int w = input.size().width;
    int h = input.size().height;

    bool in = 0;
    int gap_begin = 0;
    int gap_end = 0;

    int mingap = 1 * ppmm;
    int minframe = 10 * ppmm;
    int treshold = 230;
    int v;

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
    int framesize = holder.frameH * ppmm;
    int framedistance = holder.frameD * ppmm;
    unsigned int framemax = holder.frameN - 1;
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

    double approx = ( holder.frameD - holder.frameH ) * .8;

    for ( i = 0; i < begins.size(); i++ )
    {
        Rect box;
        box.x = 0;
        box.y = begins[i];
        box.width = w;
        box.height = ends[i] - begins[i];

        double m = mean ( preview (holderRect)(box) )[0];
        if ( m < 240 )
        {
            Rect2d frame;

            if ( !keepFrameBorders )
            {
                frame.x = ( holderRect.x / ppmm ) - ( approx / 2 );
                frame.width = ( holderRect.width / ppmm ) + approx;
            }
            else
            {
                frame.x = holder.x;
                frame.width = holder.width;
            }

            frame.y = holder.y + ( box.y / ppmm ) - ( approx / 2 );
            frame.height = ( box.height / ppmm ) + approx;

            frames.push_back(frame);

            if ( scanner_debug )
                cv::rectangle( output, box, cv::Scalar( 0, 255, 0 ), 1 );
        }
    }
}

vector<Rect2d> Scanner::guessFrames ( const Scan &preview, const vector<Slot> &holders )
{
    vector<Rect2d> ret;

    Mat dbgOutput;
    if ( scanner_debug )
        cv::cvtColor( preview, dbgOutput, cv::COLOR_GRAY2BGR);

    for ( unsigned int i = 0; i < holders.size(); i++ )
        guessFrames ( ret, preview, holders[i], dbgOutput );

    if ( scanner_debug )
        imwrite ( "frames-output.png", dbgOutput );

    return ret;
}


#define LMAX        65535.0
#define LUT_SIZE    (LMAX + 1)
typedef unsigned short pixel;

vector<pixel> getCalibLUT( Profile profile )
{
    vector<pixel> LUT(LUT_SIZE, 0);

    tk::spline s( profile.measured, profile.expected, tk::spline::cspline );
    double l, le;
    for (int i = 0; i < LUT_SIZE; ++i)
    {
        l = i / LMAX * 100;
        le = s ( l ) / 100;
        if (le < 0)
            le = 0;

        if ( le > 1 )
            le = 1;

        LUT[i] = le * LMAX;
    }

    return LUT;
}

vector<pixel> getNormLUT(pixel minValue, pixel maxValue)
{
    vector<pixel> LUT(LUT_SIZE, 0);

    double C = 1;
    if ( maxValue > minValue )
        C = LMAX / ( maxValue - minValue );

    for (int i = 0; i < LUT_SIZE; ++i)
    {
        if ( i > minValue && i <= maxValue )
            LUT[i] = round( (i - minValue) * C);
        else if ( i > maxValue )
            LUT[i] = LMAX;
        else
            LUT[i] = 0;
    }

    return LUT;
}

vector<pixel> getDefLUT( )
{
    vector<pixel> LUT(LUT_SIZE, 0);

    for (int i = 0; i < LUT_SIZE; ++i)
        LUT[i] = i;

    return LUT;
}

vector<pixel> getInvLUT( vector<pixel> &in )
{
    vector<pixel> LUT(LUT_SIZE, 0);

    for (int i = 0; i < LUT_SIZE; ++i)
        LUT[i] = LMAX - in[i];

    return LUT;
}

vector<pixel> getDensityLUT( vector<pixel> &in )
{
  vector<pixel> LUT(LUT_SIZE, 0);

  for (int i = 0; i < LUT_SIZE; ++i)
  {
      if ( in[i] )
           LUT[i] = log10( LMAX / in[i] ) * 10000;
      else
           LUT[i] = LMAX;
  }

  return LUT;
}

vector<pixel> getLogLUT( vector<pixel> &in )
{
    double C = LMAX / log10( LUT_SIZE );

    vector<pixel> LUT(LUT_SIZE, 0);

    for (int i = 0; i < LUT_SIZE; ++i)
        if ( in[i])
            LUT[i] = round(C * log10( LMAX / in[i] ));
        else
            LUT[i] = LMAX;

    return LUT;
}

vector<pixel> getGammaLUT(vector<pixel> &in, double gamma)
{
  vector<pixel> LUT(LUT_SIZE, 0);

  for (int i = 0; i < LUT_SIZE; ++i)
      LUT[i] = LMAX * pow ( in[i] / LMAX, 1 / gamma );

  return LUT;
}

void processImage(Mat& I, vector<pixel> LUT )
{
  pixel pos, neg;
  for (int i = 0; i < I.rows; ++i)
  {
    for (int j = 0; j < I.cols; ++j)
    {
      pos = I.at<pixel>(i, j);
      neg = LUT[pos];
      I.at<pixel>(i, j) = neg;
    }
  }
}

void Scanner::doprocess ( Frame &frame, int frameNumber )
{
    /*////////////////////////////////////////
     *
     * Parameters
     *
     */

    double targetw_mm = 24.00;
    double targeth_mm = 36.00;
    double crop_x100 = 12.5;

    Scan image = frame.scan();
    double ppmm = image.ppmm;
    int w = image.size().width;
    int h = image.size().height;

    int targetw_px = round ( targetw_mm * ppmm );
    int targeth_px = round ( targeth_mm * ppmm );
    int crop_x100_px = round ( crop_x100 / 100 * std::min( targetw_mm, targeth_mm) * ppmm );

    int cropw_px = targetw_px - crop_x100_px;
    int croph_px = targeth_px - crop_x100_px;

    int dbg_line = round ( 1 + 0.04 * ppmm );
    double dbg_scale = 1080.0 / std::min ( w, h );

    /*////////////////////////////////////////
     *
     * Check and convert image
     *
     */

//    if ( scanner_debug )
//        imwrite ( fmt::format ( dbgfilename, "debug-1-scan"), *image );

    Mat output;
    if ( scanner_debug )
    {
        Mat img8;
        image.convertTo(img8, CV_8UC1, 1. / 256.);
        cv::cvtColor( img8, output, cv::COLOR_GRAY2BGR);
    }

    Point center ( w / 2, h / 2);
    Rect crop ( center.x - cropw_px / 2, center.y - croph_px / 2, cropw_px, croph_px );

    /*////////////////////////////////////////
     *
     * process
     *
     */

    vector<pixel> LUT, LUT1;
    double minVal, maxVal;

    // Calibration
    LUT = getCalibLUT( profile );
    LUT = getLogLUT( LUT );
    processImage( image, LUT );

    // Min/Max avg 1/10
    Mat dst;
    double factor = .25;
    resize( image(crop), dst, cv::Size(), factor, factor, cv::INTER_LINEAR_EXACT);
    minMaxLoc(dst, &minVal, &maxVal);

    LUT = getNormLUT( minVal, maxVal );
    processImage( image, LUT );

//    imwrite( filename, rot );

    if ( scanner_debug )
    {
        Mat rot;
        cv::Scalar dbg_color ( 52, 167, 252 );
        cv::line(output, Point ( 0, crop.y ), Point ( w, crop.y ), dbg_color, dbg_line );
        cv::line(output, Point ( 0, crop.y + crop.height ), Point ( w, crop.y + crop.height ), dbg_color, dbg_line );
        cv::line(output, Point ( crop.x, 0 ), Point ( crop.x, h ), dbg_color, dbg_line );
        cv::line(output, Point ( crop.x + crop.width, 0 ), Point ( crop.x + crop.width, h ), dbg_color, dbg_line );

        rotate ( output, rot, ROTATE_90_COUNTERCLOCKWISE);
        cv::resize( rot, rot, Size (), dbg_scale, dbg_scale, cv::INTER_CUBIC );
        imwrite ( fmt::format ( "{}-debug-2-crop", frameNumber ),  rot );
    }

    rotate ( image, image, ROTATE_90_COUNTERCLOCKWISE);

    onNewScan ( image, frameNumber );
}
