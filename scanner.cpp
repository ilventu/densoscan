#include "scanner.h"
#include "spline.h"

#include <fmt/format.h>
#include <png.h>

#include <iostream>
#include <fstream>

#include <sys/stat.h>

using namespace std;
using namespace cv;

SANE_Int version_code = 0;

#define CROP_X100   12.5
#define DMAX        4.8
#define LMAX        65535.0

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

void Scanner::setSkipBegining(bool newSkipBegining)
{
    skipBegining = newSkipBegining;
}

bool Scanner::getSkipBegining() const
{
    return skipBegining;
}

void Scanner::setDetectionMode ( DetectionMode m )
{
    detectionMode = m;
}

int Scanner::optindex ( const char *optname )
{
    for ( SANE_Int i = 0; i < nOptions; i++ )
        if ( optDescriptions[i]->name && !strcmp ( optDescriptions[i]->name, optname ) )
            return i;

    throw std::runtime_error ( fmt::format ( "option \"{}\" not found", optname) );
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

    if ( optExist( SCAN_SOURCE ) )
        if ( isInDomain( SCAN_SOURCE, "Transparency Unit" ) )
            optSet ( SCAN_SOURCE, "Transparency Unit" );
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

Scanner::DomainType Scanner::getDomainType(const char *param)
{
    int i = optindex( param );
    return (DomainType) optDescriptions[i]->constraint_type;
}

int Scanner::optGetMaxI ( const char *name )
{
    return optGetMaxD ( name );
}

double Scanner::optGetMaxD ( const char *name )
{
    int index = optindex(name);
    if ( optDescriptions[index]->constraint_type != SANE_CONSTRAINT_RANGE )
        throw std::runtime_error ( fmt::format ("the \"{}\" parameter does not have a range", name) );

    if ( optDescriptions[index]->type == SANE_TYPE_FIXED )
        return SANE_UNFIX(optDescriptions[index]->constraint.range->max);
    else
        return optDescriptions[index]->constraint.range->max;
}

int Scanner::optGetMinI ( const char *name )
{
    return optGetMaxD ( name );
}

double Scanner::optGetMinD ( const char *name )
{
    int index = optindex(name);
    if ( optDescriptions[index]->constraint_type != SANE_CONSTRAINT_RANGE )
        throw std::runtime_error ( fmt::format ("the \"{}\" parameter does not have a range", name) );

    if ( optDescriptions[index]->type == SANE_TYPE_FIXED )
        return SANE_UNFIX(optDescriptions[index]->constraint.range->min);
    else
        return optDescriptions[index]->constraint.range->min;
}


double Scanner::getPpmm ( )
{
    return optGetI( SCAN_DPI ) / 25.4;
}

int Scanner::optType(const char *name)
{
    int index = optindex(name);

    return optDescriptions[index]->type;
}

int Scanner::optUnit(const char *name)
{
    int index = optindex(name);

    return optDescriptions[index]->unit;
}

double Scanner::optSet(const char *name, double value)
{
    int i = optindex(name);
    if ( optDescriptions[i]->type == SANE_TYPE_FIXED )
        return optSetFix ( i, value );
    else
        return optSetInt ( i, value );
}

int Scanner::optSet(const char *name, int value)
{
    int i = optindex(name);

    if ( optDescriptions[i]->type == SANE_TYPE_FIXED )
        return optSetFix ( i, value );
    else
        return optSetInt ( i, value );
}

string Scanner::optSet(const char *name, const char *value)
{
    SANE_Status ret = SANE_STATUS_GOOD;
    SANE_Int info;

    int i = optindex(name);

    bool ok = false;

    if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_STRING_LIST )
    {
        for ( int ns = 0; optDescriptions[i]->constraint.string_list[ns]; ns++ )
            if ( !strcmp (optDescriptions[i]->constraint.string_list[ns], value ) )
                ok = true;
    }
    else
        ok = true;

    if (ok)
        ret = sane_control_option ( hDevice, i, SANE_ACTION_SET_VALUE, (void *)value, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", name, value, sane_strstatus( ret ) ) );

    return optGetS ( i );
}

double Scanner::optSetFix(int i, double value)
{
    SANE_Status ret = SANE_STATUS_GOOD;
    SANE_Int info = 0;

    if ( optDescriptions[i]->constraint_type == SANE_CONSTRAINT_RANGE )
    {
        double quant = SANE_UNFIX( optDescriptions[i]->constraint.range->quant );
        double min = SANE_UNFIX( optDescriptions[i]->constraint.range->min );
        double max = SANE_UNFIX( optDescriptions[i]->constraint.range->max );
        if ( quant == 0 )
            quant = 1;

        if ( value < min )
            value = min;
        else if ( value > max )
            value = max;
        else
        {
            int n = value / quant;
            value = n * quant;
        }
    }

    SANE_Fixed fix = SANE_FIX((float)value);
    ret = sane_control_option ( hDevice, i, SANE_ACTION_SET_VALUE, &fix, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", optDescriptions[i]->name, value, sane_strstatus( ret ) ) );

    return value;
}

int Scanner::optSetInt(int i, int value)
{
    SANE_Status ret = SANE_STATUS_GOOD;
    SANE_Int info = 0;

    ret = sane_control_option ( hDevice, i, SANE_ACTION_SET_VALUE, &value, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", optDescriptions[i]->name, value, sane_strstatus( ret ) ) );

    return value;
}

string Scanner::optGetS(int i)
{
    SANE_Status ret = SANE_STATUS_GOOD;
    SANE_Int info;

    char value [1024];
    memset(value, 1, 1024);

//    ret = sane_control_option ( hDevice, i, SANE_ACTION_GET_VALUE, value, &info);

    if ( ret != SANE_STATUS_GOOD )
        throw std::runtime_error ( fmt::format ( "Option set {}={}: {}", optDescriptions[i]->name, value, sane_strstatus( ret ) ) );

    return value;
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
    cancelling = true;
    sane_cancel (hDevice);
}

void Scanner::setProfile(const Profile &p)
{
    profile.expected = p.expected;
    profile.measured = p.measured;
}

void Scanner::setMode(OutputMode mode)
{
    outputMode = mode;
}

void Scanner::setBrightness(int level)
{
    brightness = level;
}

void Scanner::doscan ( std::vector <Box> boxes )
{
    if ( !boxes.size() )
    {
        if ( detectionMode == stoufferT2115 )
            boxes.push_back( Box ( cv::Rect2d ( 55, 16, 40, 180 ) ) );
        else
            boxes.push_back( Box ( cv::Rect2d ( optGetMinD(SCAN_TLX), optGetMinD(SCAN_TLY), optGetMaxD(SCAN_BRX), optGetMaxD(SCAN_BRY) ) ) );
    }

    for ( int b = 0; !cancelling && b < batchScan; b++ )
    {
        std::vector <Frame> frames;

        unsigned int nboxes;
        double ymin = 0, ymax = 0;

        for ( nboxes = 0; nboxes < boxes.size(); nboxes++ )
        {
            if ( !ymin || ymin > boxes[nboxes].y )
                ymin = boxes[nboxes].y;

            if ( ymax < ( boxes[nboxes].y + boxes[nboxes].height ) )
                ymax = boxes[nboxes].y + boxes[nboxes].height;
        }

        ymin -= 5;
        ymax += 1;

        try
        {
            if ( optExist( SCAN_SOURCE ) )
                if ( isInDomain( SCAN_SOURCE, "Transparency Unit" ) )
                    optSet ( SCAN_SOURCE, "Transparency Unit" );

            if ( scanFilmType != "" )
                optSet ( SCAN_FILM_TYPE, scanFilmType.c_str() );

            optSet ( SCAN_MODE, "Gray" );

            optSet ( SCAN_DEPTH, 16 );
            optSet ( SCAN_PREVIEW, false );

            if ( optExist( SCAN_DPI ) )
                optSet ( SCAN_DPI, scanDPI );

            if ( optExist( "x-resolution" ) )
            {
                optSet ( "x-resolution", scanDPI );
                optSet ( "y-resolution", scanDPI );
            }

            optSet ( SCAN_SHARP, 2 );

            optSet ( "brightness", brightness );

            if ( !skipBegining )
                ymin = optGetMinD(SCAN_TLY);

            optSet ( SCAN_TLY, (double)ymin );
            optSet ( SCAN_BRY, (double)ymax );
            optSet ( SCAN_TLX, optGetMinD(SCAN_TLX) );
            optSet ( SCAN_BRX, optGetMaxD(SCAN_BRX) );

            int width, height;

            onProgressUpdate ( "Starting...", 0 );
            start ( &width, &height );

    //        double ppmm = getPpmm();
            double lw = optGetD(SCAN_BRX) - optGetD(SCAN_TLX);
            double ppmmw = width / lw;

            double lh = optGetD(SCAN_BRY) - optGetD(SCAN_TLY);
            double ppmmh = height / lh;

            if ( scanner_debug )
            {
                cout << "scan lw: " << lw << " ppmmw: " << ppmmh << endl;
                cout << "scan lh: " << lh << " ppmmh: " << ppmmw << endl;
            }

            for ( unsigned i = 0; i < boxes.size(); i++ )
            {
                frames.push_back( Frame ( boxes[i], ppmmw, ppmmh ) );
                if ( scanner_debug )
                {
                    Frame frame ( boxes[i], ppmmw, ppmmh );
                    cout << frames.size() << ": ";
                    cout << frame.x / ppmmw << ", ";
                    cout << frame.y / ppmmh << ", ";
                    cout << frame.width / ppmmw << ", ";
                    cout << frame.height / ppmmh << endl;
                }
            }

            int n = 0;

            time_t lastTime = time(NULL);

            nCurrentLine = ymin * ppmmh;
            while ( !cancelling && read() )
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
                        int frame = i + (frames.size() * b);
                        threads.push_back ( std::thread ( [ t, f, frame ] () { t->doprocess( *f, frame ); } ) );
                    }
                }

                n++;

                if ( lastTime != time(NULL) )
                {
                    string s;
                    if ( batchScan == 1 )
                        s = "Scanning...";
                    else
                        s = fmt::format ( "Scanning {}/{}...", b + 1, batchScan );

                    onProgressUpdate ( s, (double)n / height * 100 );
                    lastTime = time(NULL);
                }

                nCurrentLine++;
            }

            if ( !cancelling )
                for ( unsigned int i = 0; i < frames.size(); i++ )
                    if ( !frames[i].running )
                    {
                        frames[i].running = true;
                        Frame *f = &(frames[i]);
                        Scanner *t = this;
                        int frame = i + (frames.size() * b);
                        threads.push_back ( std::thread ( [ t, f, frame ] () { t->doprocess( *f, frame ); } ) );
                    }
        }
        catch ( std::runtime_error error )
        {
            cancel();
            onError ( error.what() );
        }

        for ( unsigned int i = 0; i < threads.size(); i++)
            threads[i].join();

        frames.clear();
        threads.clear();
    }

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

bool Scanner::isInDomain(const char *param, const char *value)
{
    std::vector<std::string> domain = getDomainS ( param );
    std::vector<std::string>::iterator it = find(domain.begin(), domain.end(), value );
    if (it != domain.end()) {
        return true;
    }
    return false;
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

void Scanner::scan ( std::vector <Box> boxes )
{
    cancelling = false;

    if ( boxes.size() || detectionMode > autoFullStrip )
    {
        Scanner *s = this;
        new std::thread ( [ s, boxes ] () { s->doscan(boxes); } );
    }
    else
        onScanCompleted ();
}

void Scanner::dopreview ( )
{
    onProgressUpdate ( "Starting...", 0 );

    int dpi = previewDPI;

    Scan image;

    try {
        if ( optExist( "test-picture" ) )
            optSet ( "test-picture", "Grid" );

        if ( optExist( SCAN_DPI ) )
            dpi = optSet ( SCAN_DPI, dpi );

        if ( optExist( "x-resolution" ) )
        {
            optSet ( "x-resolution", dpi );
            optSet ( "y-resolution", dpi );
        }

        if ( optExist( SCAN_MODE ) )
            optSet ( SCAN_MODE, "Gray" );
        if ( optExist( SCAN_DEPTH ) )
            optSet ( SCAN_DEPTH, 8 );

        if ( optExist( SCAN_SOURCE ) )
            if ( isInDomain( SCAN_SOURCE, "Transparency Unit" ) )
                optSet ( SCAN_SOURCE, "Transparency Unit" );

        if ( scanFilmType != "" )
            if ( optExist( SCAN_FILM_TYPE ) )
                optSet ( SCAN_FILM_TYPE, scanFilmType.c_str() );

        if ( optExist( SCAN_PREVIEW ) )
            optSet ( SCAN_PREVIEW, true );

        if ( optExist( "brightness" ) )
            optSet ( "brightness", 0 );

        if ( optExist( SCAN_SHARP ) )
            optSet ( SCAN_SHARP, 0 );

        if ( optExist( SCAN_TLY ) )
        {
            optSet ( SCAN_BRY, optGetMaxD(SCAN_BRY) );
            optSet ( SCAN_BRX, optGetMaxD(SCAN_BRX) );
            optSet ( SCAN_TLY, optGetMinD(SCAN_TLX) );
            optSet ( SCAN_TLX, optGetMinD(SCAN_TLX) );
        }
    }
    catch ( std::runtime_error error )
    {
    }

    try
    {
        int width, height;
        start ( &width, &height );

        double lw = optGetD(SCAN_BRX) - optGetD(SCAN_TLX);
        double ppmmw = width / lw;

        double lh = optGetD(SCAN_BRY) - optGetD(SCAN_TLY);
        double ppmmh = height / lh;

        if ( scanner_debug )
        {
            cout << "preview lw: " << lw << " ppmmw: " << ppmmh << endl;
            cout << "preview lh: " << lh << " ppmmh: " << ppmmw << endl;
        }

        image = Scan ( height, width, CV_8UC1, ppmmw, ppmmh );

        int x = 0, y = 0, w = image.size().width;

        time_t lastTime = 0;

        int len;

        while ( ( len = read () ) )
        {
            //assert( len == w);
            for ( x = 0; x < w; x++ )
                image.at<unsigned char>( y, x ) = pScanBuffer [x];
            y++;

            if ( lastTime != time(NULL) )
            {
                onProgressUpdate ( "Preview...", (double)y / height * 100 );
                lastTime = time(NULL);
            }
        }

        imwrite("preview.png", image );
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

    cancelling = false;
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
    if ( scanner_debug & DEBUG_HOLDER )
        cv::cvtColor( preview, output, cv::COLOR_GRAY2BGR );

    for( unsigned int i=0; i < contours.size(); ++i)
    {
        // fit bounding rectangle around contour
        cv::RotatedRect rotatedRect = cv::minAreaRect(contours[i]);

        Rect slot = rotatedRect.boundingRect();
        int area = slot.area();

        if ( slot.width == preview.size().width && slot.height == preview.size().height )
            continue;

        if (   area < ( 24 * preview.ppmmw * 36 * preview.ppmmh ) )
            continue;

        // read points and angle
        cv::Point2f rect_points[4];
        rotatedRect.points( rect_points );
        float  angle = rotatedRect.angle; // angle

        if ( slot.x < 0 )
        {
            slot.width += slot.x;
            slot.x = 0;
        }

        slots.push_back( Slot ( slot, preview.ppmmw, preview.ppmmh, angle ) );

        if ( scanner_debug & DEBUG_HOLDER)
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

    if ( scanner_debug & DEBUG_HOLDER)
    {
        imwrite ( "slots-mask.png", mask );
        imwrite ( "slots-output.png", output );
    }

    return slots;
}

std::vector<string> Scanner::getFilmTypes()
{
    if ( optExist(SCAN_FILM_TYPE) )
        return getDomainS ( SCAN_FILM_TYPE );
    else
        return std::vector<std::string>();
}

std::vector<int> Scanner::getDPIs()
{
    std::vector<int> ret;

    if ( optExist( SCAN_DPI ))
    {
        if ( getDomainType ( SCAN_DPI ) == range )
        {
            int max = optGetMaxI( SCAN_DPI );
            int dpis [] = { 300, 600, 1200, 3200, 4800, 6400, 9600, 0 };

            for ( int i = 0; dpis[i] && dpis[i] < max; i++ )
                ret.push_back ( dpis [i] );
            ret.push_back( max );
        }
        else
        {
            ret = getDomainI ( SCAN_DPI );
            while ( ret.size() && ret[0] < 300 )
                ret.erase( ret.begin());
        }
    }

    return ret;
}

Size Scanner::getPreviewSize()
{
    int w, h, t;

    if ( !hDevice )
        return cv::Size ( 768, 1024 );

    w = optGetMaxI( SCAN_BRX );
    h = optGetMaxI( SCAN_BRY );
    t = optUnit( SCAN_BRX );

    if ( t == SANE_UNIT_MM )
    {
        double ppmm = previewDPI / 25.4;
        return cv::Size ( w * ppmm, h * ppmm );
    }
    else
        return cv::Size ( w, h );
}

void Scanner::setDPI(int dpi)
{
    scanDPI = dpi;
}

void Scanner::setOutputDPI(int dpi)
{
    outputDPI = dpi;
}

void Scanner::setInterpolation(cv::InterpolationFlags i)
{
    interpol = i;
}

void Scanner::setBatchScan(int n)
{
    if ( n > 0 )
        batchScan = n;
    else
        batchScan = 1;
}

void Scanner::setPreviewDPI(int dpi)
{
    previewDPI = dpi;
}

int Scanner::getPreviewDPI()
{
    return previewDPI;
}

void Scanner::setDebug(int debug)
{
    scanner_debug = debug;
}

int Scanner::getDebug()
{
    return scanner_debug;
}

void Scanner::setFilmType(const std::string &filmType)
{
    scanFilmType = filmType;
}

void Scanner::guessFrames ( vector<Rect2d> &frames, const Scan &preview, const Slot &holder, Mat &dbgOutput )
{
    double ppmmw = preview.ppmmw;
    double ppmmh = preview.ppmmh;

    Rect holderRect = holder.px (ppmmw, ppmmh);

    holderRect.x = holderRect.x + ( holderRect.width / 2.0 ) - ( holder.frameW * ppmmw / 2.0 );
    holderRect.width = holder.frameW * ppmmw;
    if ( ( holderRect.y + holderRect.height ) > preview.size().height )
        holderRect.height = preview.size().height - holderRect.y;

    cv::Mat output;

    if ( scanner_debug & DEBUG_FRAMES )
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

    if ( scanner_debug & DEBUG_FRAMES )
        imwrite ( "frames-gray.png", input );

    int threshold = 200;
    cv::Mat mask = input > threshold;

    if ( scanner_debug & DEBUG_FRAMES )
        imwrite ( "frames-mask.png", mask );

    int w = input.size().width;
    int h = input.size().height;

    bool in = 0;
    int gap_begin = 0;
    int gap_end = 0;

    int mingap = 1 * ppmmh;
    int minframe = 10 * ppmmh;
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
                double size = ( y - gap_end )  / ppmmh;
                double gapSize = ( gap_end - gap_begin ) / ppmmh;

                gap_begin = y;
                in = true;

                if ( size <= 37 && size >= 34 && gapSize < 2.5 )
                {
                    if ( scanner_debug & DEBUG_FRAMES )
                        cv::rectangle( output, Point ( 0, gap_end + 1 ), Point ( w, gap_begin - 1 ), cv::Scalar( 255, 0, 0 ), 4 );

                    begins.push_back( gap_end );
                    ends.push_back( gap_begin );
                }
                else
                    if ( scanner_debug & DEBUG_FRAMES )
                        cv::rectangle( output, Point ( 0, gap_end + 1 ), Point ( w, gap_begin - 1 ), cv::Scalar( 0, 0, 255 ), 1 );
            }
        }
        else
        {
            if ( in )
            {
                double size = ( y - gap_begin ) / ppmmh;

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
    int framesize = holder.frameH * ppmmh;
    int framedistance = holder.frameD * ppmmh;
    unsigned int framemax = holder.frameN - 1;
    int size;
    unsigned int i = 0;


    while ( i < begins.size() )
    {
        prev = 0;
        for ( i = 0; i < begins.size(); i++ )
        {
            size = ends [ i ] - begins [ i ];

            if ( ( begins[i] - prev ) > ( framedistance * .98 ) )
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

    double decrop = ( holder.frameD - holder.frameH ) * .9;

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
                frame.x = ( holderRect.x / ppmmw ) - ( decrop / 2 );
                frame.width = ( holderRect.width / ppmmw ) + decrop;
            }
            else
            {
                frame.x = holder.x;
                frame.width = holder.width;
            }

            frame.y = holder.y + ( box.y / ppmmh ) - ( decrop / 2 );
            frame.height = ( box.height / ppmmh ) + decrop;

            frames.push_back(frame);

            if ( scanner_debug & DEBUG_FRAMES )
            {
                cv::rectangle( output, box, cv::Scalar( 0, 255, 0 ), 1 );
                cout << frames.size() << ": ";
                cout << frame.x << ", ";
                cout << frame.y << ", ";
                cout << frame.width << ", ";
                cout << frame.height << endl;
            }
        }
    }
}

void Scanner::guessRegularFrames ( std::vector<Box> &frames, const Scan &preview, const Slot &holder, cv::Mat &dbgOutput )
{
    if ( detectionMode == autoFullStrip || holder.frameN <= 1 )
    {
        Box frame ( holder, holder );
        frames.push_back(frame);
        return;
    }

    double ppmmw = preview.ppmmw;
    double ppmmh = preview.ppmmh;

    Rect holderRect = holder.px (ppmmw, ppmmh);

    if ( ( holderRect.y + holderRect.height ) > preview.size().height )
        holderRect.height = preview.size().height - holderRect.y;

//    cv::Mat output;

//    if ( scanner_debug & DEBUG_FRAMES )
//        output = dbgOutput ( holderRect );

    Mat input;
    preview ( holderRect ).copyTo( input );

    cv::threshold ( input, input, 254, 255, cv::THRESH_TOZERO_INV );

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

    if ( scanner_debug & DEBUG_FRAMES )
        imwrite ( "frames-gray.png", input );

    int threshold = 200;
    cv::Mat mask; // = input > threshold;
    cv::threshold ( input, mask, threshold, 255, cv::THRESH_TOZERO );

    if ( scanner_debug & DEBUG_FRAMES )
        imwrite ( "frames-mask.png", mask );

//    int w = input.size().width;
//    int h = input.size().height;

    int frameGap = ( holder.frameD - holder.frameH ) * ppmmh;
    double avgmax = 0;
    int maxOffset = (holder.frameH / 2) * ppmmh;
    int minOffset = -(maxOffset);
    int stripOffset = 0;

    for ( int offset = minOffset; offset < maxOffset; offset ++ )
    {
        double avg = 0;
        for ( int i = 1; i < holder.frameN; i++ )
        {
            Rect gap ( mask.size().width / 4, offset + i * (holder.frameD * ppmmh) - frameGap, mask.size().width / 2, frameGap );
            avg += mean ( mask(gap) ).val[0];
        }
        if ( avg > avgmax )
        {
            avgmax = avg;
            stripOffset = offset;
        }
    }

    double decrop = ( holder.frameD - holder.frameH );// * .9;
    cv::threshold ( preview, input, 254, 255, cv::THRESH_TOZERO_INV );

    for ( int i = 0; i < holder.frameN; i++ )
    {
        Rect box;
        box.x = holderRect.x;
        box.y = holderRect.y + stripOffset + holder.frameD * ppmmh * i;
        box.width = mask.size().width;
        box.height = holder.frameD * ppmmh - frameGap;

        if ( box.y > 0 && (box.y + box.height) < input.size().height )
        {
            double m = mean ( input (box) )[0];
            if ( m > 10 )
            {
                Rect2d frame;

                if ( !keepFrameBorders )
                {
                    frame.x = ( box.x / ppmmw ) + ( decrop / 2 );
                    frame.width = ( box.width / ppmmw ) - decrop;
                }
                else
                {
                    frame.x = box.x / ppmmw;
                    frame.width = box.width / ppmmw;
                }

                frame.y = ( box.y / ppmmh ) - ( decrop / 2 );
                frame.height = ( box.height / ppmmh ) + decrop;

                frames.push_back( Box ( frame, holder ) );

                if ( scanner_debug & DEBUG_FRAMES )
                {
                    cv::rectangle( dbgOutput, box, cv::Scalar( 0, 255, 0 ), 1 );
                    cv::rectangle( dbgOutput, Rect ( frame.x * ppmmw, frame.y * ppmmh, frame.width * ppmmw, frame.height * ppmmh), cv::Scalar( 0, 0, 255 ), 1 );
                    imwrite ( "frames-output.png", dbgOutput );
                    cout << frames.size() << ": ";
                    cout << frame.x << ", ";
                    cout << frame.y << ", ";
                    cout << frame.width << ", ";
                    cout << frame.height << endl;
                }
            }
        }
    }
}

vector<Box> Scanner::guessFrames ( const Scan &preview, const vector<Slot> &holders )
{
    vector<Box> ret;

    Mat dbgOutput;
    if ( scanner_debug & DEBUG_FRAMES )
        cv::cvtColor( preview, dbgOutput, cv::COLOR_GRAY2BGR);

    for ( unsigned int i = 0; i < holders.size(); i++ )
        guessRegularFrames ( ret, preview, holders[i], dbgOutput );

    if ( scanner_debug & DEBUG_FRAMES )
        imwrite ( "frames-output.png", dbgOutput );

    return ret;
}

#define PIXEL       float
#define PIXEL_MAT   CV_32FC1

void Scanner::doprocess ( Frame &frame, int frameNumber )
{
    double sigma = scanDPI / 2000.0, amount = 0.8;

    if ( outputMode == RAW )
    {
        Scan image = frame.scan();
        if ( outputDPI )
        {
            double ratio = outputDPI / (double)scanDPI;
            resize( image, image, Size(), ratio, ratio, interpol );
        }

        onNewScan ( image, frameNumber );
        return;
    }

    Scan image = frame.scan();

    if ( scanner_debug & DEBUG_RAWSCAN )
    {
        imwrite ( fmt::format ( "{}-debug-1-scan.png", frameNumber), frame.scan() );
        ofstream out;
        out.open (fmt::format ( "{}-debug-1-scan.json", frameNumber));
        out << fmt::format ( "{{\n\t\"ppmmw\": {},\n\t\"ppmmh\": {},\n\t\"x\": {},\n\t\"y\": {},\n\t\"width\": {},\n\t\"height\": {},\n\t\"targetw\": {},\n\t\"targeth_mm\": {}\n}}", \
                             frame.ppmmw, frame.ppmmh, frame.x, frame.y, frame.width, frame.height, frame.targetw_mm, frame.targeth_mm);
        out.close();
    }

    Mat output;
    if ( scanner_debug & DEBUG_PROCESSCROP)
    {
        Mat img8;
        image.convertTo(img8, CV_8UC1, 1. / 256.);
        cv::cvtColor( img8, output, cv::COLOR_GRAY2RGB);
    }

    image.convertTo(image, PIXEL_MAT );

    /*////////////////////////////////////////
    *
    * Density values
    *
    */

    bool bProfile = profile.expected.size();
    tk::spline s;
    if ( bProfile )
        s = tk::spline ( profile.measured, profile.expected, tk::spline::cspline );

    double l, d;
    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            l = image.at<PIXEL>(i, j) / LMAX * 100;

            if ( bProfile )
                l = s ( l );

            if (l < 0)
                l = 0;

            if ( l > 100 )
                l = 100;

            if ( l )
                d = log10 ( 100 / l );
            else
                d = DMAX;

            image.at<PIXEL>(i, j) = d;
        }

    if ( scanner_debug & DEBUG_RAWSCAN)
    {
        Mat dst;
        image.convertTo(dst, CV_16UC1, LMAX / log10 ( LMAX ) );
        imwrite ( fmt::format ( "{}-debug-2-log10.png", frameNumber ),  dst );
    }

    /*////////////////////////////////////////
    *
    * Crop & resize to get min & max
    *
    */

    double minVal, maxVal, factor;
    if ( outputMode == ENDBV )
    {
//        Scan image = frame.scan();
        double ppmmw = image.ppmmw;
        double ppmmh = image.ppmmh;
        int w = image.size().width;
        int h = image.size().height;

        int targetw_px = round ( frame.targetw_mm * ppmmw );
        int targeth_px = round ( frame.targeth_mm * ppmmh );
        int crop_x100_px = round ( CROP_X100 / 100 * std::min( frame.targetw_mm, frame.targeth_mm) * ppmmh ); // TODO: resize image

        int cropw = targetw_px - crop_x100_px;
        int croph = targeth_px - crop_x100_px;

        Point center ( w / 2, h / 2);
        Rect crop ( center.x - cropw / 2, center.y - croph / 2, cropw, croph );
        // Min/Max avg 1/10
        Mat dst;
        double crop_factor = 1000.0 / std::max( w, h ); //0.25;
        resize( image(crop), dst, cv::Size(), crop_factor, crop_factor, interpol );

        if ( scanner_debug & DEBUG_PROCESSCROP)
        {
            int dbg_line = sigma * 16;
            double dbg_scale = 540.0 / std::min ( w, h );
            cv::Scalar dbg_color ( 52, 167, 252 );
            cv::line(output, Point ( 0, crop.y + crop.height ), Point ( w, crop.y + crop.height ), dbg_color, dbg_line );
            cv::line(output, Point ( crop.x, 0 ), Point ( crop.x, h ), dbg_color, dbg_line );
            cv::line(output, Point ( crop.x + crop.width, 0 ), Point ( crop.x + crop.width, h ), dbg_color, dbg_line );
            cv::line(output, Point ( 0, crop.y ), Point ( w, crop.y ), dbg_color, dbg_line );

            if ( frame.targeth_mm > frame.targetw_mm  )
                rotate ( output, output, ROTATE_90_COUNTERCLOCKWISE);
            else
                rotate ( image, image, ROTATE_180);

            cv::resize( output, output, Size (), dbg_scale, dbg_scale, cv::INTER_CUBIC );
            imwrite ( fmt::format ( "{}-debug-3-crop.png", frameNumber ),  output );
        }

        minMaxLoc(dst, &minVal, &maxVal);

        if ( scanner_debug & DEBUG_PROCESSCROP )
        {
            dst.convertTo(dst, CV_16UC1, LMAX / log10 ( LMAX ) );
            imwrite ( fmt::format ( "{}-debug-4-crop-resize.png", frameNumber ),  dst );
        }

        factor = LMAX / ( maxVal - minVal );
    }
    else
    {
        factor = 10000;
        minVal = 0;
        maxVal = 6.5;
    }

    for (int i = 0; i < image.rows; ++i)
        for (int j = 0; j < image.cols; ++j)
        {
            d = image.at<PIXEL>(i, j);
            if ( d > maxVal )
                d = maxVal;
            else if ( d < minVal )
                d = minVal;

            image.at<PIXEL>(i, j) = ( d - minVal ) * factor;
        }

    /*////////////////////////////////////////
    *
    * Sharpen image using "unsharp mask" algorithm
    *
    */

    if (1)
    {
        Mat blurred;
        GaussianBlur(image, blurred, Size(), sigma, sigma);
        image.image() = image * ( 1 + amount) + blurred * ( -amount );
    }

    if ( frame.targeth_mm > frame.targetw_mm  )
        rotate ( image, image, ROTATE_90_COUNTERCLOCKWISE);
    else
        rotate ( image, image, ROTATE_180);

    //////////////////////////////////////////
    //
    // resize
    //
    ////

    if ( outputDPI )
    {
        double ratio = outputDPI / (double)scanDPI;
        resize( image, image, Size(), ratio, ratio, INTER_LINEAR );
    }

    image.convertTo(image, CV_16UC1);
    onNewScan ( image, frameNumber );
}

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
