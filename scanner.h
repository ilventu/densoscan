#ifndef SCANNER_H
#define SCANNER_H

#include <string>
#include <vector>
#include <thread>

#include <sane/sane.h>

#include <opencv2/opencv.hpp>
#include <opencv2/imgcodecs.hpp>

#define SCAN_DPI       "resolution"
#define SCAN_SHARP     "sharpness"
#define SCAN_DEPTH     "depth"
#define SCAN_MODE      "mode"
#define SCAN_FILM_TYPE "film-type"
#define SCAN_PREVIEW   "preview"
#define SCAN_SOURCE    "source"
#define SCAN_TLX       "tl-x"
#define SCAN_TLY       "tl-y"
#define SCAN_BRX       "br-x"
#define SCAN_BRY       "br-y"

// https://sane-project.gitlab.io/standard/api.html
// https://subscription.packtpub.com/book/application_development/9781784391454/1/ch01lvl1sec16/logarithmic-transformations

typedef struct
{
    std::vector<double> expected;
    std::vector<double> measured;
} Profile;

class Slot : public cv::Rect2d
{
public:
    double frameW = 24;
    double frameH = 36;

    double frameD = 38;
    double frameN = 6;

    double angle = 0;

public:
    Slot ( cv::Rect rect, double ppmm, double angle )
    {
        x = rect.x / ppmm;
        y = rect.y / ppmm;
        width = rect.width / ppmm;
        height = rect.height / ppmm;
    }

    cv::Rect px ( double ppmm ) const
    {
        return cv::Rect ( x * ppmm, y * ppmm, width * ppmm, height * ppmm);
    }
};

class Scan : public cv::Mat
{
public:
    double ppmm = 0;

public:
    Scan ( int height, int width, int type, double ppmm ) : cv::Mat ( height, width, type )
    {
        this->ppmm = ppmm;
    }

/*    Scan ( const Scan &in ) : cv::Mat ( in )
    {
        this->ppmm = in.ppmm;
    } */

    Scan () : cv::Mat ()
    {

    }

    Scan ( const cv::Mat &mat ) : cv::Mat (mat)
    {

    }
};

typedef struct Box
{
    double xmm;
    double ymm;
    double wmm;
    double hmm;
} Box;

class Frame : public cv::Rect
{
    double ppmm = 0;
    int bpp = 2;

    int current_line = 0;

    Scan image;

//    std::string dbgfilename;

    const unsigned char *subbuffer ( const unsigned char *buffer )
    {
        return buffer + ( x * bpp );
    }

public:

    bool running = false;

    Frame ( cv::Rect2d &rectmm, double ppmm )
    {
        x = rectmm.x * ppmm;
        y = rectmm.y * ppmm;
        width = rectmm.width * ppmm;
        height = rectmm.height * ppmm;

        this->ppmm = ppmm;
//        this->filename = filename;

//        dbgfilename = filename;
//        dbgfilename.replace( dbgfilename.find_last_of('.'), 0, "-{}");
    }

    Frame ( const Scan &image )
    {
        this->image = image;
    }

    bool isinto ( int line )
    {
        return (line > y) && (line <= ( y + height ) );
    }

    bool iscomplete ( int line )
    {
        if ( running )
            return false;
        else
            return current_line && ( line > ( y + height ) );
    }

    void addline ( const unsigned char *buffer )
    {
        if ( !current_line )
        {
            image = cv::Mat ( height, width, CV_16UC1 );
            image.ppmm = ppmm;
        }

        const unsigned short *src = (const unsigned short *)subbuffer( buffer );

        for ( int i = 0; i < width; i ++ )
            image.at<unsigned short>( current_line, i ) = src [ i ];

        current_line++;
    }

    const Scan &scan ()
    {
        return image;
    }
};

typedef struct
{
    std::string name;	/* unique device name */
    std::string vendor;	/* device vendor string */
    std::string model;	/* device model name */
    std::string type;	/* device type (e.g., "flatbed scanner") */
}
ScannerDevice;

std::vector <ScannerDevice> getDevices ();

class Scanner
{
public:
    typedef enum
    {
        ENDBV = 0,
        RAW
    }
    OutputMode;

private:
    // Device
    SANE_Handle hDevice = nullptr;
    SANE_Int nOptions = 0;
    const SANE_Option_Descriptor **optDescriptions = nullptr;

    // Scan & preview status
    SANE_Parameters parameters;
    int nCurrentLine = 0;
    SANE_Byte *pScanBuffer = nullptr;

    // Scan status
    std::vector <std::thread> threads;

    // Profile
    Profile profile;

    // Config
    int scanner_debug;
    bool keepFrameBorders = true;
    std::string scanFilmType;
    int scanDPI = 4800;
    int previewDPI = 150;
    OutputMode outputMode = ENDBV;
    int brightness = 0;

private:
    int optindex ( const char *optname );
    void start ( int *maxx = nullptr, int *maxy = nullptr );
    int read ( );

    void guessFrames ( std::vector<cv::Rect2d> &frames, const Scan &preview, const Slot &holder, cv::Mat &dbgOutput );

    void doscan ( std::vector <cv::Rect2d> boxes );
    void doprocess ( Frame &frame, int frameNumber );
    void dopreview ( );

    bool optExist ( const char *name );

    double optGetMaxD ( const char *name );
    int optGetMaxI ( const char *name );
    double optGetMinD ( const char *name );
    int optGetMinI ( const char *name );
    int optType (const char *name);
    int optUnit (const char *name);

    double optSet ( const char *name, double value );
    int optSet ( const char *name, int value );
    std::string optSet ( const char *name, const char *value );

    double optSetFix ( int i, double value );
    int optSetInt ( int i, int value );

    std::string optGetS ( int i );

    typedef enum
    {
        none = 0,
        range,
        intList,
        stringList
    }
    DomainType;

    DomainType getDomainType ( const char *param );

    double optGetD ( const char *name );
    int optGetI ( const char *name );

    double getPpmm ( );
    int getDepth ();
    int getBpp ();
    bool isColor ();

    std::vector<std::string> getDomainS ( const char *param );
    std::vector<int> getDomainI ( const char *param );
    bool isInDomain ( const char *param, const char *value );

private:
    // Callbacks
    virtual void onNewScan ( Scan &image, int frameNumber ) = 0;
    virtual void onScanCompleted () = 0;
    virtual void onPreviewCompleted ( const Scan &preview ) = 0;
    virtual void onProgressUpdate ( const std::string text, int percent ) = 0;
    virtual void onError ( std::string error ) = 0;

public:
    // Scan operations
    void open ( const char *name );
    void close ();

    void preview ( );
    void scan ( std::vector <cv::Rect2d> frames );

    void cancel ();

    // Guess film holder and frames
    std::vector<cv::Rect2d> guessFrames ( const Scan &preview, const std::vector<Slot> &holders );
    std::vector <Slot> guessSlots ( const Scan &preview );

    // Parameters domains
    std::vector<std::string> getFilmTypes ();
    std::vector<int> getDPIs ();

    cv::Size getPreviewSize ();

    // Parameters
    void setDPI ( int dpi );
    void setFilmType ( const std::string &filmType );
    void setProfile ( const Profile &p = Profile () );
    void setDebug ( int debug = true );
    void setMode ( OutputMode mode = ENDBV );
    void setBrightness ( int level );

    void dumpopts ();
};

std::vector<Slot> guessSlots ( const Scan &preview );
std::vector<cv::Rect2d> guessFrames ( const Scan &preview, const std::vector<Slot> &holders );

#endif // SCANNER_H
