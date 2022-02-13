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

#define DEBUG_FRAMES        1
#define DEBUG_HOLDER        2
#define DEBUG_RAWSCAN       4
#define DEBUG_RAWPREVIEW    8
#define DEBUG_PROCESSCROP   16

// https://sane-project.gitlab.io/standard/api.html
// https://subscription.packtpub.com/book/application_development/9781784391454/1/ch01lvl1sec16/logarithmic-transformations

typedef struct
{
    std::vector<double> expected;
    std::vector<double> measured;
} Profile;

typedef struct tagSlotDef
{
    std::string name;
    double width;
    double height;

    double frameW;
    double frameH;

    double frameD;
    double frameN;
} SlotDef;

SlotDef const slotDefs[] = { { "Epson v850 135",       26,   226,  24, 36, 38, 6 },
                             { "Epson v850 120",       58,   200,  56, 56, 62, 3 },
                             { "Penso 135&126",   32,   158,  24, 36, 38, 4 },
                             { "Penso 127",       42,   158,  40, 40, 46, 3 },
                             { "",                 0,     0,   0,  0,  0, 0 }
                           };

class Slot : public cv::Rect2d
{
public:
    double frameW = 0;
    double frameH = 0;

    double frameD = 0;
    double frameN = 1;

    double angle = 0;

public:
    Slot ( cv::Rect rect, double ppmmw, double ppmmh, double ang )
    {
        x = rect.x / ppmmw;
        y = rect.y / ppmmh;
        width = rect.width / ppmmw;
        height = rect.height / ppmmh;
        angle = ang;

        int i = 0;
        double approx = 16 / 100.0;
        while ( slotDefs[i].width )
        {
            if ( width  > ( slotDefs[i].width  - ( slotDefs[i].width  * approx ) ) &&
                 width  < ( slotDefs[i].width  + ( slotDefs[i].width  * approx ) ) &&
                 height > ( slotDefs[i].height - ( slotDefs[i].height * approx ) ) &&
                 height < ( slotDefs[i].height + ( slotDefs[i].height * approx ) ) )
                    break;
            i++;
        }

        if ( slotDefs[i].width )
        {
            frameW = slotDefs[i].frameW;
            frameH = slotDefs[i].frameH;
            frameD = slotDefs[i].frameD;
            frameN = slotDefs[i].frameN;
        }
        else
        {
            frameW = width - ( width * approx );
            frameH = height - ( height * approx );
            frameD = 0;
            frameN = 1;
        }
    }

    cv::Rect px ( double ppmmw, double ppmmh ) const
    {
        return cv::Rect ( x * ppmmw, y * ppmmh, width * ppmmw, height * ppmmh );
    }
};

class Scan : public cv::Mat
{
public:
    double ppmmw = 0;
    double ppmmh = 0;

    cv::Mat &image () { return *this; }

public:
    Scan ( int height, int width, int type, double ppmmw, double ppmmh ) : cv::Mat ( height, width, type )
    {
        this->ppmmw = ppmmw;
        this->ppmmh = ppmmh;
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

class Box : public cv::Rect2d
{
public:
    double targetw_mm;
    double targeth_mm;

    Box ( cv::Rect2d in, Slot s ) : cv::Rect2d (in)
    {
        targeth_mm = s.frameH;
        targetw_mm = s.frameW;
    }

    Box ( cv::Rect2d in ) : cv::Rect2d (in)
    {
        targeth_mm = in.height;
        targetw_mm = in.width;
    }

};

class Frame : public cv::Rect
{
    double ppmmw = 0;
    double ppmmh = 0;
    int bpp = 2;

    int current_line = 0;

    Scan image;

public:
    double targetw_mm;
    double targeth_mm;

    const unsigned char *subbuffer ( const unsigned char *buffer )
    {
        return buffer + ( x * bpp );
    }

public:

    bool running = false;

    Frame ( Box &rectmm, double ppmmw, double ppmmh )
    {
        x = rectmm.x * ppmmw;
        y = rectmm.y * ppmmh;
        width = rectmm.width * ppmmw;
        height = rectmm.height * ppmmh;

        this->ppmmw = ppmmw;
        this->ppmmh = ppmmh;

        targetw_mm = rectmm.targetw_mm;
        targeth_mm = rectmm.targeth_mm;
    }

/*    Frame ( const Scan &image )
    {
        this->image = image;
    } */

    bool isinto ( int line )
    {
        return (line > y) && (line <= ( y + height ) );
    }

    bool iscomplete ( int line )
    {
        if ( running )
            return false;
        else
            return current_line && ( line >= ( y + height ) );
    }

    virtual void addline ( const unsigned char *buffer )
    {
        if ( !current_line )
        {
            image = cv::Mat ( height, width, CV_16UC1 );
            image.ppmmw = ppmmw;
            image.ppmmh = ppmmh;
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

class FullFrame : public Frame
{
    int fromDPI;
    int toDPI;

public:
    FullFrame ( Box &rectmm, double ppmmw, double ppmmh, int fromDPI, int toDPI ) : Frame ( rectmm, ppmmw, ppmmh )
    {
        this->fromDPI = fromDPI;
        this->toDPI = toDPI;
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

typedef enum
{
    automatic = 0,
    autoFullStrip,
    fullScanArea,
    stoufferT2115
}
DetectionMode;

class Scanner
{
public:
    typedef enum
    {
        ENDBV = 0,
        UENDBV,
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
    bool skipBegining = false;
    std::string scanFilmType;
    int scanDPI = 4800;
    int outputDPI = 0;
    cv::InterpolationFlags interpol = cv::INTER_CUBIC;
    int batchScan = 1;
    int previewDPI = 150;
    OutputMode outputMode = UENDBV;
    int brightness = 0;

private:
    int optindex ( const char *optname );
    void start ( int *maxx = nullptr, int *maxy = nullptr );
    int read ( );

    void guessFrames ( std::vector<cv::Rect2d> &frames, const Scan &preview, const Slot &holder, cv::Mat &dbgOutput );
    void guessRegularFrames( std::vector<Box>  &frames, const Scan &preview, const Slot &holder, cv::Mat &dbgOutput);

    void doscan ( std::vector <Box> boxes );
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

    DetectionMode detectionMode = automatic;

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
    void scan ( std::vector <Box> frames );

    void cancel ();

    // Guess film holder and frames
    std::vector<Box> guessFrames ( const Scan &preview, const std::vector<Slot> &holders );
    std::vector <Slot> guessSlots ( const Scan &preview );

    // Parameters domains
    std::vector<std::string> getFilmTypes ();
    std::vector<int> getDPIs ();

    cv::Size getPreviewSize ();

    // Parameters
    void setDPI ( int dpi );
    void setOutputDPI ( int dpi );
    void setInterpolation ( cv::InterpolationFlags i );
    void setBatchScan ( int n );

    void setDetectionMode ( DetectionMode m );
    void setPreviewDPI ( int dpi );
    int getPreviewDPI (  );

    void setDebug ( int debug );
    int getDebug ();
    void setSkipBegining(bool newSkipBegining);
    bool getSkipBegining() const;


    void setFilmType ( const std::string &filmType );
    void setProfile ( const Profile &p = Profile () );
    void setMode ( OutputMode mode = ENDBV );
    void setBrightness ( int level );

    void dumpopts ();
};

std::vector<Slot> guessSlots ( const Scan &preview );
std::vector<cv::Rect2d> guessFrames ( const Scan &preview, const std::vector<Slot> &holders );

#endif // SCANNER_H
