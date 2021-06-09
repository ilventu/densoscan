#include <fmt/format.h>

#include "scanner.h"
#include "frame.h"
#include "spline.h"

#define LMAX        65535.0
#define LUT_SIZE    (LMAX + 1)
typedef unsigned short pixel;

//vector<pixel> getCalibLUT( Profile *profile )
//{
//    vector<pixel> LUT(LUT_SIZE, 0);

//    tk::spline s( profile->measured, profile->expected, tk::spline::cspline );
//    double l, le;
//    for (int i = 0; i < LUT_SIZE; ++i)
//    {
//        l = i / LMAX * 100;
//        le = s ( l ) / 100;
//        if (le < 0)
//            le = 0;

//        if ( le > 1 )
//            le = 1;

//        LUT[i] = le * LMAX;
//    }

//    return LUT;
//}

//vector<pixel> getNormLUT(pixel minValue, pixel maxValue)
//{
//    vector<pixel> LUT(LUT_SIZE, 0);

//    double C = 1;
//    if ( maxValue > minValue )
//        C = LMAX / ( maxValue - minValue );

//    for (int i = 0; i < LUT_SIZE; ++i)
//    {
//        if ( i > minValue && i <= maxValue )
//            LUT[i] = round( (i - minValue) * C);
//        else if ( i > maxValue )
//            LUT[i] = LMAX;
//        else
//            LUT[i] = 0;
//    }

//    return LUT;
//}

//vector<pixel> getDefLUT( )
//{
//    vector<pixel> LUT(LUT_SIZE, 0);

//    for (int i = 0; i < LUT_SIZE; ++i)
//        LUT[i] = i;

//    return LUT;
//}

//vector<pixel> getInvLUT( vector<pixel> &in )
//{
//    vector<pixel> LUT(LUT_SIZE, 0);

//    for (int i = 0; i < LUT_SIZE; ++i)
//        LUT[i] = LMAX - in[i];

//    return LUT;
//}

//vector<pixel> getDensityLUT( vector<pixel> &in )
//{
//  vector<pixel> LUT(LUT_SIZE, 0);

//  for (int i = 0; i < LUT_SIZE; ++i)
//  {
//      if ( in[i] )
//           LUT[i] = log10( LMAX / in[i] ) * 10000;
//      else
//           LUT[i] = LMAX;
//  }

//  return LUT;
//}

//vector<pixel> getLogLUT( vector<pixel> &in )
//{
//    double C = LMAX / log10( LUT_SIZE );

//    vector<pixel> LUT(LUT_SIZE, 0);

//    for (int i = 0; i < LUT_SIZE; ++i)
//        if ( in[i])
//            LUT[i] = round(C * log10( LMAX / in[i] ));
//        else
//            LUT[i] = LMAX;

//    return LUT;
//}

//vector<pixel> getGammaLUT(vector<pixel> &in, double gamma)
//{
//  vector<pixel> LUT(LUT_SIZE, 0);

//  for (int i = 0; i < LUT_SIZE; ++i)
//      LUT[i] = LMAX * pow ( in[i] / LMAX, 1 / gamma );

//  return LUT;
//}

//void processImage(Mat& I, vector<pixel> LUT )
//{
//  pixel pos, neg;
//  for (int i = 0; i < I.rows; ++i)
//  {
//    for (int j = 0; j < I.cols; ++j)
//    {
//      pos = I.at<pixel>(i, j);
//      neg = LUT[pos];
//      I.at<pixel>(i, j) = neg;
//    }
//  }
//}

//// https://www.alglib.net/interpolation/bicubic-spline-interpolation-fitting.php
//// https://subscription.packtpub.com/book/application_development/9781784391454/1/ch01lvl1sec16/logarithmic-transformations
//// https://stackoverflow.com/questions/40527333/gamma-correction-with-pow

//void fill ( Mat &image, int max )
//{
//    int w = image.size().width;
//    int h = image.size().height;

//    Mat img8, mask;
//    if ( image.depth() != CV_8UC1 )
//        image.convertTo(img8, CV_8UC1, 1. / 256.);
//    else
//        img8 = image;

//    int treshold = 32;
//    int x, y;

//    threshold ( img8, mask, treshold, 255, cv::THRESH_BINARY );
//    if ( scanner_debug )
//        imwrite ( "debug-mask.png", mask );

//    for ( x = 0; x < max; x++ )
//    {
//        double sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 250 || avg < 10 )
//            for ( y = 0; y < h; y++ )
//            {
//                image.at<unsigned short>( y, x ) = 0;
//                mask.at<unsigned char>( y, x ) = 0;
//            }
//        else
//            break;
//    }

//    for ( x = w - 1; x > ( w - max ); x-- )
//    {
//        double sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 250 || avg < 10 )
//            for ( y = 0; y < h; y++ )
//            {
//                image.at<unsigned short>( y, x ) = 0;
//                mask.at<unsigned char>( y, x ) = 0;
//            }
//        else
//            break;
//    }

//    for ( y = 0; y < max; y++ )
//    {
//        double sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 250 || avg < 10 )
//            for ( x = 0; x < w; x++ )
//            {
//                image.at<unsigned short>( y, x ) = 0;
//                mask.at<unsigned char>( y, x ) = 0;
//            }
//        else
//            break;
//    }

//    for ( y = h - 1; y > ( h - max ); y-- )
//    {
//        double sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 250 || avg < 10 )
//            for ( x = 0; x < w; x++ )
//            {
//                image.at<unsigned short>( y, x ) = 0;
//                mask.at<unsigned char>( y, x ) = 0;
//            }
//        else
//            break;
//    }

//}

///* void Frame::process ( )
//{
//    int w = image.size().width;
//    int h = image.size().height;

//    /*////////////////////////////////////////
//     *
//     * Parameters
//     *
//     */

//    double targetw_mm = 24.00;
//    double targeth_mm = 36.00;
//    double crop_x100 = 12.5;

//    int targetw_px = round ( targetw_mm * ppmm );
//    int targeth_px = round ( targeth_mm * ppmm );
//    int crop_x100_px = round ( crop_x100 / 100 * std::min( targetw_mm, targeth_mm) * ppmm );

//    int cropw_px = targetw_px - crop_x100_px;
//    int croph_px = targeth_px - crop_x100_px;

//    int dbg_line = round ( 1 + 0.04 * ppmm );
//    double dbg_scale = 1080.0 / std::min ( w, h );

//    /*////////////////////////////////////////
//     *
//     * Check and convert image
//     *
//     */

////    if ( scanner_debug )
////        imwrite ( fmt::format ( dbgfilename, "debug-1-scan"), *image );

//    Mat output;
//    if ( scanner_debug )
//    {
//        Mat img8;
//        image.convertTo(img8, CV_8UC1, 1. / 256.);
//        cv::cvtColor( img8, output, cv::COLOR_GRAY2BGR);
//    }

//    Point center ( w / 2, h / 2);
//    Rect crop ( center.x - cropw_px / 2, center.y - croph_px / 2, cropw_px, croph_px );

//    /*////////////////////////////////////////
//     *
//     * process
//     *
//     */

//    vector<pixel> LUT, LUT1;
//    double minVal, maxVal;

//    // Calibration
//    LUT = getCalibLUT( profile );
//    LUT = getLogLUT( LUT );
//    processImage( *image, LUT );

//    // Min/Max avg 1/10
//    Mat dst;
//    double factor = .25;
//    resize( image(crop), dst, cv::Size(), factor, factor, cv::INTER_LINEAR_EXACT);
//    minMaxLoc(dst, &minVal, &maxVal);

//    LUT = getNormLUT( minVal, maxVal );
//    processImage( *image, LUT );

//    Mat rot;
//    rotate ( mage, rot, ROTATE_90_COUNTERCLOCKWISE);

//    imwrite( filename, rot );

//    if ( scanner_debug )
//    {
//        cv::Scalar dbg_color ( 52, 167, 252 );
//        cv::line(output, Point ( 0, crop.y ), Point ( w, crop.y ), dbg_color, dbg_line );
//        cv::line(output, Point ( 0, crop.y + crop.height ), Point ( w, crop.y + crop.height ), dbg_color, dbg_line );
//        cv::line(output, Point ( crop.x, 0 ), Point ( crop.x, h ), dbg_color, dbg_line );
//        cv::line(output, Point ( crop.x + crop.width, 0 ), Point ( crop.x + crop.width, h ), dbg_color, dbg_line );

//        rotate ( output, rot, ROTATE_90_COUNTERCLOCKWISE);
//        cv::resize( rot, rot, Size (), dbg_scale, dbg_scale, cv::INTER_CUBIC );
//        imwrite ( fmt::format ( dbgfilename, "debug-2-crop" ),  rot );
//    }

//    delete image;
//    image = nullptr;
//}

//void Frame::processold ( )
//{
//    /*////////////////////////////////////////
//     *
//     * Parameters
//     *
//     */

//    double holderedge_mm = 0.05;
//    double maxholdermargin = 3;

//    double targetw_mm = 24.00;
//    double targeth_mm = 36.00;
//    double finalcrop_x100 = 2;

//    int holderedge_px = round ( holderedge_mm * ppmm );
//    int maxholdermargin_px = maxholdermargin * ppmm;

//    int targetw_px = round ( targetw_mm * ppmm );
//    int targeth_px = round ( targeth_mm * ppmm );
//    int finalcropw_px = targetw_px - round ( finalcrop_x100 / 100 * targetw_mm * ppmm );
//    int finalcroph_px = targeth_px - round ( finalcrop_x100 / 100 * targeth_mm * ppmm );

//    int step = round ( 0.2 * ppmm );

//    int dbg_line = round ( 1 + 0.04 * ppmm );

//    /*////////////////////////////////////////
//     *
//     * Check and convert image
//     *
//     */

//    if ( scanner_debug )
//        imwrite ( fmt::format ( dbgfilename, "debug-1-scan"), *image );

//    Mat img8, mask, output;
//    if ( image->depth() != CV_8UC1 )
//        image->convertTo(img8, CV_8UC1, 1. / 256.);
//    else
//        img8 = *image;

//    if ( scanner_debug )
//        cv::cvtColor( img8, output, cv::COLOR_GRAY2BGR);

//    int w = img8.size().width;
//    int h = img8.size().height;

//    /*////////////////////////////////////////
//     *
//     * crop film holder
//     *
//     */

//    int treshold = 16;
//    int x, y;

////    threshold ( img8, mask, treshold * 2, 255, cv::THRESH_TOZERO );
//    threshold ( img8, mask, treshold, 255, cv::THRESH_BINARY );

//    for ( x = 0; x < maxholdermargin_px; x++ )
//    {
//        double sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 127 )
//            break;
//    }

//    int left = x;

//    for ( x = w - 1; x >= ( w - maxholdermargin_px ); x-- )
//    {
//        double sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;

//        if ( avg > 127 )
//            break;
//    }

//    int right = w - x;

//    for ( y = 0; y < maxholdermargin_px; y++ )
//    {
//        double sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / w;

//        if ( avg > 127 )
//            break;
//    }

//    int top = y;

//    for ( y = h - 1; y >= (h - maxholdermargin_px ); y-- )
//    {
//        double sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / w;

//        if ( avg > 127 )
//            break;
//    }

//    int bottom = h - y;

//    Rect r ( left, top, w - left - right, h - top - bottom );
//    if ( scanner_debug )
//    {
//        cv::cvtColor( mask, mask, cv::COLOR_GRAY2BGR);
//        cv::rectangle( mask, r, cv::Scalar( 0, 0, 255), dbg_line );
//        rotate ( mask, mask, ROTATE_90_COUNTERCLOCKWISE);
//        imwrite ( fmt::format ( dbgfilename, "debug-2-holder-mask"), mask );
//    }

//    left += holderedge_px;
//    right += holderedge_px;
//    top += holderedge_px;
//    bottom += holderedge_px;

//    Rect holder_crop ( left, top, w - left - right, h - top - bottom );

//    if ( scanner_debug )
//    {
//        cv::rectangle( output, r, cv::Scalar( 0, 255, 255), dbg_line );
//        cv::rectangle( output, holder_crop, cv::Scalar( 0, 0, 255), dbg_line );
//    }

//    /*////////////////////////////////////////
//     *
//     * crop frame
//     *
//     */

//    Mat blurred;
//    int ksize = ppmm;
//    if ( (ksize % 2 ) != 1 )
//        ksize += 1;

//    GaussianBlur( img8 (holder_crop), blurred, Size ( ksize, ksize ), 0, 0 );
//    double min, max;
//    minMaxIdx( blurred, &min, &max );

//    if ( scanner_debug )
//    {
//        rotate ( blurred, blurred, ROTATE_90_COUNTERCLOCKWISE);
//        imwrite ( fmt::format ( dbgfilename, "debug-3-crop-blurred"), blurred );
//    }

//    mask = img8 (holder_crop).clone();
//    mask -= min;
//    mask *= ( 255 / (max - min) );

//    Mat lookUpTable(1, 256, CV_8U);
//    uchar* p = lookUpTable.ptr();
//    for( int i = 0; i < 256; ++i)
//        p[i] = saturate_cast<uchar>(pow(i / 255.0, 1 ) * 255.0);

//    LUT( mask, lookUpTable, mask);

//    if ( scanner_debug )
//        imwrite ( fmt::format ( dbgfilename, "debug-4-crop-gamma" ), mask );

//    int threshold = 127;
//    mask = mask > threshold;

//    if ( scanner_debug )
//        imwrite ( fmt::format ( dbgfilename, "debug-5-crop-mask" ), mask );

//    top = left = bottom = right = 1;
//    w = mask.size().width;
//    h = mask.size().height;

//    double sum = 0;

///*    while ( ( w - left - right ) > targetw_px )
//    {
//        sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, left );

//        double avgl = sum / h;

//        sum = 0;
//        for ( y = 0; y < h; y++ )
//            sum += mask.at<unsigned char>( y, w - right );

//        double avgr = sum / h;

//        if ( avgr <= threshold && avgl <= threshold )
//            break;

//        if ( avgr > avgl )
//            right += step;
//        else
//            left += step;
//    } */

//    double maxavg = 0;
//    int xmax = 0;

//    for ( x = (w - finalcropw_px ); x > 0; x-- )
//    {
//        sum = 0;
//        for ( y = h - targeth_px; y < targeth_px; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;
//        if ( avg > threshold && avg > maxavg )
//        {
//            maxavg = avg;
//            xmax = x;
//        }
////        if ( avg >  200 )
////            break;
//    }

//    left = xmax;

//    maxavg = 0;
//    xmax = w;

//    for ( x = finalcropw_px; x < w; x++ )
//    {
//        sum = 0;
//        for ( y = h - targeth_px; y < targeth_px; y++ )
//            sum += mask.at<unsigned char>( y, x );

//        double avg = sum / h;
//        if ( avg > threshold && avg > maxavg )
//        {
//            maxavg = avg;
//            xmax = x;
//        }
////        if ( avg > 200 )
////            break;
//    }

//    right = w - xmax;

//    while ( ( h - top - bottom ) > targeth_px )
//    {
//        sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( top, x );

//        double avgt = sum / w;

//        sum = 0;
//        for ( x = 0; x < w; x++ )
//            sum += mask.at<unsigned char>( h - bottom, x );

//        double avgb = sum / h;

//        if ( avgt <= threshold && avgb <= threshold )
//            break;

//        if ( avgt > avgb )
//            top += step;
//        else
//            bottom += step;
//    }

//    Rect frame_precrop ( holder_crop.x + left, holder_crop.y + top, w - left - right, h - top - bottom );
//    Point center = ( frame_precrop.br() + frame_precrop.tl() ) * 0.5;

//    Rect frame_crop;
//    frame_crop.y = center.y - finalcroph_px / 2;
//    frame_crop.x = center.x - finalcropw_px / 2;
//    frame_crop.height = finalcroph_px;
//    frame_crop.width = finalcropw_px;

//    if ( scanner_debug )
//    {
//        cv::line(output, Point ( holder_crop.x + (w - finalcropw_px), 0 ), Point ( holder_crop.x + (w - finalcropw_px ), output.size().height ), cv::Scalar ( 255, 255, 0 ), dbg_line );
//        cv::line(output, Point ( holder_crop.br().x + finalcropw_px, 0 ), Point ( holder_crop.br().x + finalcropw_px, output.size().height ), cv::Scalar ( 255, 255, 0 ), dbg_line );


//        cv::line(output, Point ( 0, holder_crop.y + top ), Point ( output.size().width, holder_crop.y + top ), cv::Scalar ( 255, 0, 255 ), dbg_line );
//        cv::line(output, Point ( 0, holder_crop.br().y - bottom ), Point ( output.size().width, holder_crop.br().y - bottom ), cv::Scalar ( 255, 0, 255 ), dbg_line );
//        cv::line(output, Point ( holder_crop.x + left, 0 ), Point ( holder_crop.x + left, output.size().height ), cv::Scalar ( 255, 0, 255 ), dbg_line );
//        cv::line(output, Point ( holder_crop.br().x - right, 0 ), Point ( holder_crop.br().x - right, output.size().height ), cv::Scalar ( 255, 0, 255 ), dbg_line );
//        cv::rectangle( output, frame_precrop, cv::Scalar( 255, 127, 127), dbg_line );
//        cv::rectangle( output, frame_crop, cv::Scalar( 0, 255, 0), dbg_line );
//        //rotate ( output, output, ROTATE_90_COUNTERCLOCKWISE);
//        imwrite ( fmt::format ( dbgfilename, "debug-6-crop-output" ), output );
//    }

//    vector<pixel> LUT, LUT1;
//    double minVal, maxVal;

//    // Calibration
//    LUT = getCalibLUT( profile );
//    LUT = getLogLUT( LUT );
//    processImage( *image, LUT );

//    // Min/Max avg 1/10
//    Mat dst;
//    double factor = .25;
//    resize( (*image)(frame_crop), dst, cv::Size(), factor, factor, cv::INTER_LINEAR_EXACT);
//    minMaxLoc(dst, &minVal, &maxVal);

//    LUT = getNormLUT( minVal, maxVal );
//    processImage( *image, LUT );


//    Mat rot;
//    rotate ( (*image)(frame_crop), rot, ROTATE_90_COUNTERCLOCKWISE);
//    imwrite( filename, rot );

//    if ( scanner_debug )
//    {
//        rotate ( *image, rot, ROTATE_90_COUNTERCLOCKWISE);
//        imwrite ( fmt::format ( dbgfilename, "debug-7-nocrop" ), rot );
//    }

//    image.release();
//}
//*/
