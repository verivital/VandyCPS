
#include <ros/ros.h>
#include <ros/console.h>
#include <cwru_opencv_common/grab_cut.h>

#include <iostream>

using namespace std;
using namespace cv;


namespace cv_grab_cut
{

const Scalar RED = Scalar(0,0,255);
const Scalar PINK = Scalar(230,130,255);
const Scalar BLUE = Scalar(255,0,0);
const Scalar LIGHTBLUE = Scalar(255,255,160);
const Scalar GREEN = Scalar(0,255,0);

const int BGD_KEY = EVENT_FLAG_CTRLKEY;
const int FGD_KEY = EVENT_FLAG_SHIFTKEY;



static void help()
{
    cout << "\nThis program demonstrates GrabCut segmentation -- select an object in a region\n"
            "and then grabcut will attempt to segment it out.\n"
            "Call:\n"
            "./grabcut <image_name>\n"
        "\nSelect a rectangular area around the object you want to segment\n" <<
        "\nHot keys: \n"
        "\tESC - quit the program\n"
        "\tr - restore the original image\n"
        "\tn - next iteration\n"
        "\n"
        "\tleft mouse button - set rectangle\n"
        "\n"
        "\tCTRL+left mouse button - set GC_BGD pixels\n"
        "\tSHIFT+left mouse button - set GC_FGD pixels\n"
        "\n"
        "\tCTRL+right mouse button - set GC_PR_BGD pixels\n"
        "\tSHIFT+right mouse button - set GC_PR_FGD pixels\n" << endl;
}


void GrabCutObject::reset()
{
    if( !mask.empty() )
        mask.setTo(Scalar::all(GC_BGD));
    bgdPxls.clear(); fgdPxls.clear();
    prBgdPxls.clear();  prFgdPxls.clear();

    isInitialized = false;
    rectState = NOT_SET;
    lblsState = NOT_SET;
    prLblsState = NOT_SET;
    iterCount = 0;
}

void GrabCutObject::setImageAndWinName( const Mat& _image, const string& _winName  )
{
    if( _image.empty() || _winName.empty() )
        return;
    image = &_image;
    winName = &_winName;
    mask.create( image->size(), CV_8UC1);
    reset();
}

void GrabCutObject::showImage() const
{
    if( image->empty() || winName->empty() )
        return;

    Mat res;
    Mat binMask;
    if( !isInitialized )
        image->copyTo( res );
    else
    {
        getMask( binMask );
        image->copyTo( res, binMask );
    }

    vector<Point>::const_iterator it;
    for( it = bgdPxls.begin(); it != bgdPxls.end(); ++it )
        circle( res, *it, radius, BLUE, thickness );
    for( it = fgdPxls.begin(); it != fgdPxls.end(); ++it )
        circle( res, *it, radius, RED, thickness );
    for( it = prBgdPxls.begin(); it != prBgdPxls.end(); ++it )
        circle( res, *it, radius, LIGHTBLUE, thickness );
    for( it = prFgdPxls.begin(); it != prFgdPxls.end(); ++it )
        circle( res, *it, radius, PINK, thickness );

    if( rectState == IN_PROCESS || rectState == SET )
        rectangle( res, Point( rect.x, rect.y ), Point(rect.x + rect.width, rect.y + rect.height ), GREEN, 2);

    imshow( *winName, res );
}

void GrabCutObject::setRectInMask()
{
    CV_Assert( !mask.empty() );
    mask.setTo( GC_BGD );
    rect.x = max(0, rect.x);
    rect.y = max(0, rect.y);
    rect.width = min(rect.width, image->cols-rect.x);
    rect.height = min(rect.height, image->rows-rect.y);
    (mask(rect)).setTo( Scalar(GC_PR_FGD) );
}

void GrabCutObject::setLblsInMask( int flags, Point p, bool isPr )
{
    vector<Point> *bpxls, *fpxls;
    uchar bvalue, fvalue;
    if( !isPr )
    {
        bpxls = &bgdPxls;
        fpxls = &fgdPxls;
        bvalue = GC_BGD;
        fvalue = GC_FGD;
    }
    else
    {
        bpxls = &prBgdPxls;
        fpxls = &prFgdPxls;
        bvalue = GC_PR_BGD;
        fvalue = GC_PR_FGD;
    }
    if( flags & BGD_KEY )
    {
        bpxls->push_back(p);
        circle( mask, p, radius, bvalue, thickness );
    }
    if( flags & FGD_KEY )
    {
        fpxls->push_back(p);
        circle( mask, p, radius, fvalue, thickness );
    }
}

void GrabCutObject::mouseClick( int event, int x, int y, int flags, void* )
{
    // TODO add bad args check


    switch( event )
    {
    case EVENT_LBUTTONDOWN: // set rect or GC_BGD(GC_FGD) labels
        {
            bool isb = (flags & BGD_KEY) != 0,
                 isf = (flags & FGD_KEY) != 0;
            if( rectState == NOT_SET && !isb && !isf )
            {
                rectState = IN_PROCESS;
                rect = Rect( x, y, 1, 1 );
            }
            if ( (isb || isf) && rectState == SET )
                lblsState = IN_PROCESS;
        }
        break;
    case EVENT_RBUTTONDOWN: // set GC_PR_BGD(GC_PR_FGD) labels
        {
            bool isb = (flags & BGD_KEY) != 0,
                 isf = (flags & FGD_KEY) != 0;
            if ( (isb || isf) && rectState == SET )
                prLblsState = IN_PROCESS;
        }
        break;
    case EVENT_LBUTTONUP:
        if( rectState == IN_PROCESS )
        {
            rect = Rect( Point(rect.x, rect.y), Point(x,y) );
            rectState = SET;
            setRectInMask();
            CV_Assert( bgdPxls.empty() && fgdPxls.empty() && prBgdPxls.empty() && prFgdPxls.empty() );
            showImage();
        }
        if( lblsState == IN_PROCESS )
        {
            setLblsInMask(flags, Point(x,y), false);
            lblsState = SET;
            showImage();
        }
        break;
    case EVENT_RBUTTONUP:
        if( prLblsState == IN_PROCESS )
        {
            setLblsInMask(flags, Point(x,y), true);
            prLblsState = SET;
            showImage();
        }
        break;
    case EVENT_MOUSEMOVE:
        if( rectState == IN_PROCESS )
        {
            rect = Rect( Point(rect.x, rect.y), Point(x,y) );
            CV_Assert( bgdPxls.empty() && fgdPxls.empty() && prBgdPxls.empty() && prFgdPxls.empty() );
            showImage();
        }
        else if( lblsState == IN_PROCESS )
        {
            setLblsInMask(flags, Point(x,y), false);
            showImage();
        }
        else if( prLblsState == IN_PROCESS )
        {
            setLblsInMask(flags, Point(x,y), true);
            showImage();
        }
        break;
    }
}

int GrabCutObject::getMask(Mat & binMask) const
{
    if( binMask.empty() || binMask.rows!=mask.rows || binMask.cols!=mask.cols ) binMask.create( mask.size(), CV_8UC1 );
    binMask = (mask & 1)*255;
}

int GrabCutObject::nextIter()
{
    if( isInitialized )
        grabCut( *image, mask, rect, bgdModel, fgdModel, 1 );
    else
    {
        if( rectState != SET )
            return iterCount;

        if( lblsState == SET || prLblsState == SET )
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_MASK );
        else
            grabCut( *image, mask, rect, bgdModel, fgdModel, 1, GC_INIT_WITH_RECT );

        isInitialized = true;
    }
    iterCount++;

    bgdPxls.clear(); fgdPxls.clear();
    prBgdPxls.clear(); prFgdPxls.clear();

    return iterCount;
}


static void on_mouse( int event, int x, int y, int flags, void* param )
{
    GrabCutObject* localPtr= static_cast<GrabCutObject*> (param);
    localPtr->mouseClick( event, x, y, flags, NULL );
}



void grabCutDemo(const Mat & srcImage, Mat & maskImage){

  help();
    

    GrabCutObject gcapp;

    const string winName = "grab_cut_image";
    namedWindow( winName, WINDOW_AUTOSIZE );
    setMouseCallback( winName, on_mouse, &gcapp );

    gcapp.setImageAndWinName( srcImage, winName );
    gcapp.showImage();

    for(;;)
    {
        int c = waitKey(0);
        switch( (char) c )
        {
        case '\x1b':
            ROS_INFO_STREAM("Exiting ..." << endl);
            goto exit_main;
        case 'r':
            cout << endl;
            gcapp.reset();
            gcapp.showImage();
            break;
        case 'n':
            int iterCount = gcapp.getIterCount();
            ROS_INFO_STREAM( "<" << iterCount << "... ");
            int newIterCount = gcapp.nextIter();
            if( newIterCount > iterCount )
            {
                gcapp.showImage();
                cout << iterCount << ">" << endl;
            }
            else
                ROS_INFO("rect must be determined>");
            break;
        }
    }
    exit_main:
    {
    gcapp.getMask(maskImage);
    destroyWindow( winName );
    return;
    }


}

};