
#include <ciso646>
#include <cmath>
#include <cstdlib>
#include <iostream>

#define BOOST_FILESYSTEM_NO_DEPRECATED 1
#include <boost/filesystem.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * For direct skewing with offset 's':
 *
 *      A = [  1   s ],     b = [ 0 ],      ^w = w + s · h,     ...     (w: width)
 *          [  s   1 ]          [ 0 ]       ^h = h + s · w;     ...     (h: height)
 *
 * where s in { 0.1, 0.2, 0.3, 0.5, 0.7 }. With the linear transformation:
 *
 *      ^p = A · p + b,     p = [ x ].
 *                              [ y ]
 *
 *
 * For reverse skewing with offset 's':
 *
 *      A1 = [  1  0 ],     A2 = [  1  s ],     b = [ 0 ],
 *           [  0 -1 ]           [ -s -1 ]          [ h ]
 *
 * With the linear transformation:
 *
 *      ^p = A2 · ( A1 · p + b ) + b.
 */


/**
 * Affine transformations
 */

namespace cv
{

#define CV_FLIP_HORIZONTAL  1
#define CV_FLIP_VERTICAL    0
#define CV_FLIP_BOTH       -1

    /**
     * Skew an image
     */

    void skew1(const Mat& src, Mat& dst, const Size& dsize,
               const double sx, const double sy, int flags = CV_INTER_LINEAR)
    {
        Mat skw(2, 3, CV_64FC1);
        skw.at<double>(0,0) = skw.at<double>(1,1) = 1;
        skw.at<double>(0,1) = sx, skw.at<double>(1,0) = sy;
        skw.at<double>(0,2) = skw.at<double>(1,2) = 0;
        Mat aux1, aux2;
        flip(src, aux1, CV_FLIP_VERTICAL);
        warpAffine(aux1, aux2, skw, dsize, flags);
        flip(aux2, dst, CV_FLIP_VERTICAL);
    }

    void skew2(const Mat& src, Mat& dst, const Size& dsize,
               const double sx, const double sy, int flags = CV_INTER_LINEAR)
    {
        Mat skw(2, 3, CV_64FC1);
        skw.at<double>(0,0) = skw.at<double>(1,1) = 1;
        skw.at<double>(0,1) = sx, skw.at<double>(1,0) = sy;
        skw.at<double>(0,2) = skw.at<double>(1,2) = 0;
        warpAffine(src, dst, skw, dsize, flags);
    }

}


int affine_image(const boost::filesystem::path& p, 
                 const boost::filesystem::path& q)
{
    using namespace boost::filesystem;
    using namespace cv;
    using namespace std;

    int status = EXIT_SUCCESS;

    if ( exists(p) )  // does p actually exist?
    {
        if ( is_regular_file(p) )   // is p a regular file? 
        {
            // Get the base filename for output files.
            const path skv_p = q / "skew1", 
                       skw_p = q / "skew2";

            const path xt = p.extension();
            path fn = p.filename();
            fn.replace_extension("");
            const string sn = fn.string();
            fn = sn.substr( 0, sn.find_last_of('-') );

            path skw_f = fn;
            skw_f += "-1";
            skw_f.replace_extension(xt);

            cout << "Processing \"" << p << "\"\n Geenerating:\n";

            path skv_f = skv_p / skw_f;
            if ( not exists(skv_f) )
                copy(p, skv_f); // create_hard_link
            cout << "  \"" << skv_f << "\"\n";

            skw_f = skw_p / skw_f;
            if ( not exists(skw_f) )
                copy(p, skw_f); // create_hard_link
            cout << "  \"" << skw_f << "\"\n";

            // Load the image
            const Mat src = imread( p.string(), CV_LOAD_IMAGE_ANYDEPTH );

            // The database includes 420 shapes; 70 basic shapes and 5 
            // derived shapes from each basic shape by skewing (in 
            // digital domain) with offsets: 0.1, 0.2, 0.3, 0.5 and 0.7.
            const double offset[5] = { 0.1, 0.2, 0.3, 0.5, 0.7 };

            // File name suffixes
            const char* fs[5] = { "-2", "-3", "-4", "-5", "-6" };

            // PNG saving options
            vector<int> opt;
            opt.push_back(CV_IMWRITE_PNG_COMPRESSION);
            opt.push_back(9);

            // Create transformed versions
            for (int i = 0; i < 5; ++i)
            {
                // Set file name for (direct) skewed figure
                skw_f = fn, skw_f += fs[i], skw_f.replace_extension(xt);
                skv_f = skv_p / skw_f;

                if (not exists(skv_f))
                {
                    // Set the dst image the same type as src and scaled size
                    Size skv_size( int(src.cols + src.rows * offset[i] + 0.5),
                                   int(src.rows + src.cols * offset[i] + 0.5) );

                    Mat skv = Mat::zeros( skv_size, src.type() );

                    // Skew the image
                    skew1(src, skv, skv_size, offset[i], offset[i], CV_INTER_LINEAR);

                    // Save the image
                    imwrite( skv_f.string(), skv, opt );
                    cout << "  \"" << skv_f << "\"\n";
                }

                // Set file name for (reverse) skewed figure
                skw_f = skw_p / skw_f;

                if (not exists(skw_f))
                {
                    // Set the dst image the same type as src and scaled size
                    Size skw_size( int(src.cols + src.rows * offset[i] + 0.5),
                                   int(src.rows + src.cols * offset[i] + 0.5) );

                    Mat skw = Mat::zeros( skw_size, src.type() );

                    // Skew the image
                    skew2(src, skw, skw_size, offset[i], offset[i], CV_INTER_LINEAR);

                    // Save the image
                    imwrite( skw_f.string(), skw, opt );
                    cout << "  \"" << skw_f << "\"\n";
                }
            }
        }

        else    // p is not a regular file!
        {
            clog << p << " exists, but is not a regular file\n";
            status = EXIT_FAILURE;
        }
    }

    else    // p does not exists!
    {
        clog << p << " does not exist\n";
        return EXIT_FAILURE;
    }

    return status;
}
