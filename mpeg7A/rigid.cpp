
#include <ciso646>
#include <cmath>
#include <cstdlib>
#include <iostream>

#define BOOST_FILESYSTEM_NO_DEPRECATED 1
#include <boost/filesystem.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


/**
 * For each case use the linear transformation:
 *
 *      ^p = A · p + b,     p = [ x ].
 *                              [ y ]
 *
 * For a scaling of factor 's':
 *
 *      A = [  s   0 ],     b = [ 0 ],      ^w = s · w,     ...     (w: width)
 *          [  0   s ]          [ 0 ]       ^h = s · h;     ...     (h: height)
 *
 * where s in { 2, 0.3, 0.25, 0.2, 0.1 }.
 *
 *
 * For a counter-clockwise rotation of 'angle':
 *
 *      A = [  c  s ],      b = [ ^w - c · w - s · h ],     c = cos(angle),
 *          [ -s  c ]           [ ^h + s · w - c · h ]      s = sin(angle).
 *
 *      ^w = | c · w | + | s · h |,     ^h = | s · w | + | c · h |;
 *
 * where angle in { 9, 36, 45, 90, 150 }.
 */


/**
 * Rigid transformations
 */

namespace cv
{

    /**
     * Rotate an image
     */

    void rotate(const Mat& src, Mat& dst, const Size& dsize, const Point2d& centre, 
                const double angle, const int flags = CV_INTER_LINEAR)
    {
        const Mat r = getRotationMatrix2D(centre, angle, 1.0);
        warpAffine(src, dst, r, dsize, flags);
    }

    void rotate(const Mat& src, Mat& dst, const Size& dsize, 
                const double angle, const int flags = CV_INTER_LINEAR)
    {
        const Point2d centre(0.5 * src.cols, 0.5 * src.rows);
        Mat r = getRotationMatrix2D(centre, angle, 1.0);
        if (dsize.width != 0)
            r.at<double>(0,2) += 0.5 * (dsize.width - src.cols);
        else if (dst.cols != 0)
            r.at<double>(0,2) += 0.5 * (dst.cols - src.cols);
        if (dsize.height != 0)
            r.at<double>(1,2) += 0.5 * (dsize.height - src.rows);
        else if (dst.rows != 0)
            r.at<double>(1,2) += 0.5 * (dst.rows - src.rows);
        warpAffine(src, dst, r, dsize, flags);
    }

}


int rigid_image(const boost::filesystem::path& p, 
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
            const path scl_p = q / "scale", 
                       rot_p = q / "rotation";

            const path xt = p.extension();
            path fn = p.filename();
            fn.replace_extension("");
            const string sn = fn.string();
            fn = sn.substr( 0, sn.find_last_of('-') );

            path rot_f = fn;
            rot_f += "-1";
            rot_f.replace_extension(xt);

            cout << "Processing \"" << p << "\"\n Geenerating:\n";

            path scl_f = scl_p / rot_f;
            if ( not exists(scl_f) )
                copy(p, scl_f); // create_hard_link
            cout << "  \"" << scl_f << "\"\n";

            rot_f = rot_p / rot_f;
            if ( not exists(rot_f) )
                copy(p, rot_f); // create_hard_link
            cout << "  \"" << rot_f << "\"\n";

            // Load the image
            Mat src = imread( p.string(), CV_LOAD_IMAGE_ANYDEPTH );

            // The database includes 420 shapes; 70 basic shapes and 5
            // derived shapes from each basic shape by scaling digital
            // images with factors 2, 0.3, 0.25, 0.2, and 0.1.
            const double scale[5] = { 2.00, 0.30, 0.25, 0.20, 0.10 };

            // The database includes 420 shapes; 70 basic shapes and 5 
            // derived shapes from each basic shape by rotation (in 
            // digital domain) with angles: 9, 36, 45 (composed of 9 
            // and 36 degree rotations), 90 and 150 degrees.
            const double angle[5] = { 9.0, 36.0, 45.0, 90.0, 150.0 },
                         sina[5] = {
                             0.15643446504023087, 0.58778525229247314,
                             0.70710678118654746, 1.0, 0.5 };

            // File name suffixes
            const char* fs[5] = { "-2", "-3", "-4", "-5", "-6" };

            // PNG saving options
            vector<int> opt;
            opt.push_back(CV_IMWRITE_PNG_COMPRESSION);
            opt.push_back(9);

            // Create transformed versions
            for (int i = 0; i < 5; ++i)
            {
                // Set file name for scaled figure
                rot_f = fn, rot_f += fs[i], rot_f.replace_extension(xt);
                scl_f = scl_p / rot_f;

                if ( not exists(scl_f) )
                {
                    // Set the dst image the same type as src and scaled size
                    Size scl_size( int(src.cols * scale[i] + 0.5),
                                   int(src.rows * scale[i] + 0.5) );

                    Mat scl = Mat::zeros( scl_size, src.type() );

                    // Scale the image
                    resize(src, scl, scl_size, scale[i], scale[i], 
                           scale[i] > 1 ? CV_INTER_LINEAR : CV_INTER_AREA);

                    // Save the image
                    imwrite( scl_f.string(), scl, opt );
                    cout << "  \"" << scl_f << "\"\n";
                }

                // Set file name for rotated figure
                rot_f = rot_p / rot_f;

                if ( not exists(rot_f) )
                {
                    // Set the dst image the same type as src and rotated size
                    const double sa = sina[i], ca = sqrt(1 - sa * sa);
                    Size rot_size( int(src.cols * ca + src.rows * sa + 0.5) ,
                                   int(src.cols * sa + src.rows * ca + 0.5) );

                    Mat rot = Mat::zeros( rot_size, src.type() );

                    // Rotate the image
                    if (i != 2) // is it a single rotation?
                        rotate(src, rot, rot_size, angle[i], CV_INTER_LINEAR);

                    else // it is a composite rotation!
                    {
                        const double sb = sina[0], cb = sqrt(1 - sb * sb);
                        Size aux_size( int(src.cols * cb + src.rows * sb + 0.5) ,
                                       int(src.cols * sb + src.rows * cb + 0.5) );

                        Mat aux = Mat::zeros( aux_size, src.type() );

                        rotate(src, aux, aux_size, angle[0], CV_INTER_LINEAR);
                        rotate(aux, rot, rot_size, angle[1], CV_INTER_LINEAR);
                    }

                    // Save the image
                    imwrite( rot_f.string(), rot, opt );
                    cout << "  \"" << rot_f << "\"\n";
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
