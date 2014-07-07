
#include <ciso646>
#define _USE_MATH_DEFINES 1
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <iomanip>

#define BOOST_FILESYSTEM_NO_DEPRECATED 1
#include <boost/filesystem.hpp>
#include <boost/filesystem/fstream.hpp>

#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>


int save_contour(const std::vector< std::vector< cv::Point > >& contours,
                 const std::vector< cv::Vec4i >& hierarchy,
                 const size_t width, const size_t height,
                 const boost::filesystem::path& q)
{
    using namespace boost::filesystem;
    using namespace std;
    using namespace cv;

    path part_p = q;

    int status = EXIT_SUCCESS;

    part_p.replace_extension(".part");

    boost::filesystem::ofstream out(part_p, ios_base::binary);

    if (not out.is_open())
        return EXIT_FAILURE;

    std::cout << part_p << '\n';

    out << "<?xml version=\"1.0\" standalone=\"yes\"?>\n"
           "<ctx>\n";

    out << "\t<canvas width=\"" << width << "\" height=\"" << height
        << "\" />\n";

    out << "\t<silhouette contours=\"" << contours.size();
    if (contours.size() > 1)
    {
        out << "\" outer-contour-list=\"";

        bool notFirst = false;
        for (unsigned i = 0; i < contours.size(); ++i)
        {
            if (hierarchy[i][3] < 0)
            {
                if (notFirst)
                    out << ' ';
                notFirst = true;
                out << (i + 1);
            }
        }
    }
    out << "\">\n";

    for (unsigned i = 0; i < contours.size(); ++i)
    {
        out << "\t\t<contour id=\"" << (i + 1) << '\"';

        const int next = hierarchy[i][0],
                  previous = hierarchy[i][1],
                  child = hierarchy[i][2],
                  parent = hierarchy[i][3];

        if (next >= 0)
            out << " next-sibling=\"" << (next + 1) << '\"';

        if (previous >= 0)
            out << " previous-sibling=\"" << (previous + 1) << '\"';

        if (child >= 0)
            out << " first-child=\"" << (child + 1) << '\"';

        if (parent >= 0)
            out << " parent=\"" << (parent + 1) << '\"';

        out << ">\n";

        const double  p = arcLength(contours[i], true);
        const Moments m = moments(contours[i]);
        const double i00 = 1.0 / m.m00;
        Point2d c(m.m10 * i00, m.m01 * i00);

        out << std::setprecision(17) << std::defaultfloat;

        out << "\t\t\t<shape area=\"" << m.m00
            << "\" perimeter=\"" << p
            << "\" compactness=\"" << (4 * M_PI * m.m00 / (p * p))
            << "\" cx=\"" << c.x << "\" cy=\"" << c.y
            << "\" />\n";

        // Cartesian geometric spatial (raw product about the origin) moments.

        out << "\t\t\t<spatial-moments m00=\"" << m.m00
            << "\" m10=\"" << m.m10
            << "\" m01=\"" << m.m01
            << "\" m20=\"" << m.m20
            << "\" m11=\"" << m.m11
            << "\" m02=\"" << m.m02
            << "\" m30=\"" << m.m30
            << "\" m21=\"" << m.m21
            << "\" m12=\"" << m.m12
            << "\" m03=\"" << m.m03 << "\" />\n";

        // Cartesian geometric moments about the centroid or central moments.
        // Note: mu00 = m00, mu10 = mu01 = 0, hence the values are not stored.

        out << "\t\t\t<central-moments mu20=\"" << m.mu20
            << "\" mu11=\"" << m.mu11
            << "\" mu02=\"" << m.mu02
            << "\" mu30=\"" << m.mu30
            << "\" mu21=\"" << m.mu21
            << "\" mu12=\"" << m.mu12
            << "\" mu03=\"" << m.mu03 << "\" />\n";

        // Normalized central moments or standard moments.
        // Note: nu00 = 1, nu10 = nu01 = 0, hence the values are not stored.

        out << "\t\t\t<normal-moments nu20=\"" << m.nu20
            << "\" nu11=\"" << m.nu11
            << "\" nu02=\"" << m.nu02
            << "\" nu30=\"" << m.nu30
            << "\" nu21=\"" << m.nu21
            << "\" nu12=\"" << m.nu12
            << "\" nu03=\"" << m.nu03 << "\" />\n";

        // 8-connected Freeman chain code.
        //
        // Direction-to-code convention is:
        //
        //      3  2  1     0 (+1,  0)      3 (-1, +1)      6 ( 0, -1)
        //      4  x  0     1 (+1, +1)      4 (-1,  0)      7 (+1, -1)
        //      5  6  7     2 ( 0, +1)      5 (-1, -1)
        //
        // In terms of (delta_x, delta_y) if next pixel compared to the
        // current and converting (dy,dx) pairs to scalar indexes thinking
        // to them as base-3 numbers according to:
        //
        //      i = 3 * (dy+1) + (dx+1) = 3dy + dx + 4
        //
        //      ---------------------------------------
        //      | deltax | deltay | code |  (base-3)  |
        //      |------------------------------------|
        //      |    0   |   +1   |   2  |      7     | 
        //      |    0   |   -1   |   6  |      1     | 
        //      |   -1   |   +1   |   3  |      6     | 
        //      |   -1   |   -1   |   5  |      0     | 
        //      |   +1   |   +1   |   1  |      8     | 
        //      |   +1   |   -1   |   7  |      2     | 
        //      |   -1   |    0   |   4  |      3     |  
        //      |   +1   |    0   |   0  |      5     | 
        //      ---------------------------------------
        //
        static const int cc[3][3] = {
            {  5,  6,  7 },
            {  4, -1,  0 },
            {  3,  2,  1 }
        };

        out << "\t\t\t<path vertices=\""
            << contours[i].size()
            << "\" chain=\"";

        auto it = contours[i].begin(),
             et = contours[i].end();

        int x = it->x,
            y = it->y;

        out << x << ' ' << y;

        while (++it != et)
        {
            const int dx = it->x - x,
                      dy = it->y - y;

            const int code = cc[dy+1][dx+1];

            if (code < 0)
                return EXIT_FAILURE;

            out << ' ' << code;

            x = it->x, y = it->y;
        }
        
        out << "\" />\n";

        out << "\t\t</contour>\n";
    }

    out << "\t</silhouette>\n";

    out << "</ctx>";

    out.close();

    rename(part_p, q);
    std::cout << q << '\n';

    return status;
}


int contour_image(const boost::filesystem::path& p, 
                  const boost::filesystem::path& q,
                  const bool invert)
{
    using namespace boost::filesystem;
    using namespace std;
    using namespace cv;

    int status = EXIT_SUCCESS;

    if ( exists(p) )  // does p actually exist?
    {
        if ( is_regular_file(p) )   // is p a regular file? 
        {
            Mat src, dst;

            // Get the base filename for output files.
            path ctx_p = q / p.filename();
            ctx_p.replace_extension(".ctx");

            // Create contour files
            if ( not exists(ctx_p) )
            {
                cout << "Processing \n" << p << "\nGenerating:\n";

                // Load the image
                src = imread( p.string(), CV_LOAD_IMAGE_ANYDEPTH );

                // Threshold the image
                threshold( src, dst, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU );
                if (invert)
                    subtract( Scalar::all(255), dst, dst );

                // Extract the contours and store them all as a list
                // (Use CV_RETR_EXTERNAL for outer contour only.)
                vector< vector<Point> > contours;
                vector< Vec4i > hierarchy;
                findContours( dst, contours, hierarchy, CV_RETR_TREE,
                              CV_CHAIN_APPROX_NONE );

                // Save the contour
                status = save_contour( contours, hierarchy, src.cols, src.rows,
                                       ctx_p );

                if ( status != EXIT_SUCCESS )
                    return EXIT_FAILURE;
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
