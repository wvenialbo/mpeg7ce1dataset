
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


struct sin_cos_t
{
    double sin, cos;
};


struct variance_t
{
    double vx, vy, vxy;
};


struct circle_t
{
    cv::Point2f center;
    float radius;
};


struct ctr_info
{
    double area;
    double perimeter;
    double compactness;
    bool isContourConvex;
    cv::Rect boundingRect;
    cv::RotatedRect localFrame;
    cv::RotatedRect minAreaRect;
    cv::RotatedRect fitEllipse;
    circle_t minEnclosingCircle;
    cv::Moments moments;
    variance_t variance;
};


inline
sin_cos_t sin_cos(const double alpha)
{
    double ca = std::cos(alpha), sa;
    if (ca > M_SQRT1_2)
    {
        sa = std::sin(alpha);
        ca = std::sqrt(1 - sa * sa);
    }
    else
        sa = std::sqrt(1 - ca * ca);
    sin_cos_t sc = { sa, ca };
    return sc;
}


inline
double contour_compactness(const double area, const double perimeter)
{
    // Compactness
    return 4 * M_PI * area / (perimeter * perimeter);
}


inline
variance_t contour_variance(const cv::Moments& moments)
{
    // Covariance matrix elements
    const double i00 =  1.0 / moments.m00;
    variance_t var = { moments.mu20 * i00,   // Variance_x
                       moments.mu02 * i00,   // Variance_y
                       moments.mu11 * i00 }; // Covariance_(x,y)
    return var;
}


inline
std::vector< cv::Point > align_contour(std::vector< cv::Point > ctr,
                                         const cv::Point2d& centroid,
                                         const double angle)
{
    //  i) Set the centroid as the origin of coordinates.
    // ii) Derotate the shape by an angle equal to -theta.
    std::vector< cv::Point > pv;
    const cv::Point pc = centroid;
    const sin_cos_t theta = sin_cos(angle);
    auto it = ctr.begin();
    while (it != ctr.end())
    {
        cv::Point pt = *it - pc;
        const double x = pt.x * theta.cos + pt.y * theta.sin;
        const double y = pt.y * theta.cos - pt.x * theta.sin;
        pv.push_back(cv::Point2d(x, y));
        ++it;
    }
    return pv;
}


inline
cv::Moments aligned_moments(const std::vector< cv::Point >& ctr)
{
    // Compute the third order moments about the principal axis.

    cv::Moments moments;
    auto it = ctr.begin(), jt = it++;
    double tm1, tm2;
    moments.m03 = moments.m12 = moments.m21 = moments.m30 = 0;
    while (it != ctr.end())
    {
        tm1 = jt->x * it->y;
        tm2 = it->x * jt->y;
        moments.m30 += jt->x * jt->x * jt->x * tm1
                     - it->x * it->x * it->x * tm2;
        moments.m03 += tm1 * it->y * it->y * it->y
                     - tm2 * jt->y * jt->y * jt->y;
        jt = it++;
    }
    it = ctr.begin();
    tm1 = jt->x * it->y;
    tm2 = it->x * jt->y;
    moments.m30 += jt->x * jt->x * jt->x * tm1
                 - it->x * it->x * it->x * tm2;
    moments.m30 = 0.125 * moments.m30;
    moments.m03 += tm1 * it->y * it->y * it->y
                 - tm2 * jt->y * jt->y * jt->y;
    moments.m03 = 0.125 * moments.m03;

    return moments;
}


inline
cv::RotatedRect contour_frame(const std::vector< cv::Point >& ctr,
                                const cv::Moments& moments,
                                const double perimeter)
{
    cv::RotatedRect rect;

    const double i00 = 1.0 / moments.m00,
                 mu0 = moments.mu20 + moments.mu02;

    // Centroid components
    const cv::Point2d centroid(moments.m10 * i00, moments.m01 * i00);
    rect.center = centroid;

    // Compute the orientation of the shape's local coordinate frame.

    // 1) Determine the direction (theta) of the axis of least inertia,
    //    i.e., the major principal axis.
    const double b = 2 * moments.mu11,
                 a = moments.mu20 - moments.mu02;
    double phi = 0;
    if (a == 0)
    {
        if (b > 0)
            phi = M_PI_4;
        else if (b < 0)
            phi = -M_PI_4;
    }
    else
    {
        phi = 0.5 * std::atan(b / a);
    }
    const sin_cos_t alpha = sin_cos(2.0 * phi);
    const double dI2_dphi2 = 2.0 * (a * alpha.cos + b * alpha.sin);
    double theta = (dI2_dphi2 < 0.0 ? phi + M_PI_2 : phi);

    // 2) Align the shape to the reference frame.
    std::vector< cv::Point > aligned = align_contour(ctr, centroid, theta);

    // 3) Compute the third order moments about the principal axis.
    cv::Moments mpc = aligned_moments(aligned);

    // 4) Determine the semi-positive direction of the major axis.

    // epsilon: zero uncertainty interval, i.e.,
    //          -epsilon < x < +epsilon ==> x := 0.
    const double epsilon = std::sqrt(moments.m00 / M_PI) / 128.0;

    // Invert the current positive axis direction?
    if (-epsilon < mpc.m30 && mpc.m30 < +epsilon)
    {
        // Yes!
        if (mpc.m03 <= -epsilon)
            theta += M_PI;
    }

    // Yes!
    else if (mpc.m30 < 0)
        theta += M_PI;

    // Regularize the orientation angle.
    if (theta <= (-2 * M_PI))
        theta += 2 * M_PI;
    else if (theta >= (2 * M_PI))
        theta -= 2 * M_PI;

    rect.angle = static_cast<float>(theta * (180.0 / M_PI));

    // Compute the elliptic parameters.
    double l0 = std::sqrt(b * b + a * a),
           l1 = 0.5 * (mu0 + l0) * i00,
           l2 = 0.5 * (mu0 - l0) * i00;

    if (l1 < l2)
    {
        l0 = l1;
        l1 = l2;
        l2 = l0;
    }

    const double frame_prm = 2 * M_PI * std::sqrt((l1 * l1 + l2 * l2) / 2),
                 k = perimeter / frame_prm;

    rect.size = cv::Size2f(static_cast<float>(k * l1),
                           static_cast<float>(k * l2));

    return rect;
}


inline
circle_t contour_enclosing_circle(const std::vector< cv::Point >& ctr)
{
    circle_t cir;
    cv::minEnclosingCircle(ctr, cir.center, cir.radius);
    return cir;
}


ctr_info contour_information(const std::vector< cv::Point >& contour)
{
    using namespace cv;

    // Spatial, central, and normalized moments
    const Moments scn_moments = moments(contour);

    const double area  = contourArea(contour),
                 perim = arcLength(contour, true);

    RotatedRect fitEllip;
    
    if (contour.size() >= 5)
        fitEllip = fitEllipse(contour);

    ctr_info data = {
        area, perim,
        contour_compactness(area, perim),
        isContourConvex(contour),
        boundingRect(contour),
        contour_frame(contour, scn_moments, perim),
        minAreaRect(contour),
        fitEllip,
        contour_enclosing_circle(contour),
        scn_moments,
        contour_variance(scn_moments),
    };

    return data;
}


int save_contour(const std::vector< std::vector< cv::Point > >& contours,
                 const std::vector< cv::Vec4i >& hierarchy,
                 const boost::filesystem::path& q)
{
    using namespace boost::filesystem;
    using namespace std;
    using namespace cv;

    path part_p = q.parent_path() / "ctx" / q.filename();

    int status = EXIT_SUCCESS;

    part_p.replace_extension(".part");

    boost::filesystem::ofstream out(part_p, ios_base::binary);

    if (not out.is_open())
        return EXIT_FAILURE;

    std::cout << part_p << '\n';

    out << "<?xml version=\"1.0\" standalone=\"yes\"?>\n"
           "<ctx>\n";

    out << "\t<silhouettes contours=\"" << contours.size()
        << "\" outerContourList=\"";

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

    out << "\">\n";

    for (unsigned i = 0; i < contours.size(); ++i)
    {
        out << "\t\t<contour id=\"" << (i + 1) << '\"';

        const int next = hierarchy[i][0],
                  previous = hierarchy[i][1],
                  child = hierarchy[i][2],
                  parent = hierarchy[i][3];

        if (next >= 0)
            out << " nextSibling=\"" << (next + 1) << '\"';

        if (previous >= 0)
            out << " previousSibling=\"" << (previous + 1) << '\"';

        if (child >= 0)
            out << " firstChild=\"" << (child + 1) << '\"';

        if (parent >= 0)
            out << " parent=\"" << (parent + 1) << '\"';

        out << ">\n";

        const ctr_info info = contour_information(contours[i]);

        out << std::setprecision(17) << std::defaultfloat;

        out << "\t\t\t<shape area=\"" << info.area
            << "\" perimeter=\"" << info.perimeter
            << "\" compactness=\"" << info.compactness
            << "\"  isConvex=\"" << (info.isContourConvex ? "true" : "false")
            << "\" />\n";

        out << "\t\t\t<boundingRect top=\"" << info.boundingRect.tl().y
            << "\" right=\"" << info.boundingRect.br().x
            << "\" bottom=\"" << info.boundingRect.br().y
            << "\" left=\"" << info.boundingRect.tl().x << "\" />\n";

        out << std::setprecision(9);

        out << "\t\t\t<localFrame angle=\"" << info.fitEllipse.angle
            << "\" centre=\"" << info.localFrame.center.x << ' '
                              << info.localFrame.center.y
            << "\" axes=\"" << info.localFrame.size.width << ' '
                            << info.localFrame.size.height << "\" />\n";

        out << "\t\t\t<minAreaRect angle=\"" << info.minAreaRect.angle
            << "\" top=\"" << info.minAreaRect.boundingRect().tl().y
            << "\" right=\"" << info.minAreaRect.boundingRect().br().x
            << "\" bottom=\"" << info.minAreaRect.boundingRect().br().y
            << "\" left=\"" << info.minAreaRect.boundingRect().tl().x
            << "\" />\n";

        if (contours[i].size() >= 5)
        {
            out << "\t\t\t<fitEllipse angle=\"" << info.fitEllipse.angle
                << "\" centre=\"" << info.fitEllipse.center.x << ' '
                                  << info.fitEllipse.center.y
                << "\" axes=\"" << info.fitEllipse.size.width << ' '
                                << info.fitEllipse.size.height << "\" />\n";
        }

        out << "\t\t\t<minCircle radius=\"" << info.minEnclosingCircle.radius
            << "\" centre=\"" << info.minEnclosingCircle.center.x << ' '
                              << info.minEnclosingCircle.center.y << "\" />\n";

        out << std::setprecision(17);

        out << "\t\t\t<rawMoments m00=\"" << info.moments.m00
            << "\" m10=\"" << info.moments.m10
            << "\" m01=\"" << info.moments.m01
            << "\" m20=\"" << info.moments.m20
            << "\" m11=\"" << info.moments.m11
            << "\" m02=\"" << info.moments.m02
            << "\" m30=\"" << info.moments.m30
            << "\" m21=\"" << info.moments.m21
            << "\" m12=\"" << info.moments.m12
            << "\" m03=\"" << info.moments.m03 << "\" />\n";

        out << "\t\t\t<centralMoments mu00=\"" << info.moments.m00
            << "\" mu10=\"0\" mu01=\"0\" mu20=\"" << info.moments.mu20
            << "\" mu11=\"" << info.moments.mu11
            << "\" mu02=\"" << info.moments.mu02
            << "\" mu30=\"" << info.moments.mu30
            << "\" mu21=\"" << info.moments.mu21
            << "\" mu12=\"" << info.moments.mu12
            << "\" mu03=\"" << info.moments.mu03 << "\" />\n";

        out << "\t\t\t<normalMoments nu00=\"1\" nu10=\"0\" "
               "nu01=\"0\" nu20=\"" << info.moments.nu20
            << "\" nu11=\"" << info.moments.nu11
            << "\" nu02=\"" << info.moments.nu02
            << "\" nu30=\"" << info.moments.nu30
            << "\" nu21=\"" << info.moments.nu21
            << "\" nu12=\"" << info.moments.nu12
            << "\" nu03=\"" << info.moments.nu03 << "\" />\n";

        out << "\t\t\t<variance var=\"" << info.variance.vx << ' '
                                        << info.variance.vy
            << "\" cov=\"" << info.variance.vxy << "\" />\n";

        // Chain code: This representation is based upon the work of Freeman.
        //
        //      3  2  1     0 (+1,  0)      3 (-1, -1)      6 ( 0, +1)
        //      4  x  0     1 (+1, -1)      4 (-1,  0)      7 (+1, +1)
        //      5  6  7     2 ( 0, -1)      5 (-1, +1)
        //
        out << "\t\t\t<path vertices=\""
            << contours[i].size()
            << "\" chain=\"";

        auto it = contours[i].begin(),
             et = contours[i].end();

        int code = 0, x = it->x, y = it->y;

        out << x << ' ' << y;

        while (++it != et)
        {
            x = it->x - x, y = it->y - y;
            switch (x)
            {
            case -1 :
                switch (y)
                {
                case -1 :
                    code = 3;
                    break;
                case  0 :
                    code = 4;
                    break;
                case +1 :
                    code = 5;
                    break;
                default :
                    return EXIT_FAILURE;
                }
                break;
            case  0 :
                switch (y)
                {
                case -1 :
                    code = 2;
                    break;
                case  0 :
                    return EXIT_FAILURE;
                case +1 :
                    code = 6;
                    break;
                default :
                    return EXIT_FAILURE;
                }
                break;
            case +1 :
                switch (y)
                {
                case -1 :
                    code = 1;
                    break;
                case  0 :
                    code = 0;
                    break;
                case +1 :
                    code = 7;
                    break;
                default :
                    return EXIT_FAILURE;
                }
                break;
            default :
                return EXIT_FAILURE;
            }
            out << ' ' << code;
            x = it->x, y = it->y;
        }
        
        out << "\" />\n";

        out << "\t\t</contour>\n";
    }

    out << "\t</silhouettes>\n";

    out << "</ctx>";

    out.close();

    path ctx_p = part_p;
    ctx_p.replace_extension(".ctx");
    rename(part_p, ctx_p);
    std::cout << ctx_p << '\n';

    return status;
}


int contour_image(const boost::filesystem::path& p, 
                  const boost::filesystem::path& q)
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
            path ctr_p = q / "ctr" / p.filename();
            ctr_p.replace_extension(".png");

            // Create contour files
            if ( not exists(ctr_p) )
            {
                cout << "Processing \n" << p << "\nGeenerating:\n";

                // Load the image
                src = imread( p.string(), CV_LOAD_IMAGE_ANYDEPTH );

                // Threshold the image
                threshold( src, dst, 0, 255, CV_THRESH_BINARY|CV_THRESH_OTSU );
                //subtract( Scalar::all(255), dst, dst );

                // Extract the contours and store them all as a list
                // (Use CV_RETR_EXTERNAL for outer contour only.)
                vector< vector<Point> > contours;
                vector< Vec4i > hierarchy;
                findContours( dst, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_NONE );

                // Save the contour
                status = save_contour( contours, hierarchy, q / p.filename() );

                if ( status != EXIT_SUCCESS )
                    return EXIT_FAILURE;

                // Draw contours
                dst = Mat::zeros( dst.rows, dst.cols, dst.type() );
                drawContours( dst, contours, -1, Scalar(128,255,255) );

                // PNG saving options
                vector<int> opt;
                opt.push_back(CV_IMWRITE_PNG_COMPRESSION);
                opt.push_back(9);

                // Save the contour image
                imwrite( ctr_p.string(), dst, opt );
                cout << ctr_p << '\n';
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
