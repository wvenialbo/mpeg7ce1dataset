
#include <ciso646>
#include <cstdlib>
#include <iostream>

#define BOOST_FILESYSTEM_NO_DEPRECATED 1
#include <boost/filesystem.hpp>

#include <opencv2/highgui/highgui.hpp>


int scan_file(const boost::filesystem::path& p, 
              const boost::filesystem::path& q);


int main(const int argc, const char* argv[])
{
    using namespace boost::filesystem;
    using namespace std;

    if (argc < 3)
    {
        cout << "Usage: mpeg7A <src path> <dst path>\n";
        return EXIT_FAILURE;
    }

    try
    {
        const path p = argv[1];
        const path q = argv[2];

        if ( not exists(p) )    // does p exist?
        {
            cout << p << " does not exist.\n";
            return EXIT_FAILURE;
        }

        while ( not exists(q) )
        {
            cout << q << " does not exist. Do you want to create it? (Y/N): ";

            char c;
            if (cin >> c)
            {
                if (c == 'y' or c == 'Y')
                {
                    create_directories(q);
                    break;
                }
                else if (c == 'n' or c == 'N')
                {
                    break;
                }
            }

            else
            {
                cin.clear(0);
            }
        }

        if ( exists(q) )    // does q exist?
        {
            if ( is_directory(q) )  // is q a directory?
            {
                if ( not exists(p / "skew1") )
                {
                    create_directory(q / "skew1");
                }
                if ( not exists(p / "skew2") )
                {
                    create_directory(q / "skew2");
                }
            }
        }

        else    // q does not exist!
        {
            clog << q << " does not exist\n";
            return EXIT_FAILURE;
        }

        if (scan_file(p, q) != EXIT_SUCCESS)
            return EXIT_FAILURE;
    }

    catch (const filesystem_error& x)
    {
        cerr << "Error: Unhandled filesystem error\n" << x.what() << '\n';
        return EXIT_FAILURE;
    }

    catch (const bad_alloc& x)
    {
        cerr << "Error: Unhandled memory error\n" << x.what() << '\n';
        return EXIT_FAILURE;
    }

    catch (const exception& x)
    {
        cerr << "Error: Unhandled standard exception\n" << x.what() << '\n';
        return EXIT_FAILURE;
    }

    catch (...)
    {
        cerr << "Error: Unhandled unknown exception\n";
        return EXIT_FAILURE;
    }

    return EXIT_SUCCESS;
}
