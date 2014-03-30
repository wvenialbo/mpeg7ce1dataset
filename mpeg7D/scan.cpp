
#include <ciso646>
#include <cstdlib>
#include <iostream>

#define BOOST_FILESYSTEM_NO_DEPRECATED 1
#include <boost/filesystem.hpp>


int affine_image(const boost::filesystem::path& p, 
                 const boost::filesystem::path& q);


int scan_file(const boost::filesystem::path& p, const boost::filesystem::path& q)
{
    using namespace boost::filesystem;
    using namespace std;

    int status = EXIT_SUCCESS;

    if ( exists(p) )    // does p actually exist?
    {
        if ( is_regular_file(p) )     // is p a regular file?
        {
            if ( affine_image(p, q) != EXIT_SUCCESS )
                status = EXIT_FAILURE;
        }

        else if ( is_directory(p) )   // is p a directory?
        {
            for ( auto it = directory_iterator(p);   // iterate through directory
                  it != directory_iterator(); ++it )
            {
                if ( is_regular_file(*it) )   // is *it a regular file?   
                {
                    if ( affine_image(*it, q) != EXIT_SUCCESS )
                        status = EXIT_FAILURE;
                }

                else if ( is_directory(*it) )   // is *it a directory?
                {
                    if (scan_file(*it, q) != EXIT_SUCCESS)
                        status = EXIT_FAILURE;
                }

                else    // *it is neither a regular file nor a directory!
                {
                    clog << *it << " exists, but is neither a regular file nor a directory\n";
                    status = EXIT_FAILURE;
                }
            }
        }

        else    // p is neither a regular file nor a directory!
        {
            clog << p << " exists, but is neither a regular file nor a directory\n";
            return EXIT_FAILURE;
        }
    }

    else    // p does not exists!
    {
        clog << p << " does not exist\n";
        return EXIT_FAILURE;
    }

    return status;
}
