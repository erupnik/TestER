#include "all.h"


extern int TestEqCollinear_main(int argc, char** argv);
extern int TestGraphBA_main(int argc, char** argv);


void PrintHelp(std::string aArg)
{
    std::cerr << "Usage: " << aArg << " "
              << "Options:\n"
              << "\t-h,--help\tShow this help message\n"
              << "\tGBA \tfor graph tests;\n"
              << "\tEQCOL\tfor tests on collinearity equation"
              << std::endl;    
}

int main(int argc, char** argv)
{

    if (argc < 2) 
    {
        PrintHelp(argv[0]);
        return 1;
    }
    


    std::string aArg1 = std::string(argv[1]);


    if ((aArg1 == "-h") || (aArg1 == "--help"))
    {
        std::cout << aArg1 << "\n";
        PrintHelp(argv[0]);
    }
    else if (aArg1 == "GBA")
    {
        std::cout << aArg1 << "\n";
        return(TestGraphBA_main(argc,argv));
        //return(1);
    }
    else if (aArg1 == "EQCOL")
    {
        std::cout << aArg1 << "\n";
        return(TestEqCollinear_main(argc,argv));
    }
    else
    {
        std::cout << "RIENNNNNN" << "\n";
        return true;
    }



    return 1;

}
