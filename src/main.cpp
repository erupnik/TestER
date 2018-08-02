#include "all.h"


extern int TestEqCollinear_main(int argc, char** argv);
extern int TestGraphBA_main(int argc, char** argv);


void PrintHelp(std::string aArg)
{
    std::cerr << "Usage: " << aArg << " "
              << "Options:\n"
              << "\t-h,--help\tShow this help message\n"
              << "\tGBA \tfor graph tests;\n"
              << "\tEQCOL\tfor tests on collinearity equation\n";
}

void PrintHelpEQCOL(std::string aArg)
{
    std::cerr << "Usage: " << aArg << " \n"   
              << "1st_arg [string] :: the BAL format file\n"
              << "2nd_arg [bool]   :: 1 - cost function on 1ELBlocks, 0 - 3ELBlocks  Def=[false]\n";
}

int main(int argc, char** argv)
{

    if (argc < 2) 
    {
        PrintHelp(argv[0]);
        return 1;
    }
    


    std::string aArg1 = std::string(argv[1]);


    if ((aArg1 == "-h") || (aArg1 == "--help") || (aArg1 == "-help"))
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
        if (argc==3) 
        {
            std::string aArg2 = std::string(argv[2]);
            if ((aArg2 == "-h") || (aArg2 == "--help") || (aArg2 == "-help"))
            {
                PrintHelpEQCOL(argv[1]);
                return 1;
            }
        }
        if (argc==4)
            std::cout << argv[3] << "\n";

        return(TestEqCollinear_main(argc,argv));
    }
    else
    {
        std::cout << "RIENNNNNN" << "\n";
        return true;
    }



    return 1;

}
