#include "TestEqCollinear.h"

/* functions */
//re-projection
//class for datset like Appli?


class ResidualErrors
{
    public :
        ResidualErrors(double aPtImX,double aPtImY) : mPtIm(new double[2]) 
        { 
            mPtIm[0] = aPtImX; 
            mPtIm[1] = aPtImY; 
        }

        template <typename T>
        bool operator ()(const T* const CPersp,
                         const T* const Angles,
                         const T* const Point,
                         T* Residuals);


    private:

    double* mPtIm;
    //double mIm;
};

template <typename T>
bool ResidualErrors::operator()(const T* const CPersp,
                                const T* const Angles,
                                const T* const Pt3D,
                                T* Residuals)
{
    
    ceres::MatrixAdapter<T,3,3> aRot;
    Rot3D(Angles,aRot);

    //aRot * (Pt3D - CPersp);
    

    return true;
}

int TestEqCollinear_main(int argc,char ** argv)
{

    return(1.0);
};
