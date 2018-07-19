#include "TestEqCollinear.h"

/* functions */
//re-projection

/* Cost function */

class ResidualError
{
    public :
        ResidualError(const Eigen::Vector2d& aPtIm) : 
          mPtIm   (aPtIm) {}
        ~ResidualError(){}

        template <typename T>
        bool operator ()(const T* const aAngleT,    
                         const T* const aCPerspT,
                         const T* const aPt3T,
                         const T* const aInCal,
                         T* Residual);


        template <typename T>
        bool operator2  (const T* const aAngleT,
                         const T* const aCPerspT,
                         const T* const aPt3T,
                         const T* const aInCal,
                         T* Residual);


    private:

    const Eigen::Vector2d& mPtIm;

    

    
};

/* AutoDiff calculated on Eigen matrices */
template <typename T>
bool ResidualError::operator()(const T* const aAngleT,
                               const T* const aCPerspT,
                               const T* const aPt3T,
                               const T* const aInCal,
                               T* Residual)
{
    typedef Eigen::Matrix<T,3,1> Mat3T;
    typedef Eigen::Matrix<T,3,1> Vec3T;
  
    Mat3T aRot; 
    Rot3D(aAngleT,aRot);

    Eigen::Map<const Mat3T> aCPersp(aCPerspT);
    Eigen::Map<const Vec3T> aPt3d(aPt3T);
    
    T aPP[2];
    T aFoc = aInCal[0];
    aPP[0] = aInCal[1];
    aPP[1] = aInCal[2];
    T aDR1 = aInCal[3];
    T aDR2 = aInCal[4];

 
    T aPtCam    = aRot * (aPt3d - aCPersp);
    T aPtCamDir (aPtCam.x/aPtCam.z,aPtCam.y/aPtCam.z);

    T aRho2 = aPtCamDir.x.sqrt() + aPtCamDir.y.sqrt();
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir = aDist*aPtCamDir;
     
    T aPtImProj = aFoc*aPtCamDir + aPP;
    Residual = mPtIm - aPtImProj;

    return true;
}

/* AutoDiff on doubles */
template <typename T>
bool ResidualError::operator2(const T* const aAngleT,
                              const T* const aCPerspT,
                              const T* const aPt3T,
                              const T* const aInCal,
                              T* Residual)
{
    T[3][3] aRot;
    Rot3D(aAngleT,aRot);

    T aPP[2];
    T aFoc = aInCal[0];
    aPP[0] = aInCal[1];
    aPP[1] = aInCal[2];
    T aDR1 = aInCal[3];
    T aDR2 = aInCal[4]; 

    T aPtCam;
    RotTr(aRot,aCPerspT,aPt3T,aPtCam);

    T aPtCamDir (aPtCam.x/aPtCam.z,aPtCam.y/aPtCam.z);
    T aRho2 = aPtCamDir.x.sqrt() + aPtCamDir.y.sqrt();
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir = aDist*aPtCamDir;

    T aPtImProj = aFoc*aPtCamDir + aPP;
    Residual = mPtIm - aPtImProj;


    return true;
}


/* Application class */
class cAppliTestEqCollinear 
{
    public:
        cAppliTestEqCollinear();
        ~cAppliTestEqCollinear(){};

        void SetCeresOptions();
        void SetOrdering();
        void BuildCeresProblem();
        
        void LoadData();        

    private:
       ceres::Solver::Summary mSummary;

       //poses
       //calibration
       //points homologues

};


int TestEqCollinear_main(int argc,char ** argv)
{

    /* 
       1/  load data
            + save the MicMac pose in the BAL format
            + use ceres BAL loader
            

       2/  build ceres problem

                * for each observation
                    - cost_function : create cost_function for a given observation and load it with that observation 
                    - camera        : create a double pointer that points to your camera in the parameter vector
                    - point         : create a double pointer that points to your point in question
                    - AddResidualBlock to your ceres problem : cost_function + loss_function + camera + point

       3/  set ceres params
       4/  solve 
       5/  get result */

    /*static ceres::CostFunction* Create(const Eigen::Vector2d& aPtIm)
    {
        return (new ceres::AutoDiffCostFunction<ResidualError,2,3,3,3,5>(
                                            new ResidualError(aPtIm))); 
    }*/

    return(1.0);
};
