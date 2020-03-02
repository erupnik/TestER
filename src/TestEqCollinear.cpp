#include "TestEqCollinear.h"


/* Application classes managing data (obs, params) taken from ceres */
class BALProblemVec;//uses Eigen vectors
class BALProblem_;  //uses double* et int*

/* Classes to manage the solver*/
class cBundleAdj;
class cBundleAdjSimple;

/* Cost function */
class cResidualError;

template <typename T>
void Rot3D(const T* const Angle, Eigen::Matrix<T,3,3>& R);

using namespace ceres;

class cResidualError
{
    public :
/*        cResidualError(Vec2d& aPtIm) : 
          mPtImX   (aPtIm[0]),
          mPtImY   (aPtIm[1]) {}*/
        cResidualError(double PtImX,double PtImY) : 
          mPtImX(PtImX),
          mPtImY(PtImY) {}

        ~cResidualError(){}

        template <typename T>
        bool operator()(const T* const aWT,
                        const T* const aPT,
                        const T* const aKT,
                        const T* const aCPX,
                        const T* const aCPY,
                        const T* const aCPZ,
                        const T* const aCal,
                        const T* const aPTerX,
                        const T* const aPTerY,
                        const T* const aPterZ,
                        T* Residual) const;
        
        template <typename T>
        bool operatorMicMac(const T* const aWT,
                        const T* const aPT,
                        const T* const aKT,
                        const T* const aCPX,
                        const T* const aCPY,
                        const T* const aCPZ,
                        const T* const aCal,
                        const T* const aPTerX,
                        const T* const aPTerY,
                        const T* const aPterZ,
                        T* Residual) const;


        template <typename T>
        bool operatorMicMac(const T* const aAngleT,    
                         const T* const aCPerspT,
                         const T* const aInCal,
                         const T* const aPt3T,
                         T* Residual) const;

        template <typename T>
        bool operator()(const T* const aAngleT,    
                         const T* const aCPerspT,
                         const T* const aInCal,
                         const T* const aPt3T,
                         T* Residual) const;

        template <typename T>
        bool operator() (const T* const aPoseT,//angles, persp cent, incal    
                         const T* const aPt3T,
                         T* Residual) const;


        template <typename T>
        bool operator2  (const T* const aAngleT,
                         const T* const aCPerspT,
                         const T* const aInCal,
                         const T* const aPt3T,
                         T* Residual) const;
    
 //       static CostFunction * Create_1ELBLOCKS_VEC(Vec2d&);
        static CostFunction * Create_1ELBLOCKS(const double,const double);
        static CostFunction * Create_3ELBLOCKS(const double,const double);
        static CostFunction * Create_9ELBLOCKS(const double,const double);

    private:

        double mPtImX;
        double mPtImY;
    

    
};

//CostFunction * cResidualError::Create_1ELBLOCKS_VEC(Vec2d& aPtIm)
//{
    /* 2- residual
       3- angles
       3- optical center
       5- PPx,PPy,focal,dr1,dr2 
       3- point 3d */

//    return  (new AutoDiffCostFunction<cResidualError,2,1,1,1,1,1,1,5,1,1,1> (new cResidualError(aPtIm)));
//}

CostFunction * cResidualError::Create_1ELBLOCKS(const double PtImX,const double PtImY)
{

    return  (new AutoDiffCostFunction<cResidualError,2,1,1,1,1,1,1,5,1,1,1> (new cResidualError(PtImX,PtImY)));

}

CostFunction * cResidualError::Create_3ELBLOCKS(const double PtImX,const double PtImY)
{

    return  (new AutoDiffCostFunction<cResidualError,2,3,3,5,3> (new cResidualError(PtImX,PtImY)));

}

CostFunction * cResidualError::Create_9ELBLOCKS(const double PtImX,const double PtImY)
{

    return  (new AutoDiffCostFunction<cResidualError,2,11,3> (new cResidualError(PtImX,PtImY)));

}



/* Bundler M2C : Parameters submitted one by one, operations on doubles */
template <typename T>
bool cResidualError::operator()(const T* const aWT,
                                const T* const aPT,
                                const T* const aKT,
                                const T* const aCPX,
                                const T* const aCPY,
                                const T* const aCPZ,
                                const T* const aCal,
                                const T* const aPTerX,
                                const T* const aPTerY,
                                const T* const aPTerZ,
                                T* Residual) const

{

    T aAngles[3] = {*aWT,*aPT,*aKT};
    T aCP[3]   = {*aCPX,*aCPY,*aCPZ};
    T aPTer[3] = {*aPTerX,*aPTerY,*aPTerZ};


    T aPtCam[3];
    ceres::AngleAxisRotatePoint(aAngles,aPTer,aPtCam);


    aPtCam[0] += aCP[0];
    aPtCam[1] += aCP[1];
    aPtCam[2] += aCP[2];

    T aPP[2];
    T aFoc = aCal[0];
    aPP[0] = aCal[1];
    aPP[1] = aCal[2];
    T aDR1 = aCal[3];
    T aDR2 = aCal[4];


    T aPtCamDir[2] = {-aPtCam[0]/aPtCam[2],
                      -aPtCam[1]/aPtCam[2]};


    T aR2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aR4 = aR2 * aR2;
    T aDist = 1.0 + aR2*aDR1 + aR4*aDR2;

    aPtCamDir[0] = aDist*aPtCamDir[0];
    aPtCamDir[1] = aDist*aPtCamDir[1];

    T aPtImProj[2];
    aPtImProj[0] = aFoc*aPtCamDir[0] + aPP[0];
    aPtImProj[1] = aFoc*aPtCamDir[1] + aPP[1];
    
    Residual[0] = -mPtImX + aPtImProj[0];
    Residual[1] = -mPtImY + aPtImProj[1];

    return true;
}


/* MicMac  M2C : Parameters submitted one by one, operations on doubles */
template <typename T>
bool cResidualError::operatorMicMac(const T* const aWT,
                                const T* const aPT,
                                const T* const aKT,
                                const T* const aCPX,
                                const T* const aCPY,
                                const T* const aCPZ,
                                const T* const aCal,
                                const T* const aPTerX,
                                const T* const aPTerY,
                                const T* const aPTerZ,
                                T* Residual) const

{

    T aAngles[3] = {*aWT,*aPT,*aKT};

    T aRot[3][3];
    Rot3D(aAngles,aRot);

    T aCP[3]   = {*aCPX,*aCPY,*aCPZ};
    T aPTer[3] = {*aPTerX,*aPTerY,*aPTerZ};


    T aPP[2];
    T aFoc = aCal[0];
    aPP[0] = aCal[1];
    aPP[1] = aCal[2];
    T aDR1 = aCal[3];
    T aDR2 = aCal[4];

    T aPtCam[3];
    RotTr(aRot,aCP,aPTer,aPtCam);

    T aPtCamDir[2] = {aPtCam[0]/aPtCam[2],
                      aPtCam[1]/aPtCam[2]};




    T aRho2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir[0] = aDist*aPtCamDir[0];
    aPtCamDir[1] = aDist*aPtCamDir[1];

    T aPtImProj[2];
    aPtImProj[0] = aFoc*aPtCamDir[0] + aPP[0];
    aPtImProj[1] = aFoc*aPtCamDir[1] + aPP[1];
    
    Residual[0] = mPtImX - aPtImProj[0];
    Residual[1] = mPtImY - aPtImProj[1];

    return true;
}


/* Parameters submitted one by one, operations on Eigen vectors */
/*template <typename T>
bool cResidualError::operator()(const T* const aWT,
                                const T* const aPT,
                                const T* const aKT,
                                const T* const aCPX,
                                const T* const aCPY,
                                const T* const aCPZ,
                                const T* const aCal,
                                const T* const aPTerX,
                                const T* const aPTerY,
                                const T* const aPTerZ,
                                T* Residual) const
{
    std::cout << "AddResidualBlock " << "\n";
    typedef Eigen::Matrix<T,3,3> Mat3T;
    typedef Eigen::Matrix<T,3,1> Vec3T;
    typedef Eigen::Matrix<T,2,1> Vec2T;



    getchar();
    std::cout << "AddResidualBlock " << *aWT << " " << *aPT << " " << *aKT << "\n";

    getchar();

    Vec3T aAngles;
    aAngles[0]=*aWT; 
    aAngles[1]=*aPT; 
    aAngles[2]=*aKT; 
    std::cout << "AddResidualBlock " << aAngles[0] << " " << aAngles[1] << " " << aAngles[2] << "\n";
    getchar();

    Mat3T aRot;
    Rot3D(aAngles,aRot);

    Vec3T aCP;
    aCP[0] = *aCPX;
    aCP[1] = *aCPY;
    aCP[2] = *aCPZ;
    Vec3T aPTer;
    aPTer[0] = *aPTerX;
    aPTer[1] = *aPTerY;
    aPTer[2] = *aPTerZ;


    Vec2T aPP;
    T aFoc = aCal[0];
    aPP[0] = aCal[1];
    aPP[1] = aCal[2];
    T aDR1 = aCal[3];
    T aDR2 = aCal[4];

    Vec3T aPtCam    = aRot * (aPTer - aCP);
    Vec2T aPtCamDir (aPtCam[0]/aPtCam[2],aPtCam[1]/aPtCam[2]);

    T aRho2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir = aDist*aPtCamDir;

    Vec2T aPtImProj = aFoc*aPtCamDir + aPP;
    Residual[0] = mPtIm[0] - aPtImProj[0];
    Residual[1] = mPtIm[1] - aPtImProj[1];


    return true;
}*/


/* Bundler M2C : AutoDiff calculated on doubles */
template <typename T>
bool cResidualError::operator()(const T* const aAngleT,
                                const T* const aCPerspT,
                                const T* const aCal,
                                const T* const aPt3T,
                                T* Residual) const
{

    T aPtCam[3];
    ceres::AngleAxisRotatePoint(aAngleT,aPt3T,aPtCam);

    aPtCam[0] += aCPerspT[0];
    aPtCam[1] += aCPerspT[1];
    aPtCam[2] += aCPerspT[2];

    T aPtCamDir[2] = {-aPtCam[0]/aPtCam[2],
                      -aPtCam[1]/aPtCam[2]}; 
   
    T aPP[2];
    T aFoc = aCal[0];
    aPP[0] = aCal[1];
    aPP[1] = aCal[2];
    T aDR1 = aCal[3];
    T aDR2 = aCal[4];


    T aRho2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir[0] = aDist*aPtCamDir[0];
    aPtCamDir[1] = aDist*aPtCamDir[1];

    T aPtImProj[2];
    aPtImProj[0] = aFoc*aPtCamDir[0] + aPP[0];
    aPtImProj[1] = aFoc*aPtCamDir[1] + aPP[1];

    Residual[0] = -mPtImX + aPtImProj[0];
    Residual[1] = -mPtImY + aPtImProj[1];


    return true;
}

template <typename T>
bool cResidualError::operator()(const T* const aPoseT,//angles, persp cent, incal    
                                const T* const aPt3T,
                                T* Residual) const
{

    T aPtCam[3];
    ceres::AngleAxisRotatePoint(aPoseT,aPt3T,aPtCam);

    aPtCam[0] += aPoseT[3];
    aPtCam[1] += aPoseT[4];
    aPtCam[2] += aPoseT[5];

    T aPtCamDir[2] = {-aPtCam[0]/aPtCam[2],
                      -aPtCam[1]/aPtCam[2]}; 
   
    T aPP[2];
    const T& aFoc = aPoseT[6];
    aPP[0] = aPoseT[7];
    aPP[1] = aPoseT[8];
    const T& aDR1 = aPoseT[9];
    const T& aDR2 = aPoseT[10];

    T aRho2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir[0] = aDist*aPtCamDir[0];
    aPtCamDir[1] = aDist*aPtCamDir[1];

    T aPtImProj[2];
    aPtImProj[0] = aFoc*aPtCamDir[0] + aPP[0];
    aPtImProj[1] = aFoc*aPtCamDir[1] + aPP[1];    
    
    Residual[0] = -mPtImX + aPtImProj[0];
    Residual[1] = -mPtImY + aPtImProj[1];

    return true;
}



/* MicMac M2C : AutoDiff calculated on Eigen matrices */
template <typename T>
bool cResidualError::operatorMicMac(const T* const aAngleT,
                                const T* const aCPerspT,
                                const T* const aInCal,
                                const T* const aPt3T,
                                T* Residual) const
{
    typedef Eigen::Matrix<T,3,3> Mat3T;
    typedef Eigen::Matrix<T,3,1> Vec3T;
    typedef Eigen::Matrix<T,2,1> Vec2T;
  
    Mat3T aRot; 
    Rot3D(aAngleT,aRot);

    Eigen::Map<const Vec3T> aCPersp(aCPerspT);
    Eigen::Map<const Vec3T> aPt3d(aPt3T);
    
    Vec2T aPP;
    T aFoc = aInCal[0];
    aPP[0] = aInCal[1];
    aPP[1] = aInCal[2];
    T aDR1 = aInCal[3];
    T aDR2 = aInCal[4];

 
    Vec3T aPtCam    = aRot * (aPt3d - aCPersp);
    Vec2T aPtCamDir (aPtCam[0]/aPtCam[2],aPtCam[1]/aPtCam[2]);

    T aRho2 = aPtCamDir[0] * aPtCamDir[0] + aPtCamDir[1] * aPtCamDir[1];
    T aRho4 = aRho2 * aRho2;
    T aDist = 1.0 + aRho2*aDR1 + aRho4*aDR2;

    aPtCamDir = aDist*aPtCamDir;
     
    Vec2T aPtImProj = aFoc*aPtCamDir + aPP;
    Residual[0] = mPtImX - aPtImProj[0];
    Residual[1] = mPtImY - aPtImProj[1];

    return true;
}

/* MicMac M2C : AutoDiff on doubles */
template <typename T>
bool cResidualError::operator2(const T* const aAngleT,
                               const T* const aCPerspT,
                               const T* const aInCal,
                               const T* const aPt3T,
                               T* Residual) const
{
    T aRot[3][3];
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
    Residual[0] = mPtImX - aPtImProj[0];
    Residual[1] = mPtImY - aPtImProj[1];


    return true;
}


/* General bundle adjustement class */
class cBundleAdj
{
    public:
        cBundleAdj();
        ~cBundleAdj(){}

        virtual void Optimize();
        virtual void BuildProblem();
        virtual void SetCeresOptions();

        
};

cBundleAdj::cBundleAdj(){}

void cBundleAdj::Optimize()
{}

void cBundleAdj::BuildProblem()
{}

void cBundleAdj::SetCeresOptions()
{}


/* Very simple bundle */
class cBundleAdjSimple : public cBundleAdj
{
    public:
        cBundleAdjSimple(BALProblemVec&);
        cBundleAdjSimple(BALProblem_&,int);
        ~cBundleAdjSimple(){}

        virtual void Optimize();


    private:    

        //allocate the solver with observations/parameters 
        virtual void BuildProblem();
        //void BuildProblem_1ELBLOCKS_VEC();
        void BuildProblem_1ELBLOCKS();

        void BuildProblem_3ELBLOCKS();
        void BuildProblem_9ELBLOCKS();
        
        virtual void SetCeresOptions();//see bundle_adjuster.cc for setting options
        void SetMinimizer();
        void SetOrdering();

        Problem       * mCeresPb;
        BALProblemVec * mBAPb;
        BALProblem_   * mBAPb_;

        int MODE;

        ceres::Solver::Options        mSolOpt;
        ceres::Solver::Summary        mSummary;
};

cBundleAdjSimple::cBundleAdjSimple(BALProblemVec& aBAP) :
    mBAPb(&aBAP)
{
}

cBundleAdjSimple::cBundleAdjSimple(BALProblem_& aBAP,int aM) :
    mBAPb_(&aBAP),
    MODE(aM)
{
}

/* Ordering - ordered partitioning of the parameter blocks
              each parameter block belongs to only one group and 
              has an assigned integer that defines its order in the set of groups aka ELIMINATION GROUPS
              Given the ordering, Ceres will make sure that parameter blocks 
              with the lowest numbers will be solved first;    
                
              If Ceres is to choose th order put everything in one EliminationGroup, otherwise use
              ParameterBlockOrdering */
void cBundleAdjSimple::SetOrdering()
{
    //automatically determine ParameterBlock order
    //ceres::Options::ordering_type = ceres::SCHUR;

    //for now only when blocks are group by 9 and 3
    if (MODE==0)
    {
        //otherwise create/add appropriate Parameter blocks
        ceres::ParameterBlockOrdering* ordering =
            new ceres::ParameterBlockOrdering;
  
        // The points come before the cameras.
        for (int aK=0; aK<mBAPb_->PtNum(); aK++)
        {
          ordering->AddElementToGroup(mBAPb_->Pt3d(aK),0);
        }
  
        for (int aK=0; aK<mBAPb_->PoseNum(); aK++) 
        {
          // When using axis-angle, there is a single parameter block for
          // the entire camera.
          ordering->AddElementToGroup(mBAPb_->Pose(aK), 1);
        }
 
        mSolOpt.linear_solver_ordering.reset(ordering);
    }

}

void cBundleAdjSimple::SetMinimizer()
{
    //S - the reduced camera matrix / the Shur complement;
    //uses the SHURR trick; solves S as a dense matrix with Cholesky factorization; i
    //for problems up to several hundreds of cameras 
    //mSolOpt.linear_solver_type = ceres::DENSE_SCHUR; 
    //uses the SHURR trick; solves S as a sparse matrix with Cholesky factorization; 
    mSolOpt.linear_solver_type = ceres::SPARSE_SCHUR; 
    //applies Preconditioned Conjugate Gradients to S; implements inexact step algorithm;
    //choose a precondition, e.g. CLUSTER_JACOBI,CLUSTER_TRIDIAGONAL that exploits camera-point visibility structure
    //mSolopt.linear_solver_type = ceres::ITERATIVE_SHUR;    

    //relaxes the requirement to decrease the obj function at each iter step; 
    //may turn very efficient in the long term;
    //mSolOpt.use_nonmonotonic_steps = true;

    mSolOpt.max_num_iterations = 25;
    mSolOpt.minimizer_progress_to_stdout = true;
    mSolOpt.num_threads = 20;


    //mSolOpt.sparse_linear_algebra_library_type = SUITE_SPARSE; //there exist two others
    //options->use_inner_iterations ?
}

void cBundleAdjSimple::SetCeresOptions()
{
    SetMinimizer();
    SetOrdering();
}


void cBundleAdjSimple::BuildProblem()
{}

/*void cBundleAdjSimple::BuildProblem_1ELBLOCKS_VEC()
{
    std::cout << "cBundleAdjSimple::BuildProblem()\n";
    mCeresPb = new ceres::Problem;

    // create cost function per observation and AddResidualBlock 
    for (int aK=0; aK<mBAPb->ObservationNum(); aK++)
    {

        CostFunction* aCostF = cResidualError::Create_1ELBLOCKS_VEC( mBAPb->ObservationIth(aK) );
        LossFunction * aLossF = NULL;// new HuberLoss(1.0);
   

        mCeresPb->AddResidualBlock(aCostF,aLossF,
                                   &(*mBAPb->PoseROfPt(aK))[0],
                                   &(*mBAPb->PoseROfPt(aK))[1],
                                   &(*mBAPb->PoseROfPt(aK))[2],
                                   &(*mBAPb->PoseCPOfPt(aK))[0],
                                   &(*mBAPb->PoseCPOfPt(aK))[1],
                                   &(*mBAPb->PoseCPOfPt(aK))[2],
                                     mBAPb->PoseCalOfPt(aK),
                                   &(*mBAPb->Pt3dOfPt2d(aK))[0],
                                   &(*mBAPb->Pt3dOfPt2d(aK))[1],
                                   &(*mBAPb->Pt3dOfPt2d(aK))[2]);

    }   
}*/

void cBundleAdjSimple::BuildProblem_1ELBLOCKS()
{
    mCeresPb = new ceres::Problem;

    /* create cost function per observation and AddResidualBlock */
    for (int aK=0; aK<mBAPb_->ObservationNum(); aK++)
    {

        CostFunction* aCostF = cResidualError::Create_1ELBLOCKS( mBAPb_->ObservationIth(aK)[0], mBAPb_->ObservationIth(aK)[1] );
        LossFunction * aLossF = NULL;// new HuberLoss(1.0);
        
        //std::cout << "*** Pose=" << mBAPb_->PoseCPOfPt(aK)[0] << " ,R=" << mBAPb_->PoseROfPt(aK)[0] << " "  << "\n" ;
//        std::cout << "*** Pt=" << mBAPb_->ObservationIth(aK)[0] << " ,R=" << mBAPb_->ObservationIth(aK)[1] << " "  << "\n" ;
        
        mCeresPb->AddResidualBlock(aCostF,aLossF,
                                   mBAPb_->PoseROfPt(aK),
                                   mBAPb_->PoseROfPt(aK)+1,
                                   mBAPb_->PoseROfPt(aK)+2,
                                   mBAPb_->PoseCPOfPt(aK), 
                                   mBAPb_->PoseCPOfPt(aK)+1, 
                                   mBAPb_->PoseCPOfPt(aK)+2, 
                                   mBAPb_->PoseCalOfPt(aK),
                                   mBAPb_->Pt3dOfPt2d(aK),
                                   mBAPb_->Pt3dOfPt2d(aK)+1,
                                   mBAPb_->Pt3dOfPt2d(aK)+2);
                                   

    }
    
}

void cBundleAdjSimple::BuildProblem_3ELBLOCKS()
{
    mCeresPb = new ceres::Problem;

    /* create cost function per observation and AddResidualBlock */
    for (int aK=0; aK<mBAPb_->ObservationNum(); aK++)
    {

        CostFunction* aCostF = cResidualError::Create_3ELBLOCKS( mBAPb_->ObservationIth(aK)[0], mBAPb_->ObservationIth(aK)[1] );
        LossFunction * aLossF = NULL;// new HuberLoss(1.0);
        
        //std::cout << "*** Pose=" << mBAPb_->PoseCPOfPt(aK)[0] << " ,R=" << mBAPb_->PoseROfPt(aK)[0] << " "  << "\n" ;
//        std::cout << "*** Pt=" << mBAPb_->ObservationIth(aK)[0] << " ,R=" << mBAPb_->ObservationIth(aK)[1] << " "  << "\n" ;
        
        mCeresPb->AddResidualBlock(aCostF,aLossF,
                                   mBAPb_->PoseROfPt(aK),
                                   mBAPb_->PoseCPOfPt(aK), 
                                   mBAPb_->PoseCalOfPt(aK),
                                   mBAPb_->Pt3dOfPt2d(aK));
                                   

    }
    
}

void cBundleAdjSimple::BuildProblem_9ELBLOCKS()
{
    mCeresPb = new ceres::Problem;

    for (int aK=0; aK<mBAPb_->ObservationNum(); aK++)
    {
        CostFunction* aCostF = cResidualError::Create_9ELBLOCKS( mBAPb_->ObservationIth(aK)[0], mBAPb_->ObservationIth(aK)[1] );
        LossFunction * aLossF = NULL;// new HuberLoss(1.0);
        
        mCeresPb->AddResidualBlock(aCostF,aLossF,
                                   mBAPb_->PoseOfPt(aK),
                                   mBAPb_->Pt3dOfPt2d(aK));
        
    }

}

void cBundleAdjSimple::Optimize()
{
    if (MODE==0)
    {
        std::cout << "Optimize 9ELBLOCKS (camera block + pt3d block)" << "\n";
        BuildProblem_9ELBLOCKS();
    }
    else if (MODE==1)
    {
        std::cout << "Optimize 3ELBLOCKS" << "\n";
        BuildProblem_3ELBLOCKS();
    }
    else if (MODE==2)
    {
        std::cout << "Optimize 1ELBLOCKS" << "\n";
        BuildProblem_1ELBLOCKS();
    }

    SetCeresOptions();

    ceres::Solve(mSolOpt,mCeresPb,&mSummary);
    std::cout << mSummary.FullReport() << "\n";
}

/* What happens here:
       1/  input : the bundle adjustement input data is kept in class BALProblemVec 
       2/  construct the ceres problem

                * for each observation
                    - create the cost_function for a given observation and load it with that observation 
                    - AddResidualBlock to the ceres problem : cost_function + loss_function + camera + point

       3/  set ceres params
       4/  solve 
       5/  get result */
int TestEqCollinear_main(int argc,char ** argv)
{
    if (argc < 3)
        throw std::invalid_argument("TestER EQCOL : not enough input arguments");
   
    int DO1BLOCK = 0;
    if (argc==4)
    {
        std::string Arg3 = std::string(argv[3]);
        if (Arg3 == "1") 
            DO1BLOCK = 1;
    }
    /* Load the input data */
    BALProblem_ aBAProb; 
    
    aBAProb.ReadBAL(argv[2]);
    aBAProb.WriteToPly("InputCloud.ply");

    /* Allocate in the solver */
    cBundleAdjSimple aBAS(aBAProb,DO1BLOCK);
    
    /* Solve */
    aBAS.Optimize();

    /* Save to ply */
    aBAProb.WriteToPly("OutputCloud.ply"); 
    


    return(1.0);
}
