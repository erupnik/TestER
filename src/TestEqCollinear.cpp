#include "TestEqCollinear.h"


/* Application classe managing data (obs, params) taken from ceres */
class BALProblemVec;

/* Classes to manage the solver*/
class cBundleAdj;
class cBundleAdjSimple;

/* Cost function */
class cResidualError;

template <typename T>
void Rot3D(const T* const Angle, Eigen::Matrix<T,3,3>& R);

using namespace ceres;

//typedef Eigen::Vector2d Vec2d;

class cResidualError
{
    public :
        cResidualError(Vec2d& aPtIm) : 
          mPtIm   (aPtIm) {}
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
        bool operator ()(const T* const aAngleT,    
                         const T* const aCPerspT,
                         const T* const aInCal,
                         const T* const aPt3T,
                         T* Residual) const;


        template <typename T>
        bool operator2  (const T* const aAngleT,
                         const T* const aCPerspT,
                         const T* const aInCal,
                         const T* const aPt3T,
                         T* Residual) const;
    
        static CostFunction * Create(Vec2d&);

    private:

        Vec2d mPtIm;

    

    
};

CostFunction * cResidualError::Create(Vec2d& aPtIm)
{
    /* 2- residual
       3- angles
       3- optical center
       5- PPx,PPy,focal,dr1,dr2 
       3- point 3d */

    //return  (new AutoDiffCostFunction<cResidualError,2,3,3,5,3> (new cResidualError(aPtIm)));
    return  (new AutoDiffCostFunction<cResidualError,2,1,1,1,1,1,1,5,1,1,1> (new cResidualError(aPtIm)));
}

/* Parameters submitted one by one */
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
    typedef Eigen::Matrix<T,3,3> Mat3T;
    typedef Eigen::Matrix<T,3,1> Vec3T;
    typedef Eigen::Matrix<T,2,1> Vec2T;

    Vec3T aAngles;
    aAngles[0]=*aWT; 
    aAngles[1]=*aPT; 
    aAngles[2]=*aKT; 

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
}

/* AutoDiff calculated on Eigen matrices */
template <typename T>
bool cResidualError::operator()(const T* const aAngleT,
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
    Residual[0] = mPtIm[0] - aPtImProj[0];
    Residual[1] = mPtIm[1] - aPtImProj[1];

    return true;
}

/* AutoDiff on doubles */
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
    Residual = mPtIm - aPtImProj;


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
        ~cBundleAdjSimple(){}

        virtual void Optimize();


    private:    

        //allocate the solver with observations/parameters 
        virtual void BuildProblem();
        
        virtual void SetCeresOptions();//see bundle_adjuster.cc for setting options
        void SetMinimizer();
        void SetOrdering();

        Problem       * mCeresPb;
        BALProblemVec * mBAPb;

        ceres::Solver::Options        mSolOpt;
        ceres::Solver::Summary        mSummary;
};

cBundleAdjSimple::cBundleAdjSimple(BALProblemVec& aBAP) :
    mBAPb(&aBAP)
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

    //otherwise create/add appropriate Parameter blocks
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
    mSolOpt.use_nonmonotonic_steps = true;

    mSolOpt.max_num_iterations = 50;
    mSolOpt.minimizer_progress_to_stdout = true;
    mSolOpt.num_threads = 20;


    mSolOpt.sparse_linear_algebra_library_type = SUITE_SPARSE; //there exist two others
    //options->use_inner_iterations ?
}

void cBundleAdjSimple::SetCeresOptions()
{
    SetMinimizer();
    SetOrdering();
}

void cBundleAdjSimple::Optimize()
{
    BuildProblem();

    SetCeresOptions();

    ceres::Solve(mSolOpt,mCeresPb,&mSummary);
}

void cBundleAdjSimple::BuildProblem()
{

    /* create cost function per observation and AddResidualBlock */
    for (int aK=0; aK<mBAPb->ObservationNum(); aK++)
    {
        CostFunction* aCostF = cResidualError::Create( mBAPb->ObservationIth(aK) );
        //CostFunction* aCostF = (new AutoDiffCostFunction<cResidualError,2,1,1,1,1,1,1,1,1,1,1,1,1,1,1> 
          //                    (new cResidualError(mBAPb->ObservationIth(aK))));
        LossFunction * aLossF = NULL;// new HuberLoss(1.0);
   

        mCeresPb->AddResidualBlock(aCostF,aLossF,
                                   &mBAPb->PoseROfPt(aK)[0],
                                   &mBAPb->PoseROfPt(aK)[1],
                                   &mBAPb->PoseROfPt(aK)[2],
                                   &mBAPb->PoseCPOfPt(aK)[0],
                                   &mBAPb->PoseCPOfPt(aK)[1],
                                   &mBAPb->PoseCPOfPt(aK)[2],
                                   mBAPb->PoseCalOfPt(aK),
                                   &mBAPb->Pt3dOfPt2d(aK)[0],
                                   &mBAPb->Pt3dOfPt2d(aK)[1],
                                   &mBAPb->Pt3dOfPt2d(aK)[2]);
    }   
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
    
    /* Load the input data */
    BALProblemVec aBAProb; 
    aBAProb.ReadBAL(argv[2]);

    aBAProb.WriteToPly("InputCloud.ply");

    if (0)
    {
    /* Allocate in the solver */
    cBundleAdjSimple aBAS(aBAProb);
    
    /* Solve */
    aBAS.Optimize();

    /* Save to ply */
    //aBAProb.WriteToPLYFile("BAPr-adj.ply"); 
    
    }


    return(1.0);
}
