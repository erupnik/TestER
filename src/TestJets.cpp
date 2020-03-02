#include "TestEqCollinear.h"
#include <chrono>
#include <iomanip> 

static const int SzJetCam=17;
 
 
using namespace ceres;
 
 
 

double RandUnif_0_1()
{
   return ((double) rand() / RAND_MAX );
}

double RandUnif_C()
{
   return (RandUnif_0_1()-0.5) * 2.0;
}

 
class cProjCamRad
{
    public :    

        cProjCamRad(double x, double y);
        void InitFromParams(std::vector<double>& Params);
 
        template <typename Type>
        bool operator()(const Type* parameters, Type* Residual) const;
        
        template <typename Type>
        bool operator()(const Type* Parameter0, const Type* Parameter1, const Type* Parameter2, const Type* Parameter3,
                             const Type* Parameter4, const Type* Parameter5, Type* Residual) const;

        Eigen::Matrix3d  mRotCur;
        double  mPixX;
        double  mPixY;

        double mParameter[SzJetCam];
};

 
static CostFunction * AddCostFn1Block(const double mPixX,const double mPixY)
{
    return  (new AutoDiffCostFunction<cProjCamRad,2,SzJetCam> (new cProjCamRad(mPixX,mPixY)));
}

static CostFunction * AddCostFnNBlocks(const double mPixX,const double mPixY)
{
    return  (new AutoDiffCostFunction<cProjCamRad,2,3,3,3,2,3,3> (new cProjCamRad(mPixX,mPixY)));
}

cProjCamRad::cProjCamRad(double x, double y) :
    mRotCur(Eigen::Matrix3d::Identity()),
    mPixX   (x),
    mPixY   (y)
{
}

void cProjCamRad::InitFromParams(std::vector<double>& aVParam) 
{
     for (int aK=0 ; aK<SzJetCam ; aK++)
        mParameter[aK] =  aVParam[aK];
}


/* 
 *   Cost function to compare jets of MicMac 
*/
template <typename Type>
bool cProjCamRad::operator()(const Type* Parameter, Type* Residual) const
{
    // Ground Coordinates of projected point
    const Type & XTer = Parameter[0];
    const Type & YTer = Parameter[1];
    const Type & ZTer = Parameter[2];

    // Coordinate of camera center
    const Type & C_XCam = Parameter[3];
    const Type & C_YCam = Parameter[4];
    const Type & C_ZCam = Parameter[5];

    // Coordinate of Omega vector coding the unknown "tiny" rotation
    const Type & Wx = Parameter[6];
    const Type & Wy = Parameter[7];
    const Type & Wz = Parameter[8];

    // Coordinate Center of distorstion
    const Type & xCD = Parameter[9];
    const Type & yCD = Parameter[10];

    // Distortions coefficients
    const Type & k2D = Parameter[11];
    const Type & k4D = Parameter[12];
    const Type & k6D = Parameter[13];

    // PP and Focal
    const Type & xPP = Parameter[14];
    const Type & yPP = Parameter[15];
    const Type & zPP = Parameter[16]; // also named as focal

    // Vector P->Cam
    Type  XPC = XTer-C_XCam;
    Type  YPC = YTer-C_YCam;
    Type  ZPC = ZTer-C_ZCam;


    // Coordinate of points in  camera coordinate system, do not integrate "tiny" rotation 
    Type  XCam0 = mRotCur(0,0)*XPC +  mRotCur(1,0)*YPC +  mRotCur(2,0)*ZPC;
    Type  YCam0 = mRotCur(0,1)*XPC +  mRotCur(1,1)*YPC +  mRotCur(2,1)*ZPC;
    Type  ZCam0 = mRotCur(0,2)*XPC +  mRotCur(1,2)*YPC +  mRotCur(2,2)*ZPC;

    // Now "tiny" rotation
    //  Wx      X      Wy * Z - Wz * Y
    //  Wy  ^   Y  =   Wz * X - Wx * Z
    //  Wz      Z      Wx * Y - Wy * X

    //  P =  P0 + W ^ P0 

    Type  XCam = XCam0 + Wy * ZCam0 - Wz * YCam0;
    Type  YCam = YCam0 + Wz * XCam0 - Wx * ZCam0;
    Type  ZCam = ZCam0 + Wx * YCam0 - Wy * XCam0;

    // Projection 

    Type xPi =  XCam/ZCam;
    Type yPi =  YCam/ZCam;


    // Coordinate relative to distorsion center
    Type xC =  xPi-xCD;
    Type yC =  yPi-yCD;
    Type Rho2C = xC*xC + yC*yC;

    // Compute the distorsion
    Type Dist = k2D*Rho2C + k4D * Rho2C*Rho2C + k6D*Rho2C*Rho2C*Rho2C;

    Type xDist =  xPi + xC * Dist;
    Type yDist =  yPi + yC * Dist;

    // Use principal point and focal
    Type xIm =  xPP  + zPP  * xDist;
    Type yIm =  yPP  + zPP  * yDist;

    Residual[0] = xIm - mPixX;
    Residual[1] = yIm - mPixY;

    return true;
}

template <typename Type>
bool cProjCamRad::operator()(const Type* Parameter0, const Type* Parameter1, const Type* Parameter2, const Type* Parameter3,
                             const Type* Parameter4, const Type* Parameter5, Type* Residual) const
{
    // Ground Coordinates of projected point
    const Type & XTer = Parameter0[0];
    const Type & YTer = Parameter0[1];
    const Type & ZTer = Parameter0[2];

    // Coordinate of camera center
    const Type & C_XCam = Parameter1[0];
    const Type & C_YCam = Parameter1[1];
    const Type & C_ZCam = Parameter1[2];

    // Coordinate of Omega vector coding the unknown "tiny" rotation
    const Type & Wx = Parameter2[0];
    const Type & Wy = Parameter2[1];
    const Type & Wz = Parameter2[2];

    // Coordinate Center of distorstion
    const Type & xCD = Parameter3[0];
    const Type & yCD = Parameter3[1];

    // Distortions coefficients
    const Type & k2D = Parameter4[0];
    const Type & k4D = Parameter4[1];
    const Type & k6D = Parameter4[2];

    // PP and Focal
    const Type & xPP = Parameter5[0];
    const Type & yPP = Parameter5[1];
    const Type & zPP = Parameter5[2]; // also named as focal

    // Vector P->Cam
    Type  XPC = XTer-C_XCam;
    Type  YPC = YTer-C_YCam;
    Type  ZPC = ZTer-C_ZCam;


    // Coordinate of points in  camera coordinate system, do not integrate "tiny" rotation 
    Type  XCam0 = mRotCur(0,0)*XPC +  mRotCur(1,0)*YPC +  mRotCur(2,0)*ZPC;
    Type  YCam0 = mRotCur(0,1)*XPC +  mRotCur(1,1)*YPC +  mRotCur(2,1)*ZPC;
    Type  ZCam0 = mRotCur(0,2)*XPC +  mRotCur(1,2)*YPC +  mRotCur(2,2)*ZPC;

    // Now "tiny" rotation
    //  Wx      X      Wy * Z - Wz * Y
    //  Wy  ^   Y  =   Wz * X - Wx * Z
    //  Wz      Z      Wx * Y - Wy * X

    //  P =  P0 + W ^ P0 

    Type  XCam = XCam0 + Wy * ZCam0 - Wz * YCam0;
    Type  YCam = YCam0 + Wz * XCam0 - Wx * ZCam0;
    Type  ZCam = ZCam0 + Wx * YCam0 - Wy * XCam0;

    // Projection 

    Type xPi =  XCam/ZCam;
    Type yPi =  YCam/ZCam;


    // Coordinate relative to distorsion center
    Type xC =  xPi-xCD;
    Type yC =  yPi-yCD;
    Type Rho2C = xC*xC + yC*yC;

    // Compute the distorsion
    Type Dist = k2D*Rho2C + k4D * Rho2C*Rho2C + k6D*Rho2C*Rho2C*Rho2C;

    Type xDist =  xPi + xC * Dist;
    Type yDist =  yPi + yC * Dist;

    // Use principal point and focal
    Type xIm =  xPP  + zPP  * xDist;
    Type yIm =  yPP  + zPP  * yDist;

    Residual[0] = xIm - mPixX;
    Residual[1] = yIm - mPixY;

    return true;
}


std::vector<double>    StdParamTestCam(double AmplNoise)
{
    srand (time(NULL));

    std::vector<double> aRes;
 
    /*std::cout <<  RandUnif_C() << "\n";
    std::cout <<  RandUnif_C() << "\n";
    std::cout <<  RandUnif_C() << "\n";
    std::cout <<  RandUnif_C() << "\n";
    std::cout <<  RandUnif_C() << "ENDDDD\n";*/

    aRes.push_back(0.0+0.01* RandUnif_C()*AmplNoise);  //X-Gr
    aRes.push_back(0.0+0.01* RandUnif_C()*AmplNoise);  //Y-Gr
    aRes.push_back(1.0+0.01* RandUnif_C()*AmplNoise);  //Z-Gr

    aRes.push_back(0.0+0.01* RandUnif_C()*AmplNoise);  //X-Cam
    aRes.push_back(0.0+0.01* RandUnif_C()*AmplNoise);  //Y-Cam
    aRes.push_back(0.0+0.01* RandUnif_C()*AmplNoise);  //Z-Cam

    aRes.push_back(0.0);   // W-x   Mandotary 000 as it is the complementary rotation
    aRes.push_back(0.0);   // W-y
    aRes.push_back(0.0);   // W-z

    aRes.push_back(0.01 * RandUnif_C()*AmplNoise);   // Centre dist X
    aRes.push_back(0.02 * RandUnif_C()*AmplNoise);   // Centre dist Y


    aRes.push_back(0.01 * RandUnif_C()*AmplNoise);   // K1
    aRes.push_back(0.01 * RandUnif_C()*AmplNoise);   // K2
    aRes.push_back(0.01 * RandUnif_C()*AmplNoise);   // K3


    aRes.push_back(3000 * (1+ 0.01 * RandUnif_C()*AmplNoise));   // PPx
    aRes.push_back(2000 * (1+ 0.01 * RandUnif_C()*AmplNoise));   // PPy
    aRes.push_back(6000 * (1+ 0.01 * RandUnif_C()*AmplNoise));   // PPz / Focale

    return aRes;

}

int TestJets_main(int argc,char** argv)
{
    std::cout << "TestJets_main" << "\n";

    for (int aKGlob=0; aKGlob<10; aKGlob++)
    {
        //create params
        std::vector<double> aVParam = StdParamTestCam(1.0);

        /*for (auto aK : aVParam)
            std::cout << aK << "\n";*/

        //create camera and initfromparams 
        cProjCamRad aCamJet(0.0,0.0);
        aCamJet.InitFromParams(aVParam);

        //create ceres problem
        ceres::Problem::Options CeresOpt;
        CeresOpt.enable_fast_removal = true;
        CeresOpt.disable_all_safety_checks = true;

        ceres::Problem CeresPb(CeresOpt);


        //do loop of AddResidualBlock and calculate time
        int aNbTest=10000;
 
        auto start = std::chrono::steady_clock::now(); 
        for (int aK=0; aK<aNbTest; aK++)
        {  
            LossFunction * aLossF = NULL;// new HuberLoss(1.0); 
            CostFunction* aCostF = AddCostFn1Block(aCamJet.mPixX,aCamJet.mPixY);
            CeresPb.AddResidualBlock(aCostF,aLossF,aCamJet.mParameter);

            /*CostFunction* aCostF = AddCostFnNBlocks(aCamJet.mPixX,aCamJet.mPixY);
            CeresPb.AddResidualBlock(aCostF,aLossF,aCamJet.mParameter,aCamJet.mParameter+3,aCamJet.mParameter+6,
                                                   aCamJet.mParameter+9,aCamJet.mParameter+11,aCamJet.mParameter+14);*/
                                                                   
        }
        auto end = std::chrono::steady_clock::now();
        double duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() / 1000.0;
 

        std::setprecision(15);
        std::cout << std::fixed  << "TimeJetsCeres= "  << duration  << ", NumParam=" << CeresPb.NumParameters() << ", NumResid=" << CeresPb.NumResiduals() <<"\n";
    }

    
    return 1.0;
}