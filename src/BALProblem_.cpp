#include "BALProblem_.h"

BALProblem_::BALProblem_() : 
    POINT_BLOCK_SIZE(3),
    CAMERA_BLOCK_SIZE(11)
{
}

bool BALProblem_::ReadBAL(const std::string& aName)
{
    FILE* fptr = fopen(aName.c_str(), "r");
    if (fptr == NULL) 
      return false;

    FileReadOK(fptr, "%d", &mPoseNum);
    FileReadOK(fptr, "%d", &mPtNum);
    FileReadOK(fptr, "%d", &mObsNum);
   
    std::cout << " +poses  : " << mPoseNum << " \n" ;
    std::cout << " +3D pts : " << mPtNum << " \n" ;
    std::cout << " +obs    : " << mObsNum << " \n" ;

    /* Initializaion */
    mObsPoseId = new int[mObsNum];
    mObsPt3dId = new int[mObsNum];
    mObs       = new double[2*mObsNum];
    mPoses     = new double[CAMERA_BLOCK_SIZE*mPoseNum];
    mPt3d      = new double[POINT_BLOCK_SIZE*mPtNum];

    /* Observations 2D */
    int   aCamId,aPtId;
    Vec2d aPt;
    for (int aK=0; aK<mObsNum; aK++)    
    {

        FileReadOK(fptr, "%d", mObsPoseId+aK);
        FileReadOK(fptr, "%d", mObsPt3dId+aK);
    
    
        for (int aDim=0; aDim<2; aDim++)
        {
            FileReadOK(fptr, "%lf", mObs+aK*2+aDim);
        }

    }


    /* Camera parameters */
    for (int aP=0; aP<mPoseNum; aP++)
    {
        
        for (int aD=0; aD<7; aD++)
            FileReadOK(fptr, "%lf", mPoses + aP*CAMERA_BLOCK_SIZE + aD);

        //PPx, PPy -> for BAL obs are wrt to PP in the middle of the image
        *(mPoses+aP*CAMERA_BLOCK_SIZE+7) = 0.0;
        *(mPoses+aP*CAMERA_BLOCK_SIZE+8) = 0.0;

        //DR1 et DR2 
        FileReadOK(fptr, "%lf", mPoses + aP*CAMERA_BLOCK_SIZE + 9);
        FileReadOK(fptr, "%lf", mPoses + aP*CAMERA_BLOCK_SIZE + 10);

    }
   

    /* 3D points */
    for (int aP=0; aP<mPtNum; aP++)
    {

        for (int aD=0; aD<POINT_BLOCK_SIZE; aD++)
            FileReadOK(fptr, "%lf", mPt3d + aP*POINT_BLOCK_SIZE + aD);


    }
    fclose(fptr);
}

void BALProblem_::WriteToPly(const std::string& aName)
{
    std::ofstream aFp(aName.c_str());

     aFp << "ply"
     << '\n' << "format ascii 1.0"
     << '\n' << "element vertex " << mPtNum + mPoseNum
     << '\n' << "property float x"
     << '\n' << "property float y"
     << '\n' << "property float z"
     << '\n' << "property uchar red"
     << '\n' << "property uchar green"
     << '\n' << "property uchar blue"
     << '\n' << "element face 0"
     << '\n' << "end_header" << std::endl;

    /* Poses */
    for (int aK=0; aK<mPoseNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << *(mPoses + aK*CAMERA_BLOCK_SIZE + 3 + aD) << ' ' ;
        }
        aFp << "255 0 0\n"; 
    }    


    /* 3D points */
    for (int aK=0; aK<mPtNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << *(mPt3d + aK*POINT_BLOCK_SIZE + aD)  << ' ' ;

        }
        aFp << "0 255 0\n"; 
    }    
    aFp.close();

}

