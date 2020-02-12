#include "BALProblemVec.h"

/*BALProblem_::BALProblem_() : 
    POINT_BLOCK_SIZE(3),
    CAMERA_BLOCK_SIZE(11)
{}*/

/*bool BALProblem_::ReadBAL(const std::string& aName)
{
    std::cout << "BALProblemVec::ReadBAL " << aName << "\n";
    FILE* fptr = fopen(aName.c_str(), "r");
    if (fptr == NULL) 
      return false;

    FileReadOK(fptr, "%d", &mPoseNum);
    FileReadOK(fptr, "%d", &mPtNum);
    FileReadOK(fptr, "%d", &mObsNum);
   
    std::cout << " +poses  : " << mPoseNum << " \n" ;
    std::cout << " +3D pts : " << mPtNum << " \n" ;
    std::cout << " +obs    : " << mObsNum << " \n" ;

    mObsPoseId = new int[mObsNum];
    mObsPt3dId = new int[mObsNum];
    mObs       = new double[2*mObsNum];
    mPoses     = new double[CAMERA_BLOCK_SIZE*mPoseNum];
    mPt3d      = new double[POINT_BLOCK_SIZE*mPtNum];

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
   

    for (int aP=0; aP<mPtNum; aP++)
    {

        for (int aD=0; aD<POINT_BLOCK_SIZE; aD++)
            FileReadOK(fptr, "%lf", mPt3d + aP*POINT_BLOCK_SIZE + aD);


    }
    fclose(fptr);
}

void BALProblem_::WriteToPly(const std::string& aName)
{
    std::cout << "BALProblemVec::WriteToPly " << aName << "\n";
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

    for (int aK=0; aK<mPoseNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << *(mPoses + aK*CAMERA_BLOCK_SIZE + 3 + aD) << ' ' ;
        }
        aFp << "255 0 0\n"; 
    }    


    for (int aK=0; aK<mPtNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << *(mPt3d + aK*POINT_BLOCK_SIZE + aD)  << ' ' ;

        }
        aFp << "0 255 0\n"; 
    }    
    aFp.close();

}*/

cPose::cPose()
{}

BALProblemVec::BALProblemVec()
{}

bool BALProblemVec::ReadBAL(const std::string& aName)
{
    std::cout << "BALProblemVec::ReadBAL " << aName << "\n";
    FILE* fptr = fopen(aName.c_str(), "r");
    if (fptr == NULL) 
      return false;

    FileReadOK(fptr, "%d", &mPoseNum);
    FileReadOK(fptr, "%d", &mPtNum);
    FileReadOK(fptr, "%d", &mObsNum);
   
    std::cout << " +poses  : " << mPoseNum << " \n" ;
    std::cout << " +3D pts : " << mPtNum << " \n" ;
    std::cout << " +obs    : " << mObsNum << " \n" ;


    /* Observations 2D */
    int   aCamId,aPtId;
    Vec2d aPt;
    for (int aK=0; aK<mObsNum; aK++)    
    {

        FileReadOK(fptr, "%d", &aCamId);
        FileReadOK(fptr, "%d", &aPtId);
    
    
        for (int aDim=0; aDim<2; aDim++)
        {
            FileReadOK(fptr, "%lf", &aPt[aDim]);
        }

        mVecObs.push_back(aPt);
        mVecObsPoseId.push_back(aCamId);
        mVecObsPt3dId.push_back(aPtId);
    }


    /* Camera parameters */
    for (int aP=0; aP<mPoseNum; aP++)
    {
        mPoses.push_back( cPose() );  
        
        Vec3d aPose;
        for (int aDim=0; aDim<3; aDim++)
            FileReadOK(fptr, "%lf", &aPose[aDim]);

        mPoses[aP].R() = aPose; 
 
        for (int aDim=0; aDim<3; aDim++)
            FileReadOK(fptr, "%lf", &aPose[aDim]);

        mPoses[aP].CP() = aPose; 
        
        double * aCal = new double[5];
        //focale
        FileReadOK(fptr, "%lf", aCal);
        //PPx, PPy -> for BAL obs are wrt to PP in the middle of the image
        *(aCal+1) = 0.0;
        *(aCal+2) = 0.0;
        //DR1 et DR2 
        FileReadOK(fptr, "%lf", aCal+3);
        FileReadOK(fptr, "%lf", aCal+4);

        mPoses[aP].Cal() = aCal; 
    
    }
   

    /* 3D points */
    Vec3d aPt3;
    for (int aPt=0; aPt<mPtNum; aPt++)
    {

        for (int aD=0; aD<3; aD++)
            FileReadOK(fptr, "%lf", &aPt3[aD]);

        mPt3d.push_back( Vec3d() );
        mPt3d[aPt] << aPt3;


    }
    fclose(fptr);
}

Vec3d* BALProblemVec::PoseCPOfPt(const int i)
{
    return &(mPoses.at(mVecObsPoseId.at(i)).CP());
}

double* BALProblemVec::PoseCPOfPt(const int i,const int d)
{
    return &(mPoses.at(mVecObsPoseId.at(i)).CP()[d]);
}

Vec3d* BALProblemVec::PoseROfPt(const int i)
{
    return &(mPoses.at(mVecObsPoseId.at(i)).R());
}

double* BALProblemVec::PoseROfPt(const int i,const int d)
{
    return &(mPoses.at(mVecObsPoseId.at(i)).R()[d]);
}

double* BALProblemVec::PoseCalOfPt(const int i)
{
    return mPoses.at(mVecObsPoseId.at(i)).Cal();
}

Vec3d* BALProblemVec::Pt3dOfPt2d(const int i)
{
    return &(mPt3d.at(mVecObsPt3dId.at(i)));
}

double* BALProblemVec::Pt3dOfPt2d(const int i,const int d)
{   
    return &(mPt3d[mVecObsPt3dId.at(i)])[d];
}


Vec2d& BALProblemVec::ObservationIth(const int i)
{
    return mVecObs.at(i);
}

void BALProblemVec::WriteToPly(const std::string& aName)
{
    std::cout << "BALProblemVec::WriteToPly " << aName << "\n";
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
            aFp << mPoses[aK].CP()[aD] << ' ' ;
        }
        aFp << "255 0 0\n"; 
    }    



    /* 3D points */
    for (int aK=0; aK<mPtNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << mPt3d[aK][aD] << ' ' ;

        }
        aFp << "0 255 0\n"; 
    }    
    aFp.close();

}
