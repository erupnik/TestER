#include "BALProblemVec.h"

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
    //int aParNum = 9*mPoseNum;
    for (int aP=0; aP<mPoseNum; aP++)
    {
        mPoses.push_back( new cPose() );  
        
        Vec3d aPose;
        for (int aDim=0; aDim<3; aDim++)
            FileReadOK(fptr, "%lf", &aPose[aDim]);

        mPoses[aP]->R() = aPose; 
        
        for (int aDim=0; aDim<3; aDim++)
            FileReadOK(fptr, "%lf", &aPose[aDim]);

        mPoses[aP]->CP() = aPose; 
        
        double * aCal = new double[3];
        for (int aDim=0; aDim<3; aDim++)
            FileReadOK(fptr, "%lf", aCal + aDim);

        mPoses[aP]->Cal() = aCal; 
    
    }
   

    /* 3D points */

    Vec3d aPt3;
    for (int aPt=0; aPt<mPtNum; aPt++)
    {

        for (int aD=0; aD<3; aD++)
            FileReadOK(fptr, "%lf", &aPt3[aD]);

        mPt3d.push_back( new Vec3d() );
        *(mPt3d[aPt]) << aPt3;

        //mPt3d.push_back( new Vec3d(aPt3) );
    
    //getchar();

        //mPt3d.push_back( new Vec3d(aPt3[0],aPt3[1],aPt3[2]) );

    }
    fclose(fptr);
}

Vec3d& BALProblemVec::PoseCPOfPt(const int i)
{
    return mPoses.at(mVecObsPoseId.at(i))->CP();
}

Vec3d& BALProblemVec::PoseROfPt(const int i)
{
    return mPoses.at(mVecObsPoseId.at(i))->R();
}

double*& BALProblemVec::PoseCalOfPt(const int i)
{
    return mPoses.at(mVecObsPoseId.at(i))->Cal();
}

Vec3d& BALProblemVec::Pt3dOfPt2d(const int i)
{
    return *(mPt3d.at(mVecObsPt3dId.at(i)));
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
            aFp << mPoses[aK]->CP()[aD] << ' ' ;
        }
        aFp << "255 0 0\n"; 
    }    



    /* 3D points */
    for (int aK=0; aK<mPtNum; aK++)
    {
        for (int aD=0; aD<3; aD++)
        {
            aFp << (*mPt3d[aK])[aD] << ' ' ;

        }
        aFp << "0 255 0\n"; 
    }    
    aFp.close();

}
