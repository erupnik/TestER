#include "all.h"
#ifndef BALProblemVec_H
#define BALProblemVec_H


class cPose
{
    public:
        cPose();
        ~cPose(){}

        Vec3d&   CP() {return mCP;}
        Vec3d&   R()  {return mR;}
        double*&  Cal(){return mCal;}

    private:
        Vec3d mCP;
        Vec3d mR;
    
        double* mCal;
        int aNumCal;
};


class BALProblemVec
{
    public:
        BALProblemVec();
        ~BALProblemVec(){}

        const int           ObservationNum() {return mObsNum;}
        Vec2d&              ObservationIth(const int);
      
        bool ReadBAL   (const std::string&); 
        void WriteToPly(const std::string&);
 
        Vec3d* PoseCPOfPt(const int);
        double* PoseCPOfPt(const int,const int);
        Vec3d* PoseROfPt(const int);
        double* PoseROfPt(const int,const int);
        double* PoseCalOfPt(const int);
        Vec3d* Pt3dOfPt2d(const int);
        double* Pt3dOfPt2d(const int,const int);


    private:
        std::vector<Vec2d>    mVecObs;
        std::vector<int>      mVecObsPoseId;
        std::vector<int>      mVecObsPt3dId;

        std::vector<cPose>    mPoses;
        std::vector<Vec3d>    mPt3d;

        int mObsNum;
        int mPtNum;
        int mPoseNum;
        int mParamNum;
    

};

#endif
