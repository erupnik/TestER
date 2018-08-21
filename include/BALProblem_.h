#ifndef BALProblem_H
#define BALProblem_H

#include "all.h"


class cPose_
{
    public:
        cPose_();
        ~cPose_(){}

        double*   CP() {return mCP;}
        double*   R()  {return mR;}
        double*   Cal(){return mCal;}

    private:
        double* mCP;//[3]; must be initialized
        double* mR;//[3];
    
        double mCal[5];
        int aNumCal;
};


class BALProblem_
{
    public:
        BALProblem_();
        ~BALProblem_(){}

        const int           PoseNum() {return mPoseNum;}
        const int           PtNum() {return mPtNum;}
        const int           ObservationNum() {return mObsNum;}
        const double* const ObservationIth(const int P) { return mObs + P*2; };
      
        bool ReadBAL   (const std::string&); 
        void WriteToPly(const std::string&);
                                                      
        double* Pose(const int P)        { return mPoses + P*CAMERA_BLOCK_SIZE; }
        double* PoseOfPt(const int P)    { return mPoses + mObsPoseId[P]*CAMERA_BLOCK_SIZE; }
        double* PoseCPOfPt(const int P)  { return mPoses + mObsPoseId[P]*CAMERA_BLOCK_SIZE + 3; }
        double* PoseROfPt(const int P)   { return mPoses + mObsPoseId[P]*CAMERA_BLOCK_SIZE; }
        double* PoseCalOfPt(const int P) { return mPoses + mObsPoseId[P]*CAMERA_BLOCK_SIZE + 6; }
        double* Pt3dOfPt2d(const int P)  { return mPt3d  + mObsPt3dId[P]*POINT_BLOCK_SIZE; }
        double* Pt3d(const int P)        { return mPt3d  + P*POINT_BLOCK_SIZE; }
                                                      

    private:
        
        double*    mObs;
        int*       mObsPoseId;
        int*       mObsPt3dId;

        double*    mPoses;
        double*    mPt3d;

        int mObsNum;
        int mPtNum;
        int mPoseNum;
        int mParamNum;
    
    
        int POINT_BLOCK_SIZE;
        int CAMERA_BLOCK_SIZE;
};
#endif
