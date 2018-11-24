#ifndef _REFERENCE_POINTS_H_
#define _REFERENCE_POINTS_H_


#include <NiTE.h>
#include <ros/console.h>

using namespace std;

struct Point{
    float x;
    float y;
    float z;
};

class ReferencePoints
{
private:
    nite::SkeletonJoint head;
    nite::SkeletonJoint neck; //Los hombros son el cuello
    nite::SkeletonJoint torso;
    nite::SkeletonJoint hip;
    nite::SkeletonJoint knee;
    nite::SkeletonJoint rightShoulder;
    nite::SkeletonJoint leftShoulder;

    float error;
    float size_x;

public:
    ReferencePoints();
    ReferencePoints(const nite::UserData& user, const float &size_x);
    ~ReferencePoints();

    void setReferencePoints(const nite::UserData& user, const float &size_x);

    nite::SkeletonJoint getHead() const;
    nite::SkeletonJoint getNeck() const;
    nite::SkeletonJoint getTorso() const;
    nite::SkeletonJoint getHip() const;
    nite::SkeletonJoint getKnee() const;
    nite::SkeletonJoint getRigthShoulder() const;
    nite::SkeletonJoint getLeftShoulder() const;


    float getError() const;
    float getSizeX() const;



};


#endif
