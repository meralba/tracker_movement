#include <tracker_movement/ReferencePoints.h>


ReferencePoints::ReferencePoints(){

}

ReferencePoints::ReferencePoints(const nite::UserData& user, const float & size_x){
    this->head= user.getSkeleton().getJoint(nite::JOINT_HEAD);
    this->neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    this->torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
    this->hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    this->knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
    this->rightShoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    this->leftShoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);

    this->error = (head.getPosition().y -neck.getPosition().y)/4;
    this->size_x = size_x;
}

ReferencePoints::~ReferencePoints(){

}

void ReferencePoints::setReferencePoints(const nite::UserData& user, const float & size_x){
    this->head= user.getSkeleton().getJoint(nite::JOINT_HEAD);
    this->neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    this->torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
    this->hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    this->knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
    this->rightShoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    this->leftShoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);

    this->error = abs(head.getPosition().y - neck.getPosition().y)/4;
        this->size_x = size_x;
}

nite::SkeletonJoint ReferencePoints::getHead() const{
    return this->head;
}

nite::SkeletonJoint ReferencePoints::getNeck() const {
    return this->neck;
}

nite::SkeletonJoint ReferencePoints::getTorso() const {
    return this->torso;
}

nite::SkeletonJoint ReferencePoints::getHip() const {
    return this->hip;

}

nite::SkeletonJoint ReferencePoints::getKnee() const {
    return this->knee;
}

nite::SkeletonJoint ReferencePoints::getRigthShoulder() const {
    return this->rightShoulder;
}

nite::SkeletonJoint ReferencePoints::getLeftShoulder() const {
    return this->leftShoulder;
}

float ReferencePoints::getError() const {
    return this->error;
}

float ReferencePoints::getSizeX() const {
    return this->size_x;
}
