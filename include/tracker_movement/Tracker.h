/** License GPLv3
  * @author Mer Alba / merAlba
  */
#ifndef _TRACKER_H_
#define _TRACKER_H_

#include <NiTE.h>
#include <vector>
#include <ros/console.h>
#include <tracker_movement/ReferencePoints.h>
#include <tracker_movement/FuzzySystem.h>
#include <stdlib.h>
#include <std_msgs/String.h>
#include <string.h>

using namespace std;


enum Pose {none, centerHands, separate, closed, center, high, down, /*new*/centerBothHands, highHands, downHands};

class Tracker
{
private:
    int max_users;
    vector<nite::SkeletonState> skeletonStates;
    //nite::Array<nite::UserData> users;
    vector<bool> visibleUsers;
    nite::UserTracker* userTracker;
    nite::Status niteStatus;
    nite::UserTrackerFrameRef userTrackerFrame;
    bool printPoints;
    bool isFront;
    bool manos;
    Pose pose;
    ReferencePoints referencePoints;
    FuzzySystem fuzzySystem;


    void showPoints(const nite::UserData& user);
    bool checkConfidence(const nite::UserData& user);
    bool userFront(const nite::UserData &user);
public:
    Tracker();
    Tracker(bool points, int num_users = 10);
    ~Tracker();

    nite::UserTrackerFrameRef getUserTrackerFrame();
    nite::UserTracker* getUserTracker();

    nite::Status run(std_msgs::String& cmdGripper, nite::UserData& user);
    //nite::Status run(string& action); //new
    void updateUserState(const nite::UserData& user, unsigned long long timestamp);

    //Getter and setter
    int getMaxUsers();


};

#endif
