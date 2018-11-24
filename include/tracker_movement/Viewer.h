/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer                                    *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/

#ifndef _VIEWER_H_
#define _VIEWER_H_

#include "NiTE.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <ros/ros.h>
#include <cmath>
#include <std_msgs/String.h>

#include <geometry_msgs/Twist.h>

#include <tracker_movement/ReferencePoints.h>
#include <tracker_movement/FuzzySystem.h>

#include <GL/glut.h>

using std::string;


//Iniciar depthHistogram y elimiinar
enum Pose {none, centerHands, separate, closed, centerLeft, highLeft, centerRight, highRight, /*new*/centerBothHands, highHands, downHands, spinRight, spinLeft};

enum Move {ninguno, parar, subir, bajar, abrirPinza, cerrarPinza, avanzar, retroceder, girarDerecha, girarIzquierda};

class Viewer{
    private:
        Viewer(const Viewer&);
    	Viewer& operator=(Viewer&);

    	static Viewer* mySelf;
    	static void glutIdle();
    	static void glutDisplay();
    	static void glutKeyboard(unsigned char key, int x, int y);

        static const int MAX_DEPTH = 10000;
    	float* depthHistogram;
    	char name[ONI_MAX_STR];
    	openni::RGB888Pixel*		pixels;
    	unsigned int		pixelsX;
    	unsigned int		pixelsY;

    	//Paused
    	bool pause;
        bool setFuzzy;
        bool printPoints;
        bool isTracking;
        bool isCalibrating;

    	openni::Device		device;
    	nite::UserTracker* userTracker;

    	nite::UserId poseUser;
    	uint64_t poseTime;

	    openni::VideoStream colorVideo;
        openni::Status openniStatus;
        nite::Status niteStatus;
        ReferencePoints referencePoints;
        FuzzySystem fuzzySystem;
        Pose poseBefore;
        Move moveBefore;
        ros::Publisher gripper_pub;
        ros::Publisher velocity_pub;
        ros::Publisher ptu_pub;

        ros::Rate r;

        float* centerMass;

    protected:
        virtual void Display();
    	virtual void DisplayPostDraw(){};	// Overload to draw over the screen image

    	virtual void OnKey(unsigned char key, int x, int y);

    	virtual openni::Status InitOpenGL(int argc, char **argv);
    	void InitOpenGLHooks();

    	void Finalize();
        bool checkConfidence(const nite::UserData& user, const float &confidence=.5);
        bool userFront(const nite::UserData &user);
        void checkFuzzyLogic(const nite::UserData &user,  const nite::UserTrackerFrameRef &userTrackerFrame);
        void showPoints(const nite::UserData& user);
        float* calculateCenterOfMass(const nite::UserData& user, float * coordinates );

    public:
    	Viewer(const char* name);
        Viewer(const char* name, ros::Publisher gripper_pub, ros::Publisher velocity_pub, ros::Publisher ptu_pub);
    	virtual ~Viewer();

    	virtual openni::Status Init(int argc, char **argv);
    	virtual openni::Status Run();	//Does not return

};

#endif
