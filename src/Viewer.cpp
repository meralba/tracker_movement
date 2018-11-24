/*******************************************************************************
*                                                                              *
*   PrimeSense NiTE 2.0 - User Viewer                                    *
*   Copyright (C) 2012 PrimeSense Ltd.                                         *
*                                                                              *
*******************************************************************************/
#include <tracker_movement/Viewer.h>
#include <GL/glut.h>

#define GL_WIN_SIZE_X	 640
#define GL_WIN_SIZE_Y	480
#define TEXTURE_SIZE	512
#define PI 3.14159265


int minNumChunk(const int &data_size,const int &chunk_size){
    return (((data_size)-1) / (chunk_size) + 1);
}

int minChunksSize(const int &data_size,const int &chunk_size){
    return (minNumChunk(data_size, chunk_size) * (chunk_size));
}

Viewer* Viewer::mySelf = NULL;


int resolutionX = 0, resolutionY = 0;

// time to hold in pose to exit program. In milliseconds.
const int poseTimeoutToCalibrate = 1000;

void Viewer::glutIdle()
{
	glutPostRedisplay();
}
void Viewer::glutDisplay()
{
	Viewer::mySelf->Display();
}
void Viewer::glutKeyboard(unsigned char key, int x, int y)
{
	Viewer::mySelf->OnKey(key, x, y);
}

Viewer::Viewer(const char* name) : poseUser(0), r(10)
{
    depthHistogram = new float[MAX_DEPTH];
    mySelf = this;
	strncpy(this->name, name, ONI_MAX_STR);
	this->userTracker = new nite::UserTracker;
    this->setFuzzy = false;
    this->printPoints = false;
    this->poseBefore = none;
    this->moveBefore = ninguno;
    float a[3] ={0};
    this->centerMass = a;
    this->isCalibrating = false;
}

Viewer::Viewer(const char* name, ros::Publisher gripper_pub, ros::Publisher velocity_pub, ros::Publisher ptu_pub) : poseUser(0), r(10)
{
    depthHistogram = new float[MAX_DEPTH];
    mySelf = this;
	strncpy(this->name, name, ONI_MAX_STR);
	this->userTracker = new nite::UserTracker;
    this->gripper_pub = gripper_pub;
    this->velocity_pub = velocity_pub;
    this->ptu_pub = ptu_pub;
    this->setFuzzy = false;
    this->printPoints = false;
    this->poseBefore = none;
    this->moveBefore = ninguno;
    float a[3] ={0};
    this->centerMass = a;
    this->isCalibrating = false;
}
Viewer::~Viewer()
{
	Finalize();

	delete[] pixels;
    delete[] depthHistogram;

	mySelf = NULL;
}

void Viewer::Finalize()
{
	delete userTracker;
    colorVideo.stop();
    colorVideo.destroy();
    device.close();
	nite::NiTE::shutdown();
	openni::OpenNI::shutdown();
}

openni::Status Viewer::Init(int argc, char **argv)
{
    pixels = NULL;
    openniStatus = openni::OpenNI::initialize();
    if (openniStatus != openni::STATUS_OK)
    {
        ROS_ERROR("Failed to initialize OpenNI\n%s\n", openni::OpenNI::getExtendedError());
        return openniStatus;
    }

    const char* deviceUri = openni::ANY_DEVICE;
    for (int i = 1; i < argc-1; ++i)
    {
        if (strcmp(argv[i], "-device") == 0)
        {   deviceUri = argv[i+1];
            break;
        }

    }
    openniStatus = device.open(deviceUri);
    if (openniStatus != openni::STATUS_OK)
    {
        ROS_ERROR("Failed to open device\n%s\n", openni::OpenNI::getExtendedError());
        return openniStatus;
    }
    //openniStatus = device.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    colorVideo.create(device,openni::SENSOR_COLOR);
    colorVideo.start();
    nite::NiTE::initialize();
    niteStatus = userTracker->create(&device);
    if (niteStatus != nite::STATUS_OK)
    {
        return openni::STATUS_ERROR;
    }

    return InitOpenGL(argc, argv);

}

openni::Status Viewer::Run()	//Does not return
{
	glutMainLoop();

	return openni::STATUS_OK;
}

float Colors[][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}, {1, 1, 1}};
int colorCount = 3;

#define MAX_USERS 10
bool visibleUsers[MAX_USERS] = {false};
bool angulo=false;
bool calibrate = false;
nite::SkeletonState skeletonStates[MAX_USERS] = {nite::SKELETON_NONE};
char userStatusLabels[MAX_USERS][100] = {{0}};

char generalMessage[100] = {0};

#define USER_MESSAGE(msg) {\
	sprintf(userStatusLabels[user.getId()], "%s", msg);\
	ROS_INFO("[%08" PRIu64 "]User #%d:\t%s\n", ts, user.getId(), msg);}

void updateUserState(const nite::UserData& user, uint64_t ts)
{
	if (user.isNew())
	{
		USER_MESSAGE("New");
	}
	else if (user.isVisible() && !visibleUsers[user.getId()])
		ROS_INFO("[%08" PRIu64 "] User #%d:\tVisible\n", ts, user.getId());
	else if (!user.isVisible() && visibleUsers[user.getId()])
		ROS_INFO("[%08" PRIu64 "] User #%d:\tOut of Scene\n", ts, user.getId());
	else if (user.isLost())
	{
		USER_MESSAGE("Lost");
	}
	visibleUsers[user.getId()] = user.isVisible();


	if(skeletonStates[user.getId()] != user.getSkeleton().getState())
	{
		switch(skeletonStates[user.getId()] = user.getSkeleton().getState())
		{
		case nite::SKELETON_NONE:
			USER_MESSAGE("Stopped tracking.")
			break;
		case nite::SKELETON_CALIBRATING:
			USER_MESSAGE("Calibrating...")
			break;
		case nite::SKELETON_TRACKED:
			USER_MESSAGE("Tracking!")
			break;
		case nite::SKELETON_CALIBRATION_ERROR_NOT_IN_POSE:
		case nite::SKELETON_CALIBRATION_ERROR_HANDS:
		case nite::SKELETON_CALIBRATION_ERROR_LEGS:
		case nite::SKELETON_CALIBRATION_ERROR_HEAD:
		case nite::SKELETON_CALIBRATION_ERROR_TORSO:
			USER_MESSAGE("Calibration Failed... :-|")
			break;
		}
	}
}

#ifndef USE_GLES
void glPrintString(void *font, const char *str)
{
	int i,l = (int)strlen(str);

	for(i=0; i<l; i++)
	{
		glutBitmapCharacter(font,*str++);
	}
}
#endif
void DrawStatusLabel(nite::UserTracker* userTracker, const nite::UserData& user)
{
	int color = user.getId() % colorCount;
	glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);

	float x,y;
	userTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &x, &y);
	x *= GL_WIN_SIZE_X/(float)resolutionX;
	y *= GL_WIN_SIZE_Y/(float)resolutionY;
	char *msg = userStatusLabels[user.getId()];
	glRasterPos2i(x-((strlen(msg)/2)*8),y);
	glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
}


void DrawLimb(nite::UserTracker* userTracker, const nite::SkeletonJoint& joint1, const nite::SkeletonJoint& joint2, int color,bool b=false)
{
	float coordinates[6] = {0};

	userTracker->convertJointCoordinatesToDepth(joint1.getPosition().x, joint1.getPosition().y, joint1.getPosition().z, &coordinates[0], &coordinates[1]);
	userTracker->convertJointCoordinatesToDepth(joint2.getPosition().x, joint2.getPosition().y, joint2.getPosition().z, &coordinates[3], &coordinates[4]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)resolutionX;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)resolutionY;
	coordinates[3] *= GL_WIN_SIZE_X/(float)resolutionX;
	coordinates[4] *= GL_WIN_SIZE_Y/(float)resolutionY;


	if (joint1.getPositionConfidence() == 1 && joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else if (joint1.getPositionConfidence() < 0.5f || joint2.getPositionConfidence() < 0.5f)
	{
		return;
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glPointSize(2);
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_LINES, 0, 2);

	glPointSize(10);
	if (joint1.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates);
	glDrawArrays(GL_POINTS, 0, 1);

	if (joint2.getPositionConfidence() == 1)
	{
		glColor3f(1.0f - Colors[color][0], 1.0f - Colors[color][1], 1.0f - Colors[color][2]);
	}
	else
	{
		glColor3f(.5, .5, .5);
	}
	glVertexPointer(3, GL_FLOAT, 0, coordinates+3);
	glDrawArrays(GL_POINTS, 0, 1);
}
void DrawSkeleton(nite::UserTracker* userTracker, const nite::UserData& userData)
{
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_HEAD), userData.getSkeleton().getJoint(nite::JOINT_NECK), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getId() % colorCount);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getId() % colorCount);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER), userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getId() % colorCount,true);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_TORSO), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getId() % colorCount);


	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_HIP), userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getId() % colorCount);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_LEFT_FOOT), userData.getId() % colorCount);

	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getId() % colorCount);
	DrawLimb(userTracker, userData.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE), userData.getSkeleton().getJoint(nite::JOINT_RIGHT_FOOT), userData.getId() % colorCount);
}

void calculateHistogram(float* pHistogram, int histogramSize, const openni::VideoFrameRef& depthFrame)
{
	const openni::DepthPixel* pDepth = (const openni::DepthPixel*)depthFrame.getData();
	int width = depthFrame.getWidth();
	int height = depthFrame.getHeight();
	// Calculate the accumulative histogram (the yellow display...)
	memset(pHistogram, 0, histogramSize*sizeof(float));
	int restOfRow = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel) - width;

	unsigned int nNumberOfPoints = 0;
	for (int y = 0; y < height; ++y)
	{
		for (int x = 0; x < width; ++x, ++pDepth)
		{
			if (*pDepth != 0)
			{
				pHistogram[*pDepth]++;
				nNumberOfPoints++;
			}
		}
		pDepth += restOfRow;
	}
	for (int nIndex=1; nIndex<histogramSize; nIndex++)
	{
		pHistogram[nIndex] += pHistogram[nIndex-1];
	}
	if (nNumberOfPoints)
	{
		for (int nIndex=1; nIndex<histogramSize; nIndex++)
		{
			pHistogram[nIndex] = (256 * (1.0f - (pHistogram[nIndex] / nNumberOfPoints)));
		}
	}
}

void angulos(nite::UserData userData){
    nite::SkeletonJoint mano, hombro, codo,torso, cuello;
    mano = userData.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
    hombro = userData.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
    codo = userData.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
    torso = userData.getSkeleton().getJoint(nite::JOINT_TORSO);
    cuello = userData.getSkeleton().getJoint(nite::JOINT_NECK);

    float anguloxy, anguloBrazo, anguloTorso;
    float vector1,vector2;
    float u1 = mano.getPosition().y-codo.getPosition().y;
    float u2 = mano.getPosition().z-codo.getPosition().z;
    float v1 = hombro.getPosition().y-codo.getPosition().y;
    float v2 = hombro.getPosition().z-codo.getPosition().z;
    anguloBrazo = acos((u1*v1+u2*v2)/(sqrt(u1*u1+u2*u2)*sqrt(v1*v1+v2*v2))) * 180.0 / PI;;
    ROS_INFO("anguloBrazo: %f\n",anguloBrazo);
    u1 = torso.getPosition().y-cuello.getPosition().y;
    u2 = torso.getPosition().z-cuello.getPosition().z;
    v1 = hombro.getPosition().y-cuello.getPosition().y;
    v2 = hombro.getPosition().z-cuello.getPosition().z;
    anguloTorso = acos((u1*v1+u2*v2)/(sqrt(u1*u1+u2*u2)*sqrt(v1*v1+v2*v2))) * 180.0 / PI;;
    ROS_INFO("anguloTorso: %f\n",anguloTorso);


}

bool Viewer::checkConfidence(const nite::UserData& user, const float &confidence){
    bool haveConfidence = false;
    const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
    const nite::SkeletonJoint& neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    const nite::SkeletonJoint& right_elbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
    const nite::SkeletonJoint& right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
    const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
    const nite::SkeletonJoint& left_elbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
    const nite::SkeletonJoint& left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
    const nite::SkeletonJoint& torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
    const nite::SkeletonJoint& left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    const nite::SkeletonJoint& right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
    const nite::SkeletonJoint& left_knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
    const nite::SkeletonJoint& right_knee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);
    if (head.getPositionConfidence() >confidence && neck.getPositionConfidence() >confidence &&
            right_shoulder.getPositionConfidence() > confidence && right_elbow.getPositionConfidence() >confidence && right_hand.getPositionConfidence() >confidence &&
            left_shoulder.getPositionConfidence() >confidence && left_elbow.getPositionConfidence() >confidence && left_hand.getPositionConfidence() >confidence &&
            torso.getPositionConfidence() >confidence && left_hip.getPositionConfidence() >confidence && right_hip.getPositionConfidence() >confidence ){
                haveConfidence = true;
            }
    return haveConfidence;
}

bool Viewer::userFront(const nite::UserData &user){

    bool front = false;
    const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
    const nite::SkeletonJoint& right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
    const nite::SkeletonJoint& left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    const nite::SkeletonJoint& neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    const nite::SkeletonJoint& torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);

    if ( abs(right_shoulder.getPosition().y-left_shoulder.getPosition().y) <= 40 &&
        abs(right_hip.getPosition().y-left_hip.getPosition().y) <= 40 &&
        abs(neck.getPosition().x-torso.getPosition().x) <= 40 && checkConfidence(user,.5) )

    {
        front = true;
    }

    return front;

}


float* Viewer::calculateCenterOfMass(const nite::UserData& user, float *coordinates)
{
	userTracker->convertJointCoordinatesToDepth(user.getCenterOfMass().x, user.getCenterOfMass().y, user.getCenterOfMass().z, &coordinates[0], &coordinates[1]);

	coordinates[0] *= GL_WIN_SIZE_X/(float)resolutionX;
	coordinates[1] *= GL_WIN_SIZE_Y/(float)resolutionY;

    return coordinates;

}

void Viewer::checkFuzzyLogic(const nite::UserData &user, const nite::UserTrackerFrameRef &userTrackerFrame){

    std::stringstream ss;
    geometry_msgs::Twist cmdVel;
    std_msgs::String cmdGripper;
    std_msgs::String cmdPTU;
    bool confidence = checkConfidence(user,.5);
    std::stringstream ptuss;

    float coordinates[3] ={0};
    calculateCenterOfMass(user,coordinates);
    if( abs(centerMass[0]-coordinates[0])>50 || abs(centerMass[1]-coordinates[1]) >50){
        ROS_INFO_STREAM("Recalibrating...\n");
        referencePoints.setReferencePoints(user, GL_WIN_SIZE_X);
        ROS_INFO_STREAM("Reference points have been established");
        fuzzySystem.setFuzzySystem(referencePoints);
        setFuzzy = true;
        calibrate = true;
        calculateCenterOfMass(user,centerMass);
    }

    if(confidence && userFront(user) && setFuzzy && !isCalibrating){


        float spinPTU = fuzzySystem.processSkeletonCenterEngine(coordinates[0]);

        if(spinPTU < 0.4){
            ptuss << "derecha" ;
            ///*/DEBUG*/ROS_INFO_STREAM("derecha");
            cmdPTU.data = ptuss.str();
            ROS_INFO_STREAM("Send to /RosAria/cmd_ptu spin to the " << cmdGripper.data);
            ptu_pub.publish(cmdPTU);
            r.sleep();
        }
        else {
            if (spinPTU >0.6){
                ptuss << "izquierda" ;
                ///*/DEBUG*/ROS_INFO_STREAM("izquierda");
                cmdPTU.data = ptuss.str();
                ROS_INFO_STREAM("Send to /RosAria/cmd_ptu spin to the " << cmdGripper.data);
                ptu_pub.publish(cmdPTU);
                r.sleep();
            }
        }
        if(fuzzySystem.processSpinRightEngine(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y,
                                                    user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                    user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y) > .5 && fuzzySystem.processSpinLeftEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y,
                                                    user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                    user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y) <=.5){
            if(poseBefore != spinRight ){
                sprintf(generalMessage, "Keep it for %d second%s to spin to the right\n", poseTimeoutToCalibrate/1000, poseTimeoutToCalibrate/1000 == 1 ? "" : "s");
                ROS_INFO("Counting down %d second to spin to the right\n", poseTimeoutToCalibrate/1000);
                poseBefore = spinRight;
                poseUser = user.getId();
                poseTime = userTrackerFrame.getTimestamp();
            }
            else{
                // tick
				if (userTrackerFrame.getTimestamp() -
                poseTime > poseTimeoutToCalibrate * 1000 && moveBefore != girarDerecha)
				{
					ROS_INFO_STREAM("Count down complete. Spining to the right...\n");
                    moveBefore = girarDerecha;
                    cmdVel.angular.z = 0.05;
                    ///*/DEBUG*/ROS_INFO_STREAM("gira derecha");
                    ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                        << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                            << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                    velocity_pub.publish(cmdVel);
                    r.sleep();
				}
            }
        }
        else{
            if(moveBefore == girarDerecha){
                moveBefore = ninguno;
                poseBefore = none;
                cmdVel.angular.x = 0;
                ///*/DEBUG*/ROS_INFO_STREAM("parar giro");
                ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                    << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                        << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                velocity_pub.publish(cmdVel);
                r.sleep();
            }
            if(fuzzySystem.processSpinLeftEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y,
                                                        user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                        user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y) > .5 && fuzzySystem.processSpinRightEngine(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y,
                                                        user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                        user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y) <= .5){
                if(poseBefore != spinLeft ){
                    sprintf(generalMessage, "Keep it for %d second%s to spin to the left\n", poseTimeoutToCalibrate/1000, poseTimeoutToCalibrate/1000 == 1 ? "" : "s");
                    ROS_INFO("Counting down %d second to spin to the left\n", poseTimeoutToCalibrate/1000);
                    poseBefore = spinLeft;
                    poseUser = user.getId();
                    poseTime = userTrackerFrame.getTimestamp();
                }
                else{
                    // tick
    				if (userTrackerFrame.getTimestamp() -
                    poseTime > poseTimeoutToCalibrate * 1000 && moveBefore != girarIzquierda)
    				{
    					ROS_INFO_STREAM("Count down complete. Spining to the left...\n");
                        moveBefore = girarIzquierda;
                        cmdVel.angular.z = -0.05;
                        ///*/DEBUG*/ROS_INFO_STREAM("gira izquierda");
                        ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                            << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                                << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                        velocity_pub.publish(cmdVel);
                        r.sleep();
    				}
                }
            }
            else{
                if(moveBefore == girarIzquierda){
                    moveBefore = ninguno;
                    poseBefore = none;
                    cmdVel.angular.x = 0;
                    ///*/DEBUG*/ROS_INFO_STREAM("parar giro");
                    ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                        << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                            << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                    velocity_pub.publish(cmdVel);
                    r.sleep();
                }
                if( fuzzySystem.processSameHeightHandsEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y,
                                                        user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y) > 0.45 ){
                    if( fuzzySystem.processSeparateHandsEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                           user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                       user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y ) > 0.5 ){
                        if(poseBefore == centerHands){
                            moveBefore = abrirPinza;
                            poseBefore = separate;
                            ss << "abrir";
                            ///*/DEBUG*/ROS_INFO_STREAM("abrir");
                            cmdGripper.data = ss.str();
                            ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                			gripper_pub.publish(cmdGripper);
                            r.sleep();
                        }
                        else{
                            if(poseBefore != separate){
                                moveBefore = ninguno;
                                poseBefore = separate;
                                ///*/DEBUG*/ROS_INFO_STREAM("separadas");
                            }
                        }
                    }
                    else{
                        if( poseBefore == separate && moveBefore == abrirPinza){
                            moveBefore = ninguno;
                            ///*/DEBUG*/ROS_INFO_STREAM("fuera abrir pinza");
                        }

                        if(fuzzySystem.processCenterHandsEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                                user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                            user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y ) > 0.5){
                            if(poseBefore == separate){
                                moveBefore = cerrarPinza;
                                poseBefore = centerHands;
                                ss << "cerrar";
                                ///*/DEBUG*/ROS_INFO_STREAM("cerrar");
                                cmdGripper.data = ss.str();
                                ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                                gripper_pub.publish(cmdGripper);
                                r.sleep();
                            }

                            if(poseBefore != centerHands){
                                poseBefore = centerHands;
                                ///*/DEBUG*/ROS_INFO_STREAM("Manos centradas");
                            }

                        }
                        else{
                            if(poseBefore == centerHands && moveBefore == cerrarPinza){
                                moveBefore = ninguno;
                                ROS_INFO_STREAM("fuera cerrar pinza");
                            }
                        }
                    }
                }
                else{
                    if( fuzzySystem.processHighLeftHandEngine(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y,
                                                                user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                                user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y) > .5){
                        if (poseBefore == centerLeft){
                            moveBefore = subir;
                            poseBefore = highLeft;
                            ss << "subir";
                            ///*/DEBUG*/ROS_INFO_STREAM("subir");
                            cmdGripper.data = ss.str();
                            ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                			gripper_pub.publish(cmdGripper);
                            r.sleep();
                        }
                        else{
                            if(poseBefore != highLeft){
                                moveBefore = ninguno;
                                poseBefore = highLeft;
                                ///*/DEBUG*/ROS_INFO_STREAM("mano arriba");
                            }
                        }
                    }
                    else {
                        if( poseBefore == highLeft && moveBefore == subir){
                            moveBefore = parar;
                            ss << "parar pinza";
                            ///*/DEBUG*/ROS_INFO_STREAM("parar pinza");
                            cmdGripper.data = ss.str();
                            ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                			gripper_pub.publish(cmdGripper);
                            r.sleep();
                        }
                        if(fuzzySystem.processCenterLeftHandEngine(user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().y,
                                                                    user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND).getPosition().x,
                                                                    user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW).getPosition().y) > .5){
                            if(poseBefore == highLeft){
                                moveBefore = bajar;
                                poseBefore = centerLeft;
                                ss << "bajar";
                                ///*/DEBUG*/ROS_INFO_STREAM("bajar");
                                cmdGripper.data = ss.str();
                                ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                                gripper_pub.publish(cmdGripper);
                                r.sleep();
                            }

                            if(poseBefore != centerLeft){
                                poseBefore = centerLeft;

                                ///*/DEBUG*/ROS_INFO_STREAM("Centrada");
                            }

                        }
                        else{
                            if( poseBefore == centerLeft && moveBefore == bajar){
                                moveBefore = parar;
                                ss << "parar pinza";
                                ///*/DEBUG*/ROS_INFO_STREAM("parar pinza");
                                cmdGripper.data = ss.str();
                                ROS_INFO_STREAM("Send to /RosAria/cmd_gripper the value " << cmdGripper.data);
                    			gripper_pub.publish(cmdGripper);
                                r.sleep();
                            }

                            if(fuzzySystem.processHighRightHandEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y,
                                                                    user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                                    user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y) > 0.5 ){
                                if(poseBefore == centerRight){
                                    moveBefore = avanzar;
                                    poseBefore = highRight;
                                    cmdVel.linear.x = 0.05;
                                    ///*/DEBUG*/ROS_INFO_STREAM("avanzar");
                                    ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                                        << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                                            << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                                    velocity_pub.publish(cmdVel);
                                    r.sleep();
                                }
                                else{
                                    if(poseBefore != highRight){
                                        moveBefore = ninguno;
                                        poseBefore = highRight;
                                        ///*/DEBUG*/ROS_INFO_STREAM("mano derecha arriba");
                                    }
                                }
                            }
                            else {
                                if( poseBefore == highRight && moveBefore == avanzar){
                                    moveBefore = parar;
                                    ss << "parar avance";
                                    ///*/DEBUG*/ROS_INFO_STREAM("parar robot");
                                    cmdVel.linear.x = 0;
                                    ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                                        << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                                            << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                                    velocity_pub.publish(cmdVel);
                                    r.sleep();
                                }
                                if(fuzzySystem.processCenterRightHandEngine(user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().y,
                                                                            user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND).getPosition().x,
                                                                            user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW).getPosition().y) > .5){
                                    if(poseBefore == highRight){
                                        moveBefore = retroceder;
                                        poseBefore = centerRight;
                                        ss << "bajar";
                                        ///*/DEBUG*/ROS_INFO_STREAM("retroceder");
                                        cmdVel.linear.x = -0.1;
                                        ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                                            << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                                                << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                                        velocity_pub.publish(cmdVel);
                                        r.sleep();
                                    }

                                    if(poseBefore != centerRight){
                                        poseBefore = centerRight;

                                        ///*/DEBUG*/ROS_INFO_STREAM("Centrada derecha");
                                    }
                                }
                                else{
                                    if( poseBefore == centerRight && moveBefore == retroceder){
                                        moveBefore = parar;
                                        ss << "parar robot";
                                        ///*/DEBUG*/ROS_INFO_STREAM("parar robot");
                                        cmdVel.linear.x = 0;
                                        ROS_INFO_STREAM("Send to /RosAria/cmd_vel the value [" << cmdVel.linear.x
                                                            << "," << cmdVel.linear.y << "," << cmdVel.linear.z << "] [" << cmdVel.angular.x
                                                                                << "," << cmdVel.angular.y << "," << cmdVel.angular.z <<"]");
                                        velocity_pub.publish(cmdVel);
                                        r.sleep();
                                    }
                                    if(poseBefore !=none){
                                        ///*/DEBUG*/ROS_INFO_STREAM("Ninguna");
                                        poseBefore = none;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}

void Viewer::showPoints(const nite::UserData& user){
    const nite::SkeletonJoint& head = user.getSkeleton().getJoint(nite::JOINT_HEAD);
    const nite::SkeletonJoint& neck = user.getSkeleton().getJoint(nite::JOINT_NECK);
    const nite::SkeletonJoint& right_shoulder = user.getSkeleton().getJoint(nite::JOINT_RIGHT_SHOULDER);
    const nite::SkeletonJoint& right_elbow = user.getSkeleton().getJoint(nite::JOINT_RIGHT_ELBOW);
    const nite::SkeletonJoint& right_hand = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HAND);
    const nite::SkeletonJoint& left_shoulder = user.getSkeleton().getJoint(nite::JOINT_LEFT_SHOULDER);
    const nite::SkeletonJoint& left_elbow = user.getSkeleton().getJoint(nite::JOINT_LEFT_ELBOW);
    const nite::SkeletonJoint& left_hand = user.getSkeleton().getJoint(nite::JOINT_LEFT_HAND);
    const nite::SkeletonJoint& torso = user.getSkeleton().getJoint(nite::JOINT_TORSO);
    const nite::SkeletonJoint& left_hip = user.getSkeleton().getJoint(nite::JOINT_LEFT_HIP);
    const nite::SkeletonJoint& right_hip = user.getSkeleton().getJoint(nite::JOINT_RIGHT_HIP);
    const nite::SkeletonJoint& left_knee = user.getSkeleton().getJoint(nite::JOINT_LEFT_KNEE);
    const nite::SkeletonJoint& right_knee = user.getSkeleton().getJoint(nite::JOINT_RIGHT_KNEE);

    ROS_INFO("Cabeza[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), head.getPosition().x, head.getPosition().y, head.getPosition().z);
    ROS_INFO("Cuello[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), neck.getPosition().x, neck.getPosition().y, neck.getPosition().z);
    ROS_INFO("Hombro derecho[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), right_shoulder.getPosition().x, right_shoulder.getPosition().y, right_shoulder.getPosition().z);
    ROS_INFO("Codo derecho[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), right_elbow.getPosition().x, right_elbow.getPosition().y, right_elbow.getPosition().z);
    ROS_INFO("Mano derecha[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), right_hand.getPosition().x, right_hand.getPosition().y, right_hand.getPosition().z);
    ROS_INFO("Hombro izquierdo[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), left_shoulder.getPosition().x, left_shoulder.getPosition().y, left_shoulder.getPosition().z);
    ROS_INFO("Codo izquierdo[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), left_elbow.getPosition().x, left_elbow.getPosition().y, left_elbow.getPosition().z);
    ROS_INFO("Mano izquierda[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), left_hand.getPosition().x, left_hand.getPosition().y, left_hand.getPosition().z);
    ROS_INFO("Torso[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), torso.getPosition().x, torso.getPosition().y, torso.getPosition().z);
    ROS_INFO("Cadera derecha[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), right_hip.getPosition().x, right_hip.getPosition().y, right_hip.getPosition().z);
    ROS_INFO("Cadera izquierda[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), left_hip.getPosition().x, left_hip.getPosition().y, left_hip.getPosition().z);
    ROS_INFO("Rodilla derecha[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), right_knee.getPosition().x, right_knee.getPosition().y, right_knee.getPosition().z);
    ROS_INFO("Rodilla izquierda[User:%d (%5.2f, %5.2f, %5.2f)]", user.getId(), left_knee.getPosition().x, left_knee.getPosition().y, left_knee.getPosition().z);

}

void Viewer::Display()
{
    nite::UserTrackerFrameRef userTrackerFrame;
    openni::VideoFrameRef depthFrame, colorFrame;
    openni::VideoMode videoMode;
    niteStatus = userTracker->readFrame(&userTrackerFrame);
    if (niteStatus != nite::STATUS_OK)
    {
        printf("GetNextData failed\n");
        return;
    }

    depthFrame = userTrackerFrame.getDepthFrame();

	if (pixels == NULL)
	{
		// Texture map init
		pixelsX = minChunksSize(depthFrame.getVideoMode().getResolutionX(), TEXTURE_SIZE);
		pixelsY = minChunksSize(depthFrame.getVideoMode().getResolutionY(), TEXTURE_SIZE);
		pixels = new openni::RGB888Pixel[pixelsX * pixelsY];
	}

	const nite::UserMap& userLabels = userTrackerFrame.getUserMap();
   //Limpia buffers
	glClear (GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
   //Especifica la matriz actual
	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
   //Reemplaza la matriz actual con la matriz identidad
	glLoadIdentity();
   //Multiplica la matriz actual con una matriz ortográfica
	glOrtho(0, GL_WIN_SIZE_X, GL_WIN_SIZE_Y, 0, -1.0, 1.0);
   //Si el frame es valido y la opción de dibujar profundidad está activa
	if ( depthFrame.isValid() )
	{
      //Calcula el histograma del actual frame de profundidad y se almacena en el primer argumento.
		calculateHistogram(depthHistogram, MAX_DEPTH, depthFrame);
	}

	memset(pixels, 0, pixelsX*pixelsY*sizeof(openni::RGB888Pixel));

	float factor[3] = {1, 1, 1};
	// check if we need to draw depth frame to texture
	if (depthFrame.isValid() )
	{
		const nite::UserId* labels = userLabels.getPixels();

		const openni::DepthPixel* depthRow = (const openni::DepthPixel*)depthFrame.getData();
		openni::RGB888Pixel* pixelsRow = pixels + depthFrame.getCropOriginY() * pixelsX;
		int rowSize = depthFrame.getStrideInBytes() / sizeof(openni::DepthPixel);

		for (int y = 0; y < depthFrame.getHeight(); ++y)
		{
			const openni::DepthPixel* depth = depthRow;
			openni::RGB888Pixel* pix = pixelsRow + depthFrame.getCropOriginX();

			for (int x = 0; x < depthFrame.getWidth(); ++x, ++depth, ++pix, ++labels)
			{
				if (*depth != 0)
				{
					if (*labels == 0)
					{

						factor[0] = Colors[colorCount][0];
						factor[1] = Colors[colorCount][1];
						factor[2] = Colors[colorCount][2];

					}
					else
					{
						factor[0] = Colors[*labels % colorCount][0];
						factor[1] = Colors[*labels % colorCount][1];
						factor[2] = Colors[*labels % colorCount][2];
					}
//					// Add debug lines - every 10cm
// 4			else if ((*pDepth / 10) % 10 == 0)
// 					{
// 						factor[0] = factor[2] = 0;
// 					}

					int histogramValue = depthHistogram[*depth];
					pix->r = histogramValue*factor[0];
					pix->g = histogramValue*factor[1];
					pix->b = histogramValue*factor[2];

					factor[0] = factor[1] = factor[2] = 1;
				}
			}

			depthRow += rowSize;
			pixelsRow += pixelsX;
		}
	}
   //Modifica textura
	glTexParameteri(GL_TEXTURE_2D, GL_GENERATE_MIPMAP_SGIS, GL_TRUE);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
	glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
   //Especifica la textura de la imagen, target, nivel, formato interno, altura, anchura, borde, formato, tipo,datos
	glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, pixelsX, pixelsY, 0, GL_RGB, GL_UNSIGNED_BYTE, pixels);

	// Display the OpenGL texture map
	glColor4f(1,1,1,1);
   //Se activa la imágen
	glEnable(GL_TEXTURE_2D);
	glBegin(GL_QUADS);

	resolutionX = depthFrame.getVideoMode().getResolutionX();
	resolutionY = depthFrame.getVideoMode().getResolutionY();

	// upper left
	glTexCoord2f(0, 0);
	glVertex2f(0, 0);
	// upper right
	glTexCoord2f((float)resolutionX/(float)pixelsX, 0);
	glVertex2f(GL_WIN_SIZE_X, 0);
	// bottom right
	glTexCoord2f((float)resolutionX/(float)pixelsX, (float)resolutionY/(float)pixelsY);
	glVertex2f(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	// bottom left
	glTexCoord2f(0, (float)resolutionY/(float)pixelsY);
	glVertex2f(0, GL_WIN_SIZE_Y);

	glEnd();
	glDisable(GL_TEXTURE_2D);

	const nite::Array<nite::UserData>& users = userTrackerFrame.getUsers();
	for (int i = 0; i < users.getSize(); ++i)
	{
		const nite::UserData& user = users[i];

		updateUserState(user, userTrackerFrame.getTimestamp());
		if (user.isNew())
		{
			userTracker->startSkeletonTracking(user.getId());
			userTracker->startPoseDetection(user.getId(), nite::POSE_PSI);
		}
		else if (!user.isLost())
		{
			DrawStatusLabel(userTracker, user);
			if (users[i].getSkeleton().getState() == nite::SKELETON_TRACKED)
			{
				DrawSkeleton(userTracker, user);
                if(setFuzzy){
                    checkFuzzyLogic(user, userTrackerFrame);
                }
                if(printPoints){
                    showPoints(user);
                    printPoints = false;
                }

            }
		}

		if (poseUser == 0 || poseUser == user.getId() && userFront(user))
		{
			const nite::PoseData& pose = user.getPose(nite::POSE_PSI);

			if (pose.isEntered())
			{
				// Start timer
				sprintf(generalMessage, "Recalibrating fuzzySystems. Keep it for %d second%s to calibrate\n", poseTimeoutToCalibrate/1000, poseTimeoutToCalibrate/1000 == 1 ? "" : "s");
				ROS_INFO("Counting down %d second to recalibrate reference points\n", poseTimeoutToCalibrate/1000);
				poseUser = user.getId();
				poseTime = userTrackerFrame.getTimestamp();
                calibrate = false;
                isCalibrating = true;
			}
			else if (pose.isExited())
			{
				memset(generalMessage, 0, sizeof(generalMessage));
				ROS_INFO_STREAM("Count-down interrupted\n");
				poseTime = 0;
				poseUser = 0;
                isCalibrating = false;
			}
			else if (pose.isHeld())
			{
				// tick
				if (userTrackerFrame.getTimestamp() -
                poseTime > poseTimeoutToCalibrate * 1000 && !calibrate)
				{
					ROS_INFO_STREAM("Count down complete. Recalibrating...\n");
					referencePoints.setReferencePoints(user, GL_WIN_SIZE_X);
                    ROS_INFO_STREAM("Reference points have been established");
                    fuzzySystem.setFuzzySystem(referencePoints);
                    setFuzzy = true;
                    calibrate = true;
                    calculateCenterOfMass(user,centerMass);
                    isCalibrating = false;
				}
			}
		}
	}


	if (generalMessage[0] != '\0')
	{
		char *msg = generalMessage;
		glColor3f(1.0f, 0.0f, 0.0f);
		glRasterPos2i(100, 20);
		glPrintString(GLUT_BITMAP_HELVETICA_18, msg);
	}

	// Swap the OpenGL display buffers
	glutSwapBuffers();

}

void Viewer::OnKey(unsigned char key, int /*x*/, int /*y*/)
{
	switch (key)
	{
	case 27:
		Finalize();
		exit (1);
    case 'a':
        angulo=true;
    case 's':
        //Screenshoot
    case 'p':
        printPoints = true;
    }

}

openni::Status Viewer::InitOpenGL(int argc, char **argv)
{
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowSize(GL_WIN_SIZE_X, GL_WIN_SIZE_Y);
	glutCreateWindow (name);
	// 	glutFullScreen();
	glutSetCursor(GLUT_CURSOR_NONE);

	InitOpenGLHooks();

	glDisable(GL_DEPTH_TEST);
	glEnable(GL_TEXTURE_2D);

	glEnableClientState(GL_VERTEX_ARRAY);
	glDisableClientState(GL_COLOR_ARRAY);

	return openni::STATUS_OK;

}
void Viewer::InitOpenGLHooks()
{
	glutKeyboardFunc(glutKeyboard);
	glutDisplayFunc(glutDisplay);
	glutIdleFunc(glutIdle);
}
