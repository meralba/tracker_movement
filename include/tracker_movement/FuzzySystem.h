/** License GPLv3
  * @author Mer Alba / merAlba
  */
#ifndef _FUZZYSYSTEMS_H_
#define _FUZZYSYSTEMS_H_


#include <fl/Headers.h>
#include <tracker_movement/ReferencePoints.h>

using namespace std;
using namespace fl;

class FuzzySystem{
private:
    float head_y;
    float neck_y;
    float torso_y;
    float hip_y;
    float knee_y;

    float min_y, max_y;

    float neck_x;
    float leftShoulder_x;
    float rightShoulder_x;

    float min_x, max_x;

    float error;
    float downElbow;
    float centerElbow;
    float highElbow;
    float downHand;
    float leftHand;
    float rightHand;
    float maxRightHand_x;
    float maxLeftHand_x;

    float size_x;

    InputVariable* rightHand_x;
    InputVariable* leftHand_x;
    InputVariable* rightHand_y;
    InputVariable* leftHand_y;
    // Se puede utilizar leftElbow como rightElbowInputVariable* rightElbow_y; //new
    InputVariable* leftElbow_y;

    InputVariable* skeletonPose;

    OutputVariable* ok;
    OutputVariable* spinPTU;

    //Engine gesto mano izquierda centro
    Engine* skeletonCenterEngine;
    RuleBlock* rulesSkeletonCenter;

    //Engine gesto mano izquierda centro
    Engine* centerLeftHandEngine;
    RuleBlock* rulesCenterLeftHand;


    //Engine gesto giro derecha
    Engine* spinRightEngine;
    RuleBlock* rulesSpinRight;

    //Engine gesto giro izquierda
    Engine* spinLeftEngine;
    RuleBlock* rulesSpinLeft;

    //Engine gesto mano derecha centro //new
    Engine* centerRightHandEngine;
    RuleBlock* rulesCenterRightHand;

    //Engine gesto mano izquierda arriba
    Engine* highLeftHandEngine;
    RuleBlock* rulesHighLeftHand;

    //Engine gesto mano derecha arriba //new
    Engine* highRightHandEngine;
    RuleBlock* rulesHighRightHand;

    //Engine gesto mano izquierda abajo
    Engine* downLeftHandEngine;
    RuleBlock* rulesDownLeftHand;

    //Engine gesto mano derecha abajo //new
    Engine* downRightHandEngine;
    RuleBlock* rulesDownRightHand;

    //Engine gesto manos centro
    Engine* centerHandsEngine;
    RuleBlock* rulesCenterHands;

    //Engine gesto manos separadas
    Engine* separateHandsEngine;
    RuleBlock* rulesSeparateHands;

    //Engine gesto manos juntas
    Engine* closeHandsEngine;
    RuleBlock* rulesCloseHands;

    //Engine manos misma altura
    Engine* sameHeightHandsEngine;
    RuleBlock* rulesSameHeightHands;

public:

    FuzzySystem();
    ~FuzzySystem();


    void setValuesPoints(const ReferencePoints &referencePoints);

    void setVariables();
    void setCenterLeftHandEngine();
    void setHighLeftHandEngine();
    void setCenterHandsEngine();
    void setSeparateHandsEngine();
    void setSameHeightHandsEngine();

    void setCenterRightHandEngine(); // new
    void setHighRightHandEngine(); //new

    void setSpinRightEngine(); // new
    void setSpinLeftEngine(); // new

    void setSkeletonCenterEngine(); // new



    void setFuzzySystem(const ReferencePoints &referencePoints);

    float processCenterLeftHandEngine(const float& leftHand_y, const float& leftHand_x, const float& leftElbow_y);
    float processHighLeftHandEngine(const float& leftHand_y, const float& leftHand_x,const float& leftElbow_y);
    float processCenterHandsEngine(const float& rightHand_x, const float& leftHand_x, const float& leftHand_y);
    float processSameHeightHandsEngine(const float& rightHand_y, const float& leftHand_y);
    float processSeparateHandsEngine(const float& rightHand_x, const float& leftHand_x, const float& leftHand_y);

    float processCenterRightHandEngine(const float& rightHand_y, const float& rightHand_x, const float& rightElbow_y); //new
    float processHighRightHandEngine(const float& rightHand_y, const float& rightHand_x, const float& rightElbow_y); //new

    float processSpinRightEngine(const float & leftHand_y, const float & leftHand_x, const float & leftElbow_y);
    float processSpinLeftEngine(const float & rightHand_y, const float & rightHand_x, const float & rightElbow_y);
    float processSkeletonCenterEngine(const float & centerMass_x);


};


#endif
