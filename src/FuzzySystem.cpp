#include <tracker_movement/FuzzySystem.h>

FuzzySystem::FuzzySystem(){
    rightHand_x = new InputVariable();
    leftHand_x = new InputVariable();
    rightHand_y = new InputVariable();
    leftHand_y = new InputVariable();
    leftElbow_y = new InputVariable();

    skeletonPose = new InputVariable();

    ok = new OutputVariable();
    spinPTU = new OutputVariable();


    skeletonCenterEngine = new Engine();
    rulesSkeletonCenter = new RuleBlock();

    spinRightEngine = new Engine();
    rulesSpinRight = new RuleBlock();

    spinLeftEngine = new Engine();
    rulesSpinLeft = new RuleBlock();

    centerLeftHandEngine = new Engine();
    rulesCenterLeftHand = new RuleBlock();

    highLeftHandEngine = new Engine();
    rulesHighLeftHand = new RuleBlock();

    downLeftHandEngine = new Engine();
    rulesDownLeftHand = new RuleBlock();
    //new
    centerRightHandEngine = new Engine();
    rulesCenterRightHand = new RuleBlock();
    //new
    highRightHandEngine = new Engine();
    rulesHighRightHand = new RuleBlock();
    //new
    downRightHandEngine = new Engine();
    rulesDownRightHand = new RuleBlock();

    centerHandsEngine = new Engine();
    rulesCenterHands = new RuleBlock();

    separateHandsEngine = new Engine();
    rulesSeparateHands = new RuleBlock();

    closeHandsEngine = new Engine();
    rulesCloseHands = new RuleBlock();

    sameHeightHandsEngine = new Engine();
    rulesSameHeightHands = new RuleBlock();
}

FuzzySystem::~FuzzySystem(){

}

void FuzzySystem::setValuesPoints(const ReferencePoints &referencePoints){
    head_y = referencePoints.getHead().getPosition().y;
    neck_y = referencePoints.getNeck().getPosition().y;
    torso_y = referencePoints.getTorso().getPosition().y;
    hip_y = referencePoints.getHip().getPosition().y;
    knee_y = referencePoints.getKnee().getPosition().y;

    min_y = torso_y - (head_y - torso_y);
    max_y = head_y + (head_y-torso_y);

    neck_x = referencePoints.getNeck().getPosition().x;
    leftShoulder_x = referencePoints.getLeftShoulder().getPosition().x;
    rightShoulder_x = referencePoints.getRigthShoulder().getPosition().x;
    min_x = neck_x - abs(leftShoulder_x - rightShoulder_x)*4;
    max_x = neck_x + abs(leftShoulder_x - rightShoulder_x)*4;

    error = referencePoints.getError();
    downElbow =hip_y+(abs(torso_y-hip_y))/4;
    centerElbow = torso_y-(abs(torso_y-hip_y))/4;
    highElbow = centerElbow+abs(centerElbow-downElbow);
    downHand = hip_y-(abs(hip_y-knee_y))/2;
    leftHand = leftShoulder_x - abs(rightShoulder_x -leftShoulder_x);
    rightHand = rightShoulder_x + abs(rightShoulder_x -leftShoulder_x);

    size_x = referencePoints.getSizeX();


}

void FuzzySystem::setVariables(){

    rightHand_x->setEnabled(true);
    rightHand_x->setName("rightHand_x");
    rightHand_x->setRange(min_x, max_x);
    cout << "rightHand_x: " <<  min_x << " " << max_x << endl <<
                min_x << " " << min_x << " " << neck_x  <<" "<< rightShoulder_x << endl <<
                neck_x +error<< " " << rightShoulder_x << " " << rightShoulder_x+error*(1/2)  <<" "<< rightHand+error << endl <<
                rightShoulder_x << " " << rightHand << " " << max_x <<" " << max_x << endl;
    rightHand_x->addTerm(new Trapezoid( "left", min_x ,min_x, neck_x, rightShoulder_x));
    rightHand_x->addTerm(new Trapezoid( "center", neck_x+error, rightShoulder_x, rightShoulder_x+error*(1/2), rightHand+error));
    rightHand_x->addTerm(new Trapezoid( "right", rightShoulder_x, rightHand, max_x, max_x));

    leftHand_x->setEnabled(true);
    leftHand_x->setName("leftHand_x");
    leftHand_x->setRange(min_x, max_x);
    leftHand_x->addTerm(new Trapezoid("left", min_x, min_x, leftHand-error, leftShoulder_x));
    leftHand_x->addTerm(new Trapezoid("center", leftHand, leftShoulder_x-error, leftShoulder_x+error*(1/2), neck_x));
    leftHand_x->addTerm(new Trapezoid("right", leftShoulder_x, neck_x, max_x, max_x));

    cout << "leftHand_x: " <<  min_x << " " << max_x <<" "  << endl <<
                min_x << " " << min_x << " " << leftHand-error<<" "  << leftShoulder_x << endl <<
                leftHand << " " << leftShoulder_x-error << " " << leftShoulder_x+error*(1/2)<<" "  << neck_x << endl <<
                leftShoulder_x << " " << neck_x << " " << max_x <<" " << max_x << endl;

    rightHand_y->setEnabled(true);
    rightHand_y->setName("rightHand_y");
    rightHand_y->setRange(knee_y-error,head_y+error);
    rightHand_y->addTerm(new Trapezoid("down",min_y, min_y, hip_y ,torso_y));
    rightHand_y->addTerm(new Trapezoid("center",hip_y,torso_y-error,torso_y+error,neck_y));
    rightHand_y->addTerm(new Trapezoid("high",torso_y,neck_y,max_y,max_y));
    // cout << "rightHand_y: " <<  min_y << " " << max_y << endl <<
    //              min_y << " " << min_y << " " << hip_y <<" "  << torso_y << endl <<
    //             hip_y << " " << torso_y-error << " " << torso_y+error<<" "  << neck_y << endl <<
    //             torso_y << " " << neck_y << " " << max_y<<" "  << max_y << endl;

    leftHand_y->setEnabled(true);
    leftHand_y->setName("leftHand_y");
    leftHand_y->setRange(min_y,max_y);
    leftHand_y->addTerm(new Trapezoid("down",min_y, min_y, hip_y ,torso_y));
    leftHand_y->addTerm(new Trapezoid("center",hip_y,torso_y-error,torso_y+error,neck_y));
    leftHand_y->addTerm(new Trapezoid("high",torso_y,neck_y,max_y,max_y));
    // cout << "leftHand_y: " <<  min_y << " " << max_y << endl <<
    //              min_y << " " << min_y << " " << hip_y <<" "  << torso_y << endl <<
    //             hip_y << " " << torso_y-error << " " << torso_y+error<<" "  << neck_y << endl <<
    //             torso_y << " " << neck_y << " " << max_y<<" "  << max_y << endl;


    leftElbow_y->setEnabled(true);
    leftElbow_y->setName("leftElbow_y");
    leftElbow_y->setRange(min_y,max_y);
    leftElbow_y->addTerm(new Trapezoid("down",min_y,min_y, torso_y, neck_y));
    leftElbow_y->addTerm(new Trapezoid("center", torso_y,neck_y-error,neck_y+error,head_y));
    leftElbow_y->addTerm(new Trapezoid("high",neck_y,head_y,max_y, max_y));

    // cout << "leftElbow_y: " <<  min_y << " " << max_y << endl <<
    //              min_y << " " << min_y << " " << torso_y <<" "  << neck_y << endl <<
    //             torso_y << " " << neck_y-error << " " << neck_y+error<<" "  << head_y << endl <<
    //             neck_y << " " << head_y << " " << max_y<<" "  << max_y << endl;

    skeletonPose->setEnabled(true);
    skeletonPose->setName("skeletonPose");
    skeletonPose->setRange(0,size_x);
    skeletonPose->addTerm(new Trapezoid("left",0 ,0, 0.25*size_x,0.5*size_x));
    skeletonPose->addTerm(new Triangle("center",0.25*size_x ,0.5*size_x,0.75*size_x));
    skeletonPose->addTerm(new Trapezoid("right",0.5*size_x,0.75*size_x,size_x,size_x));
    //
    // cout << "SkeletonPose: " << 0 << " " << size_x << endl <<
    //         0 << " " << 0 << " " << 0.25*size_x << " " << 0.5*size_x << endl <<
    //         0.25*size_x  << " " << 0.5*size_x  << " " << 0.75*size_x << endl <<
    //         0.5*size_x << " " << 0.75*size_x << " " << size_x << " " << size_x << endl;


    ok->setEnabled(true);
    ok->setName("ok");
    ok->setRange(0.000, 1.000);
    ok->fuzzyOutput()->setAccumulation(new fl::Maximum);
    ok->setDefuzzifier(new fl::Centroid(200));
    ok->setDefaultValue(fl::nan);
    ok->setLockValidOutput(false);
    ok->setLockOutputRange(true);
    ok->addTerm(new fl::Trapezoid("no", 0.000, 0.000, 0.400, 0.600));
    ok->addTerm(new fl::Trapezoid("yes", 0.400, 0.600, 1.000, 1.000));


    spinPTU->setEnabled(true);
    spinPTU->setName("spinPTU");
    spinPTU->setRange(0.000, 1.000);
    spinPTU->fuzzyOutput()->setAccumulation(new fl::Maximum);
    spinPTU->setDefuzzifier(new fl::MeanOfMaximum(200));
    spinPTU->setDefaultValue(fl::nan);
    spinPTU->setLockValidOutput(false);
    spinPTU->setLockOutputRange(true);
    spinPTU->addTerm(new fl::Trapezoid("left", 0.000, 0.000, 0.330, 0.500));
    spinPTU->addTerm(new fl::Triangle("center", 0.330, 0.500, 0.670));
    spinPTU->addTerm(new fl::Trapezoid("right", 0.500, 0.670,1, 1));

}



void FuzzySystem::setSkeletonCenterEngine(){

    skeletonCenterEngine->setName("skeletonCenterEngine");
    skeletonCenterEngine->addInputVariable(skeletonPose);
    skeletonCenterEngine->addOutputVariable(spinPTU);

	rulesSkeletonCenter->setEnabled(true);
	rulesSkeletonCenter->setName("rulesSkeletonCenter");
	rulesSkeletonCenter->setConjunction(new fl::Minimum);
	rulesSkeletonCenter->setDisjunction(new fl::Maximum);
	rulesSkeletonCenter->setActivation(new fl::Minimum);
    rulesSkeletonCenter->addRule(fl::Rule::parse("if skeletonPose is left then spinPTU is left", this->skeletonCenterEngine));
    rulesSkeletonCenter->addRule(fl::Rule::parse("if skeletonPose is center then spinPTU is center", this->skeletonCenterEngine));
    rulesSkeletonCenter->addRule(fl::Rule::parse("if skeletonPose is right then spinPTU is right", this->skeletonCenterEngine));

	skeletonCenterEngine->addRuleBlock(rulesSkeletonCenter);

}

void FuzzySystem::setCenterLeftHandEngine(){

    centerLeftHandEngine->setName("centerLeftHandEngine");
    centerLeftHandEngine->addInputVariable(leftHand_y);
    centerLeftHandEngine->addInputVariable(leftHand_x);
    centerLeftHandEngine->addInputVariable(leftElbow_y);
    centerLeftHandEngine->addOutputVariable(ok);

	rulesCenterLeftHand->setEnabled(true);
	rulesCenterLeftHand->setName("rulesCenterLeftHand");
	rulesCenterLeftHand->setConjunction(new fl::Minimum);
	rulesCenterLeftHand->setDisjunction(new fl::Maximum);
	rulesCenterLeftHand->setActivation(new fl::Minimum);
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is down and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is down and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is down and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is center and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is center and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is center and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is high and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is high and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftElbow_y is high and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is down and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is down and leftHand_x is center then ok is yes", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is down and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is center and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is center and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is center and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is high and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is high and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftElbow_y is high and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is down and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is down and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is down and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is center and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is center and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is center and leftHand_x is right then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is high and leftHand_x is left then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is high and leftHand_x is center then ok is no", this->centerLeftHandEngine));
    rulesCenterLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftElbow_y is high and leftHand_x is right then ok is no", this->centerLeftHandEngine));

	centerLeftHandEngine->addRuleBlock(rulesCenterLeftHand);

}

void FuzzySystem::setHighLeftHandEngine(){

    highLeftHandEngine->setName("highLeftHandEngine");
    highLeftHandEngine->addInputVariable(leftHand_y);
    highLeftHandEngine->addInputVariable(leftHand_x);
    highLeftHandEngine->addInputVariable(leftElbow_y);
    highLeftHandEngine->addOutputVariable(ok);


	rulesHighLeftHand->setEnabled(true);
	rulesHighLeftHand->setName("rulesHighLeftHand");
	rulesHighLeftHand->setConjunction(new fl::Minimum);
	rulesHighLeftHand->setDisjunction(new fl::Maximum);
	rulesHighLeftHand->setActivation(new fl::Minimum);
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is down then ok is yes", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is center then ok is yes", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is high then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is down then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is center then ok is no", this->highLeftHandEngine));
    rulesHighLeftHand->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is high then ok is no", this->highLeftHandEngine));

	highLeftHandEngine->addRuleBlock(rulesHighLeftHand);
}

void FuzzySystem::setCenterHandsEngine(){

    centerHandsEngine->setName("centerHandsEngine");
    centerHandsEngine->addInputVariable(rightHand_x);
    centerHandsEngine->addInputVariable(leftHand_x);
    centerHandsEngine->addInputVariable(leftHand_y);
    centerHandsEngine->addOutputVariable(ok);


	rulesCenterHands->setEnabled(true);
	rulesCenterHands->setName("rulesCenterHands");
	rulesCenterHands->setConjunction(new fl::Minimum);
	rulesCenterHands->setDisjunction(new fl::Maximum);
	rulesCenterHands->setActivation(new fl::Minimum);
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is center then ok is yes", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is high then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is down then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is center then ok is no", this->centerHandsEngine));
    rulesCenterHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is high then ok is no", this->centerHandsEngine));

	centerHandsEngine->addRuleBlock(rulesCenterHands);
}


void FuzzySystem::setSeparateHandsEngine(){

    separateHandsEngine->setName("separateHandsEngine");
    separateHandsEngine->addInputVariable(rightHand_x);
    separateHandsEngine->addInputVariable(leftHand_x);
    separateHandsEngine->addInputVariable(leftHand_y);
    separateHandsEngine->addOutputVariable(ok);


	rulesSeparateHands->setEnabled(true);
	rulesSeparateHands->setName("rulesSeparateHands");
	rulesSeparateHands->setConjunction(new fl::Minimum);
	rulesSeparateHands->setDisjunction(new fl::Maximum);
	rulesSeparateHands->setActivation(new fl::Minimum);
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is left and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is center and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is left and leftHand_x is right and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is left and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is center and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is center and leftHand_x is right and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is center then ok is yes", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is left and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is center and leftHand_y is high then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is down then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is center then ok is no", this->separateHandsEngine));
    rulesSeparateHands->addRule(fl::Rule::parse("if rightHand_x is right and leftHand_x is right and leftHand_y is high then ok is no", this->separateHandsEngine));

	separateHandsEngine->addRuleBlock(rulesSeparateHands);
}


void FuzzySystem::setSameHeightHandsEngine(){

    sameHeightHandsEngine->setName("sameHeightHandsEngine");
    sameHeightHandsEngine->addInputVariable(rightHand_y);
    sameHeightHandsEngine->addInputVariable(leftHand_y);
    sameHeightHandsEngine->addOutputVariable(ok);


	rulesSameHeightHands->setEnabled(true);
	rulesSameHeightHands->setName("rulesSameHeightHands");
	rulesSameHeightHands->setConjunction(new fl::Minimum);
	rulesSameHeightHands->setDisjunction(new fl::Maximum);
	rulesSameHeightHands->setActivation(new fl::Minimum);
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is down and leftHand_y is down then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is down and leftHand_y is center then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is down and leftHand_y is high then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is center and leftHand_y is down then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is center and leftHand_y is center then ok is yes", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is center and leftHand_y is high then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is high and leftHand_y is down then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is high and leftHand_y is center then ok is no", this->sameHeightHandsEngine));
    rulesSameHeightHands->addRule(fl::Rule::parse("if rightHand_y is high and leftHand_y is high then ok is no", this->sameHeightHandsEngine));


	sameHeightHandsEngine->addRuleBlock(rulesSameHeightHands);
}

void FuzzySystem::setCenterRightHandEngine(){ //new

    centerRightHandEngine->setName("centerRightHandEngine");
    centerRightHandEngine->addInputVariable(rightHand_y);
    centerRightHandEngine->addInputVariable(rightHand_x);
    centerRightHandEngine->addInputVariable(leftElbow_y);
    centerRightHandEngine->addOutputVariable(ok);

	rulesCenterRightHand->setEnabled(true);
	rulesCenterRightHand->setName("rulesCenterRightHand");
	rulesCenterRightHand->setConjunction(new fl::Minimum);
	rulesCenterRightHand->setDisjunction(new fl::Maximum);
	rulesCenterRightHand->setActivation(new fl::Minimum);
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is down and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is down and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is down and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is center and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is center and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is center and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is high and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is high and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is down and leftElbow_y is high and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is down and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is down and rightHand_x is center then ok is yes", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is down and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is center and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is center and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is center and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is high and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is high and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is center and leftElbow_y is high and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is down and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is down and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is down and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is center and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is center and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is center and rightHand_x is right then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is high and rightHand_x is left then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is high and rightHand_x is center then ok is no", this->centerRightHandEngine));
    rulesCenterRightHand->addRule(fl::Rule::parse("if rightHand_y is high and leftElbow_y is high and rightHand_x is right then ok is no", this->centerRightHandEngine));

	centerRightHandEngine->addRuleBlock(rulesCenterRightHand);

}

void FuzzySystem::setHighRightHandEngine(){  //new

    highRightHandEngine->setName("highRightHandEngine");
    highRightHandEngine->addInputVariable(rightHand_y);
    highRightHandEngine->addInputVariable(rightHand_x);
    highRightHandEngine->addInputVariable(leftElbow_y);
    highRightHandEngine->addOutputVariable(ok);


	rulesHighRightHand->setEnabled(true);
	rulesHighRightHand->setName("rulesHighRightHand");
	rulesHighRightHand->setConjunction(new fl::Minimum);
	rulesHighRightHand->setDisjunction(new fl::Maximum);
	rulesHighRightHand->setActivation(new fl::Minimum);
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is down then ok is yes", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is center then ok is yes", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is high then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is down then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is center then ok is no", this->highRightHandEngine));
    rulesHighRightHand->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is high then ok is no", this->highRightHandEngine));

	highRightHandEngine->addRuleBlock(rulesHighRightHand);
}

void FuzzySystem::setSpinRightEngine(){

    spinRightEngine->setName("spinRightEngine");
    spinRightEngine->addInputVariable(leftHand_y);
    spinRightEngine->addInputVariable(leftHand_x);
    spinRightEngine->addInputVariable(leftElbow_y);
    spinRightEngine->addOutputVariable(ok);


	rulesSpinRight->setEnabled(true);
	rulesSpinRight->setName("rulesSpinRight");
	rulesSpinRight->setConjunction(new fl::Minimum);
	rulesSpinRight->setDisjunction(new fl::Maximum);
	rulesSpinRight->setActivation(new fl::Minimum);
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is left and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is center and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is down and leftHand_x is right and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is left and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is center and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is center and leftHand_x is right and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is center then ok is yes", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is left and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is center and leftElbow_y is high then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is down then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is center then ok is no", this->spinRightEngine));
    rulesSpinRight->addRule(fl::Rule::parse("if leftHand_y is high and leftHand_x is right and leftElbow_y is high then ok is no", this->spinRightEngine));

	spinRightEngine->addRuleBlock(rulesSpinRight);
}


void FuzzySystem::setSpinLeftEngine(){

    spinLeftEngine->setName("spinLeftEngine");
    spinLeftEngine->addInputVariable(rightHand_y);
    spinLeftEngine->addInputVariable(rightHand_x);
    spinLeftEngine->addInputVariable(leftElbow_y);
    spinLeftEngine->addOutputVariable(ok);


	rulesSpinLeft->setEnabled(true);
	rulesSpinLeft->setName("rulesSpinLeft");
	rulesSpinLeft->setConjunction(new fl::Minimum);
	rulesSpinLeft->setDisjunction(new fl::Maximum);
	rulesSpinLeft->setActivation(new fl::Minimum);
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is left and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is left and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is left and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is center and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is down and rightHand_x is right and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is left and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is left and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is left and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is center and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is center and rightHand_x is right and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is left and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is left and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is left and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is center then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is center and leftElbow_y is high then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is down then ok is no", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is center then ok is yes", this->spinLeftEngine));
    rulesSpinLeft->addRule(fl::Rule::parse("if rightHand_y is high and rightHand_x is right and leftElbow_y is high then ok is no", this->spinLeftEngine));

	spinLeftEngine->addRuleBlock(rulesSpinLeft);
}


void FuzzySystem::setFuzzySystem(const ReferencePoints &referencePoints){
    this->setValuesPoints(referencePoints);
    this->setVariables();
    this->setCenterLeftHandEngine();
    this->setHighLeftHandEngine();
    this->setCenterHandsEngine();
    this->setSeparateHandsEngine();
    this->setSameHeightHandsEngine();

    this->setCenterRightHandEngine(); //new
    this->setHighRightHandEngine(); //new
    this->setSpinRightEngine(); //new
    this->setSpinLeftEngine(); //new
    this->setSkeletonCenterEngine();

}


float FuzzySystem::processCenterLeftHandEngine(const float& leftHand_y, const float& leftHand_x, const float& leftElbow_y){
    this->leftHand_y->setInputValue(leftHand_y);
    this->leftHand_x->setInputValue(leftHand_x);
    this->leftElbow_y->setInputValue(leftElbow_y);

    this->centerLeftHandEngine->process();

    return ok->defuzzify();
}

float FuzzySystem::processHighLeftHandEngine(const float& leftHand_y, const float& leftHand_x, const float& leftElbow_y){
    this->leftHand_y->setInputValue(leftHand_y);
    this->leftHand_x->setInputValue(leftHand_x);
    this->leftElbow_y->setInputValue(leftElbow_y);

    this->highLeftHandEngine->process();

    return ok->defuzzify();
}



float FuzzySystem::processCenterHandsEngine(const float& rightHand_x, const float& leftHand_x, const float& leftHand_y){
    this->rightHand_x->setInputValue(rightHand_x);
    this->leftHand_x->setInputValue(leftHand_x);
    this->leftHand_y->setInputValue(leftHand_y);

    this->centerHandsEngine->process();

    return ok->defuzzify();
}


float FuzzySystem::processSeparateHandsEngine(const float& rightHand_x, const float& leftHand_x, const float& leftHand_y){
    this->rightHand_x->setInputValue(rightHand_x);
    this->leftHand_x->setInputValue(leftHand_x);
    this->leftHand_y->setInputValue(leftHand_y);

    this->separateHandsEngine->process();

    return ok->defuzzify();
}

float FuzzySystem::processSameHeightHandsEngine(const float& rightHand_y, const float& leftHand_y){
    this->rightHand_y->setInputValue(rightHand_y);
    this->leftHand_y->setInputValue(leftHand_y);

    this->sameHeightHandsEngine->process();

    return ok->defuzzify();
}

//new
float FuzzySystem::processCenterRightHandEngine(const float& rightHand_y, const float& rightHand_x, const float& rightElbow_y){
    this->rightHand_y->setInputValue(rightHand_y);
    this->rightHand_x->setInputValue(rightHand_x);
    this->leftElbow_y->setInputValue(rightElbow_y);

    this->centerRightHandEngine->process();

    return ok->defuzzify();
}
//new
float FuzzySystem::processHighRightHandEngine(const float& rightHand_y, const float& rightHand_x, const float& rightElbow_y){
    this->rightHand_y->setInputValue(rightHand_y);
    this->rightHand_x->setInputValue(rightHand_x);
    this->leftElbow_y->setInputValue(rightElbow_y);

    this->highRightHandEngine->process();

    return ok->defuzzify();
}

float FuzzySystem::processSpinRightEngine(const float& leftHand_y, const float& leftHand_x, const float& leftElbow_y){
    this->rightHand_y->setInputValue(leftHand_y);
    this->rightHand_x->setInputValue(leftHand_x);
    this->leftElbow_y->setInputValue(leftElbow_y);

    this->spinRightEngine->process();

    return ok->defuzzify();
}

float FuzzySystem::processSpinLeftEngine(const float& rightHand_y, const float& rightHand_x, const float& rightElbow_y){
    this->rightHand_y->setInputValue(rightHand_y);
    this->rightHand_x->setInputValue(rightHand_x);
    this->leftElbow_y->setInputValue(rightElbow_y);

    this->spinLeftEngine->process();

    return ok->defuzzify();
}

float FuzzySystem::processSkeletonCenterEngine(const float& centerMass_x ){
    this->skeletonPose->setInputValue(centerMass_x);

    this->skeletonCenterEngine->process();
    return spinPTU->defuzzify();
}
