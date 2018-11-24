//draw Join



void drawBone(cv::Mat out ,  NUI_SKELETON_DATA  skeleton,  NUI_SKELETON_POSITION_INDEX jointFrom,NUI_SKELETON_POSITION_INDEX jointTo ){

	  NUI_SKELETON_POSITION_TRACKING_STATE jointFromState = skeleton.eSkeletonPositionTrackingState[jointFrom];

	  NUI_SKELETON_POSITION_TRACKING_STATE jointToState = skeleton.eSkeletonPositionTrackingState[jointTo];

	  if (jointFromState == NUI_SKELETON_POSITION_NOT_TRACKED || jointToState == NUI_SKELETON_POSITION_NOT_TRACKED){
          return; // nothing to draw, one of the joints is not tracked
        }

	   // Don't draw if both points are inferred
        if (jointFromState == NUI_SKELETON_POSITION_INFERRED || jointToState == NUI_SKELETON_POSITION_INFERRED){
			cv::Point2f pointFrom;
			NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[jointFrom], &pointFrom.x, &pointFrom.y, NUI_IMAGE_RESOLUTION_640x480 );
			cv::Point2f pointTo;
			NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[jointTo], &pointTo.x, &pointTo.y, NUI_IMAGE_RESOLUTION_640x480 );
			cv::line(out,pointFrom, pointTo, static_cast<cv::Scalar>( cv::Vec3b(   0, 0,  255 ) ),2,CV_AA);
			cv::circle( out, pointTo, 5, static_cast<cv::Scalar>(cv::Vec3b(   0, 255,  255 )), -1, CV_AA );
		}

		 // We assume all drawn bones are inferred unless BOTH joints are tracked
        if (jointFromState == NUI_SKELETON_POSITION_TRACKED && jointToState == NUI_SKELETON_POSITION_TRACKED)
        {
			cv::Point2f pointFrom;
			NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[jointFrom], &pointFrom.x, &pointFrom.y, NUI_IMAGE_RESOLUTION_640x480 );
			cv::Point2f pointTo;
			NuiTransformSkeletonToDepthImage( skeleton.SkeletonPositions[jointTo], &pointTo.x, &pointTo.y, NUI_IMAGE_RESOLUTION_640x480 );
			//dibujamos una linea que entre los dos punto
			cv::line(out,pointFrom, pointTo, static_cast<cv::Scalar>( cv::Vec3b(   0, 255,   0 ) ),2,CV_AA);
			//en donde inicia cada linea dibujamos un circulo
			cv::circle( out, pointTo, 5, static_cast<cv::Scalar>(cv::Vec3b(   0, 255,  255 )), -1, CV_AA );
		}

}

//draw the body
void drawSkeleton(cv::Mat &out,  nite::Skeleton skeleton){
	 //Head and Shoulders (cabeza y espalda)
     cv::line(out, skeleton.getJoint(nite::JOINT_HEAD),skeleton.getJoint(nite::JOINT_NECK), static_cast<cv::Scalar>( cv::Vec3b(   0, 0,  255 ) ),2,CV_AA);
      // drawBone(out, skeleton, NUI_SKELETON_POSITION_HEAD, NUI_SKELETON_POSITION_SHOULDER_CENTER);
      // drawBone(out, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_LEFT);
      // drawBone(out, skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SHOULDER_RIGHT);
      //
	  // //hip(cadera)
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_SHOULDER_CENTER, NUI_SKELETON_POSITION_SPINE);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_SPINE, NUI_SKELETON_POSITION_HIP_CENTER);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_LEFT);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_HIP_CENTER, NUI_SKELETON_POSITION_HIP_RIGHT);
      //
	  // //Knee (rodillas)
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_HIP_LEFT, NUI_SKELETON_POSITION_KNEE_LEFT);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_HIP_RIGHT, NUI_SKELETON_POSITION_KNEE_RIGHT);
      //
	  //  //Ankle (rodillas)
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_KNEE_LEFT, NUI_SKELETON_POSITION_ANKLE_LEFT);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_KNEE_RIGHT, NUI_SKELETON_POSITION_ANKLE_RIGHT);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_ANKLE_LEFT, NUI_SKELETON_POSITION_FOOT_LEFT);
	  // drawBone(out,skeleton, NUI_SKELETON_POSITION_ANKLE_RIGHT, NUI_SKELETON_POSITION_FOOT_RIGHT);
      //
      //
	  // //Left Arm(brazo izquierdo)
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_SHOULDER_LEFT, NUI_SKELETON_POSITION_ELBOW_LEFT);
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_ELBOW_LEFT, NUI_SKELETON_POSITION_WRIST_LEFT);
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_WRIST_LEFT, NUI_SKELETON_POSITION_HAND_LEFT);
      //
      // //Right Arm(brazo derecho)
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_SHOULDER_RIGHT, NUI_SKELETON_POSITION_ELBOW_RIGHT);
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_ELBOW_RIGHT, NUI_SKELETON_POSITION_WRIST_RIGHT);
      // drawBone(out,skeleton, NUI_SKELETON_POSITION_WRIST_RIGHT, NUI_SKELETON_POSITION_HAND_RIGHT);




}
