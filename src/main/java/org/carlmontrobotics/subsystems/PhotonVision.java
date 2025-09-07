// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import java.util.Optional;


import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.proto.PhotonPipelineResultProto;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.proto.Photon;
import org.photonvision.PhotonUtils;

public class PhotonVision extends SubsystemBase {
  /** Creates a new PhotonVision. */
  //name of the camera in the PhotonVision UI
  private PhotonCamera camera = new PhotonCamera("DriverCam"); 
  //layout of the field
  private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); 
  //transation from the center of the robot to the camera position
  //TODO ask design for translation
  private final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.0, 0.0, 0.0), new Rotation3d(0, 0, 0)); 
  //transformation from the camera to the center of the robot
  private final Transform3d kCamToRobot = kRobotToCam.inverse();
  private List<PhotonPipelineResult> latestPipelineResult = new ArrayList<>();
  /**
    * estimates the robot's pose on the field using the apriltags
    * uses MULTI_TAG_PNP_ON_COPROCESSOR which takes in account all the apriltags that the camera can sees and uses all of them to estiamte the position 
    * change it to LOWEST_AMBIGUITY if pi is too slow (I think it should be fine though)
   */
  private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);
  public PhotonVision() {
  }
  /** 
   * do NOT use this method anywhere except once in the periodic method in the photonvision subsytem, there is a reason why it is private.
   * use getResultList instead
   * @return all the apriltag targets that the camera can see 
   */
  private List<PhotonPipelineResult> getUnreadResultsList() {
    return camera.getAllUnreadResults();
  }

  private List<PhotonPipelineResult> getResultList() { 
    return latestPipelineResult;
  }

  /**
   * @return true if any of the latest pipeline results have targets, false otherwise
   */
  public boolean hasTarget() {
    for (PhotonPipelineResult result : latestPipelineResult) { //loops through all the results (I think "reults" are cameras)
      if (result.hasTargets()) {
        return true;
      }
    }
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latestPipelineResult = getUnreadResultsList();
  }
}
