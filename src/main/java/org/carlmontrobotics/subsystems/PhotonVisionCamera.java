// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

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

//NOTE: This is just me messing around with PhotonVision, no anything that is supposed to be used
public class PhotonVisionCamera extends SubsystemBase {

  Drivetrain drivetrain;

  private PhotonCamera camera; 
  private final AprilTagFieldLayout kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField); //layout of the field
  private final Transform3d kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0)); //find values later and make it configurable (or not, if we have multiple cameras) with a CONFIG (I am too lazy to do it now)
  private final Transform3d kCamToRobot = kRobotToCam.inverse();
  private PhotonPoseEstimator photonEstimator = new PhotonPoseEstimator(kTagLayout, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, kRobotToCam);


  

  List<PhotonPipelineResult> latestResult;

  public PhotonVisionCamera(Drivetrain drivetrain, String cameraName) { //maybe will not make camera passable if we end up using multiple cameras TBD, for now for simplicity it is
    this.drivetrain = drivetrain;
    this.camera = new PhotonCamera(cameraName);
  }

  public List<PhotonPipelineResult> getUnreadResultsList() {
    return camera.getAllUnreadResults();
  }

  public PhotonPipelineResult getCameraLatestResult(){ //getLatestResult() is defined in PhotoVision so it is named getCameraLatestResult() to avoid conflicts
    return latestResult.get(latestResult.size() - 1);
  }

  public PhotonTrackedTarget getBestTarget() {
    if (getCameraLatestResult().hasTargets()) {
      return getCameraLatestResult().getBestTarget();
    }
    return null; // No targets available
}

  enum PhotonDataTypes {
    yaw,
    pitch,
    area,
    skew
  }

  public int getTargetID(PhotonCamera camera, PhotonTrackedTarget target) {
    return target.getFiducialId();
  }

  public double getData(PhotonTrackedTarget target, PhotonDataTypes dataType) {
    switch (dataType) {
      case yaw:
        return target.getYaw();
      case pitch:
        return target.getPitch();
      case area:
        return target.getArea();
      case skew:
        return target.getSkew();
      default:
        System.out.println("Invalid data type");
        return 0;
    }
  }

  public Pose3d getPosRelToTarget(PhotonTrackedTarget target){
    if (kTagLayout.getTagPose(target.getFiducialId()).isPresent()) {
      Pose3d robotPose = PhotonUtils.estimateFieldToRobotAprilTag(target.getBestCameraToTarget(), kTagLayout.getTagPose(target.getFiducialId()).get(), kCamToRobot);
      return robotPose;
    } else {
      System.out.println("Tag not found");
      return null;
    }
  }
  public Pose3d getRobotPos3DWithCamera(PhotonTrackedTarget target) { //change if we get multicamera setup
   return getPosRelToTarget(target);
  }

    public Pose3d getRobotPos3D() { //test if odometry actually works on this
Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
    if (estimatedPose.isPresent()) {
      return estimatedPose.get().estimatedPose;
    }
    return getPosRelToTarget(getBestTarget());
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
    photonEstimator.setReferencePose(drivetrain.getPose());
    PhotonPipelineResult latestResult = getCameraLatestResult();
    return photonEstimator.update(latestResult);
  }

  // public void EstimateFieldToRobot(PhotonTrackedTarget target){
  //   Pose3d targetPose3d = aprilTagFieldLayout.getTagPose(target.getFiducialId()).get();
  //   Pose2d targetPose = targetPose3d.toPose2d(); // Convert to 2D pose
  //   Pose2d robotPose = PhotonUtils.estimateFieldToRobot(1, 1, 0, 0, Rotation2d.fromDegrees(getData(target, PhotonDataTypes.yaw)), drivetrain.getRotation2d(), targetPose, kCamToRobot);
  // }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    latestResult = getUnreadResultsList();


    // Only run if we have results
    if (latestResult != null && !latestResult.isEmpty()) {
        Optional<EstimatedRobotPose> estimatedPose = getEstimatedGlobalPose();
        if (estimatedPose.isPresent()) {
            drivetrain.addVisionMeasurement(
                estimatedPose.get().estimatedPose.toPose2d(), 
                estimatedPose.get().timestampSeconds
            );
        }
    }
}
}
