
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Drivetrainc.*;
import static org.carlmontrobotics.Constants.Limelightc.*;
import static org.carlmontrobotics.Constants.Limelightc.Apriltag.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;

  private final InterpolatingDoubleTreeMap shooterMap;

  private final double[] distances = {-1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1};

  // NEEDS TO SEE: Barge, Reef, Processor, Coral Dropoff
  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    LimelightHelpers.SetFiducialIDFiltersOverride(ROBOT_LL_NAME, VALID_IDS);

    shooterMap = new InterpolatingDoubleTreeMap(); // ADD VALUES AFTER TESTING!!!
    // key is distance (meters), value is angle (rads)
    shooterMap.put(2.06, .01);
    shooterMap.put(3.05, 0.26);
    shooterMap.put(2.5, 0.23);
    shooterMap.put(1.39, -0.18);
    shooterMap.put(3.45, 0.3);
    shooterMap.put(3.1, 0.3);

    SmartDashboard.putBoolean("Robot sees a Tag", seesTag()); // I need to improve this...
    SmartDashboard.putNumber("Distance to Processor", getDistanceToProcessorMeters());
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Robot sees a Tag", seesTag());
    SmartDashboard.putNumber("Distance to Processor", getDistanceToProcessorMeters());
  }
  // Very helpful accessors
  public double getTXDeg() {
    return LimelightHelpers.getTX(ROBOT_LL_NAME);
  }

  public double getTYDeg() {
    return LimelightHelpers.getTY(ROBOT_LL_NAME);
  }

  // Distance accessors for field areas
  public double getDistanceToProcessorMeters() {
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_PROCESSOR_ID
    || LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_PROCESSOR_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      return distance;
    }
    else {
      return -1;
    }
  }

  public double getDistanceToReefMeters() {
    // I should really iterate this through a for loop...
    // Red Reef Far Right (id 6)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_FAR_RIGHT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[0] = distance;
    }
    else {
      distances[0] = -1;
    }
    //Red Reef Far (id 7)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_FAR_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[1] = distance;
    }
    else {
      distances[1] = -1;
    }
    // Red Reef Far Left (id 8)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_FAR_LEFT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[2] = distance;
    }
    else {
      distances[2] = -1;
    }
    // Red Reef Close Left (id 9)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_CLOSE_LEFT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[3] = distance;
    }
    else {
      distances[3] = -1;
    }
    // Red Reef Close (id 10)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_CLOSE_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[4] = distance;
    }
    else {
      distances[4] = -1;
    }
    // Red Reef Close Right (id 11)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_REEF_CLOSE_RIGHT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[5] = distance;
    }
    else {
      distances[5] = -1;
    }
    // Blue Reef Far Left (id 17)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_FAR_LEFT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[6] = distance;
    }
    else {
      distances[6] = -1;
    }
    //Blue Reef Far (id 18)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_FAR_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[7] = distance;
    }
    else {
      distances[7] = -1;
    }
    // Blue Reef Far Right (id 19)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_FAR_RIGHT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[8] = distance;
    }
    else {
      distances[8] = -1;
    }
    // Blue Reef Close Right (id 20)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_CLOSE_RIGHT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[9] = distance;
    }
    else {
      distances[9] = -1;
    }
    // Blue Reef Close (id 21)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_CLOSE_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[10] = distance;
    }
    else {
      distances[10] = -1;
    }
    // Blue Reef Close Left (id 22)
    if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_REEF_CLOSE_LEFT_ID) {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
      distances[11] = distance;
    }
    else {
      distances[11] = -1;
    }
    // The part that does most of the fun stuff!
    double minDist = 99999.999; //Set unreasonably high
    // Finding the closest distance (not equal to -1)
    for (double distValue : distances ) {
      if(distValue!=-1 && distValue < minDist) {
        minDist = distValue;
      }
    }
    return minDist;
  }

  public double getDistanceToAlgaeMeters() {
    Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg()));
    if (angleToGoal.getDegrees() >= 0) {
      double distance = (HEIGHT_FROM_GROUND_METERS_ROBOT - ALGAE_HEIGHT)/Math.tan(Math.abs(angleToGoal.getRadians()));
      return distance;
    }
    else {
      return -1;
    }
  }

  public boolean seesTag() {
    return LimelightHelpers.getTV(ROBOT_LL_NAME);
  }
}