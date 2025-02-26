
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelightc.*;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  private final Drivetrain drivetrain;


  // NEEDS TO SEE: Barge, Reef, Processor, Coral Dropoff
  public Limelight(Drivetrain drivetrain) {
    this.drivetrain = drivetrain;
    LimelightHelpers.SetFiducialIDFiltersOverride(ROBOT_LL_NAME, VALID_IDS);

    SmartDashboard.putBoolean("Robot sees a Tag", seesTag()); // I need to improve this...
    SmartDashboard.putNumber("Distance to Processor", getDistanceToApriltag(ROBOT_LL_NAME, MOUNT_ANGLE, 0, LENS_HEIGHT_FROM_GROUND_METERS));
  }
  @Override
  public void periodic() {
    SmartDashboard.putBoolean("Robot sees a Tag", seesTag());
    SmartDashboard.putNumber("Distance to Processor", getDistanceToApriltag(ROBOT_LL_NAME, MOUNT_ANGLE, 0, LENS_HEIGHT_FROM_GROUND_METERS));
  }
  // Very helpful accessors
  public double getTXDeg() {
    return LimelightHelpers.getTX(ROBOT_LL_NAME);
  }

  public double getTYDeg() {
    return LimelightHelpers.getTY(ROBOT_LL_NAME);
  }

  // Distance accessors for field areas
  public double getDistanceToApriltag(String limelightName, double mountAngle, double tagHeight, double limelightHeight) {
    if (!seesTag()) {
      return -1;
    }
    else {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(mountAngle).plus(Rotation2d.fromDegrees(getTYDeg()));
      double distance = (tagHeight - limelightHeight) / angleToGoal.getTan();
      return distance;
    }
  }

  public double getApriltagID(String limelightName) {
    return LimelightHelpers.getFiducialID(limelightName);
  }

  public double getRotateAngleRadMT2(String limelightName) {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(limelightName); // pose of the target

    double targetX = targetPoseRobotSpace.getX(); // the forward offset between the center of the
                                                  // robot and target
    double targetZ = -targetPoseRobotSpace.getZ(); // the sideways offset

    double targetOffsetRads = MathUtil.inputModulus(Math.atan2(targetX, targetZ), -Math.PI, Math.PI);

    return targetOffsetRads;
  }

  public boolean seesTag() {
    return LimelightHelpers.getTV(ROBOT_LL_NAME);
  }
}