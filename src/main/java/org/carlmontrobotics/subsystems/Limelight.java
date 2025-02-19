
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

    SmartDashboard.putBoolean("Robot sees Processor", LimelightHelpers.getTV(ROBOT_LL_NAME));

    public void periodic() {

      SmartDashboard.putBoolean("Robot sees Processor", LimelightHelpers.getTV(ROBOT_LL_NAME));
    }
    // Very helpful accessors
    public double getTXDeg() {
      return LimelightHelpers.getTX(ROBOT_LL_NAME);
    }

    public double getTYDeg() {
      return LimelightHelpers.getTY(ROBOT_LL_NAME);
    }

    // Distance accessors for other subsystems
    public double getDistanceToProcessorMeters() {
      if (LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == RED_PROCESSOR_ID
      || LimelightHelpers.getFiducialID(ROBOT_LL_NAME) == BLUE_PROCESSOR_ID) {
        Rotation2d.angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg(ROBOT_LL_NAME)));
        double distance = (PROCESSOR_CENTER_HEIGHT_METERS - HEIGHT_FROM_GROUND_METERS_ROBOT) / angleToGoal.getTan();
        return distance;
      }
      else {
        return -1;
      }
    }

    public double getDistanceToAlgaeMeters() {
      Rotation2d angleToGoal = Rotation2d.fromDegrees(MOUNT_ANGLE_DEG_LL).plus(Rotation2d.fromDegrees(getTYDeg(ROBOT_LL_NAME)));
      if (angleToGoal.getDegrees >= 0) {
        double distance = (HEIGHT_FROM_GROUND_METERS_ROBOT - ALGAE_HEIGHT)/Math.tan(Math.abs(angleToGoal.getRadians()));
      }
      else {
        return -1;
      }
    }

    public boolean seesTag() {
      return LimelightHelpers.getTV(ROBOT_LL_NAME);
    }
  }
}