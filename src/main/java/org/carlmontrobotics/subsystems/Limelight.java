// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.carlmontrobotics.Constants.Limelightc.*;

import com.pathplanner.lib.path.PathPlannerPath;

public class Limelight extends SubsystemBase {
  
    /** Creates a new Limelight. */
    public Limelight(Drivetrain drivetrain) {
      this.drivetrain = drivetrain;

      LimelightHelpers.SetFiducialIDFiltersOverride(REEF_LL, REEF_IDS);
    }
  
    @Override
    public void periodic() {
      // This method will be called once per scheduler run
      LimelightHelpers.getTargetPose_RobotSpace(REEF_LL);
  }

  public double getRotateAngleRadMT2() {
    Pose3d targetPoseRobotSpace = LimelightHelpers.getTargetPose3d_RobotSpace(REEF_LL); // pose of the target

    double targetX = targetPoseRobotSpace.getX(); // the forward offset between the center of the
                                                  // robot and target
    double targetZ = -targetPoseRobotSpace.getZ(); // the sideways offset

    double targetOffsetRads = MathUtil.inputModulus(Math.atan2(targetX, targetZ), -Math.PI, Math.PI);

    return targetOffsetRads;
  }

  // aligh to apriltag
  public boolean seesTag() {
    return LimelightHelpers.getTV(REEF_LL);
  }

  public PathPlannerPath whatthefuck() {
    LimelightHelpers.getTargetPose3d_RobotSpace(REEF_LL);



    return new PathPlannerPath(null, null, null, null);
  }
}
