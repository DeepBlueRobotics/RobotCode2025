
package org.carlmontrobotics.subsystems;

import static org.carlmontrobotics.Constants.Limelightc.*;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {


  public Limelight() {
    LimelightHelpers.SetFiducialIDFiltersOverride(CORAL_LL, CORAL_VALID_IDS);
    LimelightHelpers.SetFiducialIDFiltersOverride(CORAL_LL, REEF_VALID_IDS);
  }

  public double getTx(String cameraName) {
    return LimelightHelpers.getTX(cameraName);
  }
  public double getX(String cameraName) {
    return LimelightHelpers.getCameraPose3d_RobotSpace(cameraName).getX();
  }
  public double getZ(String cameraName) {
    return LimelightHelpers.getCameraPose3d_RobotSpace(cameraName).getZ();
  }
  public boolean seesTag(String cameraName) {
    return LimelightHelpers.getTV(cameraName);
  }
}