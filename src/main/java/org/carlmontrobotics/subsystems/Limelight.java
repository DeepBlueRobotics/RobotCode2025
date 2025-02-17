
package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.subsystems.LimelightHelpers.LimelightResults;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Limelight extends SubsystemBase {
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  LimelightHelpers helper = new LimelightHelpers();
  double tx;
  double currDirection;
  Drivetrain drivetrain;
  double targetAngle
  PIDController rotationPID = new PIDController(kP, kI, kD);

  public Limelight (Drivetrain drivetrain){
    this.drivetrain = drivetrain;
  }

  public Pose2d getPose2D() {
    return helper.toPose2D(table.getEntry("botposeorb").getDoubleArray(new double array[6]));
  }

  public boolean aprilTagDetected() {
    return (table.getEntry("tv").getDouble(0) == 1);
  }

  public int getAprilTagId() {
    return table.getEntry("tid").getDouble(0);
  }

  public void alignRotation() {
    tx = table.getEntry("tx").getDouble(0);
    currDirection = getHeading();
    targetAngle = currDirection + tx;
    drive(0,0,rotationPID.calculate(currDirection, targetAngle));
  }

  
}
