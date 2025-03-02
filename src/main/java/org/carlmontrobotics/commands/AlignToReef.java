package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Drivetrainc.*;
import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReef extends Command {

  public final Drivetrain drivetrain;
  public final TeleopDrive teleopDrive;
  private Timer time = new Timer();
  private Limelight limelight;
  private double startTime;
  public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2]);
  double RotationSpeed = 0.0;
  public AlignToReef(Drivetrain drivetrain, Limelight limelight, double RotationSpeed) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.RotationSpeed = RotationSpeed;
    this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();
    rotationPID.enableContinuousInput(-180, 180);
    Rotation2d targetAngle = Rotation2d.fromDegrees(drivetrain.getHeading()).plus(Rotation2d.fromRadians(limelight.getRotateAngleRadMT2(REEF_LL)));
    rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
    rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
    SendableRegistry.addChild(this, rotationPID);
    time.reset();
    time.start();
    startTime = Timer.getFPGATimestamp();
    addRequirements(drivetrain);
  }
  @Override
  public void execute() {
    // double kp = SmartDashboard.getNumber("apriltag align kp",
    // rotationPID.getP());
    // double ki = SmartDashboard.getNumber("apriltag align ki",
    // rotationPID.getI());
    // double kd = SmartDashboard.getNumber("apriltag align kd",
    // rotationPID.getD());
    // if (kp != rotationPID.getP())
    // rotationPID.setP(kp);
    // if (ki != rotationPID.getI())
    // rotationPID.setI(ki);
    // if (kd != rotationPID.getD())
    // rotationPID.setD(kd);
    // double posTolerance = SmartDashboard.getNumber(
    // "apriltag align pos tolerance",
    // rotationPID.getPositionTolerance());
    // double velTolerance = SmartDashboard.getNumber(
    // "apriltag align vel tolerance",
    // rotationPID.getVelocityTolerance());
    // if (posTolerance != rotationPID.getPositionTolerance()
    // || velTolerance != rotationPID
    // .getVelocityTolerance())
    // rotationPID.setTolerance(posTolerance, velTolerance);
    // SmartDashboard.putNumber("apriltag align pos error (rad)",
    // rotationPID.getPositionError());
    // SmartDashboard.putNumber("apriltag align vel error (rad/s)",
    // rotationPID.getVelocityError());

    Rotation2d targetAngle = Rotation2d.fromDegrees(drivetrain.getHeading()).plus(Rotation2d.fromRadians(limelight.getRotateAngleRadMT2("limelight-reef")));
    rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
    double rotationDrive = rotationPID.calculate(drivetrain.getHeading());
    if (!limelight.seesTag(REEF_LL)) rotationDrive = RotationSpeed;
    if (rotationPID.atSetpoint()) rotationDrive = 0;
    if (teleopDrive == null) drivetrain.drive(0, 0, rotationDrive);
    else {
      double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
      drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1], rotationDrive);
    }
  }

  @Override
  public boolean isFinished() {
    // return false;
    // SmartDashboard.putBoolean("At Setpoint", rotationPID.atSetpoint());
    // SmartDashboard.putNumber("Error", rotationPID.getPositionError());
    return rotationPID.atSetpoint() || (Timer.getFPGATimestamp() - startTime) > 3;
  }
}