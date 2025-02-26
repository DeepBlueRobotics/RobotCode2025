// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToApriltag extends Command {
  private Limelight limelight;
  private final Drivetrain drivetrain;

  public final PIDController rotationPID = new PIDController(thetaPIDController[0], thetaPIDController[1], thetaPIDController[2]);

  double RotationSpeed = 0.0;
  
  public AlignToApriltag(Drivetrain drivetrain, Limelight limelight, double RotationSpeed) {
    this.limelight = limelight;
    this.drivetrain = drivetrain;
    this.RotationSpeed = RotationSpeed;
    this.teleopDrive = (TeleopDrive) drivetrain.getDefaultCommand();

    rotationPID.enableContinuousInput(-180, 180);
    Rotation2d targetAngle = Rotation2d.fromDegrees(drivetrain.getHeading()).plus(Rotation2d.fromRadians(limelight.getRotateAngleRadMT2()));
    rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
    rotationPID.setTolerance(positionTolerance[2], velocityTolerance[2]);
    SendableRegistry.addChild(this, rotationPID);

    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {}

  @Override
  public void execute() {
    Rotation2d targetAngle = Rotation2d
      .fromDegrees(drivetrain.getHeading())
      .plus(Rotation2d.fromRadians(limelight.getRotateAngleRadMT2()));
      rotationPID.setSetpoint(MathUtil.inputModulus(targetAngle.getDegrees(), -180, 180));
      double rotationDrive = rotationPID.calculate(drivetrain.getHeading());
      if (!limelight.seesTag()) {
        rotationDrive = RotationSpeed;
      }
      if (rotationPID.atSetpoint()) {
        rotationDrive = 0;
      }
      if (teleopDrive == null) {
        drivetrain.drive(0, 0, rotationDrive);
      }
      else {
        double[] driverRequestedSpeeds = teleopDrive.getRequestedSpeeds();
        drivetrain.drive(driverRequestedSpeeds[0], driverRequestedSpeeds[1], rotationDrive);
      }
  }

  @Override
  public void end(boolean interrupted) {}

  @Override
  public boolean isFinished() {
    return rotationPID.atSetpoint();
  }
}
