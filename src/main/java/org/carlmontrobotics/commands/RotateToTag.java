// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Limelight;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RotateToTag extends Command {
  private Drivetrain drivetrain;
  private Limelight limelight;
  public RotateToTag(Drivetrain drivetrain, Limelight limelight) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.limelight = limelight);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (limelight.seesTag(REEF_LL)) {
      double radiansOff = Units.degreesToRadians(limelight.getTx(REEF_LL));
      drivetrain.drive(0.0000001, 0, radiansOff*2);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    if (!limelight.seesTag(REEF_LL)) {
      return true;
    }
    return Math.abs(limelight.getTx(REEF_LL)) < 1;
  }
}
