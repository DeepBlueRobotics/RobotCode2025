// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Limelight;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveToleftTag extends Command {
  /** Creates a new drivetotag. */
  private Drivetrain drivetrain;
  private Limelight limelight;
  private double strafeError = 100;
  private double distanceZ = 100;
  public DriveToleftTag(Drivetrain drivetrain, Limelight limelight){
    addRequirements(this.drivetrain = drivetrain);
    addRequirements(this.limelight = limelight);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(limelight.seesTag(REEF_LL)){
      distanceZ = limelight.getZ(REEF_LL);
      strafeError = Math.tan(limelight.getTx(REEF_LL))*distanceZ;
      drivetrain.drive(distanceZ, strafeError+Constants.Limelightc.LEFT_CORAL_BRANCH, 0);
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
      return (distanceZ*100 < 2 && strafeError*100 < 2) || !limelight.seesTag(REEF_LL);
  }
}
