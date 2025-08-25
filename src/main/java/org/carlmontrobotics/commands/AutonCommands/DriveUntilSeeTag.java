// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AutonCommands;



import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DriveUntilSeeTag extends Command {
  /** Creates a new DriveUntilSeeTag. */
  Drivetrain drivetrain;
  Limelight limelight;
  int tagWantedRed;
  int tagWantedBlue;
  double fwdSpeed;
  double strSpeed;
  Timer killTimer = new Timer();
  public DriveUntilSeeTag(Drivetrain drivetrain, Limelight limelight, int tagWantedRed, int tagWantedBlue, double fwdSpeed, double strSpeed) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.drivetrain = drivetrain);
    this.limelight = limelight;
    this.tagWantedBlue = tagWantedBlue;
    this.tagWantedRed = tagWantedRed;
    this.fwdSpeed = fwdSpeed;
    this.strSpeed = strSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    killTimer.reset();
    killTimer.start();
    drivetrain.drive(fwdSpeed, strSpeed, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (killTimer.get() > 3 || limelight.seesTagId(tagWantedRed, tagWantedBlue));
  }
}
