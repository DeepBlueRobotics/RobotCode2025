// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class L4Backup extends Command {
  Drivetrain dt;
  boolean originalFieldOrientation;
  /** Creates a new L4Backup. */
  public L4Backup(Drivetrain dt) {
    addRequirements(this.dt = dt);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    dt.drive(-Units.inchesToMeters(13) * SmartDashboard.getNumber("limelight forward kp", 6), 0, 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
