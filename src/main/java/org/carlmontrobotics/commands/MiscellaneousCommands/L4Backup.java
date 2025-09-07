// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.MiscellaneousCommands;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
 
/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
/**
 * Previous command used to backup and score coral, a better method is used now
 */
public class L4Backup extends Command {
  Drivetrain dt;
  boolean originalFieldOrientation;
  Timer t = new Timer();
  double vel;
  double distance;
  /** 
   * @deprecated Robot does not work like this anymore
   * Creates a new L4Backup. 
   */
  @Deprecated
  public L4Backup(Drivetrain dt) {
    addRequirements(this.dt = dt);
    SmartDashboard.putNumber("dist", distance);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    t.reset();
    t.start();
    dt.setFieldOriented(false);
    dt.keepRotateMotorsAtDegrees(0);
    vel = 6;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dt.isAtAngle(0, 10)){
      dt.drive(-Units.inchesToMeters(vel), 0, 0);
    }else dt.drive(0,0,0);
    distance = 6.5;//SmartDashboard.getNumber("dist", 6.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return t.get() > distance/vel;
  }
}
