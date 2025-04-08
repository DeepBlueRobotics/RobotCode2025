// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class Flush extends Command {
  private Drivetrain dt;
  private Limelight ll;
  private boolean originalOrientation;

  private Timer timer = new Timer();

  double currentWidth;
  int direction;
  double initialHeading;
  double degreesOff;
  double desiredWidth = 1; // placeholder
  double distanceToTag;

  public Flush(Limelight ll, Drivetrain dt) {
    addRequirements(this.dt = dt, this.ll = ll);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.stop();
    originalOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    currentWidth = ll.getAprilWidth(REEF_LL) / ll.getAprilHeight(REEF_LL);
    degreesOff = Math.acos(currentWidth/desiredWidth);
    initialHeading = dt.getRawHeading();
    distanceToTag = LimelightHelpers.getBotPose3d(REEF_LL).getZ();
    direction = 1;

    dt.drive(0.00000001, 0, 2*direction);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double currentHeading = dt.getRawHeading();
    double counterClockwiseGoal = initialHeading - degreesOff;
    double clockwiseGoal = initialHeading + degreesOff;

    if (direction < 0 && Math.abs(currentHeading - counterClockwiseGoal) < .1) {
      timer.start();
      dt.drive(Math.cos(degreesOff)*distanceToTag, Math.sin(degreesOff) * distanceToTag, 0);
    } 
    else if (direction > 0 && Math.abs(currentHeading - clockwiseGoal) < .1) {
      timer.start();
      dt.drive(Math.cos(degreesOff)*distanceToTag, -Math.sin(degreesOff) * distanceToTag, 0);
    } 
    else if ((ll.getAprilWidth(REEF_LL) / ll.getAprilHeight(REEF_LL)) < currentWidth) {
      direction = -1;
      dt.drive(0.0000001,0,2*direction);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalOrientation);
    dt.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 1;
  }
}
