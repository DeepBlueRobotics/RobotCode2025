// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import static org.carlmontrobotics.Constants.Limelightc.*;

public class MoveToLeftBranch extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private boolean originalFieldOrientation;

  /** Creates a new MoveToLeftBranch. */
  public MoveToLeftBranch(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (ll.seesTag(CORAL_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
      double strafeErr = Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(CORAL_LL))) * ll.getDistanceToApriltag(CORAL_LL, CORAL_MOUNT_ANGLE, REEF_LL_HEIGHT_FROM_GROUND_METERS, CORAL_LL_HEIGHT_FROM_GROUND_METERS);
      dt.drive(0, strafeErr, 0);
    }
    else {
      dt.drive(0, LEFT_TO_CORAL_BRANCH,0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
