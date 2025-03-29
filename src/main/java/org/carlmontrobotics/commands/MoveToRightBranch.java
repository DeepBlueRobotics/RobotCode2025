// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import static org.carlmontrobotics.Constants.Limelightc.*;

public class MoveToRightBranch extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private boolean originalFieldOrientation;
  double strafeErr;
  double didntseetime=0;

  /** Creates a new MoveToLeftBranch. */
  public MoveToRightBranch(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
  }

  // Called when the comman d is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    SmartDashboard.putNumber("strafe left", strafeErr);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    didntseetime+=1/50;
    if (ll.seesTag(REEF_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
      didntseetime=0;
      strafeErr = Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL);
      dt.drive(0.00001, (strafeErr + RIGHT_CORAL_BRANCH) * 3, 0);
    }else{
      dt.stop();
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
    dt.drive(0,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL) + RIGHT_CORAL_BRANCH < .02 && 
    Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL) + RIGHT_CORAL_BRANCH > -.02)
    || didntseetime>1.5;//sec
  }
}