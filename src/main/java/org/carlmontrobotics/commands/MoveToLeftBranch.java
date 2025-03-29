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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

import static org.carlmontrobotics.Constants.Limelightc.*;

import java.text.NumberFormat.Style;

public class MoveToLeftBranch extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private boolean originalFieldOrientation;
  double strafeErr;
  Command gocmd;
  boolean ran = false;

  /** Creates a new MoveToLeftBranch. */
  public MoveToLeftBranch(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
    // this.gocmd.schedule();
    // this.gocmd.end(true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    SmartDashboard.putNumber("strafe left", strafeErr);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("strafe left", strafeErr);
    
    // if (!ran && ll.seesTag(REEF_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
    //   strafeErr = Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL);
    //   this.gocmd = new SequentialCommandGroup(
    //     new InstantCommand(()-> dt.drive(.0001,.5,0)),
    //     new WaitCommand(strafeErr/.5),
    //     new InstantCommand(()-> dt.drive(0,0,.00001))
    //   ); 
    //   gocmd.schedule();
    //   ran=true;
    // }

    if (ll.seesTag(REEF_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
      strafeErr = Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL);
      dt.drive(0.00001, (strafeErr + LEFT_CORAL_BRANCH) * 3, 0);
    }else{
      dt.stop();
    }
  
    //   dt.drive(0.00001, (strafeErr + LEFT_CORAL_BRANCH) * 6, 0); 
      
      
    // }else{
    //   dt.stop();
    // }
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
    return (Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL) + LEFT_CORAL_BRANCH < .1 
    && Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL))) * ll.getDistanceToApriltagMT2(REEF_LL) + LEFT_CORAL_BRANCH > -.1)
    || !ll.seesTag(REEF_LL);
  }
}