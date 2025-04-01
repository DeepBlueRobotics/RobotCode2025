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
import edu.wpi.first.math.MathUtil;

import static org.carlmontrobotics.Constants.Limelightc.*;

import java.text.NumberFormat.Style;

public class MoveToRightBranch extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private boolean originalFieldOrientation;
  double strafeErr;
  double speedOfAutoAlign;
  //int kP;
  double didntseetime=0;
  double clampNumberRight;

  /** Creates a new MoveToLeftBranch. */
  public MoveToRightBranch(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
    SmartDashboard.putNumber("clamp for right", 0.5);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    SmartDashboard.putNumber("strafe right", strafeErr);
    clampNumberRight = SmartDashboard.getNumber("clamp for right", 0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dt.isAtAngle(90, 10)){
      didntseetime += 1.0/50.0;
      if (ll.seesTag(REEF_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
        didntseetime=0;
        strafeErr = getStrafeErrorMeters();
        double speed = MathUtil.clamp((strafeErr + RIGHT_CORAL_BRANCH)*6, -clampNumberRight, clampNumberRight);
        dt.drive(0.00001, speed, 0);
      }else{
        dt.stop();
      }
    }else dt.drive(0.00001, -0.1, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
    dt.drive(0.0001,0,0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(getStrafeErrorMeters()) < .01) || didntseetime > 1.5;//sec
  }


  public double getStrafeErrorMeters(){
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+RIGHT_CORAL_BRANCH;
  }
}