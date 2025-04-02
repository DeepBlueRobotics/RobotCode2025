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
  double clampNumber;
  double speedMultiplier;

  /** Creates a new MoveToLeftBranch. */
  public MoveToRightBranch(Drivetrain dt, Limelight ll) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt);
    this.ll = ll;
    clampNumber = .35;
    speedMultiplier = 6;
    // SmartDashboard.putNumber("clamp for autoalign", 0.35);
    // SmartDashboard.putNumber("speed multiplier", 3);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    SmartDashboard.putNumber("strafe err", strafeErr);
    // clampNumberLeft = SmartDashboard.getNumber("clamp for autoalign", 0.35);
    //kP = 0;
    // speedMultiplier = SmartDashboard.getNumber("speed multiplier", 6);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (dt.isAtAngle(90, 10)){
      didntseetime += 1.0/50.0;
      if (ll.seesTag(REEF_LL)) { //TODO: test with getdistancetoapriltag vs getdistancetoapriltagmt2
        didntseetime=0;
        strafeErr = getStrafeErrorMeters();
        double speed = MathUtil.clamp(strafeErr*speedMultiplier, -clampNumber, clampNumber);
        dt.drive(0, speed, 0);
      }else{
        dt.drive(0, -0.14, 0);
      }
    }else dt.drive(0, 0.00001, 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
    // dt.drive(0.0001,0,0);
    
    SmartDashboard.putNumber("I CANT BREATHE (didnt'see?)",didntseetime);
  SmartDashboard.putNumber("THEY SHOT 'IM (strafe error good)",Math.abs(getStrafeErrorMeters()));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Math.abs(getStrafeErrorMeters()) < .02) || didntseetime > 1.5;//sec
  }


  public double getStrafeErrorMeters(){
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+RIGHT_CORAL_BRANCH;
  }
}