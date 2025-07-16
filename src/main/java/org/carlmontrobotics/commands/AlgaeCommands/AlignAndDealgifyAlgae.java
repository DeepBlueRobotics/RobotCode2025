// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlgaeCommands;

import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;


import org.carlmontrobotics.subsystems.AlgaeEffector;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.Elevatorc.elevatorOffset;
import static org.carlmontrobotics.Constants.Limelightc.*;


public class AlignAndDealgifyAlgae extends Command {
  private Drivetrain drivetrain;
  private Limelight ll;
  private AlgaeEffector algaeEffector;
  private Elevator elevator;
  private boolean topLevel;
  private Timer alignTimer;
  private Timer moveTimer;
  private Timer didntseetime;
  private double goal;
  private boolean aligned = false;
  private double forwardErr;
  private double strafeErr;
  private double strafeSpeedMultiplier;
  private double forwardSpeedMultiplier;
  private double strafeClamp;
  private double forwardClamp;
  private XboxController rumbleController;
  private boolean completedTask = false;

  /** 
   * Aligns the arm to dealgify, raises arm, raises elevator, moves forward, drops arm, and moves back
   * @param drivetrain Provide a drivetrain
   * @param ll provide limelight
   * @param algaeEffector provide AlgaeEffector
   * @param elevator
   * @param toplevel Specify which level to dealgify true for top, false for bottom
  */
  public AlignAndDealgifyAlgae(Drivetrain drivetrain, Limelight ll, AlgaeEffector algaeEffector, Elevator elevator, boolean topLevel, XboxController rumbleController) {
    addRequirements(this.drivetrain = drivetrain, this.algaeEffector = algaeEffector, this.elevator = elevator);
    this.ll = ll;
    this.topLevel = topLevel;
    this.rumbleController = rumbleController;
    forwardErr = Double.POSITIVE_INFINITY;
    strafeErr = Double.POSITIVE_INFINITY;
    strafeClamp = .35; 
    forwardClamp = 1.5; //NEED TO TUNE
    strafeSpeedMultiplier = 5;
    forwardSpeedMultiplier = 0.2;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    didntseetime.reset();
    alignTimer.reset();
    moveTimer.reset();
    if (topLevel) {
       goal = 0.8;
    }
    else {
      goal = 0.4; //TODO tune this
    }
    algaeEffector.moveArm(0.125);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (aligned) {
      didntseetime.stop();
      alignTimer.stop();
      elevator.setGoal(goal);
      if (elevator.atGoalHeight()) {
        moveTimer.start();
        drivetrain.drive(1,0,0);
        if (moveTimer.get() > 1) {
          algaeEffector.moveArm(-1.25);
          drivetrain.drive(-1,0,0);
          if (Math.abs(algaeEffector.getArmPos()) < 5) {
            completedTask = true;
          }
        }
      }
    }
    else {
      alignTimer.start();
      if ((forwardErr <= areaTolerance) && (Math.abs(strafeErr) <= strafeTolerance)) {
        aligned = true;
        drivetrain.stop();
      }
      elevator.setGoal(0);
      if (elevator.getBottomLimitSwitch()) {
        elevator.zeroPosition();
      } else {
        elevator.setMasterEncoder(elevatorOffset);
      }
      if (ll.seesTag(REEF_LL)) {
        SmartDashboard.putBoolean("SeeTag", true);
        SmartDashboard.putNumber("CurrentPercentage", LimelightHelpers.getTA(REEF_LL)); //To figure out goal
        rumbleController.setRumble(RumbleType.kBothRumble, 0);
        didntseetime.reset();
        didntseetime.stop();
        //figure out errors
        forwardErr = - LimelightHelpers.getTA(REEF_LL) + areaPercentageGoalForAlgae;
        strafeErr = getStrafeErrorMeters();
        //find speeds
        double strafeSpeed = MathUtil.clamp(strafeErr*strafeSpeedMultiplier, -strafeClamp, strafeClamp);
        double forwardSpeed = MathUtil.clamp(forwardErr*forwardSpeedMultiplier, -forwardClamp, forwardClamp);
        drivetrain.drive(forwardSpeed, strafeSpeed, 0);
      }
      else {
        didntseetime.start();
        rumbleController.setRumble(RumbleType.kBothRumble, 0.5);
      }
    }
    }

  

  private double getStrafeErrorMeters() {
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+ (RIGHT_CORAL_BRANCH); //TODO will require some tuning
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.stop();
    algaeEffector.stopArm();
    rumbleController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (didntseetime.get() > 1.5 || completedTask || alignTimer.get() > 3);
  }
}
