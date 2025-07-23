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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import static org.carlmontrobotics.Constants.Elevatorc.elevatorOffset;
import static org.carlmontrobotics.Constants.Limelightc.*;

import java.nio.file.FileSystemAlreadyExistsException;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;

public class AlignAndDealgifyAlgae extends Command {
  private Drivetrain drivetrain;
  private Limelight ll;
  private AlgaeEffector algaeEffector;
  private Elevator elevator;
  private boolean topLevel;
  private Timer alignTimer = new Timer();
  private Timer moveTimer = new Timer();
  private Timer didntseetime = new Timer();
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
  private boolean originalFieldOrientation;
  private boolean error = false;
  private boolean flush = false;

  /** 
   * Aligns the arm to dealgify, raises arm, raises elevator, moves forward, drops arm, and moves back
   * Does not require a specification as this method uses the aprilTag Id to figure out which level the algae will be at
   * @param drivetrain Provide a drivetrain
   * @param ll provide limelight
   * @param algaeEffector provide AlgaeEffector
   * @param elevator Provide elevator
   * @param rumbleController add a xBox controller for the command to rumble when does not see a AprilTag
  */
  public AlignAndDealgifyAlgae(Drivetrain drivetrain, Limelight ll, AlgaeEffector algaeEffector, Elevator elevator, XboxController rumbleController) {
    addRequirements(this.drivetrain = drivetrain, this.algaeEffector = algaeEffector, this.elevator = elevator);
    this.ll = ll;
    this.rumbleController = rumbleController;
    forwardErr = Double.POSITIVE_INFINITY;
    strafeErr = Double.POSITIVE_INFINITY;
    strafeClamp = .35; 
    forwardClamp = 1.5; //NEED TO TUNE
    strafeSpeedMultiplier = 5;
    forwardSpeedMultiplier = 0.2;
    if (getAlgaePosition()) {
      goal = DELAGIFY_HIGH_POS;
    }
    else {
      goal = DELAGIFY_LOW_POS;
    }
  }
  
  /** 
   * Aligns the arm to dealgify, raises arm, raises elevator, moves forward, drops arm, and moves back
   * @param drivetrain Provide a drivetrain
   * @param ll provide limelight
   * @param algaeEffector provide AlgaeEffector
   * @param elevator Provide elevator
   * @param toplevel Specify which level to dealgify true for top, false for bottom
   * @param rumbleController add a xBox controller for the command to rumble when does not see a AprilTag
  */
  public AlignAndDealgifyAlgae(Drivetrain drivetrain, Limelight ll, AlgaeEffector algaeEffector, Elevator elevator, boolean topLevel, XboxController rumbleController) {
    addRequirements(this.drivetrain = drivetrain, this.algaeEffector = algaeEffector, this.elevator = elevator);
    this.ll = ll;
    this.topLevel = topLevel;
    this.rumbleController = rumbleController;
    forwardErr = Double.POSITIVE_INFINITY;
    strafeErr = Double.POSITIVE_INFINITY;
    strafeClamp = .35; 
    forwardClamp = 1.5;
    strafeSpeedMultiplier = 5;
    forwardSpeedMultiplier = 0.2;
    if (topLevel) {
      goal = DELAGIFY_HIGH_POS;
   }
   else {
     goal = DELAGIFY_LOW_POS; 
   }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
    didntseetime.reset();
    didntseetime.stop();
    alignTimer.reset();
    alignTimer.stop();
    moveTimer.reset();
    moveTimer.stop();
    forwardErr = Double.POSITIVE_INFINITY;
    strafeErr = Double.POSITIVE_INFINITY;
    algaeEffector.moveArm(ARM_UP_VOLTAGE);
    completedTask = false;
    error = false;
    aligned = false;
    //flush = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    /*Code Logic
     * Begins by aligning to reef wall to be flush,
     * then goes back to raise arm
     * If aligned, raise elevator to the correct position, and move in blindly
     * After half a second the robot assumes it is all good and lowers the arm and drives back until it detects that its arm is down
     * Else:
     * Align usign the same logic as MoveToAlignReef but with other constants, once aligned report that it is aligned
     */
    if (aligned) {
      didntseetime.stop();
      alignTimer.stop();

      elevator.setGoal(goal);
      if (elevator.atGoalHeight()) {
        moveTimer.start();
        drivetrain.drive(1,0,0);
        if (moveTimer.get() > 0.7) {
          algaeEffector.moveArm(ARM_DOWN_VOLTAGE);
          drivetrain.drive(-1,0,0);
          if (Math.abs(algaeEffector.getArmPos()) < 5) {
            completedTask = true;
          }
        }
      }
    }
    else {
      alignTimer.start();
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
      if ((forwardErr <= areaTolerance) && (Math.abs(strafeErr) <= strafeToleranceAlgae)) {
        aligned = true;
        drivetrain.drive(0,0,0);
      }
    }
    // else {
    //   elevator.setGoal(0);
    //   if(elevator.atGoalHeight()) {
    //     if (elevator.getBottomLimitSwitch()) {
    //     elevator.zeroPosition();
    //     } else {
    //       elevator.setMasterEncoder(elevatorOffset);
    //     }
    //     if (ll.seesTag(REEF_LL)) {
    //       rumbleController.setRumble(RumbleType.kBothRumble, 0);
    //       didntseetime.reset();
    //       didntseetime.stop();
    //       //figure out errors
    //       forwardErr = - LimelightHelpers.getTA(REEF_LL) + areaPercentageGoal;
    //       strafeErr = getStrafeErrorMeters();
    //       //find speeds
    //       double strafeSpeed = MathUtil.clamp(strafeErr*strafeSpeedMultiplier, -strafeClamp, strafeClamp);
    //       double forwardSpeed = MathUtil.clamp(forwardErr*forwardSpeedMultiplier, -forwardClamp, forwardClamp);
    //       drivetrain.drive(forwardSpeed, strafeSpeed, 0);
    //     }
    //     else {
    //       didntseetime.start();
    //       rumbleController.setRumble(RumbleType.kBothRumble, 0.5);
    //       drivetrain.drive(0.2, -0.14, 0);
    //     }
    //   }
    //   if ((forwardErr <= areaTolerance) && (Math.abs(strafeErr) <= strafeTolerance)) {
    //     flush = true;
    //     drivetrain.drive(0,0,0);
    //     strafeErr = Double.POSITIVE_INFINITY;
    //     forwardErr = Double.POSITIVE_INFINITY;
    //   }
    // }
  }

  

  private double getStrafeErrorMeters() {
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+ (RIGHT_CORAL_BRANCH); //TODO will require some tuning
  }

  private boolean getAlgaePosition() {
    int target;
    int[] topAlgae = {7,9,11,18,20,22};
    int[] bottomAlgae = {6,8,10,17,19,21};
    target = (int) LimelightHelpers.getTA(REEF_LL);
    for (int id : topAlgae) {
      if (id == target) {
        return true;
      }
    }
    for (int id : bottomAlgae) {
      if (id == target) {
        return false;
      }
    }
    DriverStation.reportError("Aligning not with reef", false);
    error = true;
    return false;

  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(0,0,0);
    algaeEffector.stopArm();
    rumbleController.setRumble(RumbleType.kBothRumble, 0);
    drivetrain.setFieldOriented(originalFieldOrientation);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (didntseetime.get() > 1.5 || completedTask || alignTimer.get() > 3 || error);
  }
}
