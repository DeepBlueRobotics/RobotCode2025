// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;

import static org.carlmontrobotics.Constants.Limelightc.*;


import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class MoveToAlignReef extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private final Elevator elevator;
  private final XboxController rumbleController;
  private boolean originalFieldOrientation;
  double strafeErr;
  double forwardErr;
  private Timer didntseetime;
  private Timer timeoutTimer;
  double strafeClamp;
  double strafeSpeedMultiplier;
  double forwardClamp;
  double forwardSpeedMultiplier;
  private final boolean rightBranch;



  /** 
   * Aligns with one of the branches both with forward and strafe
   * 
   * @param dt Drivetrain
   * @param ll Limelight
   * @param elevator Elevator
   * @param rightBranch Branch being aligned with
   * @param rumbleController for rumbling controller when tag not seen
  */
  public MoveToAlignReef(Drivetrain dt, Limelight ll, Elevator elevator, boolean rightBranch, XboxController rumbleController) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt = dt, this.elevator = elevator);
    this.rumbleController = rumbleController;
    this.ll = ll;
    this.rightBranch = rightBranch;
    strafeClamp = .35;
    forwardClamp = 1.5; //TODO figure this one out
    strafeSpeedMultiplier = 5;//TODO tune it better
    forwardSpeedMultiplier = 0.2;//TODO figure this out
    didntseetime = new Timer();
    timeoutTimer = new Timer();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    originalFieldOrientation = dt.getFieldOriented();
    dt.setFieldOriented(false);
    didntseetime.reset();
    didntseetime.start();
    timeoutTimer.reset();
    timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getCurrentHeight() <= 0.1) {
      SmartDashboard.putBoolean("LimelightFunctional", true);
      if (ll.seesTag(REEF_LL)) {
        SmartDashboard.putBoolean("SeeTag", true);
        SmartDashboard.putNumber("CurrentPercentage", LimelightHelpers.getTA(REEF_LL)); //To figure out goal
        didntseetime.reset();
        //figure out errors
        forwardErr = - LimelightHelpers.getTA(REEF_LL) + areaPercentageGoal;
        strafeErr = getStrafeErrorMeters();
        //find speeds
        double strafeSpeed = MathUtil.clamp(strafeErr*strafeSpeedMultiplier, -strafeClamp, strafeClamp);
        double forwardSpeed = MathUtil.clamp(forwardErr*forwardSpeedMultiplier, 0, forwardClamp);
        dt.drive(forwardSpeed, strafeSpeed, 0);
      }
      else {
        rumbleController.setRumble(RumbleType.kBothRumble, 0.5);
        SmartDashboard.putBoolean("SeeTag", false);
        dt.drive(0.14, 0.14 * (rightBranch ? -1 : 1), 0);
      }
    }
    else {
      SmartDashboard.putBoolean("LimelightFunctional", false);
      dt.drive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
    dt.drive(0,0,0);
    rumbleController.setRumble(RumbleType.kBothRumble, 0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    boolean aligned_with_tag = ((forwardErr <= areaTolerance) && (Math.abs(strafeErr) <= strafeTolerance));
    return (aligned_with_tag) //Aligned with tag
          || (didntseetime.get() > 1.5) //Does not see tag
          || (timeoutTimer.get() > 3.0); //Worst case scenrio
  }
  
  public double getStrafeErrorMeters(){
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+ (rightBranch ? RIGHT_CORAL_BRANCH : LEFT_CORAL_BRANCH);
  }

}
