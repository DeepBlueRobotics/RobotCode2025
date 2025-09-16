// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;

import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.MathUtil;
import static org.carlmontrobotics.Constants.Limelightc.*;


public class MoveToLeftBranch extends Command {
  private final Drivetrain dt;
  private final Limelight ll;
  private boolean originalFieldOrientation;
  double strafeErr;
  double speedOfAutoAlign;
  //int kP;
  Timer didntseetime;
  double clampNumber;
  double speedMultiplier;
  Timer alignedtime;
  Timer timeoutTimer;
  Elevator elevator;

  /** 
   * @deprecated Use {@link MoveToAlignReef} instead.
  */
  @Deprecated
  public MoveToLeftBranch(Drivetrain dt, Limelight ll, Elevator elevator) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.dt=dt, this.elevator = elevator);
    this.ll = ll;
    clampNumber = .35;
    speedMultiplier = 6;
    didntseetime=new Timer();
    alignedtime=new Timer();
    timeoutTimer =  new Timer();
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
    didntseetime.reset();didntseetime.start();
    alignedtime.reset();
    timeoutTimer.reset();
    timeoutTimer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (elevator.getCurrentHeight() <= 0.05) {
      if (dt.isAtAngle(90, 10)){
      // didntseetime += 1.0/50.0;
      if (ll.seesTag(REEF_LL)) {
        didntseetime.reset();
        strafeErr = getStrafeErrorMeters();
        double speed = MathUtil.clamp(strafeErr*speedMultiplier, -clampNumber, clampNumber);
        dt.drive(0, speed, 0);
      }else{
        dt.drive(0, 0.14, 0);
      }
    }else dt.drive(0, 0.00001, 0);

    if ( Math.abs(getStrafeErrorMeters()) < .02 ){
      // alignedtime+=1.0/50.0;
    } else alignedtime.restart();
    }
  
  else {
    dt.drive(0, 0, 0);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    dt.setFieldOriented(originalFieldOrientation);
     dt.drive(0,0,0);
    
    SmartDashboard.putString("I CANT BREATHE","I CANT BREATHE");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return ( alignedtime.get()>.3 ) || didntseetime.get() > 1.5 || timeoutTimer.get() >= 3.0;//sec
  }


  public double getStrafeErrorMeters(){
    return Math.sin(Units.degreesToRadians(LimelightHelpers.getTX(REEF_LL)))
    * ll.getDistanceToApriltagMT2(REEF_LL)+LEFT_CORAL_BRANCH;
  }
}