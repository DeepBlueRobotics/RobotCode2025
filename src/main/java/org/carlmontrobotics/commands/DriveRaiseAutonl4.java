// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package org.carlmontrobotics.commands;
import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
public class DriveRaiseAutonl4 extends Command {

  private final Timer timer = new Timer();
  /** Creates a new DriveRaoseAutonl4. */
  private Drivetrain drivetrain;  
  private Elevator elevator;
  private boolean prev;
  private int dir;
  int MAX_SECONDS_DRIVE = 4;
  
  public DriveRaiseAutonl4(Drivetrain drivetrain, Elevator elevator, int direction) {
    this.dir = direction;
    this.drivetrain = drivetrain;
    this.elevator = elevator;
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    prev = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
    drivetrain.drive(1*dir, 0, 0);
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() >= 4.0){
      drivetrain.drive(0.00000000000000000000000000001, 0, 0);
      elevator.setGoal(Constants.Elevatorc.l4);
    }
  }
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //drivetrain.drive(.00000000000000000000000000001, 0, 0);
    drivetrain.setFieldOriented(prev);
  }
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.getGoal() == Constants.Elevatorc.l4;
  }
}