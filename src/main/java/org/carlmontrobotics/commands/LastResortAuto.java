// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.Drivetrain;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class LastResortAuto extends Command {
  private final Timer timer = new Timer();
  /** Creates a new LastResortAuto. */
  private final Drivetrain drivetrain;
  private boolean prev;
  private int dir;
  private int speed;
  private double time;
  private double distance;

  int MAX_SECONDS_DRIVE = 4;

  public LastResortAuto(Drivetrain drivetrain, int direction, int speed, double distance) {
    dir = direction;
    addRequirements(this.drivetrain = drivetrain);
    this.speed = speed;
    //this.time = time;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    prev = drivetrain.getFieldOriented();
    drivetrain.setFieldOriented(false);
    drivetrain.drive(speed*dir, 0, 0);
    time = distance/speed;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(.00000000000000000000000000001, 0, 0);
    drivetrain.setFieldOriented(prev);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.hasElapsed(time);
  }

  public String toString(){
    return "Last Resort Auto: "+dir+speed+time;
  }
}