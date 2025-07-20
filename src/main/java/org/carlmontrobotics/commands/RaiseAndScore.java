// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Elevatorc.*;

import org.carlmontrobotics.commands.ElevatorCommands.ElevatorToBottomLimitSwitch;
import org.carlmontrobotics.subsystems.CoralEffector;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class RaiseAndScore extends Command {
  /** Creates a new RaiseAndScore. */
  private Elevator elevator;
  private CoralEffector coralEffector;
  boolean thirdBranch;
  Timer shootCoral = new Timer();
  boolean coralShot = false;
  public RaiseAndScore(Elevator elevator, CoralEffector coralEffector, boolean thirdBranch) {
    addRequirements(this.elevator = elevator, this.coralEffector = coralEffector);
    this.thirdBranch = thirdBranch;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shootCoral.reset();
    shootCoral.stop();
    if (thirdBranch) {
      elevator.setGoal(l3);
    }
    else {
      elevator.setGoal(l2);
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (SmartDashboard.getBoolean("AutoScoring", true)) {
      if (elevator.atGoalHeight()) {
        coralEffector.setMotorSpeed(0.1);
        shootCoral.start();
        System.out.println(shootCoral.get());
      }
      if(shootCoral.get() > 0.5) {
        coralEffector.setMotorSpeed(0);
        elevator.setGoal(l1);
        
        coralShot = true;
      }
    }
    else {
      coralShot = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return elevator.atGoalHeight() && coralShot;
  }
}
