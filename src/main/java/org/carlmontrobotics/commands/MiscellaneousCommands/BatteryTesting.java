// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.MiscellaneousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//199 files
import org.carlmontrobotics.commands.AlignCommands.MoveToLeftBranch;
import org.carlmontrobotics.commands.AlignCommands.MoveToRightBranch;
import org.carlmontrobotics.commands.ElevatorCommands.ElevatorToPos;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;
//import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import static org.carlmontrobotics.Constants.Elevatorc.l1;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class BatteryTesting extends Command {
  /** Creates a new BatteryTesting3. */
  private double level;
  Drivetrain drivetrain;
  Elevator elevator;
  Limelight limelight;
  boolean finish = false;

  public BatteryTesting(Drivetrain drivetrain, Elevator elevator, Limelight limelight, double level) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.level = level;
    addRequirements(this.drivetrain = drivetrain, this.elevator = elevator, this.limelight = limelight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //FIXME: clean up the code 
    final int waitTime = 3;
    final int speed = 1;

    new SequentialCommandGroup(
        //drives up to the reef
        new InstantCommand(() -> drivetrain.drive(speed, 0, 0)),
        new WaitCommand(waitTime),
        new InstantCommand(() -> drivetrain.stop()),
        //goes to the right branch goes to the specified level and goes back down
        new MoveToRightBranch(drivetrain, limelight, elevator),
        new InstantCommand(() -> {
          while (!drivetrain.isAtAngle(0, 10)) {
            drivetrain.drive(0.0000000000001, 0, 0);
          }},drivetrain),
        new ElevatorToPos(elevator, level),
        new ElevatorToPos(elevator, l1),
        //drives back to human player station
        new InstantCommand(() -> {drivetrain.drive(-speed, 0, 0);}),
        new WaitCommand(waitTime),
        new InstantCommand(() -> drivetrain.stop()));
    new SequentialCommandGroup(
        //drives up to the reef
        new InstantCommand(() -> drivetrain.drive(speed, 0, 0)),
        new WaitCommand(waitTime),
        new InstantCommand(() -> drivetrain.stop()),
        //goes to the left branch goes to the specified level and goes back down
        new MoveToLeftBranch(drivetrain, limelight, elevator),
        new InstantCommand(() -> {
          while (!drivetrain.isAtAngle(0, 10)) {
            drivetrain.drive(0.0000000000001, 0, 0);
          }},drivetrain),
        new ElevatorToPos(elevator, level),
        new ElevatorToPos(elevator, l1),
        //drives back to human player station
        new InstantCommand(() -> drivetrain.drive(-speed, 0, 0)),
        new WaitCommand(waitTime),
        new InstantCommand(() -> drivetrain.stop()),
        new InstantCommand(() -> finish = true)).schedule();

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finish;
  }
}
