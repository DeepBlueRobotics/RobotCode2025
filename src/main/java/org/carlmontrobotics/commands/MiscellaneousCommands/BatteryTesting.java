// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.MiscellaneousCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

//199 files
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import org.carlmontrobotics.Constants.AlgaeEffectorc;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.Constants.Elevatorc;
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.AlignCommands.MoveToLeftBranch;
import org.carlmontrobotics.commands.AlignCommands.MoveToRightBranch;
import org.carlmontrobotics.commands.ElevatorCommands.ElevatorToPos;
import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.Drivetrain;
import org.carlmontrobotics.subsystems.Elevator;

import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import static org.carlmontrobotics.commands.DriveCommands.TeleopDrive.babyMode;
import static org.carlmontrobotics.commands.DriveCommands.TeleopDrive.babyModeSupplier;

//import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;
import com.pathplanner.lib.auto.NamedCommands;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;

//limit switch
import edu.wpi.first.wpilibj.DigitalInput;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.carlmontrobotics.Constants.Elevatorc.elevatorOffset;
import static org.carlmontrobotics.Constants.Elevatorc.l1;
import static org.carlmontrobotics.Constants.Elevatorc.l2;
import static org.carlmontrobotics.Constants.Elevatorc.l3;
import static org.carlmontrobotics.Constants.Elevatorc.l4;
import static org.carlmontrobotics.Constants.Elevatorc.testl4;
import static org.carlmontrobotics.Constants.Elevatorc.testl4RaiseHeight;
//constats
//import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;;

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
