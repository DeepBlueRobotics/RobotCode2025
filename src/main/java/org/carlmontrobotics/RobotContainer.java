// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
import org.carlmontrobotics.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.Constants.OI.Manipulator.*;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.StadiaController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
public class RobotContainer {
  private static boolean babyMode = false;

  // 1. using GenericHID allows us to use different kinds of controllers
  // 2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.port);
  public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

  private final Drivetrain drivetrain = new Drivetrain();
  private Elevator elevator = new Elevator();

  /* These are assumed to be equal to the AUTO ames in pathplanner */
  /* These must be equal to the pathPlanner path names from the GUI! */
  // Order matters - but the first one is index 1 on the physical selector - index
  // 0 is reserved for
  // null command.
  // the last auto is hard-coded to go straight. since we have __3__ Autos, port 4
  // is simplePz
  // straight
  private List<Command> autoCommands;
  private final SendableChooser<Command> autoChooser;

  public RobotContainer() {
    setBindingsDriver();
    setBindingsManipulator();
    RegisterAutoCommands();
    autoChooser = AutoBuilder.buildAutoChooser();
    RegisterCustomAutos();
    setDefaultCommands();
    SmartDashboard.putData("Auto Chooser", autoChooser);
  }

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(drivetrain,
            () -> ProcessedAxisValue(driverController, Axis.kLeftY),
            () -> ProcessedAxisValue(driverController, Axis.kLeftX),
            () -> ProcessedAxisValue(driverController, Axis.kRightX),
            () -> driverController.getRawButton(OI.Driver.slowDriveButton)));

    //elevator.setDefaultCommand(new TeleopElevator(elevator, ()->ProcessedAxisValue(manipulatorController, Axis.kLeftY)));
  }
  private void setBindingsDriver() {}
  private void setBindingsManipulator() {

    new JoystickButton(manipulatorController, OI.Manipulator.Y).onTrue(new ElevatorToPos(elevator, 0.5));
    new JoystickButton(manipulatorController, Button.kA.value).onTrue(new ElevatorToPos(elevator, 0.05));
  }

  /**
     * Flips an axis' Y coordinates upside down, but only if the select axis is a
     * joystick axis
     *
     * @param hid  The controller/plane joystick the axis is on
     * @param axis The processed axis
     * @return The processed value.
     */
    private double getStickValue(GenericHID hid, Axis axis) {
      return hid.getRawAxis(axis.value)
              * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }

  /**
   * Processes an input from the joystick into a value between -1 and 1,
   * sinusoidally instead of
   * linearly
   *
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
      double processedInput;
      // processedInput =
      // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
      processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2)
              * ((1 - Math.cos(value * Math.PI)) / 2), value);
      return processedInput;
  }

  /**
   * Combines both getStickValue and inputProcessing into a single function for
   * processing joystick
   * outputs
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double ProcessedAxisValue(GenericHID hid, Axis axis) {
      return DeadzonedAxis(inputProcessing(getStickValue(hid, axis)));
  }

  /**
   * Returns zero if a axis input is inside the deadzone
   *
   * @param hid  The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double DeadzonedAxis(double axOut) {
      return (Math.abs(axOut) <= OI.JOY_THRESH) ? 0.0 : axOut;
  }

  /**
   * Returns a new instance of Trigger based on the given Joystick and Axis
   * objects. The Trigger is
   * triggered when the absolute value of the stick value on the specified axis
   * exceeds a minimum
   * threshold value.
   *
   * @param stick The Joystick object to retrieve stick value from.
   * @param axis  The Axis object to retrieve value from the Joystick.
   * @return A new instance of Trigger based on the given Joystick and Axis
   *         objects. * @throws
   *         NullPointerException if either stick or axis is null.
   */
  private Trigger axisTrigger(GenericHID controller, Axis axis) {
      return new Trigger(() -> Math
              .abs(getStickValue(controller, axis)) > OI.MIN_AXIS_TRIGGER_VALUE);
  }


  private void RegisterAutoCommands(){
    //NamedCommands.registerCommand("IntakeAlgae", new IntakeAlgae(algaeEffector));
    //NamedCommands.registerCommand("OuttakeAlgae", new OuttakeAlgae(algaeEffector));
    //NamedCommands.registerCommand("IntakeCoral", new IntakeCoral(coralEffector));
    //NamedCommands.registerCommand("OuttakeCoral", new OuttakeCoral(coralEffector));
  }
  private void RegisterCustomAutos(){
    autoChooser.addOption("ForwardLastResortAuto", new LastResortAuto(drivetrain, 1));
    autoChooser.addOption("BackwardLastResortAuto", new LastResortAuto(drivetrain, -1));

  }

    public Command getAutonomousCommand() {
      return autoChooser.getSelected();
    }
}
