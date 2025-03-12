// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
// import org.carlmontrobotics.subsystems.*;
// import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI;

import java.util.function.BooleanSupplier;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.CoralIntake;
import org.carlmontrobotics.commands.CoralOuttake;
import org.carlmontrobotics.commands.hkasjhkasdg;
import org.carlmontrobotics.commands.CoralIntakeManual;
import org.carlmontrobotics.subsystems.CoralEffector;

//limit switch
import edu.wpi.first.wpilibj.DigitalInput;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//commands
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

//constats
import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;

public class RobotContainer {

  //1. using GenericHID allows us to use different kinds of controllers
  //2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(DRIVE_CONTROLLER_PORT);
  public final GenericHID manipulatorController = new GenericHID(MANIPULATOR_CONTROLLER_PORT);
  public final CoralEffector coralEffector = new CoralEffector();

  //public final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);

  public RobotContainer() {
    SmartDashboard.putData("Coral Intake", new CoralIntake(coralEffector));
    SmartDashboard.putData("coral out", new CoralOuttake(coralEffector));
    setDefaultCommands();
    setBindingsDriver();
    setBindingsManipulator();
  }

  private void setDefaultCommands() {
    // drivetrain.setDefaultCommand(new TeleopDrive(
    //   drivetrain,
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftY)),
    //   () -> ProcessedAxisValue(driverController, Axis.kLeftX)),
    //   () -> ProcessedAxisValue(driverController, Axis.kRightX)),
    //   () -> driverController.getRawButton(OI.Driver.slowDriveButton)
    // ));
  }
  private void setBindingsDriver() {}
  private void setBindingsManipulator() {
    // new JoystickButton(manipulatorController, OI.Manipulator.OUTAKE_BUTTON)
    //   .whileTrue(new CoralOutake(coralEffector))
    //   .whileFalse(new CoralIntake(coralEffector));
    // new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_BUTTON)
    //   .whileTrue(new ManualCoralIntake());

    axisTrigger(manipulatorController, Axis.kLeftTrigger)
    .whileTrue(new CoralIntake(coralEffector))
    .onFalse(new hkasjhkasdg(coralEffector));
    
    
    axisTrigger(manipulatorController, Axis.kRightTrigger)
      .whileTrue(new CoralIntakeManual(coralEffector))
      .onFalse(new hkasjhkasdg(coralEffector));
    
    new JoystickButton(manipulatorController, (Button.kA.value)).whileTrue(new CoralIntake(coralEffector));
  }
    
  

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * Flips an axis' Y coordinates upside down, but only if the select axis is a joystick axis
   * 
   * @param hid The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double getStickValue(GenericHID hid, Axis axis) {
    return hid.getRawAxis(axis.value) * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
  }
  /**
   * Processes an input from the joystick into a value between -1 and 1, sinusoidally instead of linearly
   * 
   * @param value The value to be processed.
   * @return The processed value.
   */
  private double inputProcessing(double value) {
    double processedInput;
    // processedInput =
    // (((1-Math.cos(value*Math.PI))/2)*((1-Math.cos(value*Math.PI))/2))*(value/Math.abs(value));
    processedInput = Math.copySign(((1 - Math.cos(value * Math.PI)) / 2) * ((1 - Math.cos(value * Math.PI)) / 2),
        value);
    return processedInput;
  }
  /**
   * Combines both getStickValue and inputProcessing into a single function for processing joystick outputs
   * 
   * @param hid The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double ProcessedAxisValue(GenericHID hid, Axis axis){
    return inputProcessing(getStickValue(hid, axis));
  }

  private Trigger axisTrigger(GenericHID controller, Axis axis) {
    return new Trigger( (BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > 0.2) );
  }
}
