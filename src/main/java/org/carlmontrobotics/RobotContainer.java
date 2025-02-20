// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
// import org.carlmontrobotics.subsystems.*;
// import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.*;

import java.util.ArrayList;
import java.util.List;

import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator.*;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.OuttakeAlgae;
import org.carlmontrobotics.subsystems.AlgaeEffector;
import org.carlmontrobotics.subsystems.CoralEffector;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;

import org.carlmontrobotics.commands.IntakeAlgae;
import org.carlmontrobotics.commands.IntakeCoral;
import org.carlmontrobotics.commands.OuttakeCoral;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS5Controller.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
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
import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  //1. using GenericHID allows us to use different kinds of controllers
  //2. Use absolute paths from constants to reduce confusion
  public final GenericHID driverController = new GenericHID(OI.Driver.driverPort);
  public final GenericHID manipulatorController = new GenericHID(OI.Manipulator.manipulatorPort);
  private final AlgaeEffector algaeEffector = new AlgaeEffector();
  private final CoralEffector coralEffector = new CoralEffector();

  private List<Command> autoCommands = new ArrayList<Command>();
  private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();
  private boolean hasSetupAutos = false;
  private final String[] autoNames = new String[] {
          /* These are assumed to be equal to the AUTO ames in pathplanner */
          "Left 1 Piece L1 Auto", "Center 1 Piece L1 Auto", "Right 1 Piece L1 Auto",
          "Left 1 Piece L2 Auto", "Center 1 Piece L2 Auto", "Right 1 Piece L2 Auto",
          "Left 1 Piece L3 Auto", "Center 1 Piece L3 Auto", "Right 1 Piece L3 Auto",
          "Left 1 Piece L4 Auto", "Center 1 Piece L4 Auto", "Right 1 Piece L4 Auto",

          "Left 2 Piece L1 Auto", "Right 2 Piece L1 Auto",
          "Left 2 Piece L2 Auto", "Right 2 Piece L2 Auto",
          "Left 2 Piece L3 Auto", "Right 2 Piece L3 Auto",
          "Left 2 Piece L4 Auto", "Right 2 Piece L4 Auto",

          "Left 3 Piece L1 Auto", "Right 3 Piece L1 Auto",
          "Left 3 Piece L2 Auto", "Right 3 Piece L2 Auto",
          "Left 3 Piece L3 Auto", "Right 3 Piece L3 Auto",
          "Left 3 Piece L4 Auto", "Right 3 Piece L4 Auto",

          "Left 4 Piece L1 Auto", "Right 4 Piece L1 Auto",
          "Left 4 Piece L2 Auto", "Right 4 Piece L2 Auto",
          "Left 4 Piece L3 Auto", "Right 4 Piece L3 Auto",
          "Left 4 Piece L4 Auto", "Right 4 Piece L4 Auto",

          "Left Forward", "Center Forward", "Right Forward",


  };
  DigitalInput[] autoSelectors = new DigitalInput[Math.min(autoNames.length, 10)];
  

  public RobotContainer() {
    setBindingsDriver();
    setBindingsManipulator();


    registerAutoCommands();
    SmartDashboard.putData(autoSelector);
    SmartDashboard.setPersistent("SendableChooser[0]");
    int i = 3;
    for (String n : autoNames) {
        autoSelector.addOption(n, i);
        i++;
    }
    ShuffleboardTab autoSelectorTab = Shuffleboard.getTab("Auto Chooser Tab");
    autoSelectorTab.add(autoSelector).withSize(2, 1);
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
  private void setBindingsDriver() {
    // new Trigger(() -> driverController.getRawButton(OI.Driver.X)).onTrue(new RunAlgae(algaeEffector, 1, false)); //wrong
    // new Trigger(() -> driverController.getRawButton(OI.Driver.Y)).onTrue(new RunAlgae(algaeEffector, 2, false));
    // new Trigger(() -> driverController.getRawButton(OI.Driver.B)).onTrue(new RunAlgae(algaeEffector, 3, false));
    // new Trigger(() -> driverController.getRawButton(OI.Driver.A)).onTrue(new RunAlgae(algaeEffector, 0, true));

  }


  private void setBindingsManipulator() {
    axisTrigger(manipulatorController, OI.Manipulator.IntakeTrigger)
      .whileTrue(new IntakeCoral(coralEffector));

    axisTrigger(manipulatorController, OI.Manipulator.OuttakeTrigger)
      .whileTrue(new OuttakeCoral(coralEffector));
    
    new JoystickButton(manipulatorController, OI.Manipulator.IntakeBumper)
      .whileTrue(new IntakeAlgae(algaeEffector));
    
    new JoystickButton(manipulatorController, OI.Manipulator.OuttakeBumper)
      .whileFalse(new OuttakeAlgae(algaeEffector));
    }
    

    private Trigger axisTrigger(GenericHID controller, Axis axis) {
      return new Trigger(() -> Math
              .abs(getStickValue(controller, axis)) > Constants.OI.MIN_AXIS_TRIGGER_VALUE);
    }


  /*public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }*/

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



  private void registerAutoCommands() {
    // NamedCommands.registerCommand("Intake", new Intake(intakeShooter));
    //NamedCommands.registerCommand("Eject", new Eject(intakeShooter));
  }

    private void setupAutos() {
        {
            for (int i = 0; i < autoNames.length; i++) {
                String name = autoNames[i];
                //Run Auto
                autoCommands.add(new PathPlannerAuto(name));
            }
        }
        {
            //No Auto
            autoCommands.add(0, new PrintCommand("Running NULL Auto!"));
        }
    }

    public Command getAutonomousCommand() {
        if (!hasSetupAutos) {
            setupAutos();
            hasSetupAutos = true;
        }
        Integer autoIndex = autoSelector.getSelected();

        if (autoIndex != null && autoIndex != 0) {
            new PrintCommand("Running selected auto: " + autoSelector.toString());
            return autoCommands.get(autoIndex.intValue());
        }
        return new PrintCommand("No auto :(");
    }
}

