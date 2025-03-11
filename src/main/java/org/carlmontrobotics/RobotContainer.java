// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics;

//199 files
import org.carlmontrobotics.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import org.carlmontrobotics.commands.OuttakeAlgae;
import org.carlmontrobotics.commands.ShootAlgae;
import org.carlmontrobotics.subsystems.AlgaeEffector;
import org.carlmontrobotics.commands.ArmToPosition;
import org.carlmontrobotics.commands.DealgaficationIntake;
import org.carlmontrobotics.commands.GroundIntakeAlgae;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
    private static boolean babyMode = false;

    // 1. using GenericHID allows us to use different kinds of controllers
    // 2. Use absolute paths from constants to reduce confusion
    public final GenericHID driverController = new GenericHID(OI.Driver.port);
    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

    private final Drivetrain drivetrain = new Drivetrain();
    private final AlgaeEffector algaeEffector = new AlgaeEffector();
    private final Elevator elevator = new Elevator();

    /* These are assumed to be equal to the AUTO ames in pathplanner */
    /* These must be equal to the pathPlanner path names from the GUI! */
    // Order matters - but the first one is index 1 on the physical selector - index
    // 0 is reserved for
    // null command.
    // the last auto is hard-coded to go straight. since we have __3__ Autos, port 4
    // is simplePz
    // straight
    private List<Command> autoCommands;
    private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

    private boolean hasSetupAutos = false;
    private final String[] autoNames = new String[] {};

    public RobotContainer() {
        //auto setup stuff
        {
            // Put any configuration overrides to the dashboard and the terminal
            SmartDashboard.putData("CONFIG overrides", Config.CONFIG);
            SmartDashboard.putData(drivetrain);
            System.out.println(Config.CONFIG);

            SmartDashboard.putData("BuildConstants", BuildInfo.getInstance());

            SmartDashboard.setDefaultBoolean("babymode", babyMode);
            SmartDashboard.setPersistent("babymode");
            // safe auto setup... stuff in setupAutos() is not safe to run here - will break
            // robot
            registerAutoCommands();
            SmartDashboard.putData(autoSelector);
            SmartDashboard.setPersistent("SendableChooser[0]");

            autoSelector.addOption("Nothing", 0);
            autoSelector.addOption("Raw Forward", 1);
            autoSelector.addOption("PP Simple Forward", 2);// index corresponds to index in autoCommands[]

            int i = 3;
            for (String n : autoNames) {
                autoSelector.addOption(n, i);
                i++;
            }

            ShuffleboardTab autoSelectorTab = Shuffleboard.getTab("Auto Chooser Tab");
            autoSelectorTab.add(autoSelector).withSize(2, 1);
        }

        SmartDashboard.putData(algaeEffector);
        setDefaultCommands();
        setBindingsDriver();
        setBindingsManipulator();
    }

    
    private void setDefaultCommands() {
        drivetrain.setDefaultCommand(new TeleopDrive(drivetrain,
            () -> ProcessedAxisValue(driverController, Axis.kLeftY),
            () -> ProcessedAxisValue(driverController, Axis.kLeftX),
            () -> ProcessedAxisValue(driverController, Axis.kRightX),
            () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
        
        elevator.setDefaultCommand(new TeleopElevator(elevator, () -> ProcessedAxisValue(manipulatorController, Axis.kLeftY)));
    }
    private void setBindingsDriver() {}
    private void setBindingsManipulator() {
        // intake
        new JoystickButton(manipulatorController, INTAKE_BUMPER)
                .whileTrue(new SequentialCommandGroup(
                        new ArmToPosition(algaeEffector, ARM_INTAKE_ANGLE),
                        new GroundIntakeAlgae(algaeEffector)));

        // outake

        // dealgify

        new JoystickButton(manipulatorController, OI.Manipulator.OuttakeBumper)
                .whileFalse(new OuttakeAlgae(algaeEffector));

        new JoystickButton(manipulatorController, OI.a_button)
                .onTrue(new DealgaficationIntake(algaeEffector));

        new JoystickButton(manipulatorController, OI.b_button)
                .onTrue(new ShootAlgae(algaeEffector));
        new JoystickButton(manipulatorController, OI.y_button)
                .onTrue(new InstantCommand(() -> {
                    algaeEffector.stopMotors();
                }));
    }

    // #region Input Processing

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

    // #region Auto Things

    private void registerAutoCommands() {
        //// AUTO-USABLE COMMANDS
        // NamedCommands.registerCommand("Intake", new Intake(intakeShooter));
    }

    private void setupAutos() {
        //// CREATING PATHS from files
        if (!hasSetupAutos) {
            autoCommands = new ArrayList<Command>();// clear old/nonexistent autos

            for (int i = 0; i < autoNames.length; i++) {
                String name = autoNames[i];

                autoCommands.add(new PathPlannerAuto(name));

                /* // Charles' opinion: we shouldn't have it path find to the starting pose at the start of match
                 * new SequentialCommandGroup( 
                 *      AutoBuilder.pathfindToPose(
                 *          PathPlannerAuto.getStaringPoseFromAutoFile(name),
                 *          PathPlannerAuto.getPathGroupFromAutoFile(name).get(0).
                 *          getPreviewStartingHolonomicPose(),
                 *          Autoc.pathConstraints), 
                 *      new PathPlannerAuto(name));
                 */
            }
            hasSetupAutos = true;

            // NOTHING
            autoCommands.add(0, new PrintCommand("Running NULL Auto!"));
            // RAW FORWARD command
            // autoCommands.add(1, new SequentialCommandGroup(
            //                 new InstantCommand(() -> drivetrain.drive(-.0001, 0, 0)), new WaitCommand(0.5),
            //                 new LastResortAuto(drivetrain))); 
            //FIXME LAST RESORT AUTO NEEDS TO BE DEFINED
            // dumb PP forward command
            autoCommands.add(2, new PrintCommand("PPSimpleAuto not Configured!"));
        }
        // force regeneration each auto call
        autoCommands.set(2, constructPPSimpleAuto());// overwrite this slot each time auto runs
    }

public Command constructPPSimpleAuto() {
        /**
         * PATHPLANNER SETTINGS Robot Width (m): .91 Robot Length(m): .94 Max Module Spd
         * (m/s): 4.30
         * Default Constraints Max Vel: 1.54, Max Accel: 6.86 Max Angvel: 360, Max
         * AngAccel: 360
         * (guesses!)
         */
        // default origin is on BLUE ALIANCE DRIVER RIGHT CORNER
        Pose2d currPos = drivetrain.getPose(); 

        //FIXME running red PP file autos seems to break something, so the robot drivetrain drives in the wrong direction.
            //running blue PP autos is fine though
        //Note: alliance detection and path generation work correctly!
        //Solution: Redeploy after auto.
        Pose2d endPos = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
                        ? currPos.transformBy(new Transform2d(1, 0, new Rotation2d(0)))
                        : currPos.transformBy(new Transform2d(-1, 0, new Rotation2d(0)));

        List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currPos, endPos);

        // Create the path using the bezier points created above, /* m/s, m/s^2, rad/s, rad/s^2 */
        PathPlannerPath path = new PathPlannerPath(bezierPoints,
                Autoc.pathConstraints, null, new GoalEndState(0, currPos.getRotation()));
        
        path.preventFlipping = false;// don't flip, we do that manually already.

        return new SequentialCommandGroup(
            new InstantCommand(()->drivetrain.drive(-.0001, 0, 0)),//align drivetrain wheels.
            AutoBuilder.followPath(path).beforeStarting(new WaitCommand(1)));
    }

    public Command getAutonomousCommand() {
        setupAutos();

        Integer autoIndex = autoSelector.getSelected();

        if (autoIndex != null && autoIndex != 0) {
            new PrintCommand("Running selected auto: " + autoSelector.toString());
            return autoCommands.get(autoIndex.intValue());
        }
        return new PrintCommand("No auto :(");
    }
}