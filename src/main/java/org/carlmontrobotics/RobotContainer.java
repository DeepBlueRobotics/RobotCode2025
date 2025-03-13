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
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;

//import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.BooleanSupplier;

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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
//control bindings
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.carlmontrobotics.Constants.Elevatorc.l1;
import static org.carlmontrobotics.Constants.Elevatorc.l2;
//constats
//import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class RobotContainer {
    private static boolean babyMode = false;

    // 1. using GenericHID allows us to use different kinds of controllers
    // 2. Use absolute paths from constants to reduce confusion
    public final GenericHID driverController = new GenericHID(Driver.port);
    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);
    private final Drivetrain drivetrain =  new Drivetrain();


    //private final Drivetrain drivetrain = new Drivetrain();

    /* These are assumed to be equal to the AUTO ames in pathplanner */
    /* These must be equal to the pathPlanner path names from the GUI! */
    // Order matters - but the first one is index 1 on the physical selector - index
    // 0 is reserved for
    // null command.
    // the last auto is hard-coded to go straight. since we have __3__ Autos, port 4
    // is simplePz
    // straight
    // private List<Command> autoCommands;
    // private SendableChooser<Integer> autoSelector = new SendableChooser<Integer>();

    // private boolean hasSetupAutos = false;
    // private final String[] autoNames = new String[] {};
    // private final AlgaeEffector algaeEffector = new AlgaeEffector();
     private final Elevator elevator = new Elevator();
     private SendableChooser<Command> autoChooser = new SendableChooser<>();
     
     
       //1. using GenericHID allows us to use different kinds of controllers
       //2. Use absolute paths from constants to reduce confusion
       
       public final CoralEffector coralEffector = new CoralEffector();
     
       // public final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
     
         public RobotContainer() {
             {
                 // Put any configuration overrides to the dashboard and the terminal
                 // SmartDashboard.putData("CONFIG overrides", Config.CONFIG);
                 // SmartDashboard.putData(drivetrain);
                 // System.out.println(Config.CONFIG);
     
                 // SmartDashboard.putData("BuildConstants", BuildInfo.getInstance());
     
                 // SmartDashboard.setDefaultBoolean("babymode", babyMode);
                 // SmartDashboard.setPersistent("babymode");
                 // // safe auto setup... stuff in setupAutos() is not safe to run here - will break
                 // // robot
                 // registerAutoCommands();
                 // SmartDashboard.putData(autoSelector);
                 // SmartDashboard.setPersistent("SendableChooser[0]");
     
                 // autoSelector.addOption("Nothing", 0);
                 // autoSelector.addOption("Raw Forward", 1);
                 // autoSelector.addOption("PP Simple Forward", 2);// index corresponds to index in autoCommands[]
     
                 // int i = 3;
                 // for (String n : autoNames) {
                 //     autoSelector.addOption(n, i);
                 //     i++;
                 // }
     
                 // ShuffleboardTab autoSelectorTab = Shuffleboard.getTab("Auto Chooser Tab");
                 // autoSelectorTab.add(autoSelector).withSize(2, 1);
             }
     
             RegisterAutoCommands();
             autoChooser = AutoBuilder.buildAutoChooser();
        RegisterCustomAutos();
         SmartDashboard.putData("Auto Chooser", autoChooser);    SmartDashboard.putData("Coral Intake", new CoralIntake(coralEffector));
          SmartDashboard.putData("coral out", new CoralOuttake(coralEffector));
        setDefaultCommands();
        setBindingsDriver();
        setBindingsManipulator();
    }

   

    private void setBindingsDriver() {
        // new Trigger(() -> driverController.getRawButton(OI.Driver.X)).onTrue(new
        // RunAlgae(algaeEffector, 1, false)); //wrong
        // new Trigger(() -> driverController.getRawButton(OI.Driver.Y)).onTrue(new
        // RunAlgae(algaeEffector, 2, false));
        // new Trigger(() -> driverController.getRawButton(OI.Driver.B)).onTrue(new
        // RunAlgae(algaeEffector, 3, false));
        // new Trigger(() -> driverController.getRawButton(OI.Driver.A)).onTrue(new
        // RunAlgae(algaeEffector, 0, true));
        new JoystickButton(driverController, 3).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kForward)); //press x to run
        new JoystickButton(driverController, 4).whileTrue(elevator.sysIdQuasistatic(SysIdRoutine.Direction.kReverse)); //press y to run
        new JoystickButton(driverController, 1).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kForward)); //press a to run
        new JoystickButton(driverController, 2).whileTrue(elevator.sysIdDynamic(SysIdRoutine.Direction.kReverse)); //press b to run

    }

   
        // new JoystickButton(manipulatorController, INTAKE_BUMPER)
        //         .whileTrue(new SequentialCommandGroup(
        //                 new ArmToPosition(algaeEffector, ARM_INTAKE_ANGLE),
        //                 new GroundIntakeAlgae(algaeEffector)));

        // outake

        // dealgify

    //     new JoystickButton(manipulatorController, OI.Manipulator.OuttakeBumper)
    //             .whileFalse(new OuttakeAlgae(algaeEffector));

    //     new JoystickButton(manipulatorController, XboxController.Button.kA.value)
    //             .onTrue(new DealgaficationIntake(algaeEffector));

    //     new JoystickButton(manipulatorController, XboxController.Button.kB.value)
    //             .onTrue(new ShootAlgae(algaeEffector));
    //     new JoystickButton(manipulatorController, XboxController.Button.kY.value)
    //             .onTrue(new InstantCommand(() -> {
    //                 algaeEffector.stopMotors();
    //             }));
    

    /**
     * Flips an axis' Y coordinates upside down, but only if the select axis is a
     * joystick axis
     *
     * @param hid  The controller/plane joystick the axis is on
     * @param axis The processed axis
     * @return The processed value.
     */
   

    /**
     * Processes an input from the joystick into a value between -1 and 1,
     * sinusoidally instead of
     * linearly
     *
     * @param value The value to be processed.
     * @return The processed value.
     */
   

    /**
     * Combines both getStickValue and inputProcessing into a single function for
     * processing joystick
     * outputs
     *
     * @param hid  The controller/plane joystick the axis is on
     * @param axis The processed axis
     * @return The processed value.
     */
    

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
   

    private void RegisterAutoCommands() {
      //I made some of the constants up, so change once merged
      //AlgaeEffector
      // NamedCommands.registerCommand("GroundIntakeAlgae", new GroundIntakeAlgae(algaeEffector));
      // NamedCommands.registerCommand("DealgaficationIntake", new DealgaficationIntake(algaeEffector));
      // NamedCommands.registerCommand("ShootAlgae", new ShootAlgae(algaeEffector));
      //CoralEffector
      // NamedCommands.registerCommand("CoralIntake", new CoralIntake(coralEffector));
      // NamedCommands.registerCommand("CoralOutake", new CoralOutake(coralEffector));
      // //AlgaeArm
      // NamedCommands.registerCommand("ArmToDeAlgafy", new ArmToPosition(AlgaeEffector, Armc.DeAlgafy_Angle));
      // NamedCommands.registerCommand("ArmToIntake", new ArmToPosition(AlgaeEffector, Armc.Intake_Angle));
      // NamedCommands.registerCommand("ArmToShoot", new ArmToPosition(AlgaeEffector, Armc.Shoot_Angle));
      // //Elevator
      // NamedCommands.registerCommand("ElevatorIntake", new ElevatorLPos(Elevator, Elevatorc.IntakePos));
      // NamedCommands.registerCommand("ElevatorL1", new ElevatorLPos(Elevator, Elevatorc.L1Pos));
      // NamedCommands.registerCommand("ElevatorL2", new ElevatorLPos(Elevator, Elevatorc.L2Pos));
      // NamedCommands.registerCommand("ElevatorL3", new ElevatorLPos(Elevator, Elevatorc.L3Pos));
      // NamedCommands.registerCommand("ElevatorL4", new ElevatorLPos(Elevator, Elevatorc.L4Pos));

      // //Limelight
      // NamedCommands.registerCommand("AlignToCoralStation", new AlignToCoralStation(Limelight, drivetrain));
      // NamedCommands.registerCommand("AlignToReef", new AlignToReef(Limelight, drivetrain));
      // NamedCommands.registerCommand("MoveToLeftBranch", new MoveToLeftBranch(Limelight, LimelightHelpers, drivetrain));
      // NamedCommands.registerCommand("MoveToRightBranch", new MoveToRightBranch(Limelight, LimelightHelpers, drivetrain));

      //Sequential and Parralel Commands
        /*NamedCommands.registerCommand("L2", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL2(Elevator, Elevatorc.L2Pos),
                                new ArmToPos(arm, Armc.DeAlgafy_Angle)),
                                new DealgaficationIntake(algaeEffector),
                        new CoralOutake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/

        /*NamedCommands.registerCommand("L3", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL3(Elevator, Elevatorc.L3Pos),
                                new ArmToPos(arm, Armc.DeAlgafy_Angle)),
                                new DealgaficationIntake(algaeEffector),
                        new CoralOutake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/

        /*NamedCommands.registerCommand("L4", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL3(Elevator, Elevatorc.L4Pos),
                                new ArmToPos(arm, Armc.DeAlgafy_Angle)),
                                new DealgaficationIntake(algaeEffector),
                        new CoralOutake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/
                        
        /*NamedCommands.registerCommand("IntakeCoral",
        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToCoralStation(drivetrain, limelight),
                                new IntakeCoral(coralEffector))));*/

    }

    private void RegisterCustomAutos(){
        // autoChooser.addOption("ForwardLastResortAuto", new LastResortAuto(drivetrain, 1));
        // autoChooser.addOption("BackwardLastResortAuto", new LastResortAuto(drivetrain, -1));
    
    }
    //private void setupAutos() {
        //// CREATING PATHS from files
        // if (!hasSetupAutos) {
        //     autoCommands = new ArrayList<Command>();// clear old/nonexistent autos

        //     for (int i = 0; i < autoNames.length; i++) {
        //         String name = autoNames[i];

        //         autoCommands.add(new PathPlannerAuto(name));

                /*
                 * // Charles' opinion: we shouldn't have it path find to the starting pose at
                 * the start of match
                 * new SequentialCommandGroup(
                 * AutoBuilder.pathfindToPose(
                 * PathPlannerAuto.getStaringPoseFromAutoFile(name),
                 * PathPlannerAuto.getPathGroupFromAutoFile(name).get(0).
                 * getPreviewStartingHolonomicPose(),
                 * Autoc.pathConstraints),
                 * new PathPlannerAuto(name));
                 */
            //}
        //     hasSetupAutos = true;

        //     // NOTHING
        //     autoCommands.add(0, new PrintCommand("Running NULL Auto!"));
        //     // RAW FORWARD command
        //     // autoCommands.add(1, new SequentialCommandGroup(
        //     // new InstantCommand(() -> drivetrain.drive(-.0001, 0, 0)), new
        //     // WaitCommand(0.5),
        //     // new LastResortAuto(drivetrain)));
        //     // dumb PP forward command
        //     autoCommands.add(2, new PrintCommand("PPSimpleAuto not Configured!"));
        // }
        // // force regeneration each auto call
        // autoCommands.set(2, constructPPSimpleAuto());// overwrite this slot each time auto runs
    //}

    //public Command constructPPSimpleAuto() {
        /**
         * PATHPLANNER SETTINGS Robot Width (m): .91 Robot Length(m): .94 Max Module Spd
         * (m/s): 4.30
         * Default Constraints Max Vel: 1.54, Max Accel: 6.86 Max Angvel: 360, Max
         * AngAccel: 360
         * (guesses!)
         */
        // default origin is on BLUE ALIANCE DRIVER RIGHT CORNER
        // Pose2d currPos = drivetrain.getPose();

        // FIXME running red PP file autos seems to break something, so the robot
        // drivetrain drives in the wrong direction.
        // running blue PP autos is fine though
        // Note: alliance detection and path generation work correctly!
        // Solution: Redeploy after auto.
        // Pose2d endPos = (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
        //         ? currPos.transformBy(new Transform2d(1, 0, new Rotation2d(0)))
        //         : currPos.transformBy(new Transform2d(-1, 0, new Rotation2d(0)));

        // List<Waypoint> bezierPoints = PathPlannerPath.waypointsFromPoses(currPos, endPos);

        // // Create the path using the bezier points created above, /* m/s, m/s^2, rad/s,
        // // rad/s^2 */
        // PathPlannerPath path = new PathPlannerPath(bezierPoints,
        //         Autoc.pathConstraints, null, new GoalEndState(0, currPos.getRotation()));

        // path.preventFlipping = false;// don't flip, we do that manually already.

        // return new SequentialCommandGroup(
        //         new InstantCommand(() -> drivetrain.drive(-.0001, 0, 0)), // align drivetrain wheels.
        //         AutoBuilder.followPath(path).beforeStarting(new WaitCommand(1)));
        //return new PrintCommand("I HAT EEEYTHI");
    //}

    

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
  }

  private void setBindingsManipulator() {
    // new JoystickButton(manipulatorController, OI.Manipulator.OUTAKE_BUTTON)
    //   .whileTrue(new CoralOutake(coralEffector))
    //   .whileFalse(new CoralIntake(coralEffector));
    // new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_BUTTON)
    //   .whileTrue(new ManualCoralIntake());
    axisTrigger(manipulatorController, Axis.kLeftTrigger)
    .whileTrue(new CoralOuttake(coralEffector))
    .whileFalse(new CoralIntake(coralEffector));
    axisTrigger(manipulatorController, Axis.kRightTrigger)
    .whileTrue(new CoralIntakeManual(coralEffector));
new JoystickButton(manipulatorController, OI.Manipulator.Y).onTrue(new ElevatorToPos(elevator, l2));
        new JoystickButton(manipulatorController, Button.kA.value).onTrue(new ElevatorToPos(elevator, l1));
        new JoystickButton(manipulatorController, Button.kB.value).onTrue(new ElevatorToPos(elevator, .5));
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
