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

import org.carlmontrobotics.Constants.AlgaeEffectorc;
import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.Constants.Elevatorc;
import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Driver;
import org.carlmontrobotics.Constants.OI.Manipulator;
import org.carlmontrobotics.commands.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;

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
<<<<<<< HEAD

=======
import org.carlmontrobotics.commands.OuttakeAlgae;
import org.carlmontrobotics.subsystems.AlgaeEffector;
import org.carlmontrobotics.commands.ArmToPosition;
import org.carlmontrobotics.commands.DealgaficationIntake;
import org.carlmontrobotics.commands.GroundIntakeAlgae;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.*;


import org.carlmontrobotics.subsystems.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;

import org.carlmontrobotics.Constants.Drivetrainc.Autoc;
import org.carlmontrobotics.Constants.Elevatorc;
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
import com.pathplanner.lib.auto.NamedCommands;


import org.carlmontrobotics.Constants.OI;
import org.carlmontrobotics.Constants.OI.Manipulator;

>>>>>>> AlgaeBranch
//limit switch
import edu.wpi.first.wpilibj.DigitalInput;
//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
<<<<<<< HEAD
=======
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.XboxController.Axis;
>>>>>>> AlgaeBranch
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
import edu.wpi.first.wpilibj2.command.button.Trigger;

<<<<<<< HEAD
import static org.carlmontrobotics.Constants.Elevatorc.elevatorOffset;
=======
>>>>>>> AlgaeBranch
import static org.carlmontrobotics.Constants.Elevatorc.l1;
import static org.carlmontrobotics.Constants.Elevatorc.l2;
import static org.carlmontrobotics.Constants.Elevatorc.l3;
import static org.carlmontrobotics.Constants.Elevatorc.l4;
<<<<<<< HEAD
import static org.carlmontrobotics.Constants.Elevatorc.testl4;
import static org.carlmontrobotics.Constants.Elevatorc.testl4RaiseHeight;
=======
>>>>>>> AlgaeBranch
//constats
//import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

<<<<<<< HEAD
import static org.carlmontrobotics.commands.TeleopDrive.babyMode;
import static org.carlmontrobotics.commands.TeleopDrive.babyModeSupplier;



public class RobotContainer {
    //private static boolean babyModeg = true;
    
    // 1. using GenericHID allows us to use different kinds of controllers
    // 2. Use absolute paths from constants to reduce confusion
    public final GenericHID driverController = new GenericHID(Driver.port);
    
    public final XboxController driverRumble = new XboxController(Driver.port); //For rumbling the controller

    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

    public final Drivetrain drivetrain =  new Drivetrain();
    //public final AlgaeEffector algaeEffector = new AlgaeEffector();
    private final Limelight limelight = new Limelight(drivetrain);


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

=======
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

>>>>>>> AlgaeBranch
    // private boolean hasSetupAutos = false;
    // private final String[] autoNames = new String[] {};
    // private final AlgaeEffector algaeEffector = new AlgaeEffector();
     private final Elevator elevator = new Elevator();
<<<<<<< HEAD
=======
     private final AlgaeEffector algaeEffector = new AlgaeEffector();
>>>>>>> AlgaeBranch
     private SendableChooser<Command> autoChooser = new SendableChooser<>();
     
     
       //1. using GenericHID allows us to use different kinds of controllers
       //2. Use absolute paths from constants to reduce confusion
       
       public final CoralEffector coralEffector = new CoralEffector();
<<<<<<< HEAD
      //TODO activate arm
       //public final AlgaeEffector algaeEffector = new AlgaeEffector();
=======
>>>>>>> AlgaeBranch
     
       // public final DigitalInput limitSwitch = new DigitalInput(LIMIT_SWITCH_PORT);
     
         public RobotContainer() {
<<<<<<< HEAD
             {
                SmartDashboard.putData("intake", new CoralIntake(coralEffector));
=======
            SmartDashboard.putData(algaeEffector);
             {
>>>>>>> AlgaeBranch
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
     
<<<<<<< HEAD
        RegisterAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("null forward auto", new LastResortAuto(drivetrain, -1, 4, 8));
        RegisterCustomAutos();
        SmartDashboard.putData("Auto Chooser", autoChooser);    SmartDashboard.putData("Coral Intake", new CoralIntake(coralEffector));
        SmartDashboard.putData("coral out", new AutonCoralOuttake(coralEffector));
        setDefaultCommands();
        setBindingsDriver();
        setBindingsManipulator();
        // SmartDashboard.putData("align right", new MoveToRightBranch(drivetrain, limelight));
        // SmartDashboard.putData("align left", new MoveToLeftBranch(drivetrain, limelight));
        final int waitTime = 3;
        final int speed = 2;
        SmartDashboard.putData("Battery testing go vroooom", new SequentialCommandGroup(
            // Right L2 then reset
             new BatteryTesting(drivetrain, elevator, limelight, l2),
             new BatteryTesting(drivetrain, elevator, limelight, l3),
             new BatteryTesting(drivetrain, elevator, limelight, l4)
        ));
        SmartDashboard.putData("Rotate Command",new RotateToTag(drivetrain, limelight));
    }
   
   

    private void setBindingsDriver() {
        new JoystickButton(driverController, Driver.resetFieldOrientationButton)
            .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
        axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON)
            .onTrue(new InstantCommand(()->drivetrain.setFieldOriented(false)))
            .onFalse(new InstantCommand(()->drivetrain.setFieldOriented(true)));


        new JoystickButton(driverController, 7)
            .onTrue(new MoveToLeftBranch(drivetrain, limelight, elevator));
        new JoystickButton(driverController, 8)
            .onTrue(new MoveToRightBranch(drivetrain, limelight, elevator));

        axisTrigger(driverController, Driver.LEFT_TRIGGER_BUTTON)
            .onTrue(new InstantCommand(() -> drivetrain.setExtraSpeedMult(.5)))//normal max turn is .5
            .onFalse(new InstantCommand(() -> drivetrain.setExtraSpeedMult(0)));
        new POVButton(driverController, 180)
            .whileTrue(new ParallelCommandGroup(
        new InstantCommand(() -> drivetrain.stop()),
        new TeleopDrive(drivetrain, ()->0, ()->0, ()->0, ()->true)));
        new POVButton(driverController, 0).onTrue(new Flush(limelight, drivetrain));


        
        new JoystickButton(driverController, Driver.b).onTrue(
            new MoveToAlignReef(drivetrain, limelight, elevator, true, //To align with right branch
                driverRumble));
        new JoystickButton(driverController, Driver.x).onTrue(
            new MoveToAlignReef(drivetrain, limelight, elevator, false, //To align with right branch
                driverRumble));
        
        //TODO test rotation, need to tune pid for that
        // new POVButton(driverController, 0)
        //     .onTrue(new RotateToNearest60(drivetrain));
        // new POVButton(driverController, 180).onTrue(new RotateToFieldRelativeAngle(Rotation2d.fromDegrees(90), drivetrain));
        // axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON)
        //   .onTrue(new InstantCommand(() -> drivetrain.stop()))

        //Dont need that
        // new JoystickButton(driverController, Driver.a)
        //     .onTrue(new SequentialCommandGroup(
        //         new WaitUntilAtAngle(drivetrain),
        //         new L4Backup(drivetrain)
        // ));
        /*new JoystickButton(driverController, Driver.a)
        .onTrue(new L4Backup(drivetrain));
        }*/
    }

   
=======
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
       
      new JoystickButton(driverController, Driver.resetFieldOrientationButton)
                .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
    }

   
        
    

>>>>>>> AlgaeBranch
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
    

<<<<<<< HEAD
    // /**
    //  * Returns zero if a axis input is inside the deadzone
    //  *
    //  * @param hid  The controller/plane joystick the axis is on
    //  * @param axis The processed axis
    //  * @return The processed value.
    //  */
    // private double DeadzonedAxis(double axOut) {
    //     return (Math.abs(axOut) <= OI.JOY_THRESH) ? 0.0 : axOut;
    // }
=======
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
>>>>>>> AlgaeBranch

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
      NamedCommands.registerCommand("CoralIntake", new CoralIntake(coralEffector));
<<<<<<< HEAD
      NamedCommands.registerCommand("CoralOutake", new AutonCoralOuttake(coralEffector));
=======
      NamedCommands.registerCommand("CoralOutake", new CoralOuttake(coralEffector));
>>>>>>> AlgaeBranch
      
      //AlgaeArm
      // NamedCommands.registerCommand("ArmToDeAlgafy", new ArmToPosition(AlgaeEffector, Armc.DeAlgafy_Angle));
      // NamedCommands.registerCommand("ArmToIntake", new ArmToPosition(AlgaeEffector, Armc.Intake_Angle));
      // NamedCommands.registerCommand("ArmToShoot", new ArmToPosition(AlgaeEffector, Armc.Shoot_Angle));

      //Elevator
      NamedCommands.registerCommand("ElevatorIntake", new ElevatorToPos(elevator, Elevatorc.downPos));
      NamedCommands.registerCommand("ElevatorL1", new ElevatorToPos(elevator, Elevatorc.l1));
      NamedCommands.registerCommand("ElevatorL2", new ElevatorToPos(elevator, Elevatorc.l2));
      NamedCommands.registerCommand("ElevatorL3", new ElevatorToPos(elevator, Elevatorc.l3));
      NamedCommands.registerCommand("ElevatorL4", new ElevatorToPos(elevator, Elevatorc.l4));

      // //Limelight
      // NamedCommands.registerCommand("AlignToCoralStation", new AlignToCoralStation(Limelight, drivetrain));
      // NamedCommands.registerCommand("AlignToReef", new AlignToReef(Limelight, drivetrain));
      // NamedCommands.registerCommand("MoveToLeftBranch", new MoveToLeftBranch(Limelight, LimelightHelpers, drivetrain));
      // NamedCommands.registerCommand("MoveToRightBranch", new MoveToRightBranch(Limelight, LimelightHelpers, drivetrain));


      //Sequential and Parralel Commands
<<<<<<< HEAD
      //-------------------------------------------------------------------------------------------------
      //Intake CMD
      NamedCommands.registerCommand("IntakeCoral",
      new SequentialCommandGroup(
                      new ParallelCommandGroup(
                              new CoralIntake(coralEffector))));

      //L1 CMDS
      NamedCommands.registerCommand("L1 Right",
              new SequentialCommandGroup(
                  new MoveToRightBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l1), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));

      NamedCommands.registerCommand("L1 Left",
              new SequentialCommandGroup(
                  new MoveToLeftBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l1), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));

      // //L2 Dealgify CMDs 
      // NamedCommands.registerCommand("L2 Dealgify Left",
      //         new SequentialCommandGroup(
      //             new MoveToLeftBranch(drivetrain, limelight),
      //             new ParallelCommandGroup(
      //                   new ElevatorToPos(elevator, Elevatorc.l2), 
      //                   new SequentialCommandGroup(
      //                       new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
      //                       new DealgaficationIntake(algaeEffector))),
      //             new AutonCoralOuttake(coralEffector), 
      //             new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      // NamedCommands.registerCommand("L2 Dealgify Right",
      //         new SequentialCommandGroup(
      //             new MoveToRightBranch(drivetrain, limelight),
      //             new ParallelCommandGroup(
      //                   new ElevatorToPos(elevator, Elevatorc.l2), 
      //                   new SequentialCommandGroup(
      //                       new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
      //                       new DealgaficationIntake(algaeEffector))),
      //             new AutonCoralOuttake(coralEffector), 
      //             new ElevatorToPos(elevator, Elevatorc.downPos)));

      //L2 No-Dealify CMDS
      NamedCommands.registerCommand("L2 No-Dealgify Left", 
              new SequentialCommandGroup(
                  new MoveToLeftBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l2), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      NamedCommands.registerCommand("L2 No-Dealgify Right", 
              new SequentialCommandGroup(
                  new MoveToRightBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l2), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));    

      //L3 Dealgify CMDs 
      // NamedCommands.registerCommand("L3 Dealgify Left", 
      //         new SequentialCommandGroup(
      //             new MoveToLeftBranch(drivetrain, limelight),
      //             new ParallelCommandGroup(
      //                   new ElevatorToPos(elevator, Elevatorc.l3), 
      //                   new SequentialCommandGroup(
      //                       new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
      //                       new DealgaficationIntake(algaeEffector))),
      //             new AutonCoralOuttake(coralEffector), 
      //             new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      // NamedCommands.registerCommand("L3 Dealgify Right", 
      //         new SequentialCommandGroup(
      //             new MoveToRightBranch(drivetrain, limelight),
      //             new ParallelCommandGroup(
      //                   new ElevatorToPos(elevator, Elevatorc.l3), 
      //                   new SequentialCommandGroup(
      //                       new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
      //                       new DealgaficationIntake(algaeEffector))),
      //             new AutonCoralOuttake(coralEffector), 
      //             new ElevatorToPos(elevator, Elevatorc.downPos)));

      //L3 No-Dealify CMDS
      NamedCommands.registerCommand("L3 No-Dealgify Left", 
              new SequentialCommandGroup(
                  new MoveToLeftBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l3), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      NamedCommands.registerCommand("L3 No-Dealgify Right", 
              new SequentialCommandGroup(
                  new MoveToRightBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l3), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));    

    //  //L4 Backup CMDS
    //   NamedCommands.registerCommand("L4 Backup Left", 
    //           new SequentialCommandGroup(
    //               new MoveToLeftBranch(drivetrain, limelight),
    //               new L4BackupUsingArea(drivetrain, limelight),
    //               new ParallelCommandGroup(
    //                     new ElevatorToPos(elevator, Elevatorc.l4), 
    //                     new SequentialCommandGroup(
    //                         new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
    //                         new DealgaficationIntake(algaeEffector))),
    //               new AutonCoralOuttake(coralEffector), 
    //               new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      // NamedCommands.registerCommand("L4 Backup Right", 
      //         new SequentialCommandGroup(
      //             new MoveToRightBranch(drivetrain, limelight),
      //             new L4BackupUsingArea(drivetrain, limelight),
      //             new ParallelCommandGroup(
      //                   new ElevatorToPos(elevator, Elevatorc.l4), 
      //                   new SequentialCommandGroup(
      //                       new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
      //                       new DealgaficationIntake(algaeEffector))),
      //             new AutonCoralOuttake(coralEffector), 
      //             new ElevatorToPos(elevator, Elevatorc.downPos)));

      //L4 No-Backup CMDS
      NamedCommands.registerCommand("L4 No-Backup Left", 
              new SequentialCommandGroup(
                  new MoveToLeftBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l4), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));
      
      NamedCommands.registerCommand("L4 No-Backup Right", 
              new SequentialCommandGroup(
                  new MoveToRightBranch(drivetrain, limelight, elevator),
                  new ElevatorToPos(elevator, Elevatorc.l4), 
                  new AutonCoralOuttake(coralEffector), 
                  new ElevatorToPos(elevator, Elevatorc.downPos)));    


//-------------------------------------------------------------------------------------------
        //TODO: Make new scoring Autos
        //TODO: Change CMDS in PP
        /*NamedCommands.registerCommand("IntakeCoralNoLL",
=======
        NamedCommands.registerCommand("IntakeCoralNoLL",
>>>>>>> AlgaeBranch
        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new CoralIntake(coralEffector))));

        NamedCommands.registerCommand("L1NoLL&NoAlgae", new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ElevatorToPos(elevator, Elevatorc.l1),
                        new CoralOuttake(coralEffector),
                        new ElevatorToPos(elevator, Elevatorc.downPos))));

        NamedCommands.registerCommand("L2NoLL&NoAlgae", new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ElevatorToPos(elevator, Elevatorc.l2),
                        new CoralOuttake(coralEffector),
                        new ElevatorToPos(elevator, Elevatorc.downPos))));
                    
        NamedCommands.registerCommand("L3NoLL&NoAlgae", new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ElevatorToPos(elevator, Elevatorc.l3),
                        new CoralOuttake(coralEffector),
                        new ElevatorToPos(elevator, Elevatorc.downPos))));

        NamedCommands.registerCommand("L4NoLL&NoAlgae", new SequentialCommandGroup(
                new SequentialCommandGroup(
                        new ElevatorToPos(elevator, Elevatorc.l4),
                        new CoralOuttake(coralEffector),
<<<<<<< HEAD
                        new ElevatorToPos(elevator, Elevatorc.downPos))));*/                          
=======
                        new ElevatorToPos(elevator, Elevatorc.downPos))));                             
>>>>>>> AlgaeBranch

        /*NamedCommands.registerCommand("L2LL&&NoAlgae", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL2(Elevator, Elevatorc.l2)),
                                //new ArmToPos(arm, Armc.DeAlgafy_Angle),
                                //new DealgaficationIntake(algaeEffector),
                        new CoralOuttake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/

        /*NamedCommands.registerCommand("L3LL&&NoAlgae", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL2(Elevator, Elevatorc.l3)),
                                //new ArmToPos(arm, Armc.DeAlgafy_Angle),
                                //new DealgaficationIntake(algaeEffector),
                        new CoralOuttake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/

        /*NamedCommands.registerCommand("L4LL&&NoAlgae", new SequentialCommandGroup(
        new ParallelDeadlineGroup(
                new WaitCommand(3.0),
                new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToReef(drivetrain, limelight),
                                new ElevatorL2(Elevator, Elevatorc.l4)),
                                //new ArmToPos(arm, Armc.DeAlgafy_Angle),
                                //new DealgaficationIntake(algaeEffector),
                        new CoralOuttake(coralEffector),
                        new ElevatorIntake(Elevator, Elevatorc.IntakePos)))));*/
                                                    
        /*NamedCommands.registerCommand("IntakeCoralLL",
        new SequentialCommandGroup(
                        new ParallelCommandGroup(
                                new AlignToCoralStation(drivetrain, limelight),
                                new IntakeCoral(coralEffector))));*/

    }

    private void RegisterCustomAutos(){
<<<<<<< HEAD
        /*autoChooser.addOption("DriveRaiseAutonL2Center", new SequentialCommandGroup(
          new LastResortAuto(drivetrain, 1, 4, 2.5), 
          new ElevatorToPos(elevator, l2)));
        autoChooser.addOption("DriveRaiseAutonL2Left/Right", new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 4, 3), 
            new ElevatorToPos(elevator, l2)));

        autoChooser.addOption("DriveRaiseAutonL2ScoreCenter", 
            new SequentialCommandGroup(new LastResortAuto(drivetrain, 1, 4, 2.5), 
            new ElevatorToPos(elevator, l2), new CoralFastOutake(coralEffector)));
        autoChooser.addOption("DriveRaiseAutonL2ScoreLeft/Right", 
            new SequentialCommandGroup(new LastResortAuto(drivetrain, 1, 4, 3), 
            new ElevatorToPos(elevator, l2), new CoralFastOutake(coralEffector)));

        autoChooser.addOption("DriveRaiseAutonL4Center", new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 4, 2.5),  
            new ElevatorToPos(elevator, l4)));
        autoChooser.addOption("DriveRaiseAutonL4Left/Right", new SequentialCommandGroup(
              new LastResortAuto(drivetrain, 1, 4, 3),  
              new ElevatorToPos(elevator, l4)));

        autoChooser.addOption("DriveRaiseAutonL4ScoreCenter", 
            new SequentialCommandGroup(new LastResortAuto(drivetrain, 1, 4, 2.5),  
            new ElevatorToPos(elevator, l4), new CoralFastOutake(coralEffector)));
        autoChooser.addOption("DriveRaiseAutonL4ScoreLeft/Right", 
            new SequentialCommandGroup(new LastResortAuto(drivetrain, 1, 4, 3),  
            new ElevatorToPos(elevator, l4), new CoralFastOutake(coralEffector)));*/

        //TODO: Friday Testing


        //-----------------------------------------

        //Center Last Resort
        autoChooser.addOption("ForwardLastResortAutoCenter", new LastResortAuto(drivetrain, 1, 1, 2.5));
        autoChooser.addOption("BackwardLastResortAutoCenter", new LastResortAuto(drivetrain, -1, 1, 2.5));

        //Left/Right Last Resort
        autoChooser.addOption("ForwardLastResortAutoLeft/Right", new LastResortAuto(drivetrain, 1, 4, 3));
        autoChooser.addOption("BackwardLastResortAutoLeft/Right", new LastResortAuto(drivetrain, -1, 1, 3)); 
        
        //Center L1
        autoChooser.addOption("CustomL1ScoreCenterLeftBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.0),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l1),
            new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos))); 

        autoChooser.addOption("CustomL1ScoreCenterRightBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.0),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l1),
            new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    

        //Left/Right L1
        autoChooser.addOption("CustomL1ScoreLeftBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l1),
            new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));  

        autoChooser.addOption("CustomL1ScoreRightBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l1),
            new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos))); 

        // //Center L2(Dealgify)
        // autoChooser.addOption("CustomDealgifyingL2ScoreCenterLeftBranch", 
        //     new SequentialCommandGroup(
        //     new LastResortAuto(drivetrain, 1, 1, 2.0),  
        //     new MoveToLeftBranch(drivetrain, limelight),
        //         new ParallelCommandGroup(
        //             new ElevatorToPos(elevator, l2),
        //             new SequentialCommandGroup(
        //                 new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
        //                 new DealgaficationIntake(algaeEffector))),
        //         new AutonCoralFastOutake(coralEffector),
        //     new ElevatorToPos(elevator, Elevatorc.downPos)));    

        // autoChooser.addOption("CustomDealgifyingL2ScoreCenterRightBranch", 
        //     new SequentialCommandGroup(
        //     new LastResortAuto(drivetrain, 1, 1, 2.5),  
        //     new MoveToRightBranch(drivetrain, limelight),
        //         new ParallelCommandGroup(
        //             new ElevatorToPos(elevator, l2),
        //             new SequentialCommandGroup(
        //                 new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
        //                 new DealgaficationIntake(algaeEffector))),
        //         new AutonCoralFastOutake(coralEffector),
        //     new ElevatorToPos(elevator, Elevatorc.downPos)));  
            
        //Left/Right L2
        autoChooser.addOption("CustomL2ScoreLeftBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l2),
            new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    

        autoChooser.addOption("CustomL2ScoreRightBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l2),
            new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));      

        //Center L3
        autoChooser.addOption("CustomL3ScoreCenterLeftBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 4.0),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l3),
            new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));  

        autoChooser.addOption("CustomL3ScoreCenterRightBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 4.0),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
            new ElevatorToPos(elevator, l3),
            new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    
            
        //Left/Right L3(Dealgify)
        // autoChooser.addOption("CustomDealgifyingL3ScoreLeftBranchL/R", 
        //     new SequentialCommandGroup(
        //     new LastResortAuto(drivetrain, 1, 1, 2.5),  
        //     new MoveToLeftBranch(drivetrain, limelight),
        //         new ParallelCommandGroup(
        //             new ElevatorToPos(elevator, l3),
        //             new SequentialCommandGroup(
        //                 new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
        //                 new DealgaficationIntake(algaeEffector))),
        //         new AutonCoralFastOutake(coralEffector),
        //     new ElevatorToPos(elevator, Elevatorc.downPos)));    

        // autoChooser.addOption("CustomDealgifyingL3ScoreRightBranchL/R", 
        //     new SequentialCommandGroup(
        //     new LastResortAuto(drivetrain, 1, 1, 2.5),  
        //     new MoveToRightBranch(drivetrain, limelight),
        //         new ParallelCommandGroup(
        //             new ElevatorToPos(elevator, l2),
        //             new SequentialCommandGroup(
        //                 new ArmToPosition(algaeEffector, AlgaeEffectorc.ARM_DEALGAFYING_ANGLE),
        //                 new DealgaficationIntake(algaeEffector))),
        //         new AutonCoralFastOutake(coralEffector),
        //     new ElevatorToPos(elevator, Elevatorc.downPos)));

        //Center L4
        autoChooser.addOption("SamoL4AutonLeft", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
                new ElevatorToPos(elevator, testl4),
                new AutonCoralOuttake(coralEffector),
                new ParallelCommandGroup(
                    new ElevatorToPos(elevator, testl4 + testl4RaiseHeight),
                    new AutonCoralFastOutake(coralEffector)
                ),
            new ElevatorToPos(elevator, Elevatorc.downPos))); 
            
        autoChooser.addOption("SamoL4AutonRight", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
                new ElevatorToPos(elevator, testl4),
                new AutonCoralOuttake(coralEffector),
                new ParallelCommandGroup(
                    new ElevatorToPos(elevator, testl4 + testl4RaiseHeight),
                    new AutonCoralFastOutake(coralEffector)
                ),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    

        //Left/Right L4
        autoChooser.addOption("CustomL4ScoreLeftBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
                new ElevatorToPos(elevator, l4),
                new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos))); 

        autoChooser.addOption("CustomL4ScoreRightBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
                new ElevatorToPos(elevator, l4),
                new AutonCoralFastOutake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));  

        //Center L4(Backup)
        autoChooser.addOption("CustomBackupL4ScoreCenterLeftBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.0),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
                new L4Backup(drivetrain),
                new ElevatorToPos(elevator, l4),
                new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos))); 

        autoChooser.addOption("CustomBackupL4ScoreCenterRightBranch", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.0),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
                new L4Backup(drivetrain),
                new ElevatorToPos(elevator, l4),
                new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    

        //Left/Right L4(Backup)
        autoChooser.addOption("CustomBackupL4ScoreLeftBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToLeftBranch(drivetrain, limelight, elevator),
                new L4Backup(drivetrain),
                new ElevatorToPos(elevator, l4),
                new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));    
        autoChooser.addOption("CustomBackupL4ScoreRightBranchL/R", 
            new SequentialCommandGroup(
            new LastResortAuto(drivetrain, 1, 1, 2.5),  
            new MoveToRightBranch(drivetrain, limelight, elevator),
                new L4Backup(drivetrain),
                new ElevatorToPos(elevator, l4),
                new AutonCoralOuttake(coralEffector),
            new ElevatorToPos(elevator, Elevatorc.downPos)));
    }
//------------------------------------------------------------------------------------------------
/*deep blue is what DYNAMITE 
  deep blue is what DYNAMITE 
  deep blue is tick tick tick tick tick tick tick tick 
  BOOM dynamite
  OH YEAH
  OH YEAHHHHHHHHH*/
//5 BIG BOOMS

/*SHARK IN THE TANK 3
1
2
3
SHARK IN THE TANK 
SHARK IN THE TANK
SHARK IN THE TANK
 */

/* deep blue is sus? Sabotage
   deep blue is sus? Sabotage
   deep blue is... do do do do do do do, do-do-do, Sabotage!
   (apperently actual official cheer? ¯\_( ͡°_ʖ ͡°)_/¯)
*/

       /* autoChooser.addOption("forward4sec+autoalign+l4+Left", new SequentialCommandGroup(
          new LastResortAuto(drivetrain, 1, 4,3),
          new MoveToLeftBranch(drivetrain, limelight),
          new LastResortAuto(drivetrain, -1, 1, 0.33),
          new InstantCommand(()->elevator.setGoal(ElevatorPos.L4))
        ));

        autoChooser.addOption("forward4sec+autoalign+l4", new SequentialCommandGroup(
          new LastResortAuto(drivetrain, 1, 4,4),
          new MoveToLeftBranch(drivetrain, limelight),
          new L4Backup(drivetrain),
          new InstantCommand(()->elevator.setGoal(ElevatorPos.L4))
        ));   
        
        autoChooser.addOption("forward4sec+autoalign+l2+Left", new SequentialCommandGroup(
          new LastResortAuto(drivetrain, 1, 4,3),
          new MoveToRightBranch(drivetrain, limelight),
          new LastResortAuto(drivetrain, -1, 1, 0.33),
          new InstantCommand(()->elevator.setGoal(ElevatorPos.L2))
        )); 

        autoChooser.addOption("forward4sec+autoalign+l2+Right", new SequentialCommandGroup(
          new LastResortAuto(drivetrain, 1, 4,3),
          new MoveToRightBranch(drivetrain, limelight),
          new LastResortAuto(drivetrain, -1, 1, 0.33),
          new InstantCommand(()->elevator.setGoal(ElevatorPos.L2))
        ));   
    }*/
=======
         autoChooser.addOption("DriveRaiseAutonL4", new DriveRaiseAutonl4(drivetrain, elevator, 1));
         autoChooser.addOption("ForwardLastResortAuto", new LastResortAuto(drivetrain, 1));
         autoChooser.addOption("BackwardLastResortAuto", new LastResortAuto(drivetrain, -1));
    
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
>>>>>>> AlgaeBranch

    

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
<<<<<<< HEAD
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),//.06 drift purple, .10 drift black
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
    //   SmartDashboard.putString("Camera Video Stream", "http://wpilibpi.local:1181/stream.mjpg");
    // SmartDashboard.putString("Camera Settings page", "http://wpilibpi.local");
=======
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
      SmartDashboard.putString("Camera Video Stream", "http://wpilibpi.local:1181/stream.mjpg");
    SmartDashboard.putString("Camera Settings page", "http://wpilibpi.local");
>>>>>>> AlgaeBranch
  }

  private void setBindingsManipulator() {
    // new JoystickButton(manipulatorController, OI.Manipulator.OUTAKE_BUTTON)
    //   .whileTrue(new CoralOutake(coralEffector))
<<<<<<< HEAD
    //   .whileFalse(new CoralIntake(coralEffector));P
    // new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_BUTTON)
    //   .whileTrue(new ManualCoralIntake());
    
    // axisTrigger(manipulatorController, Axis.kLeftTigger)
    // .whileTrue(new CoralOuttake(coralEffector));
    new JoystickButton(manipulatorController, Button.kRightBumper.value)
    .whileFalse(new CoralIntake(coralEffector))
    //.whileFalse(new ConditionalCommand(coralEffector.setMotorSpeed(0.1), new InstantCommand(), coralEffector.distanceSensorSeesCoralSupplier()))
    .whileTrue(new CoralIntakeManual(coralEffector));
    new JoystickButton(manipulatorController, Button.kLeftBumper.value)
    .whileTrue(new CoralIntakeBackwards(coralEffector));
    if (!babyMode){ 

    axisTrigger(manipulatorController, Axis.kRightTrigger)
    .whileTrue(new CoralFastOutake(coralEffector));
    }
    new JoystickButton(manipulatorController, Button.kB.value).onTrue(new ElevatorToPos(elevator, l3));
    new JoystickButton(manipulatorController, Button.kA.value).onTrue(new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator)));
    new JoystickButton(manipulatorController, Button.kX.value).onTrue(new ElevatorToPos(elevator, l2)); 
    new JoystickButton(manipulatorController, OI.Manipulator.Y).onTrue(new ConditionalCommand(new ElevatorToPos(elevator, l1), new ElevatorToPos(elevator, l4), babyModeSupplier));
    new POVButton(manipulatorController, 180).onTrue(new ConditionalCommand(new ElevatorToPos(elevator, l1), new ElevatorToPos(elevator, testl4), babyModeSupplier));
    new POVButton(manipulatorController, 0).whileTrue(new ConditionalCommand(new ElevatorToPos(elevator, l1), new ParallelCommandGroup(
        new ElevatorToPos(elevator, testl4 + testl4RaiseHeight),
        new CoralOuttake(coralEffector, .15)), babyModeSupplier));    
  }
  
=======
    //   .whileFalse(new CoralIntake(coralEffector));
    // new JoystickButton(manipulatorController, OI.Manipulator.INTAKE_BUTTON)
    //   .whileTrue(new ManualCoralIntake());
    axisTrigger(manipulatorController, Axis.kRightTrigger)
    .whileTrue(new CoralOuttake(coralEffector));
    //.whileFalse(new CoralIntake(coralEffector));
    axisTrigger(manipulatorController, Axis.kLeftTrigger)
    .whileTrue(new CoralOuttake(coralEffector));
    new JoystickButton(manipulatorController, Button.kRightBumper.value)
    .whileTrue(new CoralIntakeManual(coralEffector));
    new JoystickButton(manipulatorController, Button.kLeftBumper.value)
    .whileTrue(new CoralIntakeBackwards(coralEffector));
new JoystickButton(manipulatorController, OI.Manipulator.Y).onTrue(new ElevatorToPos(elevator, l4));
        new JoystickButton(manipulatorController, Button.kA.value).onTrue(new ElevatorToPos(elevator, l1));
        new JoystickButton(manipulatorController, Button.kB.value).onTrue(new ElevatorToPos(elevator, l3));
        new JoystickButton(manipulatorController, Button.kX.value).onTrue(new ElevatorToPos(elevator, l2));

    SmartDashboard.putData("Ground Intake Alage Command",new SequentialCommandGroup(
        new ArmToPosition(algaeEffector, ARM_INTAKE_ANGLE),
       new GroundIntakeAlgae(algaeEffector), 
       new ArmToPosition(algaeEffector, ARM_HOLDING_ALGAE_ANGLE)
       //the command seems to take care of arm angles by itself. please change the command.
       //FLJ: this is fine unless a dedicated command turns out to be needed.
    ));
    
    SmartDashboard.putData("Algae Outtake Command",new SequentialCommandGroup(
        new ArmToPosition(algaeEffector, ARM_OUTTAKE_ANGLE),
       new OuttakeAlgae(algaeEffector)
       //the command seems to take care of arm angles by itself. please change the command.
       //FLJ: this is fine unless a dedicated command turns out to be needed.
    ));
  }
    
>>>>>>> AlgaeBranch
  

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    // Command cmd = autoChooser.getSelected();
    // System.out.println("RUNNING AUTO: "+cmd.getName()+" |||>str: "+cmd.toString());
    // Command cmd = new LastResortAuto(drivetrain, -1, 4, 8);
    // System.out.println("running getAutounmousCommand");
    // //return cmd;
    // return autoChooser.getSelected();
    // try{
    //     // Load the path you want to follow using its name in the GUI
    //     PathPlannerPath path = PathPlannerPath.fromPathFile("Center Piece(1)");
    //     System.out.println("COOL");
    //     // Create a path following command using AutoBuilder. This will also trigger event markers.
    //     return AutoBuilder.followPath(path);
    // } catch (Exception e) {
    //     DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
    //     return Commands.none();
    // }
    //return new LastResortAuto(drivetrain, 1, 1, 4);
    
    return  new SequentialCommandGroup(
        new LastResortAuto(drivetrain, 1, 1, 4),  
        new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
            new ElevatorToPos(elevator, testl4),
            new AutonCoralOuttake(coralEffector),
            new ParallelCommandGroup(
                new AutonCoralFastOutake(coralEffector),
                new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)
            ),
        new ElevatorToPos(elevator, Elevatorc.downPos));    
        

  }

  /**
   * Flips an axis' Y coordinates upside down if the select axis is a joystick axis and applies the deadband value to the joystick axis
   * 
   * @param hid The controller/plane joystick the axis is on
   * @param axis The processed axis
   * @return The processed value.
   */
  private double getStickValue(GenericHID hid, Axis axis) {
    double deadbandVal = MathUtil.applyDeadband(hid.getRawAxis(axis.value), Constants.OI.JOY_THRESH);
    return deadbandVal * (axis == Axis.kLeftY || axis == Axis.kRightY ? -1 : 1);
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
<<<<<<< HEAD
    return new Trigger( (BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > 0.2) );
  }

  
=======
    return new Trigger(() -> Math
            .abs(getStickValue(controller, axis)) > Constants.OI.MIN_AXIS_TRIGGER_VALUE);
  }
>>>>>>> AlgaeBranch
}
