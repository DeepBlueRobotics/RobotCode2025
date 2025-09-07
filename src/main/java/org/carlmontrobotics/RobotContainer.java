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
import org.carlmontrobotics.commands.RaiseAndScore;
import org.carlmontrobotics.commands.AlgaeCommands.AlignAndDealgifyAlgae;
import org.carlmontrobotics.commands.AlgaeCommands.ArmMove;
import org.carlmontrobotics.commands.AlignCommands.GlobalLocalAlign;
// import org.carlmontrobotics.commands.AlgaeCommands.ArmToPosition;
import org.carlmontrobotics.commands.AlignCommands.GoToCoralStation;
import org.carlmontrobotics.commands.AlignCommands.MoveToAlignReef;
import org.carlmontrobotics.commands.AlignCommands.MoveToLeftBranch;
import org.carlmontrobotics.commands.AlignCommands.MoveToRightBranch;
import org.carlmontrobotics.commands.AlignCommands.PathPlannerToReef;
import org.carlmontrobotics.commands.AlignCommands.PathPlannerToSearchingPose;
import org.carlmontrobotics.commands.AlignCommands.RotateToTag;
import org.carlmontrobotics.commands.AutonCommands.DriveUntilSeeTag;
import org.carlmontrobotics.commands.AutonCommands.PushIntoStation;
import org.carlmontrobotics.commands.CoralCommands.AutonCoralFastOutake;
import org.carlmontrobotics.commands.CoralCommands.AutonCoralOuttake;
import org.carlmontrobotics.commands.CoralCommands.CoralFastOutake;
import org.carlmontrobotics.commands.CoralCommands.CoralIntake;
import org.carlmontrobotics.commands.CoralCommands.CoralIntakeBackwards;
import org.carlmontrobotics.commands.CoralCommands.CoralIntakeManual;
import org.carlmontrobotics.commands.CoralCommands.CoralOuttake;
import org.carlmontrobotics.commands.DriveCommands.TeleopDrive;
import org.carlmontrobotics.commands.ElevatorCommands.ElevatorToBottomLimitSwitch;
import org.carlmontrobotics.commands.ElevatorCommands.ElevatorToPos;
import org.carlmontrobotics.commands.MiscellaneousCommands.BatteryTesting;
import org.carlmontrobotics.commands.MiscellaneousCommands.L4Backup;
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

//controllers
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.XboxController.Axis;
import edu.wpi.first.wpilibj.XboxController.Button;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
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
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import static org.carlmontrobotics.Constants.AlgaeEffectorc.ARM_DOWN_VOLTAGE;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.ARM_UP_VOLTAGE;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.DELAGIFY_HIGH_POS;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.DELAGIFY_LOW_POS;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.LOWER_ANGLE_LIMIT;
import static org.carlmontrobotics.Constants.AlgaeEffectorc.UPPER_ANGLE_LIMIT;
import static org.carlmontrobotics.Constants.Elevatorc.elevatorOffset;
import static org.carlmontrobotics.Constants.Elevatorc.l1;
import static org.carlmontrobotics.Constants.Elevatorc.l2;
import static org.carlmontrobotics.Constants.Elevatorc.l3;
import static org.carlmontrobotics.Constants.Elevatorc.testl4;
import static org.carlmontrobotics.Constants.Elevatorc.testl4RaiseHeight;
import org.carlmontrobotics.commands.AlgaeCommands.ArmMove;
//constats
//import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import static org.carlmontrobotics.Constants.OI.Driver.*;
import static org.carlmontrobotics.Constants.OI.Manipulator.*;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;



public class RobotContainer {
    //private static boolean babyMode = true; don't think this does anything, real baby mode is in teleop drive
    
    public final GenericHID driverController = new GenericHID(Driver.port);
    public final XboxController driverRumble = new XboxController(Driver.port); //For rumbling the controller
    public final GenericHID manipulatorController = new GenericHID(Manipulator.port);

    

    private final AlgaeEffector algaeEffector = new AlgaeEffector();
    public final Elevator elevator = new Elevator();
    public final Limelight limelight = new Limelight();
    public final Drivetrain drivetrain =  new Drivetrain(elevator, limelight);
    public final CoralEffector coralEffector = new CoralEffector();

    private SendableChooser<Command> autoChooser = new SendableChooser<>();   
    public boolean alignOverride = true;
    public boolean autoScoring = true;
    private static final ShuffleboardTab compTab = Shuffleboard.getTab("Competition");

    public RobotContainer() {
        //#region compShuffleboard
        compTab.addDouble("Match Time", () -> Timer.getMatchTime())
        .withPosition(0, 0)
        .withSize(2, 1)
        .withWidget(BuiltInWidgets.kTextView);
        compTab.addBoolean("Sees Tag", () -> limelight.seesTag(REEF_LL))
        .withPosition(3, 0)
        .withSize(2,2)
        .withWidget(BuiltInWidgets.kBooleanBox);
        compTab.addCamera("PhotonVision Stream", "SigmaCamera", "http://photonvision.local:1182/stream.mjpg")
        .withPosition(3,2)
        .withSize(3,3);
        compTab.addCamera("Limelight Stream", "REEF", "10.1.99.12:5800");
        compTab.addBoolean("FC", () -> limelight.seesTagId(7,18))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("FR", () -> limelight.seesTagId(6,17))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("FL", () -> limelight.seesTagId(8,19))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("BR", () -> limelight.seesTagId(11,22))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("BC", () -> limelight.seesTagId(10,21))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("BL", () -> limelight.seesTagId(9,20))
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(1, 1);
        compTab.addBoolean("CoralIN", () -> coralEffector.limitSwitchSeesCoral())
        .withWidget(BuiltInWidgets.kBooleanBox)
        .withSize(2, 2);
        //#endregion
        //#region AutoRegistration
        RegisterAutoCommands();
        autoChooser = AutoBuilder.buildAutoChooser();
        autoChooser.setDefaultOption("ScoreL4CenterCustom", new SequentialCommandGroup(
                new WaitCommand(3),
                new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
                new ElevatorToPos(elevator, testl4),
                new AutonCoralOuttake(coralEffector),
                new ParallelCommandGroup(
                    new AutonCoralFastOutake(coralEffector),
                    new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)
                ),
            new ElevatorToPos(elevator, Elevatorc.downPos)));
        RegisterCustomAutos();

        SmartDashboard.putData("Auto Chooser", autoChooser); 

        SmartDashboard.putBoolean("AlignOverride", alignOverride);
        SmartDashboard.putBoolean("AutoScoring", autoScoring);
        SmartDashboard.putBoolean("AlignOverride", true);
        //#endregion
        setDefaultCommands();
        setBindingsDriver();
        setBindingsManipulator();

        // SmartDashboard.putData("Battery testing go vroooom", new SequentialCommandGroup(
        //      new BatteryTesting(drivetrain, elevator, limelight, l2),
        //      new BatteryTesting(drivetrain, elevator, limelight, l3),
        //      new BatteryTesting(drivetrain, elevator, limelight, l4)
        // ));
        // SmartDashboard.putData("Rotate Command",new RotateToTag(drivetrain, limelight));
    }
   
   
    //#region ButtonBindings
    private void setBindingsDriver() {
        new JoystickButton(driverController, Driver.resetFieldOrientationButton)
            .onTrue(new InstantCommand(drivetrain::resetFieldOrientation));
        axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON)
            .onTrue(new InstantCommand(()->drivetrain.setFieldOriented(false)))
            .onFalse(new InstantCommand(()->drivetrain.setFieldOriented(true)));


        new JoystickButton(driverController, 7)
            .onTrue(new GlobalLocalAlign(drivetrain, limelight, elevator, coralEffector, false));
        new JoystickButton(driverController, 8)
            .onTrue(new GlobalLocalAlign(drivetrain, limelight, elevator, coralEffector, true));

        axisTrigger(driverController, Driver.LEFT_TRIGGER_BUTTON)
            .onTrue(new InstantCommand(() -> drivetrain.setExtraSpeedMult(.5)))//normal max turn is .5
            .onFalse(new InstantCommand(() -> drivetrain.setExtraSpeedMult(0)));
        new JoystickButton(driverController, Button.kB.value).onTrue(new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble));
        
            //new MoveToAlignReef(drivetrain, limelight, elevator, true, //To align with right branch
            //    driverRumble)));
        new JoystickButton(driverController, Button.kX.value).onTrue(new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, false, //To align with left branch
                driverRumble)));

        new POVButton(driverController, 180).onTrue(new AlignAndDealgifyAlgae(drivetrain, limelight, algaeEffector, elevator, false, driverRumble));
        new POVButton(driverController, 0).onTrue(new AlignAndDealgifyAlgae(drivetrain, limelight, algaeEffector, elevator, true, driverRumble));
        // new JoystickButton(driverController, Driver.y).onTrue(new AlignAndDealgifyAlgae(drivetrain, limelight, algaeEffector, elevator, driverRumble));
        // new JoystickButton(driverController, Driver.a).whileTrue(
        //     new InstantCommand(() -> drivetrain.drive(0,0,0)));
        //     //Teleop drive thing might a bad idea 
        // conditional buttons for going to coral station or branch depending on if the robot has a coral inside or not
        // this is for the left branch or left station (left station refers to the coral station to the left of the driver)
        new JoystickButton(driverController, Driver.y)
            .onTrue(
                new SequentialCommandGroup(
                    new PathPlannerToSearchingPose(drivetrain, elevator, driverRumble, false, limelight, 
                    () -> ProcessedAxisValue(driverController, Axis.kLeftY),
                    () -> ProcessedAxisValue(driverController, Axis.kLeftX),
                    () -> ProcessedAxisValue(driverController, Axis.kRightX))
            ));
            new JoystickButton(driverController, Driver.a)
            .onTrue(
                new SequentialCommandGroup(
                    new PathPlannerToSearchingPose(drivetrain, elevator, driverRumble, true, limelight, 
                        () -> driverController.getRawAxis(0), 
                        () -> driverController.getRawAxis(1),
                        ()-> driverController.getRawAxis(5))
                    // new MoveToAlignReef(drivetrain, limelight, elevator, true, //To align with right branch
                    // driverRumble)
                )
            );
        // new JoystickButton(driverController, Driver.y)
        //     .onTrue(new ConditionalCommand(
        //         new PathPlannerToReef(drivetrain, limelight, false, 
        //         () -> driverController.getRawAxis(0), 
        //         () -> driverController.getRawAxis(1),
        //         ()-> driverController.getRawAxis(5)), 
        //         new GoToCoralStation(drivetrain, false,
        //         () -> driverController.getRawAxis(0),
        //         () -> driverController.getRawAxis(1),
        //         ()-> driverController.getRawAxis(5)), 
        //         () -> coralEffector.limitSwitchSeesCoral() || SmartDashboard.getBoolean("AlignOverride", true)));
                
        // // this is for the right branch or right station
        // new JoystickButton(driverController, Driver.a)
        //     .onTrue(new ConditionalCommand(
        //         new PathPlannerToReef(drivetrain, limelight, true, 
        //         () -> driverController.getRawAxis(0), 
        //         () -> driverController.getRawAxis(1),
        //         ()-> driverController.getRawAxis(5)), 
        //         new GoToCoralStation(drivetrain, true,
        //         () -> driverController.getRawAxis(0),
        //         () -> driverController.getRawAxis(1),
        //         ()-> driverController.getRawAxis(5)), 
        //         () -> coralEffector.limitSwitchSeesCoral() || SmartDashboard.getBoolean("AlignOverride", true)));
        
    }

    private void setBindingsManipulator() {
        new JoystickButton(manipulatorController, Button.kRightBumper.value)
            .whileFalse(new CoralIntake(coralEffector)) //1not needed cause default command
        //.whileFalse(new ConditionalCommand(coralEffector.setMotorSpeed(0.1), new InstantCommand(), coralEffector.distanceSensorSeesCoralSupplier()))
        .whileTrue(new CoralIntakeManual(coralEffector));
        new JoystickButton(manipulatorController, Button.kLeftBumper.value)
        .whileTrue(new CoralIntakeBackwards(coralEffector));
        if (!babyMode){ 
    
        axisTrigger(manipulatorController, Axis.kRightTrigger)
        .whileTrue(new CoralFastOutake(coralEffector));
        }
    
        new JoystickButton(manipulatorController, Button.kA.value).onTrue(new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator)));
        new JoystickButton(manipulatorController, Button.kX.value).onTrue(new ElevatorToPos(elevator, l2));
        new JoystickButton(manipulatorController, Button.kB.value).onTrue(new ElevatorToPos(elevator, l3));
        //l4 in one button
        new JoystickButton(manipulatorController, OI.Manipulator.Y)
        .onTrue(
            new SequentialCommandGroup(
                new ElevatorToPos(elevator, testl4), 
                new AutonCoralOuttake(coralEffector), 
                new ParallelCommandGroup(
                    new ElevatorToPos(elevator, testl4 + testl4RaiseHeight), 
                    new AutonCoralFastOutake(coralEffector)), 
                new ElevatorToPos(elevator, Elevatorc.l1),
                new ElevatorToBottomLimitSwitch(elevator),
                //for testing vvv
                new InstantCommand(() -> System.out.println("Command complete"))
                ));
        //old l4
        // axisTrigger(manipulatorController, Axis.kLeftTrigger).onTrue(new ElevatorToPos(elevator, l4)); 
        axisTrigger(manipulatorController, Axis.kLeftTrigger).onTrue(new InstantCommand(() -> coralEffector.toggleAutoIntake())); 
    
        new POVButton(manipulatorController, 180).onTrue(new ElevatorToPos(elevator, DELAGIFY_HIGH_POS));
        new POVButton(manipulatorController, 0).onTrue(new ElevatorToPos(elevator, DELAGIFY_LOW_POS));
        new POVButton(manipulatorController, 90).whileTrue(new ArmMove(algaeEffector, ARM_UP_VOLTAGE));
        new POVButton(manipulatorController, 270).whileTrue(new ArmMove(algaeEffector, ARM_DOWN_VOLTAGE));
    
        //manual new l4
        new JoystickButton(manipulatorController, XboxController.Button.kBack.value).onTrue(new ElevatorToPos(elevator, testl4));
        new JoystickButton(manipulatorController, XboxController.Button.kStart.value).whileTrue(new ParallelCommandGroup(
            new ElevatorToPos(elevator, testl4 + testl4RaiseHeight),
            new CoralOuttake(coralEffector, .15)));  
    }
    //#endregion
    //#region AutoMaking
    private void RegisterAutoCommands() {
        //CoralEffector
        //NamedCommands.registerCommand("AutoCoralIntake", new CoralIntake(coralEffector)); not needed cause defualt commadn
        NamedCommands.registerCommand("CoralOutake", new AutonCoralOuttake(coralEffector));

        //Elevator
        NamedCommands.registerCommand("ElevatorIntake", new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator)));
        NamedCommands.registerCommand("ElevatorL1", new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator)));
        NamedCommands.registerCommand("ElevatorL2", new ElevatorToPos(elevator, Elevatorc.l2));
        NamedCommands.registerCommand("ElevatorL3", new ElevatorToPos(elevator, Elevatorc.l3));

        //Scoring Coral
        
        //Right
        NamedCommands.registerCommand("ScoreL4Right", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
            new ElevatorToPos(elevator, testl4),
            new AutonCoralOuttake(coralEffector),
            new ParallelCommandGroup(
                new AutonCoralFastOutake(coralEffector),
                new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
            )
        );
        NamedCommands.registerCommand("ScoreL3Right", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
            new ElevatorToPos(elevator, Elevatorc.l3),
            new AutonCoralOuttake(coralEffector),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
        ));
        NamedCommands.registerCommand("ScoreL2Right", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
            new ElevatorToPos(elevator, Elevatorc.l3),
            new AutonCoralOuttake(coralEffector),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
        ));

        //Left
        NamedCommands.registerCommand("ScoreL4Left", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, false, driverRumble),
            new ElevatorToPos(elevator, testl4),
            new AutonCoralOuttake(coralEffector),
            new ParallelCommandGroup(
                new AutonCoralFastOutake(coralEffector),
                new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
            )
        );
        NamedCommands.registerCommand("ScoreL3Left", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, false, driverRumble),
            new ElevatorToPos(elevator, Elevatorc.l3),
            new AutonCoralOuttake(coralEffector),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
        ));
        NamedCommands.registerCommand("ScoreL2Left", new SequentialCommandGroup(
            new MoveToAlignReef(drivetrain, limelight, elevator, false, driverRumble),
            new ElevatorToPos(elevator, Elevatorc.l3),
            new AutonCoralOuttake(coralEffector),
            new SequentialCommandGroup(new ElevatorToPos(elevator, l1), new ElevatorToBottomLimitSwitch(elevator))
        ));
        //For Station
        NamedCommands.registerCommand("WaitForCoral", new PushIntoStation(drivetrain, coralEffector));

        //For ALGAE in near future
        //TODO Algae
    }

    private void RegisterCustomAutos(){
        autoChooser.addOption("1L4Right", new SequentialCommandGroup(
            new DriveUntilSeeTag(drivetrain, limelight, 9, 22, 0, -0.5),
            new MoveToAlignReef(drivetrain, limelight, elevator, false, driverRumble),
            new ElevatorToPos(elevator, testl4), 
            new AutonCoralOuttake(coralEffector), 
            new ParallelCommandGroup(
                new ElevatorToPos(elevator, testl4 + testl4RaiseHeight), 
                new AutonCoralFastOutake(coralEffector)), 
            new ElevatorToPos(elevator, Elevatorc.l1),
            new ElevatorToBottomLimitSwitch(elevator)
        ));
        autoChooser.addOption("1L4Left", new SequentialCommandGroup(
            new DriveUntilSeeTag(drivetrain, limelight, 11, 20, 0, 0.5),
            new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
            new ElevatorToPos(elevator, testl4), 
            new AutonCoralOuttake(coralEffector), 
            new ParallelCommandGroup(
                new ElevatorToPos(elevator, testl4 + testl4RaiseHeight), 
                new AutonCoralFastOutake(coralEffector)), 
            new ElevatorToPos(elevator, Elevatorc.l1),
            new ElevatorToBottomLimitSwitch(elevator)
        ));
    }
    //#endregion
    //#region DefualtCommands
  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),//.06 drift purple, .10 drift black
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton),
        elevator,
        coralEffector,
        manipulatorController
      ));

      //coralEffector.setDefaultCommand(new CoralIntake(coralEffector));
    //   SmartDashboard.putString("Camera Video Stream", "http://wpilibpi.local:1181/stream.mjpg");
    // SmartDashboard.putString("Camera Settings page", "http://wpilibpi.local");
  }
  //#endregion
  //#region getAutoCommand
  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    Command cmd = autoChooser.getSelected();
    //System.out.println("RUNNING AUTO: "+cmd.getName()+" |||>str: "+cmd.toString());
    // Command cmd = new LastResortAuto(drivetrain, -1, 4, 8);
    // System.out.println("running getAutounmousCommand");
    return cmd;
    //return autoChooser.getSelected();
    //return new LastResortAuto(drivetrain, 1, 1, 4);
    // return new SequentialCommandGroup(
    //             new WaitCommand(3),
    //             //new LastResortAuto(drivetrain, 1, 1, 4),
    //             new MoveToAlignReef(drivetrain, limelight, elevator, false, driverRumble),
    //             new ElevatorToPos(elevator, testl4),
    //             new AutonCoralOuttake(coralEffector),
    //             new ParallelCommandGroup(
    //                 new AutonCoralFastOutake(coralEffector),
    //                 new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)
    //             ),
    //         new ElevatorToPos(elevator, Elevatorc.downPos));
    
    // return  new SequentialCommandGroup(
    //     //Wait for other robots to pass by
    //     new WaitCommand(3),
    //     new MoveToAlignReef(drivetrain, limelight, elevator, true, driverRumble),
    //         new ElevatorToPos(elevator, testl4),
    //         new AutonCoralOuttake(coralEffector),
    //         new ParallelCommandGroup(
    //             new AutonCoralFastOutake(coralEffector),
    //             new ElevatorToPos(elevator, testl4 + testl4RaiseHeight)
    //         ),
    //     new ElevatorToPos(elevator, Elevatorc.downPos));    
  }
  //#endregion
  //#region HelpfulMethods
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
    return new Trigger( (BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > 0.2) );
  }
  //#endregion
}
