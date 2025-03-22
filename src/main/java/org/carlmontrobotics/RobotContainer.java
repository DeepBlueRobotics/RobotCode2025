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

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
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

import static org.carlmontrobotics.Constants.Elevatorc.l1;
import static org.carlmontrobotics.Constants.Elevatorc.l2;
import static org.carlmontrobotics.Constants.Elevatorc.l3;
import static org.carlmontrobotics.Constants.Elevatorc.l4;
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
    public final Drivetrain drivetrain =  new Drivetrain();
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

    // private boolean hasSetupAutos = false;
    // private final String[] autoNames = new String[] {};
    // private final AlgaeEffector algaeEffector = new AlgaeEffector();
     private final Elevator elevator = new Elevator();
     private SendableChooser<Command> autoChooser = new SendableChooser<>();
     
     
       //1. using GenericHID allows us to use different kinds of controllers
       //2. Use absolute paths from constants to reduce confusion
       
       public final CoralEffector coralEffector = new CoralEffector();
      //TODO activate arm
       //public final AlgaeEffector algaeEffector = new AlgaeEffector();
     
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
        autoChooser.setDefaultOption("null forward auto", new LastResortAuto(drivetrain, -1, 4, 8));
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
       axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON)
                .onTrue(new InstantCommand(()->drivetrain.setFieldOriented(false)))
                .onFalse(new InstantCommand(()->drivetrain.setFieldOriented(true)));


        new JoystickButton(driverController, Driver.x)
          .whileTrue(new MoveToLeftBranch(drivetrain, limelight));

        new JoystickButton(driverController, Driver.b)
          .whileTrue(new MoveToRightBranch(drivetrain, limelight));
        axisTrigger(driverController, Driver.LEFT_TRIGGER_BUTTON)
          .onTrue(new InstantCommand(() -> drivetrain.setExtraSpeedMult(.5)))//normal max turn is .5
          .onFalse(new InstantCommand(() -> drivetrain.setExtraSpeedMult(0)));
        new POVButton(driverController, 180)
          .whileTrue(new ParallelCommandGroup(
            new InstantCommand(() -> drivetrain.stop()),
            new TeleopDrive(drivetrain, ()->0, ()->0, ()->0, ()->true)));
        // axisTrigger(driverController, Driver.RIGHT_TRIGGER_BUTTON)
        //   .onTrue(new InstantCommand(() -> drivetrain.stop()))


      /*new JoystickButton(driverController, Driver.a)
        .onTrue(new L4Backup(drivetrain));
    }*/
    }

   
        
    

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
      NamedCommands.registerCommand("CoralOutake", new CoralOuttake(coralEffector));
      
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
        NamedCommands.registerCommand("IntakeCoralNoLL",
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
                        new ElevatorToPos(elevator, Elevatorc.downPos))));                             

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
        autoChooser.addOption("DriveRaiseAutonL2Center", new SequentialCommandGroup(
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
            new ElevatorToPos(elevator, l4), new CoralFastOutake(coralEffector)));

        autoChooser.addOption("ForwardLastResortAutoCenter", new LastResortAuto(drivetrain, 1, 4, 2.5));
        autoChooser.addOption("BackwardLastResortAutoCenter", new LastResortAuto(drivetrain, -1, 4, 2.5));
        
        autoChooser.addOption("ForwardLastResortAutoLeft/Right", new LastResortAuto(drivetrain, 1, 4, 3));
        autoChooser.addOption("BackwardLastResortAutoLeft/Right", new LastResortAuto(drivetrain, -1, 4, 3));        
//------------------------------------------------------------------------------------------------
/*deep blue is what DYNAMITE 
  deep blue is what DYNAMITE 
  deep blue is tick tick tick tick tick tick 
  BOOM dynamite
  OH YEAH
  OH YEAHHHHHHHHH       
*/ 
        autoChooser.addOption("forward4sec+autoalign+l4+Left", new SequentialCommandGroup(
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
    }

    

  private void setDefaultCommands() {
    drivetrain.setDefaultCommand(new TeleopDrive(
      drivetrain,
      () -> ProcessedAxisValue(driverController, Axis.kLeftY),//.06 drift purple, .10 drift black
      () -> ProcessedAxisValue(driverController, Axis.kLeftX),
      () -> ProcessedAxisValue(driverController, Axis.kRightX),
      () -> driverController.getRawButton(OI.Driver.slowDriveButton)));
      SmartDashboard.putString("Camera Video Stream", "http://wpilibpi.local:1181/stream.mjpg");
    SmartDashboard.putString("Camera Settings page", "http://wpilibpi.local");
  }

  private void setBindingsManipulator() {
    // new JoystickButton(manipulatorController, OI.Manipulator.OUTAKE_BUTTON)
    //   .whileTrue(new CoralOutake(coralEffector))
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

        
       
  }
    
  

  public Command getAutonomousCommand() {
    // return Commands.print("No autonomous command configured");
    // Command cmd = autoChooser.getSelected();
    // System.out.println("RUNNING AUTO: "+cmd.getName()+" |||>str: "+cmd.toString());
    Command cmd = new LastResortAuto(drivetrain, -1, 4, 8);
    System.out.println("running getAutounmousCommand");
    return cmd;
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

  private Trigger axisTrigger(GenericHID controller, Axis axis) {
    return new Trigger( (BooleanSupplier)(() -> Math.abs(getStickValue(controller, axis)) > 0.2) );
  }

  
}
