// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.math.util.Units;

import java.lang.reflect.Method;
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;

import static org.carlmontrobotics.Config.CONFIG;
import static org.carlmontrobotics.Constants.Drivetrainc.velocityTolerance;
import static org.carlmontrobotics.Constants.Elevatorc.*;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.pathplanner.lib.path.GoalEndState;
import com.playingwithfusion.CANVenom.BrakeCoastMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.estimator.KalmanFilterLatencyCompensator;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //Master
  private SparkMax masterMotor;
  private SparkMaxConfig masterConfig = new SparkMaxConfig();
  private RelativeEncoder masterEncoder;
  //Follower
  private SparkMax followerMotor;
  private SparkMaxConfig followerConfig = new SparkMaxConfig();
  private RelativeEncoder followerEncoder;
  // Limit Switches
  // private DigitalInput topLimitSwitch; no upper limit switch
  private AnalogInput bottomLimitSwitch = new AnalogInput(elevatorBottomLimitSwitchPort);
  private double maxVelocityMetersPerSecond = 5;            
  //Vars
  private double heightGoal=0;
  private int elevatorState;
  //PID
  private PIDController pidElevatorController;
  private ElevatorFeedforward feedforwardElevatorController;
  // private Timer timer;
  private Timer encoderTimer; 
  private final SysIdRoutine sysIdRoutine;
  private double lastMeasuredTime;
  private double currTime;
  private double lastElevPos;
  private double lastElevVel;

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutVoltage m_appliedVoltage = Volts.mutable(0);//AH: its a holder, not a number.
  //Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance m_distance = Meters.mutable(0);//AH: 2 for 2 elevator motors
  //Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity m_velocity = MetersPerSecond.mutable(0);//AH: ITS A HOLDER :o
  // private double goalHeight;
  //MetersPerSecond.mutable(0);
  
  //AH: need a config to run a test
  private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    Volts.of(1).per(Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Seconds.of(5)//AH: maximum sysID test time
  );

  //private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Elevator SysID");

  // private void sysIdSetup() {
  //   sysIdTab.add("Quasistatic backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse)).withSize(2, 1);
  //   sysIdTab.add("Quasistatic forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward)).withSize(2, 1);
  //   sysIdTab.add("Dynamic forward", sysIdDynamic(SysIdRoutine.Direction.kForward)).withSize(2, 1);
  //   sysIdTab.add("Dynamic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse)).withSize(2, 1);
  // }

  public Elevator() {
    
    encoderTimer = new Timer();
    // SmartDashboard.putNumber("Goal", goalHeight);
    //motors
    // masterMotor = new SparkMax(masterPort, MotorType.kBrushless);
    masterMotor = MotorControllerFactory.createSparkMax(masterPort, MotorConfig.NEO);
    masterEncoder = masterMotor.getEncoder();

    // followerMotor = new SparkMax(Constants.Elevatorc.followerPort, MotorType.kBrushless);
    followerMotor = MotorControllerFactory.createSparkMax(followerPort, MotorConfig.NEO);
    followerEncoder = followerMotor.getEncoder();

    configureMotors();
    //Calibration
    // topLimitSwitch = new DigitalInput(elevatorTopLimitSwitchPort);
   //bottomLimitSwitch = new DigitalInput(elevatorBottomLimitSwitchPort);
    // timer = new Timer();
    // timer.start();
    //SmartDashboard.putBoolean("Is Unsafe", false);

    //PID
    pidElevatorController = new PIDController(kP, kI, kD);
    //FeedForward
    feedforwardElevatorController = new ElevatorFeedforward(kS, kG, kV, kA);
    

    sysIdRoutine = new SysIdRoutine(
      defaultSysIdConfig,//AH: use custom voltage config
      new SysIdRoutine.Mechanism(
        voltage -> {
          masterMotor.setVoltage(voltage);
        },
        log -> {
          log.motor("Elevator-Mastr")//AH: you have 2 motors, must log both
            .voltage(
              m_appliedVoltage
                .mut_replace(masterMotor.getBusVoltage() * masterMotor.getAppliedOutput(), Volts))//AH: GET() IS NOT VOLTAGE
                .linearPosition(m_distance.mut_replace(masterEncoder.getPosition(), Meters))
                .linearVelocity(m_velocity.mut_replace(masterEncoder.getVelocity(), MetersPerSecond));//AH: use metric units always
        },
        this)
      );
    
  if (CONFIG.isSysIdTesting()) {
    //sysIdSetup();
  }

  }

  /**
   * Configure motors for elevator configuration. 
   * Use stall limit of 25 amps to avoid chain skipping.
   */
  private void configureMotors () {
    //Master Config
    masterConfig
        .inverted(masterInverted)
        .smartCurrentLimit(25)
        .idleMode(masterIdleMode);
    masterConfig.encoder
        .positionConversionFactor(masterPositionConversionFactor)
        .velocityConversionFactor(masterVelocityConversionFactor);
    masterConfig.closedLoop
        .pid(kP, kI, kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    masterConfig.openLoopRampRate(.375);
    masterMotor.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    //Follower Config
    followerConfig.apply(masterConfig);
    followerConfig.follow(masterPort, followerInverted);
    followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  /**
   * Sets a goal in meters
   * @param goal Goal in meters
   */
  public void setGoal(double goal) {
    heightGoal = goal;
  }

  /**
   * Sets a goal in meters using ElevatorPos enums
   * @param goal ElevatorPos enum
   */
  public void setGoal(ElevatorPos goal) {
    heightGoal = goal.getPositioninMeters();
  }
  
  /**
   * Gets the height goal of the elevator by using the local variable heightGoal
   * @return The current goal of where the elevator wants to be at
   */
  public double getGoal() {
    return heightGoal;
  }
  
  /**
   * Calculates the current height of the elevator
   * @return the height of the elevator in meters from its starting point
   */
  public double getCurrentHeight() {
    return masterEncoder.getPosition();//conversion done in config/constants
  }

 
  //Magnetic encoder not usable
  // public boolean elevatorAtMin() {
  //   return !bottomLimitSwitch.get();//limit switches are opposite
  // }


/**
 * Calculates the needed voltage to power the motors to get to goal.
 * Uses feedforward and feedback controllers to do so.
 */
  public void goToGoal() {
    //System.out.println("GOing to GOAL");
    //System.out.println(heightGoal);
    double vel = pidElevatorController.calculate(masterEncoder.getPosition(), heightGoal);
    double feed = feedforwardElevatorController.calculate(0);
    //followerMotor.setVoltage(vel+feed);
    masterMotor.setVoltage(vel + feed);

  }

  /**
   * Manually sets speed to the elevator motors
   * @param speed voltage percentage between 0.0 to 1.0
   */
  public void setSpeed(double speed){
    masterMotor.set(speed);
  }

  /**
   * Stops the elevator motors, stays on break mode
   */
  public void stopElevator(){
    masterMotor.set(0);
  }

  /**
   * Checks if elevator reached the right height
   * @return if at goal height with 2 cm of tolerance
   */
  public boolean atGoalHeight() {
    return (Math.abs(getCurrentHeight() - heightGoal) <= elevatorTolerance);

  }


  public void setElevatorIdleMode(boolean brake) {
    IdleMode mode;
    if (brake) {
      mode = IdleMode.kBrake;
    }
    else {
      mode = IdleMode.kCoast;
    }
    masterConfig.idleMode(mode);
    followerConfig.idleMode(mode);
    masterMotor.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }
  // private boolean isEncoderDisconnected() {
  //   double currentElevPos = getCurrentHeight();
  //   double currentRelativeElevVel = masterEncoder.getVelocity();
    
  //   if ((currentRelativeElevVel != 0)) {
  //           lastMeasuredTime = currTime;
  //           lastElevPos = currentElevPos;
  //           lastElevVel = currentRelativeElevVel;
  //           return false;
  //       } else {
  //         if(lastMeasuredTime > currTime+1 && lastElevPos == currentElevPos) {
  //           return true;
  //         }
  //       }
  //       // currTime - lastMeasuredTime <
  //                                    // DISCONNECTED_ENCODER_TIMEOUT_SEC;

        
  // }

//safetyMethod is used to check during sysid if the elevator height and voltage are at the safe threshold

  /**
   * Zeros out the position of the motor
   */
  public void zeroPosition() {
    masterEncoder.setPosition(0);
  }

  public boolean getBottomLimitSwitch(){
    return bottomLimitSwitch.getValue() < bottomLimitSwitchTriggerPoint;
  }
  public void setMasterEncoder(double pos) {
    masterEncoder.setPosition(pos);
  }
  /**
   * Safety method make sure the elevator did not go crazy
   * @return if the elevator is safe
   */
  public boolean isSafe() {
    if((getCurrentHeight()>1.33 && masterEncoder.getVelocity() > 0) || (getCurrentHeight() < 0 && masterEncoder.getVelocity() < 0)) {
      return false;
      
    }
    return true;
  }
  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
  //   // BooleanSupplier bruh = Elevator::safetyMethod();
  //   return sysIdRoutine.quasistatic(direction).onlyWhile((BooleanSupplier)()->isSafe());
  //   //use onlyWhile to decorate the command and therefore add safety limits (for height and voltage)
  //   //TO-DO: fix safety method (add velocity) and also other bugs
  // }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
  //   return sysIdRoutine.dynamic(direction).onlyWhile((BooleanSupplier)()->isSafe());
  // } 
  /**
   * Gets the velocity of the elevator
   * @return velocity of the elevator in meters per second
   */
  public double getEleVel() {
    return masterEncoder.getVelocity();
  }


  @Override
  public void periodic() {
    SmartDashboard.putNumber("bottom switch", bottomLimitSwitch.getValue());
    // if (elevatorAtMax()){
    //   SmartDashboard.putString("ElevatorState", "🔴STOP🔴");
    // }
    // //masterMotor.set(0);
    // goalHeight = SmartDashboard.getNumber("Goal", 0);
    // System.out.println(goalHeight);
    //setGoal(.75);
    //SmartDashboard.putBoolean("SAFE?", isSafe());

    // if (elevatorAtMin()) {
    //   SmartDashboard.putString("ElevatorState", "🟢GO🟢");
    // }
    // else {
    //   SmartDashboard.putString("ElevatorState", "🟡AT MIN🟡");
    // }//add one for max height
    //add one for if unsafe
    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
    SmartDashboard.putNumber("Elevator Height goal", heightGoal);
    // System.out.println("eheight: "+getCurrentHeight()+" goal: "+goalHeight);
   // SmartDashboard.putNumber("Since Calibrated", timer.get());
    // updateEncoders();
   goToGoal();
   //masterEncoder.setPosition(0);
    //masterMotor.set(0.1);
    if(!isSafe() && masterMotor.getBusVoltage() > 0) {
      masterMotor.set(0); 
     // System.err.println("Bad Bad nightmare bad. Elevator unsafe");
    }
    if (getBottomLimitSwitch()) {
      zeroPosition();
    }
  
  }
}
