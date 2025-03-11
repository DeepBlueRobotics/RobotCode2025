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

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
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
  private SparkMaxConfig masterConfig;
  private RelativeEncoder masterEncoder;
  //Follower
  private SparkMax followerMotor;
  private SparkMaxConfig followerConfig;
  private RelativeEncoder followerEncoder;
  // Limit Switches
  // private DigitalInput topLimitSwitch; no upper limit switch
  private DigitalInput bottomLimitSwitch;
  private double maxVelocityMetersPerSecond = 10;
  //Vars
  private double heightGoal;
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
  private final MutVoltage[] m_appliedVoltage = new MutVoltage[2];//AH: its a holder, not a number.
  //Volts.mutable(0);
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutDistance[] m_distance = new MutDistance[2];//AH: 2 for 2 elevator motors
  //Meters.mutable(0);
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutLinearVelocity[] m_velocity = new MutLinearVelocity[2];//AH: ITS A HOLDER :o
  //MetersPerSecond.mutable(0);
  
  //AH: need a config to run a test
  private SysIdRoutine.Config defaultSysIdConfig = new SysIdRoutine.Config(
    Volts.of(1).per(Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Seconds.of(5)//AH: maximum sysID test time
  );

  private ShuffleboardTab sysIdTab = Shuffleboard.getTab("Elevator SysID");

  private void sysIdSetup() {
    sysIdTab.add("Quasistatic backward", sysIdQuasistatic(SysIdRoutine.Direction.kReverse)).withSize(2, 1);
    sysIdTab.add("Quasistatic forward", sysIdQuasistatic(SysIdRoutine.Direction.kForward)).withSize(2, 1);
    sysIdTab.add("Dynamic forward", sysIdDynamic(SysIdRoutine.Direction.kForward)).withSize(2, 1);
    sysIdTab.add("Dynamic backward", sysIdDynamic(SysIdRoutine.Direction.kReverse)).withSize(2, 1);
  }

  public Elevator() {
    encoderTimer = new Timer();
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
    bottomLimitSwitch = new DigitalInput(elevatorBottomLimitSwitchPort);
    // timer = new Timer();
    // timer.start();


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
              m_appliedVoltage[0]
                .mut_replace(masterMotor.getBusVoltage() * masterMotor.getAppliedOutput(), Volts))//AH: GET() IS NOT VOLTAGE
                .linearPosition(m_distance[0].mut_replace(masterEncoder.getPosition(), Meters))
                .linearVelocity(m_velocity[0].mut_replace(masterEncoder.getVelocity(), MetersPerSecond));//AH: use metric units always
          
          log.motor("Elevator-Follwr")//AH: you have 2 motors, must log both
            .voltage(
              m_appliedVoltage[1]
                .mut_replace(followerMotor.getBusVoltage() * followerMotor.getAppliedOutput(), Volts))//AH: GET() IS NOT VOLTAGE
                .linearPosition(m_distance[1].mut_replace(followerEncoder.getPosition(), Meters))
                .linearVelocity(m_velocity[1].mut_replace(followerEncoder.getVelocity(), MetersPerSecond));
        }, 
        this)
      );
    
  if (CONFIG.isSysIdTesting()) {
    sysIdSetup();
  }

  }

  private void configureMotors () {
    //Master Config
    masterConfig
        .inverted(masterInverted)
        .idleMode(masterIdleMode);
    masterConfig.encoder
        .positionConversionFactor(masterPositionConversionFactor)
        .velocityConversionFactor(masterVelocityConversionFactor);
    masterConfig.closedLoop
        .pid(kP, kI, kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    masterMotor.configure(masterConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    //I don't know if this is needed. Response: Not rly. Only the follow.
    //Follower Config
    followerConfig.apply(masterConfig);
    followerConfig.follow(masterPort, followerInverted);
    followerMotor.configure(followerConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoal(double goal) {
    heightGoal = goal;
  }

  public void setGoal(ElevatorPos goal) {
    heightGoal = goal.getPositioninMeters();
  }
  
  public double getGoal() {
    return heightGoal;
  }
  
  public double getCurrentHeight() {
    return masterEncoder.getPosition();//conversion done in config/constants
  }

  // public boolean elevatorAtMax() {
  //   return !topLimitSwitch.get();
  // }

  public boolean elevatorAtMin() {
    return !bottomLimitSwitch.get();//limit switches are opposite
  }

  // public void updateEncoders() {//what the fuck??
  //   // if (elevatorAtMax()) {
  //   //   masterEncoder.setPosition(maxElevatorHeightInches);
  //   //   timer.reset();
  //   //   timer.start();
  //   // }
  //   if (elevatorAtMin()) {
  //     masterEncoder.setPosition(minElevatorHeightInches);
  //     timer.reset();
  //     timer.start();
  //   }
  // }

  public void goToGoal() {
    if(heightGoal<masterEncoder.getPosition()) {
    masterMotor.setVoltage(
      pidElevatorController.calculate(masterEncoder.getPosition(), heightGoal) + 
      feedforwardElevatorController.calculate(heightGoal));
    }
  }

  public void setSpeed(double speed){
    masterMotor.set(speed);
  }

  public void stopElevator(){
    masterMotor.set(0);
  }

  // public double getPos() {
  //   return masterEncoder.getPosition();
  // }

  public boolean atGoalHeight() {
    // if (heightGoal == maxElevatorHeightInches) {
    //   return elevatorAtMax();
    // }
    if (heightGoal == minElevatorHeightInches) {
      return elevatorAtMin();
    }
    
    else {
      return (Math.abs(getCurrentHeight() - heightGoal) <= elevatorTolerance);
    }

  }
  //private boolean isEncoderDisconnected() {
  //   double currentElevPos = getPos();
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
  public boolean isUNSAFE(){
    if (Units.inchesToMeters(maxElevatorHeightInches) >= masterEncoder.getPosition() 
    || maxVelocityMetersPerSecond <= masterEncoder.getVelocity() 
    || Units.inchesToMeters(minElevatorHeightInches) <=masterEncoder.getPosition()){
      return false;
    }
    return true;
  }

  public boolean isSAFE() {
    return !isUNSAFE();
  }

  
  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    // BooleanSupplier bruh = Elevator::safetyMethod();
    return sysIdRoutine.quasistatic(direction).onlyWhile((BooleanSupplier)()->isSAFE());
    //use onlyWhile to decorate the command and therefore add safety limits (for height and voltage)
    //TO-DO: fix safety method (add velocity) and also other bugs
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction).onlyWhile((BooleanSupplier)()->isSAFE());
  } 
  public double getEleVel() {
    return masterEncoder.getVelocity();
  }


  @Override
  public void periodic() {
    // if (elevatorAtMax()){
    //   SmartDashboard.putString("ElevatorState", "🔴STOP🔴");
    // }
    if (elevatorAtMin()) {
      SmartDashboard.putString("ElevatorState", "🟢GO🟢");
    }
    else {
      SmartDashboard.putString("ElevatorState", "🟡AT MIN🟡");
    }//add one for max height
    //add one for if unsafe
    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
   // SmartDashboard.putNumber("Since Calibrated", timer.get());
    // updateEncoders();
    goToGoal();

    if(isUNSAFE() && masterMotor.getBusVoltage() > 0) {
      masterMotor.set(0); 
      System.err.println("Bad Bad nightmare bad. Elevator unsafe");
      //hey tell them they're unsafe and a bad happened
    }
  //   if (isEncoderDisconnected()) {
  //     masterMotor.set(0);
      
  // }
  }
}
