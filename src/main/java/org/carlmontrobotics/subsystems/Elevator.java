// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;


import static edu.wpi.first.units.Units.InchesPerSecond;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;
import java.util.function.Consumer;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;

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
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.units.measure.MutLinearVelocity;
import edu.wpi.first.units.measure.MutVoltage;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
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
  private DigitalInput topLimitSwitch;
  private DigitalInput bottomLimitSwitch;
  //Vars
  private double heightGoal;
  private int elevatorState;
  //PID
  private PIDController pidElevatorController;
  private ElevatorFeedforward feedforwardElevatorController;
  private Timer timer;
  
  private final SysIdRoutine sysIdRoutine;

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
    Volts.of(1).per(Units.Seconds),//ramp rate, volts/sec
    Volts.of(1), //starting voltage, volts
    Units.Seconds.of(5)//AH: maximum sysID test time
  );

  public Elevator() {
    //motors
    masterMotor = new SparkMax(Constants.Elevatorc.masterPort, MotorType.kBrushless);
    masterConfig = new SparkMaxConfig();
    masterEncoder = masterMotor.getEncoder();
    followerMotor = new SparkMax(Constants.Elevatorc.followerPort, MotorType.kBrushless);
    followerConfig = new SparkMaxConfig();
    followerEncoder = followerMotor.getEncoder();
    configureMotors();

    //Calibration
    topLimitSwitch = new DigitalInput(Constants.Elevatorc.elevatorTopLimitSwitchPort);
    bottomLimitSwitch = new DigitalInput(Constants.Elevatorc.elevatorBottomLimitSwitchPort);
    timer = new Timer();
    timer.start();


    //PID
    pidElevatorController = new PIDController(Constants.Elevatorc.kP, Constants.Elevatorc.kI, Constants.Elevatorc.kD);
    //FeedForward
    feedforwardElevatorController = new ElevatorFeedforward(Constants.Elevatorc.kS, Constants.Elevatorc.kG, Constants.Elevatorc.kV, Constants.Elevatorc.kA);
    

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
          
          log.motor("Elevator-Mastr")//AH: you have 2 motors, must log both
            .voltage(
              m_appliedVoltage[1]
                .mut_replace(followerMotor.getBusVoltage() * followerMotor.getAppliedOutput(), Volts))//AH: GET() IS NOT VOLTAGE
                .linearPosition(m_distance[1].mut_replace(followerEncoder.getPosition(), Meters))
                .linearVelocity(m_velocity[1].mut_replace(followerEncoder.getVelocity(), MetersPerSecond));
        }, 
        this)
      );
  }

  private void configureMotors () {
    //Master Config
    masterConfig
        .inverted(Constants.Elevatorc.masterInverted)
        .idleMode(Constants.Elevatorc.masterIdleMode);
    masterConfig.encoder
        .positionConversionFactor(Constants.Elevatorc.masterPositionConversionFactor)
        .velocityConversionFactor(Constants.Elevatorc.masterVelocityConversionFactor);
    masterConfig.closedLoop
        .pid(Constants.Elevatorc.kP, Constants.Elevatorc.kI,Constants.Elevatorc.kD)
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    //I don't know if this is needed
    //Follower Config
    followerConfig
      .inverted(Constants.Elevatorc.followerInverted)
      .idleMode(Constants.Elevatorc.followerIdleMode);
    followerConfig.encoder
      .positionConversionFactor(Constants.Elevatorc.followerPositionConversionFactor)
      .velocityConversionFactor(Constants.Elevatorc.followerVelocityConversionFactor);
    followerConfig.follow(Constants.Elevatorc.masterPort, Constants.Elevatorc.followerInverted);
    followerMotor.configure(followerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  public void setGoal(double goal) {
    heightGoal = goal;
  }

  public void setGoal(ElevatorPos goal) {
    heightGoal = goal.getPosition();
  }
  
  public double getGoal() {
    return heightGoal;
  }
  
  public double getCurrentHeight() {
    return masterEncoder.getPosition();
  }

  public boolean elevatorAtMax() {
    return !topLimitSwitch.get();
  }

  public boolean elevatorAtMin() {
    return !bottomLimitSwitch.get();
  }

  public void updateEncoders() {
    if (elevatorAtMax()) {
      masterEncoder.setPosition(Constants.Elevatorc.maxElevatorHeightInches);
      timer.reset();
      timer.start();
    }
    else if (elevatorAtMin()) {
      masterEncoder.setPosition(Constants.Elevatorc.minElevatorHeightInches);
      timer.reset();
      timer.start();
    }
  }

  public void getToGoal() {
    masterMotor.setVoltage(
      pidElevatorController.calculate(masterEncoder.getPosition(), heightGoal) + 
      feedforwardElevatorController.calculate(heightGoal));
  }

  public void setSpeed(double speed){
    masterMotor.set(speed);
  }

  public void stopElevator(){
    masterMotor.set(0);
  }

  public double getEncoderValue() {
    return masterEncoder.getPosition();
  }

  public boolean atGoalHeight() {
    if (heightGoal == Constants.Elevatorc.maxElevatorHeightInches) {
      return elevatorAtMax();
    }
    else if (heightGoal == Constants.Elevatorc.minElevatorHeightInches) {
      return elevatorAtMin();
    }
    
    else {
      return (Math.abs(getEncoderValue()) - heightGoal <= Constants.Elevatorc.elevatorTolerance);
    }

  }


  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutine.dynamic(direction);
  } 
  


  @Override
  public void periodic() {
    if (elevatorAtMax()){
      SmartDashboard.putString("ElevatorState", "🔴STOP🔴");
    }
    else if (elevatorAtMin()) {
      SmartDashboard.putString("ElevatorState", "🟢GO🟢");
    }
    else {
      SmartDashboard.putString("ElevatorState", "🟡CAUTION🟡");
    }
    SmartDashboard.putNumber("Elevator Height", getCurrentHeight());
    SmartDashboard.putNumber("Since Calibrated", timer.get());
    updateEncoders();
    getToGoal();
  }
}
