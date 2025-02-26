// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
  private SparkClosedLoopController pidElevatorController;
  private Timer timer;
  
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
    pidElevatorController = masterMotor.getClosedLoopController();
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
    pidElevatorController.setReference(heightGoal, ControlType.kPosition);
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
