// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  //Master
  private SparkMax masterMotor = new SparkMax(Constants.Elevatorc.masterPort, MotorType.kBrushless);
  private SparkMaxConfig masterConfig = new SparkMaxConfig();
  private RelativeEncoder masterEncoder = masterMotor.getEncoder();
  //Follower
  private SparkMax followerMotor = new SparkMax(Constants.Elevatorc.followerPort, MotorType.kBrushless);
  private SparkMaxConfig followerConfig = new SparkMaxConfig();
  private RelativeEncoder followerEncoder = followerMotor.getEncoder();

  //Absolute Encoder
  private AbsoluteEncoder primaryEncoder = masterMotor.getAbsoluteEncoder();

  //Vars
  private double heightGoal;
  //PID
  private final SparkClosedLoopController pidElevatorController = masterMotor.getClosedLoopController();
  
  public Elevator() {
    configureMotors();
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
  
  public double getGoal() {
    return heightGoal;
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

  @Override
  public void periodic() {
    getToGoal();
  }
}
