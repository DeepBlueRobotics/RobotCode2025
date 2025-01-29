// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

public class Elevator extends SubsystemBase {
  /** Creates a new Elevator. */
  private SparkMax masterMotor = new SparkMax(Constants.Elevatorc.masterPort, MotorType.kBrushless);
  private SparkMaxConfig masterConfig = new SparkMaxConfig();
  private SparkMax followerMotor = new SparkMax(Constants.Elevatorc.followerPort, MotorType.kBrushless);
  private SparkMaxConfig followerConfig = new SparkMaxConfig();
  
  public Elevator() {}
    //Not working :(
    // followerMotor.follow(masterMotor, followerInverted);
    // masterConfig
    //     .inverted(masterInverted)
    //     .IdleMode(masterIdleMode);
    // masterConfig.encoder
    //     .positionConversionFactor(masterPositionConversionFactor)
    //     .velocityConversionFactor(masterVelocityConversionFactor);
    // masterConfig.closedLoop
    //     .pid(Constants.Elevatorc.kP,Constants.Elevatorc.kI,Constants.Elevatorc.kD)
    //     .feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    // masterMotor.configure(masterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
