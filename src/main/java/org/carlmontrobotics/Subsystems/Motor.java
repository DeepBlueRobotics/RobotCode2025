// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.Subsystems;

import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Motor extends SubsystemBase {
  /** Creates a new Motor. */
  private final SparkMax motor;
  private final SparkBaseConfig config;
  public Motor() {
    motor = new SparkMax(0, MotorType.kBrushless);
    config = new SparkMaxConfig();
    config.encoder.positionConversionFactor(360);
    SmartDashboard.putNumber("Goal Pos", 0);
    SmartDashboard.putNumber("KP", 0);
    SmartDashboard.putNumber("KI", 0);
    SmartDashboard.putNumber("KD", 0);
  }

  @Override
  public void periodic() {
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(
      SmartDashboard.getNumber("KP", 0),
      SmartDashboard.getNumber("KI", 0), 
      SmartDashboard.getNumber("KD", 0));
    motor.configure(config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
    double goal = SmartDashboard.getNumber("Goal Pos", 0);
    motor.getClosedLoopController().setReference(goal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
  }
}
