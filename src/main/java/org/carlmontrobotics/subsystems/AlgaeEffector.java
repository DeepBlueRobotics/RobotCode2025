package org.carlmontrobotics.subsystems;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.SimDeviceSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class AlgaeEffector extends SubsystemBase {
    private SparkMax armMotor = new SparkMax(14, MotorType.kBrushless);
    private RelativeEncoder armEncoder = armMotor.getEncoder();
    private RelativeEncoder absoluteEncoder = armMotor.getAlternateEncoder();
    private SparkClosedLoopController pidArm;
    public AlgaeEffector() {
        SparkMaxConfig armConfig = new SparkMaxConfig();
        armConfig.encoder.positionConversionFactor(360/3);
        armConfig.closedLoop.pid(0, 0, 0);
        armConfig.closedLoop.feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder);
        armMotor.configure(armConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        pidArm = armMotor.getClosedLoopController();


    }
    public void RunPID(double angle) {
        pidArm.setReference(angle, ControlType.kPosition);
    }
    @Override
    public void periodic() {
        SmartDashboard.putNumber("RelativeEncoderPos", armEncoder.getPosition());
        SmartDashboard.putNumber("AbsoluteEncoderPos", absoluteEncoder.getPosition());
    }
}
