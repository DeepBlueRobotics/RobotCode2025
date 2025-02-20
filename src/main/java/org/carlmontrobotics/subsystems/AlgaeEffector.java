package org.carlmontrobotics.subsystems;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

import java.beans.Encoder;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.RobotContainer;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
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
    private final SparkFlex topMotor = new SparkFlex(Constants.AlgaeEffectorc.upperMotorID, MotorType.kBrushless);
    private final SparkFlex bottomMotor = new SparkFlex(Constants.AlgaeEffectorc.lowerMotorID, MotorType.kBrushless); 
    private final SparkFlex pincherMotor = new SparkFlex(Constants.AlgaeEffectorc.pinchMotorID, MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(Constants.AlgaeEffectorc.armMotorID, MotorType.kBrushless);

    
    private final RelativeEncoder topEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder pincherEncoder = pincherMotor.getEncoder();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final Encoder armAbsoluteEncoder = new Encoder();

    private final SparkClosedLoopController pidControllerTop = topMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerBottom = bottomMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerPincher = pincherMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerArm = armMotor.getClosedLoopController();
    
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.TopkS], Constants.kV[Constants.AlgaeEffectorc.TopkS], Constants.kA[Constants.AlgaeEffectorc.TopkS]);
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.BottomkS], Constants.kV[Constants.AlgaeEffectorc.BottomkS], Constants.kA[Constants.AlgaeEffectorc.BottomkS]);
    private final SimpleMotorFeedforward pincherFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.PincherkS], Constants.kV[Constants.AlgaeEffectorc.PincherkS], Constants.kA[Constants.AlgaeEffectorc.PincherkS]);
    //TODO: add feedforward

    DigitalInput limitSwitch = new DigitalInput(1); 
    
    public boolean limitDetects() {
        return limitSwitch.get(); 
    }
    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        SparkFlexConfig pincherMotorConfig = new SparkFlexConfig();
        SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();
        SparkFlexConfig topMotorConfig = new SparkFlexConfig();
        SparkMaxConfig  armMotorConfig = new SparkMaxConfig();
        

        topMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.TopkS],
            Constants.kI[Constants.AlgaeEffectorc.TopkS],
            Constants.kD[Constants.AlgaeEffectorc.TopkS]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        topMotor.configure(topMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        bottomMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.BottomkS],
            Constants.kI[Constants.AlgaeEffectorc.BottomkS],
            Constants.kD[Constants.AlgaeEffectorc.BottomkS]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        bottomMotor.configure(bottomMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);   
    
        pincherMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.PincherkS],
            Constants.kI[Constants.AlgaeEffectorc.PincherkS],
            Constants.kD[Constants.AlgaeEffectorc.PincherkS]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }
    //----------------------------------------------------------------------------------------

    public void setTopRPM(double toprpm) {
        pidControllerTop.setReference(toprpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        topFeedforward.calculate(toprpm);
    }

    public void setBottomRPM(double bottomrpm) {
        pidControllerBottom.setReference(bottomrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        bottomFeedforward.calculate(bottomrpm);
    }

    public void setPincherRPM(double pincherrpm) {
        pidControllerPincher.setReference(pincherrpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
        pincherFeedforward.calculate(pincherrpm);    
    }

    public void runRPM() {
        //TODO: Change RPM according to design
        setTopRPM(1000);
        setBottomRPM(2100);
        setPincherRPM(2100);
    }

    public void stopMotors() {
        setTopRPM(0);
        setBottomRPM(0);
        setPincherRPM(0);
    }

    public boolean checkIfAtTopRPM(double rpm) {
        return topEncoder.getVelocity() == rpm;
    }

    public boolean checkIfAtBottomRPM(double rpm) {
        return bottomEncoder.getVelocity() == rpm;
    }

    public void setSpeed(double speed) {
        topMotor.set(speed);
        bottomMotor.set(speed);
        pincherMotor.set(speed);
    }
}
