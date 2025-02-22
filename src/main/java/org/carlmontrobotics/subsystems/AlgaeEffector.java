package org.carlmontrobotics.subsystems;


import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;


import static org.carlmontrobotics.RobotContainer.*;

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
import com.revrobotics.spark.SparkMaxAlternateEncoder;

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
import edu.wpi.first.wpilibj.Encoder;


public class AlgaeEffector extends SubsystemBase {
    private final SparkFlex topMotor = new SparkFlex(Constants.AlgaeEffectorc.upperMotorID, MotorType.kBrushless);
    private final SparkFlex bottomMotor = new SparkFlex(Constants.AlgaeEffectorc.lowerMotorID, MotorType.kBrushless); 
    private final SparkFlex pincherMotor = new SparkFlex(Constants.AlgaeEffectorc.pinchMotorID, MotorType.kBrushless);
    private final SparkMax armMotor = new SparkMax(Constants.AlgaeEffectorc.armMotorID, MotorType.kBrushless);

    private SparkFlexConfig pincherMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig bottomMotorConfig = new SparkFlexConfig();
    private SparkFlexConfig topMotorConfig = new SparkFlexConfig();
    private SparkMaxConfig  armMotorConfig = new SparkMaxConfig();
    
    private final RelativeEncoder topEncoder = topMotor.getEncoder();
    private final RelativeEncoder bottomEncoder = bottomMotor.getEncoder();
    private final RelativeEncoder pincherEncoder = pincherMotor.getEncoder();
    private final RelativeEncoder armEncoder = armMotor.getEncoder();
    private final RelativeEncoder armAbsoluteEncoder = armMotor.getAlternateEncoder();

    private final SparkClosedLoopController pidControllerTop = topMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerBottom = bottomMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerPincher = pincherMotor.getClosedLoopController();
    private final SparkClosedLoopController pidControllerArm = armMotor.getClosedLoopController();
    
    private final SimpleMotorFeedforward topFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.topArrayOrder], Constants.kV[Constants.AlgaeEffectorc.topArrayOrder], Constants.kA[Constants.AlgaeEffectorc.topArrayOrder]);
    private final SimpleMotorFeedforward bottomFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.bottomArrayOrder], Constants.kV[Constants.AlgaeEffectorc.bottomArrayOrder], Constants.kA[Constants.AlgaeEffectorc.bottomArrayOrder]);
    private final SimpleMotorFeedforward pincherFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.pincherArrayOrder], Constants.kV[Constants.AlgaeEffectorc.pincherArrayOrder], Constants.kA[Constants.AlgaeEffectorc.pincherArrayOrder]);
    private final SimpleMotorFeedforward armFeedforward = new SimpleMotorFeedforward(Constants.kS[Constants.AlgaeEffectorc.armArrayOrder], Constants.kV[Constants.AlgaeEffectorc.armArrayOrder], Constants.kA[Constants.AlgaeEffectorc.armArrayOrder]);
    //feedforward for arm was added

    private double armGoalAngle = 0;

    //--------------------------------------------------------------------------------------------
    public AlgaeEffector() {
        configureMotors();

    }
    //----------------------------------------------------------------------------------------

    private void configureMotors () {
        topMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.topArrayOrder],
            Constants.kI[Constants.AlgaeEffectorc.topArrayOrder],
            Constants.kD[Constants.AlgaeEffectorc.topArrayOrder]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        topMotor.configure(topMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        
        bottomMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.bottomArrayOrder],
            Constants.kI[Constants.AlgaeEffectorc.bottomArrayOrder],
            Constants.kD[Constants.AlgaeEffectorc.bottomArrayOrder]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        bottomMotor.configure(bottomMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);   
    
        pincherMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.pincherArrayOrder],
            Constants.kI[Constants.AlgaeEffectorc.pincherArrayOrder],
            Constants.kD[Constants.AlgaeEffectorc.pincherArrayOrder]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);

        armMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.armArrayOrder],
            Constants.kI[Constants.AlgaeEffectorc.armArrayOrder],
            Constants.kD[Constants.AlgaeEffectorc.armArrayOrder]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        pincherMotor.configure(pincherMotorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        armMotorConfig.idleMode(IdleMode.kBrake);
        armMotorConfig.closedLoop.pid(
            Constants.kP[Constants.AlgaeEffectorc.armArrayOrder],
            Constants.kI[Constants.AlgaeEffectorc.armArrayOrder],
            Constants.kD[Constants.AlgaeEffectorc.armArrayOrder]
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
    }

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

    public void setArmAngle(double armangle) {
        armGoalAngle = armangle;
    }

    public double getArmPos() {

        return MathUtil.inputModulus(armAbsoluteEncoder.getPosition(),
                Constants.AlgaeEffectorc.ARM_DISCONT_RAD, Constants.AlgaeEffectorc.ARM_DISCONT_RAD + 2 * Math.PI);
        
                
    }

    public double getArmVel() {
        return armAbsoluteEncoder.getVelocity();
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

    public void setMotorSpeed(double topSpeed, double bottomSpeed, double pincherSpeed) {
        topMotor.set(topSpeed);
        bottomMotor.set(bottomSpeed);
        pincherMotor.set(pincherSpeed);
    }


    @Override
    public void periodic() {
        pidControllerArm.setReference(armGoalAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        pincherFeedforward.calculate(goalAngle); //What is this for
    }

}
