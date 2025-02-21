
package org.carlmontrobotics.subsystems;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.servohub.ServoHub.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.units.measure.Velocity;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static org.carlmontrobotics.Constants.*;

import org.carlmontrobotics.Constants.CoralEffectorConstants;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;

public class CoralEffector extends SubsystemBase {
  
    public SparkFlex coralMotor = new SparkFlex(CoralEffectorConstants.coralMotorPort, MotorType.kBrushless);
    public DigitalInput coralLimitSwitch = new DigitalInput(CoralEffectorConstants.coralLimitSwitchPort);
    public TimeOfFlight distanceSensor = new TimeOfFlight(CoralEffectorConstants.coralDistanceSensorPort);
    
    public static boolean distanceSensorSees;
    public static boolean limitSwitchSees;
    public final RelativeEncoder coralEncoder = coralMotor.getEncoder();
    // private double CoralGoalRPM = 100;
    private double coralOutput = coralMotor.getAppliedOutput();
    private boolean coralIn;
  
    SparkFlexConfig config = new SparkFlexConfig();
    public CoralEffector(){
    config
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(CoralEffectorConstants.kp, CoralEffectorConstants.ki, CoralEffectorConstants.kd);
        
    coralMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
   }

  public double getEncoderPos() {
    return coralEncoder.getPosition();
  }
  public void setMotorSpeed(double speed) {
    coralMotor.set(speed);
  }
  public void setReferencePosition(double reference) {
    coralMotor.getClosedLoopController().setReference(reference, SparkBase.ControlType.kPosition);
  }

  public boolean coralIsIn() {
    return coralIn;
  }
  public void setCoralIn(boolean coralIsInside) {
    coralIn = coralIsInside;
  }

  @Override
  public void periodic() {
    //coralMotor.getClosedLoopController().setReference(1, ControlType.kVelocity);
    distanceSensorSees = distanceSensor.getRange() < CoralEffectorConstants.coralDistanceSensorDistance;
    limitSwitchSees = !coralLimitSwitch.get();
    // CoralGoalRPM = coralEncoder.getVelocity();
    coralOutput = coralMotor.getAppliedOutput();

    SmartDashboard.putBoolean("Distance sensor", distanceSensorSees);
    SmartDashboard.putNumber("distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("limit switch", limitSwitchSees);
    // SmartDashboard.putNumber("Coral goal RPM", CoralGoalRPM);
    SmartDashboard.putNumber("Coral Speed", coralEncoder.getVelocity());
    SmartDashboard.putNumber("coral output", coralOutput);
    SmartDashboard.getNumber("P", CoralEffectorConstants.kp);
  }
}