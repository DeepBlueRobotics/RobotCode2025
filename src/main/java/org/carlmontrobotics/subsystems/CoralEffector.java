package org.carlmontrobotics.subsystems;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

//import static org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static org.carlmontrobotics.Constants.*;

import static org.carlmontrobotics.Constants.CoralEffectorc.*;

import java.util.function.BooleanSupplier;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase;

public class CoralEffector extends SubsystemBase {
    //public SparkFlex coralMotor =  MotorControllerFactory.createSparkFlex(CORAL_MOTOR_PORT);
    public SparkFlex coralMotor = new SparkFlex(CORAL_MOTOR_PORT, MotorType.kBrushless);
    public DigitalInput coralLimitSwitch = new DigitalInput(CORAL_LIMIT_SWITCH_PORT);
    public TimeOfFlight distanceSensor = new TimeOfFlight(CORAL_DISTANCE_SENSOR_PORT);
    public static boolean enableAutoIntake = true;
    
    
    public final RelativeEncoder coralEncoder = coralMotor.getEncoder();
    
    private double coralOutput;
    public boolean coralIn;
    private double targetPos=0;
  
    SparkFlexConfig config = new SparkFlexConfig();
    public CoralEffector(){
    config
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .voltageCompensation(12)
        .smartCurrentLimit(50);
    config.closedLoop
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .pid(KP, KI, KD);
        
    coralMotor.configure(config, SparkBase.ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);
    SmartDashboard.putBoolean("autoIntake", enableAutoIntake);
    enableAutoIntake = SmartDashboard.getBoolean("autoIntake", enableAutoIntake);
   }

  /**
   * Gets encoder position of the motor
   * @return encoder position
   */
  public double getEncoderPos() {
    return coralEncoder.getPosition();
  }

  /**
   * Sets motor speed using voltage percentage
   * @param speed voltage from 0.0 - 1.0
   */
  public void setMotorSpeed(double speed) {
    coralMotor.set(speed);
  }

  /**
   * Sets reference to where the motor will stay at
   * @param reference the position of the motor the motor will stay at
   */
  public void setReferencePosition(double reference) {
    targetPos = reference;
    coralMotor.getClosedLoopController().setReference(reference, SparkBase.ControlType.kPosition);
  }

  /**
   * Checks if the motor has reached its goal
   * @return if the coral motor is at the targetpos 
   */
  public boolean motorAtGoal(){
    return Math.abs(coralEncoder.getPosition()-targetPos) <= CORAL_INTAKE_ERR;
  }
  /** 
   * Checks if sees coral, generally meaning that don't lift elevator as the coral is to deep inside the elevator
   * @return if coral is seen by distance sensor
   */
  public boolean distanceSensorSeesCoral(){
    return distanceSensor.getRange() < CORAL_DISTANCE_SENSOR_DISTANCE;
  }

  /**
   * Makes a lambda function to check if distanceSensor sees coral
   * @return booleanSupplier
   */
  public BooleanSupplier distanceSensorSeesCoralSupplier(){
    return () -> distanceSensorSeesCoral();
  }

  // public boolean coralIsIn() {
  //   return coralIn;
  // }
  // public void setCoralIn(boolean coralIsInside) {
  //   coralIn = coralIsInside;
  // }

  /**
   * Checks if limitswitch is activated
   * @return boolean
   */
  public boolean limitSwitchSeesCoral() {
    return coralLimitSwitch.get();
  }

  /**
   * 
   * @return If coral is fully inside the robot
   */
  public boolean coralSecured() {
    return limitSwitchSeesCoral() && !distanceSensorSeesCoral();
  }

  public void toggleAutoIntake(){
    enableAutoIntake = !enableAutoIntake;
  }


  @Override
  public void periodic() {
    //coralMotor.getClosedLoopController().setReference(1, ControlType.kVelocity);
    // limitSwitchSees = !coralLimitSwitch.get();
    // CoralGoalRPM = coralEncoder.getVelocity();
    coralOutput = coralMotor.getAppliedOutput();

    SmartDashboard.putBoolean("Distance sensor", distanceSensorSeesCoral());
    SmartDashboard.putNumber("distance", distanceSensor.getRange());
    SmartDashboard.putBoolean("coral in", coralIn);
    SmartDashboard.putBoolean("limit switch", limitSwitchSeesCoral());
    // SmartDashboard.putNumber("Coral goal RPM", CoralGoalRPM);
    SmartDashboard.putNumber("Coral Speed", coralEncoder.getVelocity());
    SmartDashboard.putNumber("coral output", coralOutput);
    SmartDashboard.getNumber("P", KP);
    enableAutoIntake = SmartDashboard.getBoolean("autoIntake", enableAutoIntake);
    if (distanceSensor.getRange() == 0){
      DriverStation.reportWarning("The distance sensor no worky (it at 0)", false);
    }
  }
}