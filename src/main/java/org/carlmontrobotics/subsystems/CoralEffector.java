package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;
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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import static org.carlmontrobotics.Constants.CoralEffectorc.*;

public class CoralEffector extends SubsystemBase {
    private SparkFlex effectorMotor = new SparkFlex(Constants.CoralEffectorc.effectorMotorID, MotorType.kBrushless);
    private final RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
    private final SparkClosedLoopController pidControllerEffector = effectorMotor.getClosedLoopController();
    // private final SimpleMotorFeedforward effectorFeedforward = new SimpleMotorFeedforward(kS[OUTTAKE], kV[OUTTAKE], kA[OUTTAKE]);
    private double goalOutakeRPM = effectorEncoder.getVelocity();
    DigitalInput limitSwitch = new DigitalInput(0); //TODO: change channel after wired
    private Timer effectorTOFTimer = new Timer();
    private TimeOfFlight effectorDistanceSensor = new TimeOfFlight(Constants.CoralEffectorc.effectorDistanceSensorID);
    
    private double lastValidDistance = Double.POSITIVE_INFINITY;

    public boolean coralDetects() {
        return lastValidDistance < DETECT_DISTANCE_INCHES;
    }

    public boolean limitDetects() {
        return limitSwitch.get(); 
    }

    public CoralEffector() {
        SparkFlexConfig c = new SparkFlexConfig();
        c.closedLoop.pid(
            Constants.CoralEffectorc.kP,
            Constants.CoralEffectorc.kI,
            Constants.CoralEffectorc.kD
            ).feedbackSensor(FeedbackSensor.kPrimaryEncoder);
        effectorMotor.configure(c, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters);
        //Will add distance sensor later

    }

    public void setSpeed(double speed) {
        effectorMotor.set(speed);
    }

    public void setRPM(double rpm) {
        pidControllerEffector.setReference(rpm, ControlType.kVelocity, ClosedLoopSlot.kSlot0);
    }

    public void stopEffector() {
        setRPM(0);
    }

    public void setRPM() {
        setRPM(2100);
    }

    public void updateValues() {
        if (effectorDistanceSensor.isRangeValid()) {
            if (lastValidDistance != 5.75) {
                effectorTOFTimer.reset();
            } else
                effectorTOFTimer.start();
                lastValidDistance = Units.metersToInches(effectorDistanceSensor.getRange());
        }
    }

    @Override
    public void periodic() {
        updateValues();
        limitDetects();
    }
}
