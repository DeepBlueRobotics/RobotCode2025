package org.carlmontrobotics.subsystems;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.lib199.MotorConfig;
import org.carlmontrobotics.lib199.MotorControllerFactory;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkFlex;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkFlex;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;



public class CoralEffector extends SubsystemBase {
    private SparkFlex effectorMotor = new SparkFlex(Constants.CoralEffectorc.effectorMotorID, MotorType.kBrushless);
    private final RelativeEncoder effectorEncoder = effectorMotor.getEncoder();
    private final SparkPIDController pidControllerEffector = effectorMotor.getPIDController();
    private final SimpleMotorFeedforward effectorFeedforward = new SimpleMotorFeedforward(kS[OUTTAKE], kV[OUTTAKE], kA[OUTTAKE]);
    private double goalOutakeRPM = effectorEncoder.getVelocity();

    private Timer effectorTOFTimer = new Timer();
    private TimeOfFlight effectorDistanceSensor = new TimeOfFlight(Constants.CoralEffectorc.effectorDistanceSensorID);

    private double lastValidDistanceIntake = Double.POSITIVE_INFINITY;
    private double lastValidDistanceOuttake = Double.POSITIVE_INFINITY;

    public CoralEffector() {
        pidControllerEffector.setP(Constants.CoralEffectorc.kP);
        pidControllerEffector.setI(Constants.CoralEffectorc.kI);
        pidControllerEffector.setD(Constants.CoralEffectorc.kD);
        //Will add distance sensor later

    }

    public void motorSetEffector(double speed) {
        effectorMotor.set(speed);
    }
    
    public void shootIntake() {
        motorSetEffector(0.5);
    }   

    public void setRPMEffector(double rpm) {
        pidControllerEffector.setReference(rpm, CANSparkBase.ControlType.kVelocity, 0,
            effectorFeedforward.calculate(rpm / 60.0));
    }

    public void stopEffector() {
        setRPMEffector(0);
    }

    public void intakeShootEffector() {
        setRPMEffector(2100);
    }
}
