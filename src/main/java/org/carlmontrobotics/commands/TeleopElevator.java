package org.carlmontrobotics.commands;

import java.util.function.DoubleSupplier;

import static org.carlmontrobotics.Constants.Elevatorc.*;
import org.carlmontrobotics.subsystems.Elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class TeleopElevator extends Command {
    private Elevator elevator;
    private DoubleSupplier joystickSupplier;
    private TrapezoidProfile.State goalState;
    private double lastTime;
    public TeleopElevator(Elevator elevator, DoubleSupplier joystickSupplier) {
        this.joystickSupplier = joystickSupplier;
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {
        goalState = new TrapezoidProfile.State(elevator.getCurrentHeight(), elevator.getEleVel());
        lastTime = Timer.getFPGATimestamp();
    }
    public double getReqSpeeds() {
        return MAX_ACCEL_RAD_P_S*joystickSupplier.getAsDouble(); //(MAX_ACCEL in radians/s^2 times joystick)
    }
    @Override
    public void execute() {
    double speeds = getReqSpeeds();
    //SmartDashboard.putNumber("speeds", speeds);

    if (speeds == 0) {// if no input, don't set any goals.
      lastTime = Timer.getFPGATimestamp();// update deltaT even when not running
      return;
    }

    double currTime = Timer.getFPGATimestamp();
    double deltaT = currTime - lastTime;// only move by a tick of distance at once
    lastTime = currTime;

    double goalEleRad = goalState.position + speeds * deltaT;// speed*time = dist

    goalEleRad = MathUtil.clamp(goalEleRad, minElevatorHeightInches, maxElevatorHeightInches);
    // goalArmRad = MathUtil.clamp(goalArmRad,
    //     armSubsystem.getArmPos() + Math.pow(armSubsystem.getMaxVelRad(), 2) / MAX_FF_ACCEL_RAD_P_S,
    //     armSubsystem.getArmPos() - Math.pow(armSubsystem.getMaxVelRad(), 2) / MAX_FF_ACCEL_RAD_P_S);

    goalState.position = goalEleRad;
    goalState.velocity = 0;
    // don't put in constants bc it's always zero
    elevator.setGoal(goalState.position);
    }
    @Override
    public void end(boolean interrupted) {

    }
    @Override
    public boolean isFinished() {
        return false;
    }
}
