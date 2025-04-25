package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import com.playingwithfusion.TimeOfFlight;

import static org.carlmontrobotics.commands.TeleopDrive.babyMode;
import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import edu.wpi.first.wpilibj2.command.Command;

public class ElevatorToPos extends Command {
    private Elevator elevator;
    private double pos;
    private TimeOfFlight distanceSensor = new TimeOfFlight(CORAL_DISTANCE_SENSOR_PORT);
    public ElevatorToPos(Elevator elevator, double pos) {
        this.elevator = elevator;
        this.pos = pos;
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {

    }
    @Override
    public void execute() {
        if (babyMode) { if (distanceSensor.getRange() > CORAL_DISTANCE_SENSOR_DISTANCE) {
            elevator.setGoal(pos); }
            else {elevator.stopElevator();}
        }
         
        else {
            elevator.setGoal(pos);
        }
    }
    public boolean isFinished() {
        return elevator.atGoalHeight();
    }

}
