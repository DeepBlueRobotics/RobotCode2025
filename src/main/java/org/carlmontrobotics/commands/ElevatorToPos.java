package org.carlmontrobotics.commands;

import org.carlmontrobotics.Constants.Elevatorc.ElevatorPos;
import org.carlmontrobotics.subsystems.Elevator;

import com.playingwithfusion.TimeOfFlight;

import static org.carlmontrobotics.commands.TeleopDrive.babyMode;
import static org.carlmontrobotics.Constants.CoralEffectorc.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ElevatorToPos extends Command {
    private Elevator elevator;
    private double pos;
    private boolean enableL4;
    private TimeOfFlight distanceSensor = new TimeOfFlight(CORAL_DISTANCE_SENSOR_PORT);
    public ElevatorToPos(Elevator elevator, double pos) {
        this.elevator = elevator;
        this.pos = pos;
        addRequirements(this.elevator = elevator);
    }
    @Override
    public void initialize() {
        SmartDashboard.putBoolean("Enable L4", false);
    }
    @Override
    public void execute() {
    enableL4 = SmartDashboard.getBoolean("Enable L4", false);
        if (!babyMode || enableL4 || pos < 1){
            if (babyMode) { if (distanceSensor.getRange() > CORAL_DISTANCE_SENSOR_DISTANCE) {
                elevator.setGoal(pos); }
                else {elevator.stopElevator();}
            }
            
            else {
                elevator.setGoal(pos);
            }
    }
    }
    public boolean isFinished() {
        return elevator.atGoalHeight();
    }

}
