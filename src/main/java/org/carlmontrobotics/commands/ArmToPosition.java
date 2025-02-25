package org.carlmontrobotics.commands;

import org.carlmontrobotics.subsystems.AlgaeEffector;

import edu.wpi.first.wpilibj2.command.Command;


public class ArmToPosition extends Command{
    AlgaeEffector algaeArm;
    private double goalPosition; //radians - need angle for dealgfication and ground pickup
    private boolean isShooting;
    private boolean isIntake;

    public ArmToPosition(AlgaeEffector algaeArm, double goalPosition){
        addRequirements(this.algaeArm = algaeArm);
        this.goalPosition = goalPosition;
        this.isShooting = shoot;
        this.isIntake = intake;



    }

    @Override
    public void initialize(){
        algaeArm.setArmTarget(goalPosition); //sets target to position in rads
        

    }
    public void execute(){

    }
    public void end(){

    }
    public boolean isFinished(){
        return algaeArm.armAtGoal();

    }
}
