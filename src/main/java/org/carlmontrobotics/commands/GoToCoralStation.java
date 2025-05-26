// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.CORAL_LL;
import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
import static edu.wpi.first.units.Units.Rotation;
import static org.carlmontrobotics.Constants.AligningCords.*;
import java.util.List;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import org.carlmontrobotics.Constants;
import org.carlmontrobotics.subsystems.Drivetrain;

import org.carlmontrobotics.subsystems.Limelight;
import org.carlmontrobotics.subsystems.LimelightHelpers;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

import edu.wpi.first.wpilibj2.command.Command;

public class GoToCoralStation extends Command {
  private Drivetrain dt;

  
  private final SwerveDrivePoseEstimator poseEstimator = dt.getPoseEstimator();
  


  

  private Pose2d targetLocation;
  private boolean rightStation; //right station refers to the coral station on the right of the driver from the blue alliance perspective
  private DoubleSupplier xStick; //x axis on left joystick
  private DoubleSupplier yStick; //y axis on left joystick
  private DoubleSupplier rStick; //x axis on right joystick
  
  private Command currentPath; 
  private PathConstraints constraints = new PathConstraints(1, 1, 1, 1); //TODO tune this for fastest possible alignment

  public GoToCoralStation(Drivetrain drivetrain , boolean rightStation,
    DoubleSupplier xStick, DoubleSupplier yStick, DoubleSupplier rStick //For cancellation purposes
    ) {
    addRequirements(dt=drivetrain);
    this.xStick = xStick;
    this.yStick = yStick;
    this.rStick = rStick;
    this.rightStation = rightStation;
    
  }

  @Override
  public void initialize() {
    //calculate the target position
    calculateStationPose();
    //go to the target position
    runPathToStation();
  }

  @Override
  public void execute() {
    
  }

  @Override
  public void end(boolean interrupted) {
    currentPath.cancel();
  }

  @Override
  public boolean isFinished() {
    return (currentPath.isFinished()) 
    || (Math.abs(xStick.getAsDouble()) > 0.1 ) 
    || (Math.abs(yStick.getAsDouble()) > 0.1 )
    || (Math.abs(rStick.getAsDouble()) > 0.1); //adjust the joystick requirements if nessesary
  }

  private void runPathToStation(){
      
    PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
    List.of(poseEstimator.getEstimatedPosition(), targetLocation)), 
    constraints, 
    null, 
    new GoalEndState(0, targetLocation.getRotation()));

    currentPath = AutoBuilder.followPath(path); 
    currentPath.schedule();
      

  }


  private void calculateStationPose(){
    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    
    //gets the current x and y coords of the robot
    double y1 = currentPose.getY(); 
    double x1 = currentPose.getX();
    //these are for the slope and intercept values of the line of positions where the robot can be to intake coral
    double m;
    double b;
    //x and y coordinates of the target position
    double x2;
    double y2;

    double m2; //slope of the line perpendicular to the line of positions that intersects the robot's current position
    //There are two coral stations per alliance so this will calculate the target position based on which station the driver chooses
    if (rightStation) {
    
      m = -0.751606;
      b = 1.93563;
      //these are the slope and intercept values of the line perpendicular to the line of positions that intersects the robot's current position
      m2 = 1.33048; //slope of line perpendicular to m
      double b2 = y1 - m2* x1; //calculates y intercept of the perpendicular line by using the modified equation b = y - mx
      
      x2 = (b2 - b) / (m - m2); //calculates x coordinate of intersection point which is found by using a system of equations and solving for x
      y2 = m * x2 + b; //subsitutes x2 into the equation for the perpendicular line to find y

      //the domain of the line where the robot can intake coral is 0.702 <= x <= 1.636 and 1.408 <= y <= 7.06
      //This will clamp the values to be in the domain so it forces the robot to go to a valid target position
      x2 = MathUtil.clamp(x2, 0.702, 1.636);
      y2 = MathUtil.clamp(y2, 1.408, 7.06); 
      targetLocation = new Pose2d(x2, y2, Rotation2d.fromDegrees(54)); //sets the target position
     
    }
    else {
      m = 0.783726;
      b = 6.08182;
      m2 = -1.27595613773;

      double b2 = y1 - m2* x1; //calculates y intercept of the perpendicular line by using the modified equation b = y - mx
      x2 = (b2 - b) / (m - m2); //calculates x coordinate of intersection point which is found by using a system of equations and solving for x
      y2 = m * x2 + b; //subsitutes x2 into the equation for the perpendicular line to find y

      
      x2 = MathUtil.clamp(x2, 0.702, 1.636);
      y2 = MathUtil.clamp(y2, 6.632, 7.364); 
    }
    targetLocation = new Pose2d(x2, y2, Rotation2d.fromDegrees(-54)); //sets the target position
    
    

    

  }
  

  

}
