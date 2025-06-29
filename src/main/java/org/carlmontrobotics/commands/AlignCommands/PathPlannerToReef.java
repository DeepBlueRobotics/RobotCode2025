// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands.AlignCommands;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
import static org.carlmontrobotics.Constants.AligningCords.*;

import java.lang.reflect.Field;
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
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.FlippingUtil;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerToReef extends Command {
  private Drivetrain dt;
  private Limelight ll;
  private boolean searchingState = true;
  private final SwerveDrivePoseEstimator poseEstimator;
  private Field2d targetField;

  private boolean blueAlliance;
  private final Pose2d[] searchPoses = {ID6_17Search, ID7_18Search, ID8_19Search, ID9_20Search, ID10_21Search, ID11_22Search};
  private final List<Integer> blueIDs = List.of(17,18,19,20,21,22);
  private final List<Integer> redIDs = List.of(8,7,6,11,10,9);//I cannot do a freakin array cause it has no indexing option
  private final Pose2d[] rightPoses = {ID6_17Right, ID7_18Right, ID8_19Right, ID9_20Right, ID10_21Right, ID11_22Right};
  private final Pose2d[] leftPoses = {ID6_17Left, ID7_18Left, ID8_19Left, ID9_20Left, ID10_21Left, ID11_22Left};
  private Pose2d targetLocation;
  private int targetID;
  private boolean rightBranch;
  private DoubleSupplier xStick;
  private DoubleSupplier yStick;
  private DoubleSupplier rStick;
  private Command currentPath; 
  private PathConstraints constraints = new PathConstraints(1, 1, 2, 2); //TODO tune this for fastest possible alignment

  public PathPlannerToReef(Drivetrain drivetrain, Limelight limelight, boolean rightBranch,
    DoubleSupplier xStick, DoubleSupplier yStick, DoubleSupplier rStick //For cancellation purposes
    ) {
    addRequirements(dt=drivetrain, ll=limelight);
    this.xStick = xStick;
    this.yStick = yStick;
    this.rStick = rStick;
    this.rightBranch = rightBranch;
    targetID = -1;
    poseEstimator = dt.getPoseEstimator();
    targetField = new Field2d();
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    System.out.println(targetLocation);
    System.out.println(targetLocation == null);

  }

  @Override
  public void initialize() {
    SmartDashboard.putData("targetField", targetField);
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    SmartDashboard.putBoolean("searchisyes", searchingState);
    setIfBlueAlliance(); 
    goToClosestReef_NoLimelightNeeded();
    // if (ll.seesTag(REEF_LL)) {
    //   runPathToClosestReef();
    //   searchingState = false;
    // } 
    // else {
    //   // runToClosestSearchingPosition();
    //   searchingState = true;
    //   System.out.println("AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA");
  
    // }
  }

  @Override
  public void execute() {
    SmartDashboard.putBoolean("target location is null",targetLocation == null);
    SmartDashboard.putBoolean("searching state", searchingState);
    // if (searchingState && ll.seesTag(REEF_LL) && (int) LimelightHelpers.getFiducialID(REEF_LL) != targetID) {
    //   currentPath.cancel();
    //   runPathToClosestReef();
    //   searchingState = false;

    // }
    // else if (searchingState && ll.seesTag(REEF_LL)) {
    //   searchingState = false; // what does this do? Turns off searching state so that the other function wont turn on
    // }
    // if (!searchingState && !ll.seesTag(REEF_LL)) {
    //   currentPath.cancel();
    //   runToClosestSearchingPosition();
    //   searchingState = true;
    // }
    //I feel like the problem with this thing is that its gonna activate if the tag is flickering which is bad
    SmartDashboard.putData("targetField", targetField);
    System.out.println(targetLocation);
    System.out.println(targetLocation == null);
  }

  @Override
  public void end(boolean interrupted) {
    if (searchingState && currentPath!=null) {
      currentPath.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return currentPath == null || (
      currentPath.isFinished()) || 
      (Math.abs(xStick.getAsDouble()) > 0.1 ) || 
      (Math.abs(yStick.getAsDouble()) > 0.1 ) || 
      (Math.abs(rStick.getAsDouble()) > 0.1 );
  }

  private boolean setIfBlueAlliance(){
    if (DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
      blueAlliance = true;
      SmartDashboard.putBoolean("BlueAlliance", true);
    }
    else if (DriverStation.getAlliance().get() == DriverStation.Alliance.Red) {
      blueAlliance = false;
      SmartDashboard.putBoolean("BlueAlliance", false);
    }
    return blueAlliance;
  }

  private void runPathToClosestReef() {
      targetID = (int) LimelightHelpers.getFiducialID(REEF_LL);
      SmartDashboard.putNumber("TargetId", targetID);
      if (rightBranch) {
        if (blueIDs.contains(targetID)) {
          targetLocation = rightPoses[blueIDs.indexOf(targetID)];
        }
        else if ( redIDs.contains(targetID)) {
          targetLocation = rightPoses[redIDs.indexOf(targetID)];
        }
      }
      else {
        if (blueIDs.contains(targetID)) {
          targetLocation = leftPoses[blueIDs.indexOf(targetID)];
        }
        else if (redIDs.contains(targetID)) {
          targetLocation = leftPoses[redIDs.indexOf(targetID)];
        }
      }
      if (targetLocation != null) {
      targetField.setRobotPose(targetLocation);
      //double poopy = targetLocation.getY();
      if (redIDs.contains(targetID)) {
        targetLocation = FlippingUtil.flipFieldPose(targetLocation);
      }
    
      //targetLocation = new Pose2d(0,0);
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        List.of(poseEstimator.getEstimatedPosition(), targetLocation)),
        constraints, 
        null, 
        new GoalEndState(0, targetLocation.getRotation()));
        path.preventFlipping = true;
      //SmartDashboard.putBoolean("mehappy", blueAlliance);
  
      // FollowPathCommand command = new FollowPathCommand(
      //   path, 
      //   () -> poseEstimator.getEstimatedPosition(), 
      //   () -> dt.getSpeeds(),  
      //   (speeds, feedforwards) -> dt.drive(dt.getSwerveStates(speeds)),
      //   new PPHolonomicDriveController(
      //     new PIDConstants(5.0, 0.0, 0.0),
      //     new PIDConstants(5.0, 0.0, 0.0)), 
      //   Constants.Drivetrainc.Autoc.robotConfig, 
      //   () -> false
      //   ); // Not sure if dt should be added here as a requirement (extra parameter) since it is already a requirement in the pathplanneralign command
      currentPath = AutoBuilder.followPath(path); //works like this with already built autobuilder
      currentPath.schedule();
  }

  
  // private void runToClosestSearchingPosition() {
  //   targetLocation = findClosestPose(poseEstimator.getEstimatedPosition(), searchPoses);
  //   if (rightBranch) {
  //       if (blueIDs.contains(targetID)) {
  //         finalLocation = rightPoses[blueIDs.indexOf(targetID)];
  //       }
  //       else if (redIDs.contains(targetID)) {
  //         finalLocation = rightPoses[redIDs.indexOf(targetID)];
  //       }
  //     }
  //   else {
  //     if (blueIDs.contains(targetID)) {
  //       finalLocation = leftPoses[blueIDs.indexOf(targetID)];
  //     }
  //     else if (redIDs.contains(targetID)) {
  //       finalLocation = leftPoses[redIDs.indexOf(targetID)];
  //     }
  //   }
  //   PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(List.of(poseEstimator.getEstimatedPosition(), targetLocation, finalLocation)), 
  //   constraints, 
  //   null, 
  //   new GoalEndState(0, finalLocation.getRotation()));
  //   currentPath = AutoBuilder.followPath(path); //works like this with already built autobuilder
  //   currentPath.schedule();
  // }

  // public Pose2d findClosestPose(Pose2d target, Pose2d[] poses) {
  //   int closestIndex = -1;
  //   double minDistance = Double.MAX_VALUE;

  //   for (int i = 0; i < poses.length; i++) {
  //     Pose2d pose = poses[i];
  //     double distance = pose.getTranslation().getDistance(target.getTranslation());
  
  //     if (distance < minDistance) {
  //         minDistance = distance;
  //         closestIndex = i;
  //     }
  //   }
  //   if (closestIndex != -1) {
  //     if (blueAlliance) {
  //       targetID = blueIDs.get(closestIndex);
  //     }
  //     else if (!blueAlliance) {
  //       targetID = redIDs.get(closestIndex);
  //     }
  //     return poses[closestIndex];
  //   } 
  //   else {
  //     return null;
  //   }
    
  }

  private void goToClosestReef_NoLimelightNeeded(){

    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    Pose2d[] branchPoses = rightBranch ? rightPoses : leftPoses;

    double closestDistance = Double.MAX_VALUE;
    Pose2d closestBranch = null;

    for (Pose2d branchPose : branchPoses) {

      if (setIfBlueAlliance() == false){
        branchPose = FlippingUtil.flipFieldPose(branchPose); //flips the pose if the alliance is red
      }

      //get the distance from the current pose to the branch pose
      double distance = currentPose.getTranslation().getDistance(branchPose.getTranslation());
      //if the distance is less than any of the other distances checked it will set the closest branch to that pose
      if (distance < closestDistance) {
        closestDistance = distance;
        closestBranch = branchPose;
      }
    }

    targetLocation = closestBranch;
    targetField.setRobotPose(targetLocation);

    PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
      List.of(poseEstimator.getEstimatedPosition(), targetLocation)),
      constraints, 
      null, 
      new GoalEndState(0, targetLocation.getRotation()));
    path.preventFlipping = true;
    
    currentPath = AutoBuilder.followPath(path); 
    currentPath.schedule();

  }

}
