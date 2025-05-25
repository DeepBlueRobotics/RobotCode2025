// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package org.carlmontrobotics.commands;

import static org.carlmontrobotics.Constants.Limelightc.REEF_LL;
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

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj2.command.Command;

public class PathPlannerAlign extends Command {
  private Drivetrain dt;
  private Limelight ll;
  private boolean searchingState = true;
  private final SwerveDrivePoseEstimator poseEstimator = dt.getPoseEstimator();
  


  private final Pose2d[] searchPoses = {ID6_17Search, ID7_18Search, ID8_19Search, ID9_20Search, ID10_21Search, ID11_22Search};

  private Pose2d targetLocation;
  private boolean rightBranch;
  private DoubleSupplier xStick;
  private DoubleSupplier yStick;
  private DoubleSupplier rStick;
  private Command currentPath; 
  private PathConstraints constraints = new PathConstraints(1, 1, 1, 1); //TODO tune this for fastest possible alignment

  public PathPlannerAlign(Drivetrain drivetrain, Limelight limelight, boolean rightBranch,
    DoubleSupplier xStick, DoubleSupplier yStick, DoubleSupplier rStick //For cancellation purposes
    ) {
    addRequirements(dt=drivetrain, ll=limelight);
    this.xStick = xStick;
    this.yStick = yStick;
    this.rStick = rStick;
    this.rightBranch = rightBranch;
  }

  @Override
  public void initialize() {
    if (ll.seesTag(REEF_LL)) {
      searchingState = false;
    } 
    else {
      searchingState = true;
      runToClosestSearchingPosition();
    }
  }

  @Override
  public void execute() {
    if (searchingState && ll.seesTag(REEF_LL)) {
      currentPath.cancel();
      runPathToClosestReef();
      searchingState = false;

    }
    if (!searchingState && !ll.seesTag(REEF_LL)) {
      searchingState = true;
      currentPath.cancel();
      runToClosestSearchingPosition();
    }
  }

  @Override
  public void end(boolean interrupted) {
    if (searchingState) {
      currentPath.cancel();
    }
  }

  @Override
  public boolean isFinished() {
    return (!searchingState && currentPath.isFinished()) || (Math.abs(xStick.getAsDouble()) > 0.1 ) || (Math.abs(yStick.getAsDouble()) > 0.1 ) || (Math.abs(rStick.getAsDouble()) > 0.1 );
  }

  private void runPathToClosestReef() {
      if (rightBranch) {
        switch ((int) LimelightHelpers.getFiducialID(REEF_LL)) { //this is beautiful
          case 6:
            targetLocation = ID6_17Right;
            break;
          case 7:
            targetLocation = ID7_18Right;
            break;
          case 8:
            targetLocation = ID8_19Right;
            break;
          case 9:
            targetLocation = ID9_20Right;
            break;
          case 10:
            targetLocation = ID10_21Right;
            break;
          case 11:
            targetLocation = ID11_22Right;
            break;
          case 17:
            targetLocation = ID6_17Right;
            break;
          case 18:
            targetLocation = ID7_18Right;
            break;
          case 19:
            targetLocation = ID8_19Right;
            break;
          case 20:
            targetLocation = ID9_20Right;
            break;
          case 21:
            targetLocation = ID10_21Right;
            break;
          case 22:
            targetLocation = ID11_22Right;
            break;
          default:
            break;
        }
      }
      else {
        switch ((int) LimelightHelpers.getFiducialID(REEF_LL)) { //this is beautiful
          case 6:
            targetLocation = ID6_17Left;
            break;
          case 7:
            targetLocation = ID7_18Left;
            break;
          case 8:
            targetLocation = ID8_19Left;
            break;
          case 9:
            targetLocation = ID9_20Left;
            break;
          case 10:
            targetLocation = ID10_21Left;
            break;
          case 11:
            targetLocation = ID11_22Left;
            break;
          case 17:
            targetLocation = ID6_17Left;
            break;
          case 18:
            targetLocation = ID7_18Left;
            break;
          case 19:
            targetLocation = ID8_19Left;
            break;
          case 20:
            targetLocation = ID9_20Left;
            break;
          case 21:
            targetLocation = ID10_21Left;
            break;
          case 22:
            targetLocation = ID11_22Left;
            break;
          default:
            break;
      }
    }
      PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(
        List.of(poseEstimator.getEstimatedPosition(), targetLocation)), 
        constraints, 
        null, 
        new GoalEndState(0, targetLocation.getRotation()));
  
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
  private void runToClosestSearchingPosition() {
    targetLocation = findClosestPose(poseEstimator.getEstimatedPosition(), searchPoses);
    PathPlannerPath path = new PathPlannerPath(PathPlannerPath.waypointsFromPoses(List.of(poseEstimator.getEstimatedPosition(), targetLocation)), 
    constraints, 
    null, 
    new GoalEndState(0, targetLocation.getRotation()));
    currentPath = AutoBuilder.followPath(path); //works like this with already built autobuilder
    currentPath.schedule();
  }

  public static Pose2d findClosestPose(Pose2d target, Pose2d[] poses) {
    Pose2d closest = null;
    double minDistance = Double.MAX_VALUE;

    for (Pose2d pose : poses) {
        double distance = pose.getTranslation().getDistance(target.getTranslation());
        if (distance < minDistance) {
            minDistance = distance;
            closest = pose;
        }
    }

    return closest;
  }

}
