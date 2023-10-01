package com.summitrobotics.common.commands;

import java.util.function.Consumer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.summitrobotics.common.swerve.Swerve;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * Command to have the swerve drivetrain follow a PathPlannerLib trajectory.
 * This is based off of example code from https://github.com/mjansen4857/pathplanner/wiki/PathPlannerLib:-Java-Usage.
 */
public class FollowPathPlannerTrajectory extends SequentialCommandGroup {
    public FollowPathPlannerTrajectory(Swerve drivetrain, PathPlannerTrajectory traj, boolean resetPose, boolean allianceMirror, double[] drivePID, double[] rotationPID) {
        addCommands(
            // Pose should be reset to the start point if this is the first trajectory to run
            new InstantCommand(() -> {
              if (resetPose) {
                if (allianceMirror) {
                    drivetrain.setPose(PathPlannerTrajectory.transformTrajectoryForAlliance(traj, DriverStation.getAlliance()).getInitialPose());
                } else {
                    drivetrain.setPose(traj.getInitialPose());
                }
              }
            }),
            new PPSwerveControllerCommand(
                traj,
                drivetrain::getPose,
                drivetrain.getConstellation().kinematics,
                new PIDController(drivePID[0], drivePID[1], drivePID[2]), // X controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new PIDController(drivePID[0], drivePID[1], drivePID[2]), // Y controller (usually the same values as X controller)
                new PIDController(rotationPID[0], rotationPID[1], rotationPID[2]), // Rotation controller. Tune these values for your robot. Leaving them 0 will only use feedforwards.
                new Consumer<SwerveModuleState[]>() {
                    @Override
                    public void accept(SwerveModuleState[] states) {
                        drivetrain.getConstellation().setModuleStates(states);
                    }
                },
                allianceMirror, // Should the path be automatically mirrored depending on alliance color? Optional, defaults to true
                drivetrain // Requires the drive subsystem
            )
        );
    }
}
