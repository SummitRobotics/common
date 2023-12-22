package com.summitrobotics.common.swerve;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.summitrobotics.common.utilities.Functions;

/** A swerve drivetrain subsystem will extend this class. */
public abstract class Swerve extends SubsystemBase {
    public abstract SwerveConstellation getConstellation();
    public abstract Rotation2d getGyroscopeRotation();

    protected ChassisSpeeds chassisSpeeds = new ChassisSpeeds();
    protected Translation2d rotationPoint = new Translation2d();
    protected Field2d field2d = new Field2d();

    private SwerveDrivePoseEstimator poseEstimator;
    protected SwerveDrivePoseEstimator getPoseEstimator() {
        if (poseEstimator == null) {
            poseEstimator = new SwerveDrivePoseEstimator(
                getConstellation().kinematics,
                getGyroscopeRotation(),
                getConstellation().modulePositions(),
                new Pose2d(),
                VecBuilder.fill(0.02, 0.02, 0.01),
                VecBuilder.fill(0.1, 0.1, 0.01)
            );
        }
        return poseEstimator;
    }

    public Pose2d getPose() {
        return getPoseEstimator().getEstimatedPosition();
    }

    public void setPose(Pose2d pose) {
        getPoseEstimator().resetPosition(getGyroscopeRotation(), getConstellation().modulePositions(), pose);
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
        this.rotationPoint = new Translation2d();
    }

    public void drive(ChassisSpeeds chassisSpeeds, Translation2d rotationPoint) {
        this.chassisSpeeds = chassisSpeeds;
        this.rotationPoint = rotationPoint;
    }

    public ChassisSpeeds getCurrentVelocity() {
        return getConstellation().chassisSpeeds();
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        SwerveConstellation constellation = getConstellation();
        getPoseEstimator().update(getGyroscopeRotation(), constellation.modulePositions());
        field2d.setRobotPose(getPoseEstimator().getEstimatedPosition());
        if (
            Functions.withinTolerance(chassisSpeeds.vxMetersPerSecond, 0, 0.01) &&
            Functions.withinTolerance(chassisSpeeds.vyMetersPerSecond, 0, 0.01) &&
            Functions.withinTolerance(chassisSpeeds.omegaRadiansPerSecond, 0, 0.01)
        ) {
            constellation.stopModules();
        } else {
            constellation.setModuleStates(chassisSpeeds, rotationPoint);
        }
        constellation.recalibrate();
    }

    public void stop() {
        this.drive(new ChassisSpeeds(0, 0, 0));
    }

    @Override
    public void initSendable(SendableBuilder builder) {
        builder.addDoubleProperty("Odometry X", () -> getPose().getX(), null);
        builder.addDoubleProperty("Odometry Y", () -> getPose().getY(), null);
        builder.addDoubleProperty("Odometry Heading (Rads)", () -> getPose().getRotation().getRadians(), null);
        builder.addDoubleProperty("Velocity X", () -> getCurrentVelocity().vxMetersPerSecond, null);
        builder.addDoubleProperty("Velocity Y", () -> getCurrentVelocity().vyMetersPerSecond, null);
        builder.addDoubleProperty("Velocity Heading (Deg)", () -> getCurrentVelocity().omegaRadiansPerSecond * 180 / Math.PI, null);
        SmartDashboard.putData("Field", field2d);
    }
}
