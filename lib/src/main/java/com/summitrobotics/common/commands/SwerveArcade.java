package com.summitrobotics.common.commands;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.kauailabs.navx.frc.AHRS;
import com.summitrobotics.common.oi.inputs.OIAxis;
import com.summitrobotics.common.oi.inputs.OITrigger;
import com.summitrobotics.common.oi.inputs.RisingEdgeTrigger;
import com.summitrobotics.common.oi.inputs.OIAxis.PrioritizedAxis;
import com.summitrobotics.common.oi.inputs.OITrigger.PrioritizedTrigger;
import com.summitrobotics.common.swerve.Swerve;

public class SwerveArcade extends CommandBase {
    Swerve drivetrain;
    AHRS gyro;
    PrioritizedAxis fwd, str, rcw;
    PrioritizedTrigger resetPose, flipMode, rotaTrigger;
    RisingEdgeTrigger flipModeRisingEdge, resetPoseRisingEdge;
    SlewRateLimiter fwdLimiter, strLimiter;

    final double MAX_SPEED;
    boolean fieldOriented = true;

    public SwerveArcade(Swerve drivetrain, AHRS gyro, OIAxis fwd, OIAxis str, OIAxis rcw, OITrigger resetPose, OITrigger flipMode, OITrigger rotateTrigger) {
        this.drivetrain = drivetrain;
        this.gyro = gyro;
        this.fwd = fwd.prioritize(0);
        this.str = str.prioritize(0);
        this.rcw = rcw.prioritize(0);

        this.resetPose = resetPose.prioritize(0);
        this.flipMode = flipMode.prioritize(0);
        this.rotaTrigger = rotateTrigger.prioritize(0);

        this.flipModeRisingEdge = new RisingEdgeTrigger(this.flipMode.getTrigger());
        this.resetPoseRisingEdge = new RisingEdgeTrigger(this.resetPose.getTrigger());

        this.fwdLimiter = new SlewRateLimiter(4.5);
        this.strLimiter = new SlewRateLimiter(4.5);

        addRequirements(drivetrain);
        MAX_SPEED = drivetrain.getConstellation().MAX_SPEED_METERS_PER_SECOND;
    }

    @Override
    public void execute() {
        if (flipModeRisingEdge.get()) {
            fieldOriented = !fieldOriented;
        }
        if (resetPoseRisingEdge.get()) {
            drivetrain.setPose(new Pose2d());
            gyro.calibrate();
            gyro.reset();
            gyro.setAngleAdjustment(180);
            // Sets drivetrain back to 0, reducing acumulated error
            drivetrain.setPose(new Pose2d(0, 0, new Rotation2d(Math.PI)));
        }

        double turnVal = 0;

        if (Math.abs(rcw.get()) > 0.845) {
            turnVal = Math.pow(rcw.get(), 3) * 0.7;
        } else {
            turnVal = 0.5 * rcw.get();
        }

        ChassisSpeeds speed = new ChassisSpeeds(
            -strLimiter.calculate(-str.get() * MAX_SPEED / 4),
            -fwdLimiter.calculate(fwd.get() * MAX_SPEED / 4),
            -turnVal / 10
        );
        if (fieldOriented) {
            speed = ChassisSpeeds.fromFieldRelativeSpeeds(speed, drivetrain.getPose().getRotation());
            System.out.println("Rotation: " + drivetrain.getPose().getRotation());
        }
        if (rotaTrigger.getTrigger().getAsBoolean()) {
            System.out.println("HERE");
            drivetrain.drive(speed, new Translation2d(3, 0));
        } else {
            drivetrain.drive(speed);
        }
    }

    @Override
    public void end(final boolean interrupted) {
        drivetrain.stop();
        fwd.destroy();
        str.destroy();
        rcw.destroy();
        flipMode.destroy();
        resetPose.destroy();
        rotaTrigger.destroy();
    }
}
