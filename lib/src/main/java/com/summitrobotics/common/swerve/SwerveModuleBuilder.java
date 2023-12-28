package com.summitrobotics.common.swerve;

import java.util.function.Consumer;
import java.util.function.Supplier;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/** Constructs an individual SwerveModule. */
public class SwerveModuleBuilder {

    public static enum SWERVE_MODULE_PRESETS {
        SDS_MK4i_L1(8.14, 150.0/7.0, Units.inchesToMeters(4), new double[]{0.0, 0.0, 0.0}, 0.533333333333),
        SDS_MK4i_L2(6.75, 150.0/7.0, Units.inchesToMeters(4), new double[]{0.25, 0, 0.005}, 0.533333333333),
        SDS_MK4i_L3(6.12, 150.0/7.0, Units.inchesToMeters(4), new double[]{0.0, 0.0, 0.0}, 0.533333333333);

        public final double driveGearRatio;
        public final double turnGearRatio;
        public final double WheelDiameter;
        public final double[] turnPID;
        public final double turnToDrive;

        SWERVE_MODULE_PRESETS(double driveGearRatio, double turnGearRatio, double WheelDiameter, double[] turnPID, double turnToDrive) {
            this.driveGearRatio = driveGearRatio;
            this.turnGearRatio = turnGearRatio;
            this.WheelDiameter = WheelDiameter;
            this.turnPID = turnPID;
            this.turnToDrive = turnToDrive;
        }
    }

    private Translation2d location;
    private double driveGearRatio = 0; // A ratio of 2 means that the drive motor spins twice as fast as the wheel
    private double turnGearRatio = 0; // A ratio of 2 means that the turn motor spins twice as fast as the rotate wheel
    private double WheelDiameter = 0; // In meters
    private SimpleMotorFeedforward driveFeedforward = new SimpleMotorFeedforward(0, 0);
    private SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0);
    private double driveP = 0;
    private double driveI = 0;
    private double driveD = 0;
    private double turnP = 0;
    private double turnI = 0;
    private double turnD = 0;
    private double driveMotorMaxRPM = 0;
    private double turnToDriveRatio = 0; // This is here because for some modules turning the turn motor also spins the wheel.
    
    private CANSparkMax sparkMaxDriveMotor;
    private CANSparkMax sparkMaxTurnMotor;
    private TalonFX falconDriveMotor;
    private TalonFX falconTurnMotor;
    
    private Supplier<Double> turnEncoderAbsolute; // In radians the encoder reads positive is counterclockwise

    private boolean built = false; // If the module has been built

    // The CANCoder angle is a discontinuous angle; PID controllers don't like the sudden jump between 0 and 360.
    // Both angleConsumer and recalibrate convert to the version of that angle within 180 of the current position.
    // Formula to eliminate jumps: (integer number of 360s to produce something close to the current angle) * 360 + (smallest representation of current angle)
    // TODO - check this conversion
    private double makeAngleContinuous(double current, double target) {
        return Math.round((current - target % (2 * Math.PI)) / (2 * Math.PI)) * 2 * Math.PI + target % (2 * Math.PI);
    }

    public SwerveModuleBuilder() {}
    public SwerveModuleBuilder(Translation2d location) {
        this.location = location;
    }
    public SwerveModuleBuilder(Translation2d location, SWERVE_MODULE_PRESETS preset) {
        this.location = location;
        this.driveGearRatio = preset.driveGearRatio;
        this.turnGearRatio = preset.turnGearRatio;
        this.WheelDiameter = preset.WheelDiameter;
        this.turnP = preset.turnPID[0];
        this.turnI = preset.turnPID[1];
        this.turnD = preset.turnPID[2];
        this.turnToDriveRatio = preset.turnToDrive;
    }

    public SwerveModuleBuilder location(Translation2d location) {
        this.location = location;
        return this;
    }
    public SwerveModuleBuilder driveGearRatio(double driveGearRatio) {
        this.driveGearRatio = driveGearRatio;
        return this;
    }
    public SwerveModuleBuilder turnGearRatio(double turnGearRatio) {
        this.turnGearRatio = turnGearRatio;
        return this;
    }
    public SwerveModuleBuilder wheelDiameter(double WheelDiameter) {
        this.WheelDiameter = WheelDiameter;
        return this;
    }
    public SwerveModuleBuilder driveFeedforward(SimpleMotorFeedforward driveFeedforward) {
        this.driveFeedforward = driveFeedforward;
        return this;
    }
    public SwerveModuleBuilder turnFeedforward(SimpleMotorFeedforward turnFeedforward) {
        this.turnFeedforward = turnFeedforward;
        return this;
    }
    public SwerveModuleBuilder drivePID(double driveP, double driveI, double driveD) {
        this.driveP = driveP;
        this.driveI = driveI;
        this.driveD = driveD;
        return this;
    }
    public SwerveModuleBuilder drivePID(double[] pid) {
        if (pid.length != 3) return this;
        this.driveP = pid[0];
        this.driveI = pid[1];
        this.driveD = pid[2];
        return this;
    }
    public SwerveModuleBuilder turnPID(double turnP, double turnI, double turnD) {
        this.turnP = turnP;
        this.turnI = turnI;
        this.turnD = turnD;
        return this;
    }
    public SwerveModuleBuilder turnPID(double[] pid) {
        if (pid.length != 3) return this;
        this.turnP = pid[0];
        this.turnI = pid[1];
        this.turnD = pid[2];
        return this;
    }
    public SwerveModuleBuilder driveNEO1650(int deviceID) {
        this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        this.driveMotorMaxRPM = 5_676;
        return this;
    }
    public SwerveModuleBuilder turnNEO1650(int deviceID) {
        this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        return this;
    }
    public SwerveModuleBuilder driveFalcon500(int deviceID) {
        this.falconDriveMotor = new TalonFX(deviceID);
        this.driveMotorMaxRPM = 6_380;
        return this;
    }
    public SwerveModuleBuilder turnFalcon500(int deviceID) {
        this.falconTurnMotor = new TalonFX(deviceID);
        return this;
    }
    public SwerveModuleBuilder driveNEO550(int deviceID) {
        this.sparkMaxDriveMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        this.driveMotorMaxRPM = 11_000;
        return this;
    }
    public SwerveModuleBuilder turnNEO550(int deviceID) {
        this.sparkMaxTurnMotor = new CANSparkMax(deviceID, CANSparkMax.MotorType.kBrushless);
        return this;
    }
    public SwerveModuleBuilder turnEncoderAbsolute(Supplier<Double> turnEncoderAbsolute) {
        this.turnEncoderAbsolute = turnEncoderAbsolute;
        return this;
    }
    public SwerveModuleBuilder CANCoder(int deviceID, double offset) {
        CANCoder canCoder = new CANCoder(deviceID);
        canCoder.configFactoryDefault();
        // Keep things simple with only positive values, although -180 to 180 might also work.
        canCoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360); 
        canCoder.configMagnetOffset(Math.toDegrees(offset));
        this.turnEncoderAbsolute = () -> {
            return Math.toRadians(canCoder.getAbsolutePosition());
        };
        return this;
    }

    public SwerveModule build() {
        if (built) {
            throw new IllegalStateException("SwerveModule has already been built");
        }
        if (location == null) {
            throw new IllegalStateException("Location has not been set");
        }
        if (driveGearRatio == 0) {
            throw new IllegalStateException("Drive gear ratio has not been set");
        }
        if (turnGearRatio == 0) {
            throw new IllegalStateException("Turn gear ratio has not been set");
        }
        if (WheelDiameter == 0) {
            throw new IllegalStateException("Wheel diameter has not been set");
        }
        if (driveMotorMaxRPM == 0) {
            throw new IllegalStateException("Drive motor max RPM has not been set");
        }
        if (sparkMaxDriveMotor == null && falconDriveMotor == null) {
            throw new IllegalStateException("Drive motor has not been set");
        }
        if (sparkMaxTurnMotor == null && falconTurnMotor == null) {
            throw new IllegalStateException("Turn motor has not been set");
        }
        if (turnEncoderAbsolute == null) {
            throw new IllegalStateException("Turn encoder has not been set");
        }
        if (sparkMaxDriveMotor != null && falconDriveMotor != null) {
            throw new IllegalStateException("Drive motor has been set twice");
        }
        if (sparkMaxTurnMotor != null && falconTurnMotor != null) {
            throw new IllegalStateException("Turn motor has been set twice");
        }
        
        Supplier<Double> speedSupplier;
        Supplier<Rotation2d> angleSupplier;
        Supplier<Rotation2d> angleSpeedSupplier;
        Supplier<Double> distanceSupplier;
        Consumer<Double> speedConsumer;
        Consumer<Rotation2d> angleConsumer;
        double maxSpeedMPS = 0;

        // This uses the turnEncoderAbsolute to reset the motor encoder to make sure they match.
        // The turnEncoderAbsolute times (1/turnEncoderAbsoluteRatio) plus the offset is the angle of the wheel in radians.
        // The normal motor encoder needs to be multiplied by the turnGearRatio then converted from rotations to radians.
        Runnable recalibrate; 

        if (sparkMaxDriveMotor != null) {
            RelativeEncoder encoder = sparkMaxDriveMotor.getEncoder();
            encoder.setPositionConversionFactor((WheelDiameter * Math.PI) / (driveGearRatio)); // TODO Check to make sure this is accurate
            encoder.setVelocityConversionFactor((WheelDiameter * Math.PI) / (driveGearRatio * 60)); // TODO Check to make sure this is accurate
            distanceSupplier = encoder::getPosition;
            speedSupplier = encoder::getVelocity;
            SparkMaxPIDController pidController = sparkMaxDriveMotor.getPIDController();
            pidController.setP(driveP, 0);
            pidController.setI(driveI, 0);
            pidController.setD(driveD, 0);
            pidController.setFeedbackDevice(encoder);
            speedConsumer = (Double speed) -> {
                pidController.setReference(speed, ControlType.kVelocity, 0, driveFeedforward.calculate(speed));
            };
        } else if (falconDriveMotor != null) {
            // TODO
            distanceSupplier = null;
            speedSupplier = null;
            speedConsumer = null;
            throw new IllegalStateException("Falcons are not supported yet");
        } else {
            throw new IllegalStateException("Drive motor has not been set");
        }

        if (sparkMaxTurnMotor != null) {
            RelativeEncoder encoder = sparkMaxTurnMotor.getEncoder();
            sparkMaxTurnMotor.setInverted(true);
            encoder.setPositionConversionFactor((2 * Math.PI) / (turnGearRatio)); // TODO Check to make sure this is accurate
            encoder.setVelocityConversionFactor((2 * Math.PI) / (turnGearRatio)); // TODO Check to make sure this is accurate
            angleSupplier = () -> {
                double angle = encoder.getPosition() % (2 * Math.PI);
                if (angle > Math.PI) {
                    angle -= 2 * Math.PI;
                } else if (angle < -Math.PI) {
                    angle += 2 * Math.PI;
                }
                return new Rotation2d(angle);
            };
            angleSpeedSupplier = () -> {
                return new Rotation2d(encoder.getVelocity());
            };
            SparkMaxPIDController pidController = sparkMaxTurnMotor.getPIDController();
            pidController.setP(turnP, 0);
            pidController.setI(turnI, 0);
            pidController.setD(turnD, 0);
            pidController.setFeedbackDevice(encoder);
            angleConsumer = (Rotation2d angle) -> {
                double reference = makeAngleContinuous(encoder.getPosition(), angle.getRadians());
                pidController.setReference(reference, ControlType.kPosition, 0, turnFeedforward.calculate(reference));
            };
            recalibrate = () -> { encoder.setPosition(makeAngleContinuous(encoder.getPosition(), turnEncoderAbsolute.get())); };
        } else if (falconTurnMotor != null) {
            // TODO
            angleSupplier = null;
            angleConsumer = null;
            angleSpeedSupplier = null;
            throw new IllegalStateException("Falcons are not supported yet");
        } else {
            throw new IllegalStateException("Turn motor has not been set");
        }

        maxSpeedMPS = (driveMotorMaxRPM * WheelDiameter * Math.PI) * 2 / (driveGearRatio * 60) * 4;

        built = true;

        if (location == null || speedSupplier == null || angleSupplier == null || distanceSupplier == null || speedConsumer == null || angleConsumer == null || angleSpeedSupplier == null || recalibrate == null || maxSpeedMPS == 0) {
            throw new IllegalStateException("Something went wrong building the SwerveModule");
        }

        Supplier<Double> compensatedDistanceSupplier = () -> {
            return distanceSupplier.get() + (angleSupplier.get().getRotations() * turnToDriveRatio * WheelDiameter * Math.PI);
        };

        Supplier<Double> compensatedSpeedSupplier = () -> {
            return speedSupplier.get() + (angleSpeedSupplier.get().getRotations() * turnToDriveRatio * WheelDiameter * Math.PI / 60);
        };

        return new SwerveModule(
            location,
            compensatedSpeedSupplier,
            angleSupplier,
            compensatedDistanceSupplier,
            speedConsumer,
            angleConsumer,
            recalibrate,
            maxSpeedMPS,
            sparkMaxDriveMotor != null ? sparkMaxDriveMotor : falconDriveMotor,
            sparkMaxTurnMotor != null ? sparkMaxTurnMotor : falconTurnMotor
        );
    }
}
