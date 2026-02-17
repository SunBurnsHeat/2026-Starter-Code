package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.AbsoluteEncoderConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.ShooterConstants;


public final class Configs {
    public static final class MAXSwereveModule{
        public static final SparkFlexConfig drivingConfig = new SparkFlexConfig();
        public static final SparkFlexConfig turningConfig = new SparkFlexConfig();

        static{
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(40);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0) // radians per second
                    // This applies to REV Through Bore Encoder V1 (use REV_ThroughBoreEncoderV2 for V2):
                    .apply(AbsoluteEncoderConfig.Presets.REV_ThroughBoreEncoder);

            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }

    }

    public static final class ShooterMaxConfig {

        public static final SparkMaxConfig shooterConfig = new SparkMaxConfig();
        public static final SparkMaxConfig follwerConfig = new SparkMaxConfig();
        public static final SparkMaxConfig kickerCofig = new SparkMaxConfig();

        static{
                shooterConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                shooterConfig.encoder
                        .positionConversionFactor(1)
                        .velocityConversionFactor(1/60);
                shooterConfig.closedLoop.maxMotion
                        .maxAcceleration(500);
                shooterConfig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0, 0, 0)
                        .iZone(0)
                        .outputRange(-0.8, 0.8);

                follwerConfig
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12)
                        .follow(ShooterConstants.kShooterCANID, true);
                follwerConfig.closedLoop.maxMotion
                        .maxAcceleration(500);
                
                kickerCofig
                        .idleMode(IdleMode.kBrake)
                        .smartCurrentLimit(40)
                        .voltageCompensation(12);
                kickerCofig.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(0, 0, 0)
                        .outputRange(-0.7, 0.7);
                }
        
        }

        public static final class IntakeConfigs {
        
                public static final SparkMaxConfig indexerConfigs = new SparkMaxConfig();
                public static final SparkMaxConfig intakeConfigs = new SparkMaxConfig();
                public static final SparkMaxConfig intakeRollerConfigs = new SparkMaxConfig();

                static {
                        indexerConfigs
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12);
                        indexerConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.5, 0, 0)
                                .outputRange(-0.6, 0.6);

                        intakeConfigs
                                .idleMode(IdleMode.kCoast)
                                .smartCurrentLimit(40)
                                .voltageCompensation(12);
                        intakeConfigs.encoder
                                .positionConversionFactor(360/IntakeConstants.kIntakeGearReduction)
                                .velocityConversionFactor((360/IntakeConstants.kIntakeGearReduction)/60);
                        intakeConfigs.softLimit
                                .forwardSoftLimit(IntakeConstants.kIntakeFinalPosition)
                                .reverseSoftLimit(IntakeConstants.kIntakeInitialPosition)
                                .forwardSoftLimitEnabled(false)
                                .reverseSoftLimitEnabled(false);
                        intakeConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(0.9, 0, 0)
                                .outputRange(-0.8, 0.8);

                        intakeRollerConfigs
                                .idleMode(IdleMode.kBrake)
                                .smartCurrentLimit(25)
                                .voltageCompensation(12);
                        intakeRollerConfigs.encoder
                                .positionConversionFactor(1/IntakeConstants.kIntakeRollerGearReduction)
                                .velocityConversionFactor((1/IntakeConstants.kIntakeRollerGearReduction)/60);
                        intakeRollerConfigs.closedLoop
                                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                                .pid(1, 0, 0)
                                .outputRange(-0.7, 0.7);
                }
        }
}
