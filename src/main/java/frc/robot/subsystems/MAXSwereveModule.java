package frc.robot.subsystems;

import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import frc.robot.Configs;


public class MAXSwereveModule {

    private final SparkFlex kDrivingFlex;
    private final SparkFlex kTurningMAX;

    private final RelativeEncoder kDriveEncoder;
    private final AbsoluteEncoder kTurningEncoder;

    private final SparkClosedLoopController kDrivingClosedLoopController;
    private final SparkClosedLoopController kTurningClosedLoopController;

    private double chassisoffset = 0.0;
    private SwerveModuleState targetstate = new SwerveModuleState(0.0, new Rotation2d());

    public MAXSwereveModule(int turningCANid, int drivingCANid, double angleOffset) {
        kTurningMAX = new SparkFlex(turningCANid, MotorType.kBrushless);
        kDrivingFlex = new SparkFlex(drivingCANid, MotorType.kBrushless);


        kDriveEncoder = kDrivingFlex.getEncoder();
        kTurningEncoder = kTurningMAX.getAbsoluteEncoder();
        
        kTurningClosedLoopController = kTurningMAX.getClosedLoopController();
        kDrivingClosedLoopController = kDrivingFlex.getClosedLoopController();

        kDrivingFlex.configure(Configs.MAXSwereveModule.drivingConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kTurningMAX.configure(Configs.MAXSwereveModule.turningConfig, 
        ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        chassisoffset = angleOffset;
        targetstate.angle = new Rotation2d(kTurningEncoder.getPosition());
        kDriveEncoder.setPosition(0);
        
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(kDriveEncoder.getVelocity(), 
        new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(kDriveEncoder.getPosition(), 
        new Rotation2d(kTurningEncoder.getPosition() - chassisoffset));
    }

    // sets the desired state of the module
    public void setState(SwerveModuleState tState){

        // creates a new swerve module state that holds the adjusted state
        SwerveModuleState correctState = new SwerveModuleState();

        // set to target speed from "tState"
        correctState.speedMetersPerSecond = tState.speedMetersPerSecond;
        // added chassis offset
        correctState.angle = tState.angle.plus(Rotation2d.fromRadians(chassisoffset));

        // optimize with minimal movement
        correctState.optimize(new Rotation2d(kTurningEncoder.getPosition()));

        // sets desired speed of driving motor and angle for turning motor with the PID controllers
        kDrivingClosedLoopController.setSetpoint(correctState.speedMetersPerSecond, ControlType.kVelocity);
        kTurningClosedLoopController.setSetpoint(correctState.angle.getRadians(), ControlType.kPosition); // there is no such thing as angle controltype

        // updated the target state
        targetstate = correctState;
    }

    // resets the drive encoder position to zero
    public void resetEncoder() {
        kDriveEncoder.setPosition(0);
        // the turning encoder is not resetted because it is important to keep the turning state offset acknowledged
    }
}
