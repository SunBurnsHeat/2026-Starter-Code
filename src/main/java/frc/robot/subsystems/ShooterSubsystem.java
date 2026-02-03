package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants.ShooterConstants;
import frc.robot.Configs.ShooterMaxConfig;

public class ShooterSubsystem extends SubsystemBase{
    private final SparkMax kShooterMax;
    private final SparkMax kFollowerMax;
    private final SparkMax kKickerMax;

    private final SparkClosedLoopController kShooterController;
    private final SparkClosedLoopController kKickerController;

    private double targetSetpoint;

    public ShooterSubsystem(){
        kShooterMax = new SparkMax(ShooterConstants.kShooterCANID, MotorType.kBrushless);
        kFollowerMax = new SparkMax(ShooterConstants.kShooterFollowerCANID, MotorType.kBrushless);

        kKickerMax = new SparkMax(ShooterConstants.kKickerCANID, MotorType.kBrushless);

        kShooterMax.configure(ShooterMaxConfig.shooterConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kFollowerMax.configure(ShooterMaxConfig.follwerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kKickerMax.configure(ShooterMaxConfig.kickerCofig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        kShooterController = kShooterMax.getClosedLoopController();
        kKickerController = kKickerMax.getClosedLoopController();

        targetSetpoint = 0;
    }

    public void setKickerRollers(double setpoint){
        kKickerController.setSetpoint(setpoint, ControlType.kDutyCycle);
    }

    public void setShooterVelocity(double velocityRPM){
        targetSetpoint = velocityRPM;
    }

    public boolean atSpeed(){
        return Math.abs(kShooterMax.getEncoder().getVelocity() - targetSetpoint) < ShooterConstants.kShooterSpeedDeadband;
    }

    public void stopShooterRollers() {
        kKickerController.setSetpoint(0, ControlType.kDutyCycle);
        targetSetpoint = 0;
    }

    public void kickFuel(boolean checkForSpeed) {
        if(targetSetpoint > 0 && checkForSpeed) {
            if(atSpeed()) {
                setKickerRollers(ShooterConstants.kickerKickSpeed);
            }
        }
        else {
            setKickerRollers(ShooterConstants.kickerKickSpeed);
        }
    }

    @Override
    public void periodic() {
        kShooterController.setSetpoint(targetSetpoint, ControlType.kVelocity);

        SmartDashboard.putNumber("Shooter Velocity", targetSetpoint);
    }
}