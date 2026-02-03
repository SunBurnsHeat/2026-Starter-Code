package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;


public class AlignToTagCommand extends Command {
    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    private final PIDController yController;    // only strafe
    private final PIDController xController;

    private final double positionTolerance = 0.05; // Meters (5 cm)
    private double targetOffsetY;        // desired static position from center of target
    private double targetOffsetX; 
    private double targetSetpointY;      // setpoint fed to PID
    private double targetSetpointX; 
    private double targetY;             // actual position from photonvision
    private double targetX;
    private double targetErrorY;        // error input to PID
    private double targetErrorX;
    private double pidOutY;             // PID control output
    private double pidOutX; 
    private double ySpeed;          // clipped PID control output
    private double xSpeed; 
    private double maxSpeedMultiplier;
    
    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem) {
        this(driveSubsystem, visionSubsystem, 0.0, 34, 1);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY, double targetOffsetX) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetY = targetOffsetY;
        this.targetOffsetX = 0.34;
        this.maxSpeedMultiplier = 1;

        this.yController = new PIDController(.90, 0, 1e-4);
        this.xController = new PIDController(.90, 0, 1e-4);
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }


    public AlignToTagCommand(DriveSubsystem driveSubsystem, VisionSubsystem visionSubsystem, double targetOffsetY, double targetOffsetX, double maxSpeedMultiplier) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetY = targetOffsetY;
        this.targetOffsetX = targetOffsetX;
        this.maxSpeedMultiplier = maxSpeedMultiplier;

        this.yController = new PIDController(.90, 0, 1e-5);
        this.xController = new PIDController(.90, 0, 1e-5);
        xController.setTolerance(positionTolerance);
        yController.setTolerance(positionTolerance);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        yController.reset();
        xController.reset();
        targetErrorY = 0.0;
        targetErrorX = 0.0;
        pidOutY = 0.0;
        pidOutX = 0.0;
        targetSetpointY = 0.0;
        targetSetpointX = 0.0;
        ySpeed = 0.0;
        xSpeed = 0.0;
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);
    }

    @Override
    public void execute() {
        if (visionSubsystem.hasTarget()) {

            targetSetpointY = targetOffsetY;      // static command
            targetSetpointX = targetOffsetX;
            targetY = VisionSubsystem.getTarget_y();
            targetX = VisionSubsystem.getTarget_x();
            pidOutY = yController.calculate(targetY, targetSetpointY);
            pidOutX = xController.calculate(targetX, targetSetpointX);
            targetErrorY = yController.getError();
            targetErrorX = xController.getError();

            ySpeed = Math.max(-0.35*maxSpeedMultiplier, Math.min(0.35*maxSpeedMultiplier, pidOutY)); // Meters/sec
            xSpeed = Math.max(-0.35*maxSpeedMultiplier, Math.min(0.35*maxSpeedMultiplier, pidOutX));

            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);

            double xCmd = xSpeed / DriveConstants.kMaxSpeedMetersPerSec;
            double yCmd = ySpeed / DriveConstants.kMaxSpeedMetersPerSec;

            xCmd = MathUtil.clamp(xCmd, -1.0, 1.0);
            yCmd = MathUtil.clamp(yCmd, -1.0, 1.0);

            driveSubsystem.drive(xCmd, yCmd, 0, false, true);

            // driveSubsystem.drive(-xSpeed, -ySpeed, 0, false, true);    // try kinematic rate limit??

        } else {
            pidOutY = 0.0;
            pidOutX = 0.0;
            ySpeed = 0.0;
            xSpeed = 0.0;
            targetErrorY = 0.0;
            targetErrorX = 0.0;
            driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
        }
        // mpk - comment out after verifing target values
        // SmartDashboard.putNumber("targetSetpoint", targetSetpoint);
        // SmartDashboard.putNumber("target_y", targetY);
        // SmartDashboard.putNumber("targetError", targetError);
        // SmartDashboard.putNumber("pidOut", pidOut);
        // SmartDashboard.putNumber("ySpeed", ySpeed);

    }

    @Override
    public boolean isFinished() {
        if (!visionSubsystem.hasTarget() || (yController.atSetpoint() && xController.atSetpoint())) {
            return true; 
        } else {
            return false;
        }

    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0.0, 0.0, 0.0, false, false);
    }
}