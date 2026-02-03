package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.DriveConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class AlignAndFaceTagCommand extends Command {

    private final DriveSubsystem driveSubsystem;
    private final VisionSubsystem visionSubsystem;

    // PID controllers
    private final PIDController xController;     // forward/back
    private final PIDController yController;     // strafe
    private final PIDController thetaController; // rotation

    // Tolerances
    private static final double POSITION_TOLERANCE = 0.05; // meters
    private static final double ANGLE_TOLERANCE = 1.0;     // degrees

    // Setpoints
    private final double targetOffsetX;
    private final double targetOffsetY;

    public AlignAndFaceTagCommand(
            DriveSubsystem driveSubsystem,
            VisionSubsystem visionSubsystem,
            double targetOffsetX,
            double targetOffsetY
    ) {
        this.driveSubsystem = driveSubsystem;
        this.visionSubsystem = visionSubsystem;
        this.targetOffsetX = targetOffsetX;
        this.targetOffsetY = targetOffsetY;

        // X/Y PIDs (meters)
        xController = new PIDController(0.9, 0.0, 1e-5);
        yController = new PIDController(0.9, 0.0, 1e-5);
        xController.setTolerance(POSITION_TOLERANCE);
        yController.setTolerance(POSITION_TOLERANCE);

        // Theta PID (degrees → normalized output)
        thetaController = new PIDController(0.02, 0.0, 0.0);
        thetaController.enableContinuousInput(-180, 180);
        thetaController.setTolerance(ANGLE_TOLERANCE);

        addRequirements(driveSubsystem, visionSubsystem);
    }

    @Override
    public void initialize() {
        xController.reset();
        yController.reset();
        thetaController.reset();
    }

    @Override
    public void execute() {
        if (!visionSubsystem.hasTarget()) {
            driveSubsystem.drive(0, 0, 0, false, false);
            return;
        }

        double currentX = VisionSubsystem.getTarget_x();   // meters
        double currentY = VisionSubsystem.getTarget_y();   // meters
        double currentYaw = VisionSubsystem.getTarget_rawYaw(); // degrees

        double xOut = xController.calculate(currentX, targetOffsetX);
        double yOut = yController.calculate(currentY, targetOffsetY);
        // double thetaOut = thetaController.calculate(currentYaw, 0.0);

        double omegaRadPerSec =
            thetaController.calculate(
                Units.degreesToRadians(currentYaw), 0.0
            );


        double xCmd = xOut / DriveConstants.kMaxSpeedMetersPerSec;
        double yCmd = yOut / DriveConstants.kMaxSpeedMetersPerSec;
        double rotCmd =
                omegaRadPerSec / DriveConstants.kMaxAngSpeedRadiansPerSec;

        xCmd = MathUtil.clamp(xCmd, -0.5, 0.5);
        yCmd = MathUtil.clamp(yCmd, -0.5, 0.5);
        rotCmd = MathUtil.clamp(rotCmd, -0.65, 0.65);

        driveSubsystem.drive(
                -xCmd,      // forward/back
                -yCmd,      // strafe
                rotCmd,   // rotation 
                false,
                true
        );
    }

    @Override
    public boolean isFinished() {
        return !visionSubsystem.hasTarget() || 
                (xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint());
    }

    @Override
    public void end(boolean interrupted) {
        driveSubsystem.drive(0, 0, 0, false, false);
    }
}
