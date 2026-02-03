package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends Command{

    private IntakeSubsystem intakeSubsystem;
    private boolean isOscillating;
    
    private boolean wasAtPosition = false;
    private boolean oscillateHigh = false;
    private double targetPosition;

    public IntakeCommand(IntakeSubsystem intakeSubsystem, boolean isOscillating) {
        this.intakeSubsystem = intakeSubsystem;
        this.isOscillating = isOscillating;
        this.targetPosition = IntakeConstants.kIntakeFinalPosition;
        addRequirements(intakeSubsystem);
    }

    @Override
    public void initialize() {
        intakeSubsystem.setIntakeRoller(IntakeConstants.intakeSpeed);
    }

    @Override
    public void execute() {
        if (isOscillating) {
        boolean atPos = intakeSubsystem.intakeAtPosition();

            if (atPos && !wasAtPosition) {
                oscillateHigh = !oscillateHigh;

                targetPosition = oscillateHigh ? 
                    IntakeConstants.kIntakeFinalPosition: IntakeConstants.kIntakeFinalPosition - 3;
                
                intakeSubsystem.setIntakePosition(targetPosition);
            }

            wasAtPosition = atPos;
        }
    }

    @Override
    public void end(boolean canceled) {
        intakeSubsystem.setIntakeRoller(0.0);
    }
}
