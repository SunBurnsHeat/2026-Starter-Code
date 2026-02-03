package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;

public class DefaultDriveCommand extends Command{

    private final DriveSubsystem driveSubsystem;
    private XboxController driverController = new XboxController(OIConstants.kDriverControllerPort);

    public DefaultDriveCommand(DriveSubsystem subsystem){
        this.driveSubsystem = subsystem;
        addRequirements(subsystem);
    }

    @Override
    public void execute(){
        int fineTurn = 0; // robot's turning sensitivity when received controller input
        if( driverController.getXButton() ){
            fineTurn += 1; // if button X is pressed, the slow turning constant is set to 1 or left_turn
        }
        if( driverController.getBButton() ){
            fineTurn -= 1;  // if button B is pressed, the slow turning constant is set to -1 or right_turn
        }

        double multiplier = 0.4; // speed adjustment constant for regular control
        double povMultiplier = 0.5; // speed adjustment constant for POV (D-Pad) control
        // when both triggers are pressed, ....
        if( (driverController. getLeftTriggerAxis() > 0.75 ) && (driverController.getRightTriggerAxis() > 0.75) ) {
            multiplier = 1; // turbo mode (100% speed) for regular control
            povMultiplier = 1.5; // turbo mode (150% speed) for POV control
        }
        // when only one trigger is pressed, ....
        else if( (driverController.getLeftTriggerAxis() > 0.75) || (driverController.getRightTriggerAxis() > 0.75) ) {
        multiplier = 0.65; // moderate mode (65% speed) for regular control
        povMultiplier = 1;  // normal mode (100% speed) for POV control
        }

        // if no D-pad direction pressed, ....
        if(driverController.getPOV() == -1){
            // if there is not turning input, ....
            if(fineTurn == 0){
                // drive without the fine_turn constant
               driveSubsystem.drive(
                    -multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
                    -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
                    -multiplier*0.85*MathUtil.applyDeadband(driverController.getRightX(), 0.01),
                    true, true); 
            }
            // otherwise, ....
            else {
                // drive with the constant
                driveSubsystem.drive(
                    -multiplier*MathUtil.applyDeadband(driverController.getLeftY(), 0.015),
                    -multiplier*MathUtil.applyDeadband(driverController.getLeftX(), 0.015),
                    povMultiplier*fineTurn,
                    true, true);
            }
        }
        // otherwise(if D-pad direction pressed), ....
        else {
            // when the D-pad's angle is ....
            switch(driverController.getPOV()){
                // drive towards those direction
                case(0):
                    driveSubsystem.drive(povMultiplier*0.25, 0, povMultiplier*fineTurn, true, true); 
                case(45):
                    driveSubsystem.drive(povMultiplier*0.25, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
                case(90):
                    driveSubsystem.drive(0, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
                case(135):
                    driveSubsystem.drive(povMultiplier*-0.25, povMultiplier*-0.25, povMultiplier*fineTurn, true, true);
                case(180):
                    driveSubsystem.drive(povMultiplier*-0.25, 0, povMultiplier*fineTurn, true, true);
                case(225):
                    driveSubsystem.drive(povMultiplier*-0.25, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
                case(270):
                    driveSubsystem.drive(0, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
                case(315):
                    driveSubsystem.drive(povMultiplier*0.25, povMultiplier*0.25, povMultiplier*fineTurn, true, true);
            }
        }
    }

    @Override
    // this class's command is not ended until interrupted
    public boolean isFinished() {
        return false;
    }
}