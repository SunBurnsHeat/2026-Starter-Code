package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.IntakeConstants;

public class IntakeSubsystem extends SubsystemBase{

    private final SparkMax kIndexerMax;
    private final SparkMax kIntakeMax;
    private final SparkMax kIntakeRollerMax;

    private final SparkClosedLoopController kIndexerController;
    private final SparkClosedLoopController kIntakeController;
    private final SparkClosedLoopController kIntakeRollerController;

    private final RelativeEncoder intakeEncoder;

    private double targetPosition;

    private boolean agitateHigh = false;
    private boolean wasAtPosition = false;

    public IntakeSubsystem(){
        kIndexerMax = new SparkMax(IntakeConstants.kIndexerCANID, MotorType.kBrushless);
        kIntakeMax = new SparkMax(IntakeConstants.kIntakeCANID, MotorType.kBrushless);
        kIntakeRollerMax = new SparkMax(IntakeConstants.kIntakeRollerCANID, MotorType.kBrushless);

        kIndexerMax.configure(Configs.IntakeConfigs.indexerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kIntakeMax.configure(Configs.IntakeConfigs.intakeConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        kIntakeRollerMax.configure(Configs.IntakeConfigs.intakeRollerConfigs, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        intakeEncoder = kIntakeMax.getEncoder();
        intakeEncoder.setPosition(0);

        kIndexerController = kIndexerMax.getClosedLoopController();
        kIntakeController = kIntakeMax.getClosedLoopController();
        kIntakeRollerController = kIntakeRollerMax.getClosedLoopController();

        targetPosition = 0;
    }

    public void setIndexer(double setPoint){
        kIndexerController.setSetpoint(setPoint, ControlType.kDutyCycle);
    }

    public void setIntakeRoller(double setPoint){
        kIntakeRollerController.setSetpoint(setPoint, ControlType.kDutyCycle);
    }

    public boolean intakeAtPosition(){
        return Math.abs((intakeEncoder.getPosition()) - targetPosition) < IntakeConstants.kIntakePositionDeadband;
    }

    private double getNextAgitatePosition() {
        agitateHigh = !agitateHigh;

        return agitateHigh
            ? IntakeConstants.kIntakeFinalPosition
            : 2 * IntakeConstants.kIntakeFinalPosition / 3;
    }

    public void agitateFuel(boolean runRollers){
        if (runRollers) {
            setIntakeRoller(IntakeConstants.intakeRollerAgitationSpeed);
        }

        boolean atPos = intakeAtPosition();

        if (atPos && !wasAtPosition) {
            targetPosition = getNextAgitatePosition();
        }

        wasAtPosition = atPos;
    }

    public void setIntakePosition(double position){
        targetPosition = position;
    }

    @Override
    public void periodic(){
        kIntakeController.setSetpoint(targetPosition, ControlType.kPosition);
        SmartDashboard.putNumber("Intake Pos", intakeEncoder.getPosition());
    }

}
