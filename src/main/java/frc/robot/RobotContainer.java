package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.events.EventTrigger;
import com.revrobotics.spark.SparkMaxAlternateEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.AlignToTagCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.VisionSubsystem;

public class RobotContainer {

  private final VisionSubsystem vision = new VisionSubsystem();
  private final DriveSubsystem robotDrive = new DriveSubsystem(vision);


  private final CommandXboxController driverControllerCommand =
    new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SendableChooser<Command> autoChooser;

  private final SendableChooser<AutoPos> autoPosition;

  private double autoDelay;

  
  public RobotContainer() {
    configureBindings();

    autoPosition = new SendableChooser<AutoPos>();
    autoPosition.addOption("Left", AutoPos.Left);
    autoPosition.addOption("Center", AutoPos.Center);
    autoPosition.addOption("Right", AutoPos.Right);
    autoPosition.setDefaultOption("Center", AutoPos.Center);
    SmartDashboard.putData("Auto Pos", autoPosition);

    autoChooser = AutoBuilder.buildAutoChooser("Example_Auto");
    SmartDashboard.putData("Auto Mode", autoChooser);
    SmartDashboard.putNumber("Auto Delay", 0);

  }

  private void configureBindings() {

    robotDrive.setDefaultCommand(new DefaultDriveCommand(robotDrive));

    driverControllerCommand.a().whileTrue(new AlignToTagCommand(robotDrive, vision));
    driverControllerCommand.y().whileTrue(new RunCommand(() -> robotDrive.setX()));
    driverControllerCommand.start().onTrue(new InstantCommand(() -> robotDrive.zeroHeading(), robotDrive));
  }

  // private boolean leftTrigger() {
  //   return copilotController.getRawAxis(2) > 0.75;
  // }
  // private boolean rightTrigger() {
  //   return copilotController.getRawAxis(3) > 0.75;
  // }
  // private boolean R1Down() {
  //   return copilotController.getRawAxis(5) > 0.75;
  // }
  // private boolean R1Up() {
  //   return copilotController.getRawAxis(5) < -0.75;
  // }
  // private boolean R1Left(){
  //   return copilotController.getRawAxis(4) < -0.75;
  // }
  // private boolean R1Right(){
  //   return copilotController.getRawAxis(4) > 0.75;
  // }
  // private boolean L1Down() {
  //   return copilotController.getRawAxis(1) > 0.75;
  // }
  // private boolean L1Up() {
  //   return copilotController.getRawAxis(1) < -0.75;
  // }


  public Command getAutonomousCommand() {

    autoDelay = SmartDashboard.getNumber("Auto Delay", 0);

    robotDrive.zeroHeading();
    if (autoPosition.getSelected() == AutoPos.Center) {
      robotDrive.setFieldRelativeOffset(180);
    }
    else if (autoPosition.getSelected() == AutoPos.Left) {
      robotDrive.setFieldRelativeOffset(-135);
    }
    else if (autoPosition.getSelected() == AutoPos.Right) {
      robotDrive.setFieldRelativeOffset(135);
    }
    return new WaitCommand(autoDelay).andThen(autoChooser.getSelected());
  }


  public enum AutoPos{
    Left, Center, Right
  }

}
