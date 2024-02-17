package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.AmpIntakeCommand;
import frc.robot.commands.AmpShooterCommand;
import frc.robot.commands.DriveBoostCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.FloorIntakeBACKWARDSCommand;
import frc.robot.commands.FloorIntakeFORWARDCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.commands.ShooterTopIntakeCommand;
import frc.robot.subsystems.Amp;
import frc.robot.subsystems.Drive;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;

public class RobotContainer {
  private Drive m_drive = new Drive();
  private Shooter m_shooter = new Shooter();
  private Amp m_amp = new Amp();
  private FloorIntake m_floorIntake = new FloorIntake();

  

  private final SendableChooser<Integer> autoChooser = new SendableChooser<>();

  private final XboxController m_controller0 = new XboxController(0);
  private final XboxController m_controller1 = new XboxController(1);


  public RobotContainer() {
    // Register Named Commands

    NamedCommands.registerCommand("ShooterOn", m_shooter.shooterOnCommand());
    NamedCommands.registerCommand("ShooterOff", m_shooter.shooterOffCommand());

    SmartDashboard.putData("Auton", autoChooser);
    autoChooser.setDefaultOption("TEST", 1);
    autoChooser.addOption("TopBlue", 2);

    configureButtonBindings();
  }

  public void configureButtonBindings() {
    m_drive.setDefaultCommand(
      new DriveCommand(m_drive, () -> -m_controller0.getLeftY(), () -> -m_controller0.getRightX(), () -> m_controller0.getLeftTriggerAxis()>0.8) //Drive/Brake
    );


    new Trigger(() -> m_controller0.getRightTriggerAxis()>0.8).whileTrue(new DriveBoostCommand()); //Boost

    new Trigger(() -> m_controller1.getBButton()).whileTrue(new FloorIntakeBACKWARDSCommand(m_floorIntake)); //Spit
    new Trigger(() -> m_controller1.getXButton()).whileTrue(new FloorIntakeFORWARDCommand(m_floorIntake)); //Intake
    new Trigger(() -> m_controller1.getRightTriggerAxis()>0.8).whileTrue(new ShooterCommand(m_shooter)); //Shooter
    new Trigger(() -> m_controller1.getLeftTriggerAxis()>0.8).whileTrue(new ShooterTopIntakeCommand(m_shooter)); //Shooter Top Intake
    new Trigger(() -> m_controller1.getRightBumper()).whileTrue(new AmpShooterCommand(m_amp)); //Amp Shooter
    new Trigger(() -> m_controller1.getLeftBumper()).whileTrue(new AmpIntakeCommand(m_amp)); //Amp Intake
  }

  public Command getAutonomousCommand() {
    switch(autoChooser.getSelected()) {
      case 1: return AutoBuilder.buildAuto("TEST");
      case 2: return AutoBuilder.buildAuto("TopBuild");

      default: return AutoBuilder.buildAuto("TEST");
    }

  }
}