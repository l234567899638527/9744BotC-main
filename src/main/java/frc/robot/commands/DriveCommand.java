// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drive;
import java.util.function.DoubleSupplier;
import java.util.function.BooleanSupplier;


public class DriveCommand extends Command {
  private Drive driveSubsystem;
  
  private DoubleSupplier controllerLeftY;
  private DoubleSupplier controllerRightX;
  private BooleanSupplier brakeMode;

  /** Creates a new DriveCommands. */
  public DriveCommand(Drive driveSubsystem2, DoubleSupplier controllerLeftY2, DoubleSupplier controllerRightX2, BooleanSupplier brake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem2;
    addRequirements(driveSubsystem);

    this.controllerLeftY = controllerLeftY2;
    this.controllerRightX = controllerRightX2;
    this.brakeMode = brake;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(brakeMode.getAsBoolean()){
      driveSubsystem.brake();
    }
    else{
      driveSubsystem.coastMode();
    }

    driveSubsystem.drive(controllerLeftY.getAsDouble(), controllerRightX.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
