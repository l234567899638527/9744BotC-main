// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Amp extends SubsystemBase {

  private final WPI_TalonSRX amp = new WPI_TalonSRX(Constants.Mod7);

  /** Creates a new Amp. */
  public Amp() {}

  public void ampShooterOn() {
    amp.set(-1.0);
  }
  
  public void ampIntakeOn() {
    amp.set(0.3);
  }

  public void ampOff() {
    amp.set(0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
