// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class FloorIntake extends SubsystemBase {
  private final WPI_TalonSRX floorIntake = new WPI_TalonSRX(Constants.Mod8);

  public FloorIntake() {}
  
  public void floorIntakeFORWARD() {
    floorIntake.set(0.65);
  }
  
  public void floorIntakeBACKWARDS() {
    floorIntake.set(-0.65);
  }

  public void floorIntakeOff() {
    floorIntake.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
