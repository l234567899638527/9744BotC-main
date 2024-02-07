// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {
  private double dSpeed = 0.8; 


  private final static WPI_TalonSRX m_leftDrive = new WPI_TalonSRX(1);
  private final static WPI_TalonSRX m_rightDrive = new WPI_TalonSRX(2);
  private final static WPI_TalonSRX m_leftDrive2 = new WPI_TalonSRX(3);
  private final static WPI_TalonSRX m_rightDrive2 = new WPI_TalonSRX(4);
  private final DifferentialDrive m_robotDrive = new DifferentialDrive(m_leftDrive, m_rightDrive);

  private final static WPI_TalonSRX shooter1 = new WPI_TalonSRX(5);
  private final static WPI_TalonSRX shooter2 = new WPI_TalonSRX(6);
  private final WPI_TalonSRX amp = new WPI_TalonSRX(7);


  private final XboxController m_controller0 = new XboxController(0);
  private final XboxController m_controller1 = new XboxController(1);
  private final Timer m_timer = new Timer();



  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftDrive);
    SendableRegistry.addChild(m_robotDrive, m_rightDrive);
  }

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_leftDrive2.follow((m_leftDrive));
    m_rightDrive2.follow(m_rightDrive);
    m_rightDrive.setInverted(true);
    m_leftDrive2.setInverted(false);
    m_rightDrive2.setInverted(true);
    m_leftDrive2.setNeutralMode(NeutralMode.Coast);
    m_rightDrive2.setNeutralMode(NeutralMode.Coast);
  
  }

  /** This function is run once each time the robot enters autonomous mode. */
  @Override
  public void autonomousInit() {
    m_timer.restart();
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // Drive for 2 seconds
    if (m_timer.get() < 2.0) {
      // Drive forwards half speed, make sure to turn input squaring off
      m_robotDrive.arcadeDrive(0.5, 0.0, false);
    } else {
      m_robotDrive.stopMotor(); // stop robot
    }
  }

  /** This function is called once each time the robot enters teleoperated mode. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during teleoperated mode. */
  @Override
  public void teleopPeriodic() {
    double getLeftY = Math.pow(m_controller0.getLeftY(),3)*dSpeed;
    double getRightX = Math.pow(m_controller0.getRightX(),3)*dSpeed;

    m_robotDrive.curvatureDrive(getLeftY, getRightX,true);
    

    //DRIVER
    //Boost
    if(m_controller0.getRightBumper()) {
      dSpeed = 1.0;
    }
    else{
      dSpeed = 0.6;
    }

     //Brake
    if(m_controller0.getLeftBumper()) {
     m_leftDrive2.setNeutralMode(NeutralMode.Brake);
     m_rightDrive2.setNeutralMode(NeutralMode.Brake);
     m_leftDrive.setNeutralMode(NeutralMode.Brake);
     m_rightDrive.setNeutralMode(NeutralMode.Brake);
    }
    else {
     m_leftDrive2.setNeutralMode(NeutralMode.Coast);
     m_rightDrive2.setNeutralMode(NeutralMode.Coast);
     m_leftDrive.setNeutralMode(NeutralMode.Coast);
     m_rightDrive.setNeutralMode(NeutralMode.Coast);
    }


    //OPERATOR
    //Shooter
    if(m_controller1.getRightTriggerAxis()>0.8){
      shooter1.set(1.0);
    }
    else if(m_controller1.getXButton()){
      shooter2.set(1.0);
    }
    //Top Intake
    else if(m_controller1.getLeftTriggerAxis()>0.8){ 
      shooter1.set(-0.8);
      shooter2.set(-0.8);
    }
    else{
    shooter1.set(0);
    shooter2.set(0);
    }

    //AMP
    //Amp Shooter
    if(m_controller1.getRightBumper()) { 
      amp.set(1.0);
    }
    //Amp Intake
    else if(m_controller1.getLeftBumper()) {
      amp.set(-0.3);
    }
    else{
      amp.set(0);
    }


  }

//AUTONOMOUS FUNCTIONS
  public static void shootOn(){
    shooter1.set(1.0);
  }

  public static void shooterOff(){
    shooter1.set(0.0);
  }


  public static void reverse(){
    m_leftDrive.set(-0.5);
    m_rightDrive.set(-0.5);    
  }















  /** This function is called once each time the robot enters test mode. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}
  
}
