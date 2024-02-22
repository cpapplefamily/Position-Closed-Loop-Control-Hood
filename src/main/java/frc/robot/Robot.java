/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;



public class Robot extends TimedRobot {
  private static final int deviceID = 11;
  private CANSparkFlex m_motor;
  private SparkPIDController m_pidController;
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput;
  private XboxController m_xbox;
  private AbsoluteEncoder m_alternateEncoder;

  @Override
  public void robotInit() {
    m_xbox = new XboxController(0);
    // initialize motor
    m_motor = new CANSparkFlex(deviceID, MotorType.kBrushless);
    
    /**
     * The restoreFactoryDefaults method can be used to reset the configuration parameters
     * in the SPARK MAX to their factory default state. If no argument is passed, these
     * parameters will not persist between power cycles
     */
    m_motor.restoreFactoryDefaults();
    
    m_motor.setInverted(true);

    m_alternateEncoder = m_motor.getAbsoluteEncoder(Type.kDutyCycle);
    
    /**
     * In order to use PID functionality for a controller, a SparkPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_motor.getPIDController();
  
    /**
     * By default, the PID controller will use the Hall sensor from a NEO for its
     * feedback device. Instead, we can set the feedback device to the alternate
     * encoder object
     */
    m_pidController.setFeedbackDevice(m_alternateEncoder);

    /**
     * From here on out, code looks exactly like running PID control with the 
     * built-in NEO encoder, but feedback will come from the alternate encoder
     */ 

    // PID coefficients
    kP = 3.5; 
    kI = 0.0;
    kD = 50; 
    kIz = 0; 
    kFF = 0; 
    kMaxOutput = .5; 
    kMinOutput = -.5;

    // set PID coefficients
    m_pidController.setP(kP);
    m_pidController.setI(kI);
    m_pidController.setD(kD);
    m_pidController.setIZone(kIz);
    m_pidController.setFF(kFF);
    m_pidController.setOutputRange(kMinOutput, kMaxOutput);

    // display PID coefficients on SmartDashboard
    SmartDashboard.putNumber("P Gain", kP);
    SmartDashboard.putNumber("I Gain", kI);
    SmartDashboard.putNumber("D Gain", kD);
    SmartDashboard.putNumber("I Zone", kIz);
    SmartDashboard.putNumber("Feed Forward", kFF);
    SmartDashboard.putNumber("Max Output", kMaxOutput);
    SmartDashboard.putNumber("Min Output", kMinOutput);
    SmartDashboard.putNumber("Set Rotations", 0);
    SmartDashboard.putNumber("Set deg", 0);
    SmartDashboard.putBoolean("Closed Loop", false);
    m_motor.burnFlash();
  }

  @Override
  public void teleopPeriodic() {
    // read PID coefficients from SmartDashboard
    double p = SmartDashboard.getNumber("P Gain", 0);
    double i = SmartDashboard.getNumber("I Gain", 0);
    double d = SmartDashboard.getNumber("D Gain", 0);
    double iz = SmartDashboard.getNumber("I Zone", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);
    double max = SmartDashboard.getNumber("Max Output", 0);
    double min = SmartDashboard.getNumber("Min Output", 0);
    
    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { m_pidController.setP(p); kP = p; }
    if((i != kI)) { m_pidController.setI(i); kI = i; }
    if((d != kD)) { m_pidController.setD(d); kD = d; }
    if((iz != kIz)) { m_pidController.setIZone(iz); kIz = iz; }
    if((ff != kFF)) { m_pidController.setFF(ff); kFF = ff; }
    if((max != kMaxOutput) || (min != kMinOutput)) { 
      m_pidController.setOutputRange(min, max); 
      kMinOutput = min; kMaxOutput = max; 
    }
    
    /**
     * PIDController objects are commanded to a set point using the 
     * SetReference() method.
     * 
     * The first parameter is the value of the set point, whose units vary
     * depending on the control type set in the second parameter.
     * 
     * The second parameter is the control type can be set to one of four 
     * parameters:
     *  com.revrobotics.CANSparkMax.ControlType.kDutyCycle
     *  com.revrobotics.CANSparkMax.ControlType.kPosition
     *  com.revrobotics.CANSparkMax.ControlType.kVelocity
     *  com.revrobotics.CANSparkMax.ControlType.kVoltage
     */
    double encoderOffset = .015;
    double deg = SmartDashboard.getNumber("Set deg", 0);
    double rotations = (deg/360)+encoderOffset;//SmartDashboard.getNumber("Set Rotations", 0);
    
    if(SmartDashboard.getBoolean("Closed Loop", false)){
      if(m_xbox.getAButton()){
        deg = 0.0;
        rotations = (deg/360)+encoderOffset;
        System.out.println(rotations);
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      }
      if(m_xbox.getYButton()){
        deg = 60;
        rotations = (deg/360)+encoderOffset;
        System.out.println(rotations);
        m_pidController.setReference(rotations, CANSparkMax.ControlType.kPosition);
      }
    }else{
      m_motor.set(MathUtil.applyDeadband(-m_xbox.getLeftY(), .1));
    }
    
    SmartDashboard.putNumber("Motor Output", m_motor.getAppliedOutput());
    SmartDashboard.putNumber("SetPoint", rotations);
    SmartDashboard.putNumber("Encoder Position", m_alternateEncoder.getPosition());
    SmartDashboard.putBoolean("Is inverted", m_motor.getInverted());
  }
}
