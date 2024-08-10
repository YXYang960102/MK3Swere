// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AngleConstants;
import frc.robot.Constants.IDConstants;

public class AngleSubsystem extends SubsystemBase {
  private CANSparkMax AngleMotor = new CANSparkMax(IDConstants.kAngleMotorPort, MotorType.kBrushless);
  private RelativeEncoder AngleEncoder = AngleMotor.getEncoder();
  // private SparkAbsoluteEncoder AngleAbsEncoder = AngleMotor.getAbsoluteEncoder(Type.kDutyCycle);
  private SparkPIDController AnglePIDcontroller = AngleMotor.getPIDController();

  public double kP, kI, kD, kIZone, kFF, kAngleMaxOutput, kAngleMinOutput;

  /** Creates a new AngleSubsystem. */
  public AngleSubsystem() {
    AngleMotor.setInverted(true);
    AngleMotor.setIdleMode(IdleMode.kBrake);

    // AngleEncoder.setPosition(AngleAbsEncoder.getPosition());

    AngleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, true);
    AngleMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);

    AngleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float) AngleConstants.kUPLimit);
    AngleMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float) AngleConstants.kDownLimit);

    kP = AngleConstants.kP;
    kI = AngleConstants.kI;
    kD = AngleConstants.kD;
    kIZone = AngleConstants.kIZone;
    kFF = AngleConstants.kFF;
    kAngleMaxOutput = AngleConstants.kAngleMaxOutput;
    kAngleMinOutput = AngleConstants.kAngleMinOutput;

    SmartDashboard.putNumber("Angle P Gain", kP);
    SmartDashboard.putNumber("Angle I Gain", kI);
    SmartDashboard.putNumber("Angle D Gain", kD);
    SmartDashboard.putNumber("Angle I Zone", kIZone);
    SmartDashboard.putNumber("Angle Feed Forward", kFF);
    SmartDashboard.putNumber("Angle Max Output", kAngleMaxOutput);
    SmartDashboard.putNumber("Angle Min Output", kAngleMinOutput);

    // set Angle Spark PID
    AnglePIDcontroller.setP(kP);
    AnglePIDcontroller.setI(kI);
    AnglePIDcontroller.setD(kD);
    AnglePIDcontroller.setIZone(kIZone);
    AnglePIDcontroller.setFF(kFF);
    AnglePIDcontroller.setOutputRange(kAngleMaxOutput, kAngleMinOutput);
    AnglePIDcontroller.setFeedbackDevice(AngleEncoder);
    AngleMotor.burnFlash();

  }

  // public double getAngleAbsolutePosition() {
  //   return AngleAbsEncoder.getPosition();
  // }

  public double getAnglePosition() {
    return AngleEncoder.getPosition();
  }

  @Override
  public void periodic() {
    // SmartDashboard.putNumber("Angle Abs Position", getAngleAbsolutePosition());
     SmartDashboard.putNumber("Angle Position", getAnglePosition());
    // This method will be called once per scheduler run

    double p = SmartDashboard.getNumber("Angle P ", 0);
    double i = SmartDashboard.getNumber("Angle I ", 0);
    double d = SmartDashboard.getNumber("Angle D ", 0);
    double iz = SmartDashboard.getNumber("Angle I Zone ", 0);
    double ff = SmartDashboard.getNumber("Angle Feed Forward ", 0);
    double max = SmartDashboard.getNumber("Angle Max Output", 0);
    double min = SmartDashboard.getNumber("Angle Min Output", 0);

    if ((p != kP)) {
      AnglePIDcontroller.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      AnglePIDcontroller.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      AnglePIDcontroller.setD(d);
      kD = d;
    }
    if ((iz != kIZone)) {
      AnglePIDcontroller.setIZone(iz);
      kIZone = iz;
    }
    if ((ff != kFF)) {
      AnglePIDcontroller.setFF(ff);
      kFF = ff;
    }
    if ((max != kAngleMaxOutput) || (min != kAngleMinOutput)) {
      AnglePIDcontroller.setOutputRange(min, max);
      kAngleMinOutput = min;
      kAngleMaxOutput = max;
    }
  }


  public void setSpeaker() {
    AnglePIDcontroller.setReference(AngleConstants.kSpeakerPosition, ControlType.kDutyCycle);
  }

  public void setStage() {
    AnglePIDcontroller.setReference(AngleConstants.kStagePosition, ControlType.kDutyCycle);
  }

  public void setDefult() {
    AnglePIDcontroller.setReference(AngleConstants.kDefultPosition, ControlType.kDutyCycle);
  }

  public void AngleUP() {
    AngleMotor.set(AngleConstants.kAngleMotorRate);
  }

  public void AngleDown() {
    AngleMotor.set(-AngleConstants.kAngleMotorRate);
  }

  public void stop() {
    AngleMotor.set(0);
  }

  public void hold() {
    AnglePIDcontroller.setReference(getAnglePosition(), ControlType.kPosition);
  }
}
