// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDConstants;
import frc.robot.Constants.ShooterConstants;

public class ShooterCrawlSubsystem extends SubsystemBase {
  private CANSparkMax ShooterCrawlMotor = new CANSparkMax(IDConstants.kShooterCrawlMotorPort, MotorType.kBrushless);

  /** Creates a new ShooterCrawlSubsystem. */
  public ShooterCrawlSubsystem() {
    ShooterCrawlMotor.setInverted(true);
    ShooterCrawlMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void CrawlFwd() {
    ShooterCrawlMotor.set(ShooterConstants.kShooterCrawlMotorRate);
  }

  public void CrawlRev() {
    ShooterCrawlMotor.set(-ShooterConstants.kShooterCrawlMotorRate);
  }

  public void CrawlStop() {
    ShooterCrawlMotor.set(0);
  }
}
