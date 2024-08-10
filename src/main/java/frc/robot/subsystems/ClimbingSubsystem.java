// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimbingConstants;
import frc.robot.Constants.IDConstants;

public class ClimbingSubsystem extends SubsystemBase {
  private CANSparkMax ClimbingRMotor = new CANSparkMax(IDConstants.kClimbingRMotorPort, MotorType.kBrushless);
  private CANSparkMax ClimbingLMotor = new CANSparkMax(IDConstants.kClimbingLMotorPort, MotorType.kBrushless);

  /** Creates a newClimbingSubsystem. */
  public ClimbingSubsystem() {
    ClimbingRMotor.setInverted(true);
    ClimbingRMotor.setIdleMode(IdleMode.kBrake);

    ClimbingLMotor.setInverted(true);
    ClimbingLMotor.setIdleMode(IdleMode.kBrake);

    ClimbingLMotor.follow(ClimbingRMotor, true);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("Hanging Position", getHangingPosition());
  }


  public void ClimbUP() {
    ClimbingRMotor.set(ClimbingConstants.kClimbingLMotorRate);
  }

  public void ClimbDown() {
    ClimbingRMotor.set(-ClimbingConstants.kClimbingLMotorRate);
  }

  public void ClimbStop() {
    ClimbingRMotor.set(0);
  }
}
