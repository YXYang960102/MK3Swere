// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.util.struct.StructDescriptorDatabase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants;
import frc.robot.Constants.ShooterConstants.SpeedSet;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterNormal extends Command {
  private ShooterSubsystem shooterSubsystem;
   private boolean isShooteSpeaker;
  private boolean isShooteStage; 
  // private BooleanSupplier OnStop;
  // private SpeedSet speed;
  /** Creates a new ShooterNormal. */
  public ShooterNormal(ShooterSubsystem shooterSubsystem, boolean isShooteSpeaker, boolean isShooteStage) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooterSubsystem = shooterSubsystem;
    this.isShooteSpeaker = isShooteSpeaker;
    this.isShooteStage = isShooteStage;
    // this.OnStop = OnStop;
    // this.speed = speed;

    addRequirements(shooterSubsystem);


    // SmartDashboard.putNumber("Shooter Set RPM T",
    //     SmartDashboard.getNumber("Shooter Set RPM T", ShooterConstants.kShooterMotorDefaultRPM));
    // SmartDashboard.putNumber("Shooter Set RPM B",
    //     SmartDashboard.getNumber("Shooter Set RPM B", ShooterConstants.kShooterMotorDefaultRPM));
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // shooterSubsystem.setSpeed(speed.topSpeed, speed.bottomSpeed);
    if(isShooteSpeaker) 
      shooterSubsystem.ShooteSpeaker();
    if(isShooteStage)
      shooterSubsystem.ShooteStage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if(speed == SpeedSet.kManual) {
    //   int rpmT = (int) SmartDashboard.getNumber("Shooter set RPM T", ShooterConstants.kShooterMotorDefaultRPM);
    //   int rpmB = (int) SmartDashboard.getNumber("Shooter Set RPM B", ShooterConstants.kShooterMotorDefaultRPM);
    //   shooterSubsystem.setSpeed(rpmT, rpmB);
    // }
  }
  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
     return false; //OnStop != null ? OnStop.getAsBoolean() : false;
  }
}
