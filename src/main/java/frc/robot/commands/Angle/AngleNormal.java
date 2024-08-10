// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants.AngleMode;
import frc.robot.subsystems.AngleSubsystem;

public class AngleNormal extends Command {
  AngleSubsystem angleSubsystem;
  AngleMode angleMode;
  /** Creates a new AngleNormal. */
  public AngleNormal(AngleSubsystem angleSubsystem, AngleMode angleMode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSubsystem = angleSubsystem;
    this.angleMode = angleMode;
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(angleMode == AngleMode.kUP)
      angleSubsystem.AngleUP();
    if(angleMode == AngleMode.kDown)
      angleSubsystem.AngleDown();
      
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angleSubsystem.stop();
    // angleSubsystem.hold();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
