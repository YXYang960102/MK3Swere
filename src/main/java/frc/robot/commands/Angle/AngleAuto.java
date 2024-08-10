// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Angle;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.AngleConstants.AngleMode;
import frc.robot.subsystems.AngleSubsystem;

public class AngleAuto extends Command {
  private AngleSubsystem angleSubsystem;
  private AngleMode mode;

  /** Creates a new AngleAuto. */
  public AngleAuto(AngleSubsystem angleSubsystem, AngleMode mode) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angleSubsystem = angleSubsystem;
    this.mode = mode;

    addRequirements(angleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mode == AngleMode.kDefult)
      angleSubsystem.setDefult();
    if (mode == AngleMode.kSpeaker)
      angleSubsystem.setSpeaker();
    if (mode == AngleMode.kStage)
      angleSubsystem.setStage();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
