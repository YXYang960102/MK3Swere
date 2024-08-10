// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPSubsystem;

public class ArmNormal extends Command {
  private AMPSubsystem ampSubsystem;
  private boolean isUP;
  /** Creates a new ArmNormal. */
  public ArmNormal(AMPSubsystem ampSubsystem, boolean isUP) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.ampSubsystem = ampSubsystem;
    this.isUP = isUP;

    addRequirements(ampSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isUP){
      ampSubsystem.ArmUP();
    }else{
      ampSubsystem.ArmDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ampSubsystem.Armstop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
