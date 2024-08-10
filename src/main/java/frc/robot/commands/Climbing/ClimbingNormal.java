// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climbing;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbingSubsystem;

public class ClimbingNormal extends Command {
  private ClimbingSubsystem climbingSubsystem;
  private boolean isUP;

  /** Creates a new ClimbingNormal. */
  public ClimbingNormal(ClimbingSubsystem climbingSubsystem, boolean isUP) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climbingSubsystem = climbingSubsystem;
    this.isUP = isUP;

    addRequirements(climbingSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isUP){
      climbingSubsystem.ClimbUP();
    }else{
      climbingSubsystem.ClimbDown();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbingSubsystem.ClimbStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
