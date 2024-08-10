// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AMP;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPSubsystem;

public class AMPCrawl extends Command {
  private AMPSubsystem AMPSubsystem;
  private boolean isRev;
  /** Creates a new AMPCrawl. */
  public AMPCrawl(AMPSubsystem AMPSubsystem, boolean isRev) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.AMPSubsystem = AMPSubsystem;
    this.isRev = isRev;

    addRequirements(AMPSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isRev){
      AMPSubsystem.AMPCrawlRev();
    }else{
      AMPSubsystem.AMPCrawlFwd();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    AMPSubsystem.AMPCrawlStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
