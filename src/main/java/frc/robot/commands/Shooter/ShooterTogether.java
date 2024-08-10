// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCrawlSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterTogether extends Command {
  // private ShooterSubsystem shooterSubsystem;
  private ShooterCrawlSubsystem shooterCrawlSubsystem;
  private AMPSubsystem ampSubsystem;
  private IntakeSubsystem intakeSubsystem;
  private boolean isTogether;

  /** Creates a new ShooterTogether. */
  public ShooterTogether(
      ShooterCrawlSubsystem shooterCrawlSubsystem,
      AMPSubsystem ampSubsystem,
      IntakeSubsystem intakeSubsystem,
      boolean isTogether) {
    // Use addRequirements() here to declare subsystem dependencies.
    // this.shooterSubsystem = shooterSubsystem;
    this.shooterCrawlSubsystem = shooterCrawlSubsystem;
    this.ampSubsystem = ampSubsystem;
    this.intakeSubsystem = intakeSubsystem;
    this.isTogether = isTogether;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(isTogether) {
      // shooterSubsystem.NormalShoote();
      shooterCrawlSubsystem.CrawlFwd();
      ampSubsystem.AMPCrawlFwd();
      intakeSubsystem.IntakeFwd();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // shooterSubsystem.stop();
    shooterCrawlSubsystem.CrawlStop();
    ampSubsystem.AMPCrawlStop();
    intakeSubsystem.IntakeStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
