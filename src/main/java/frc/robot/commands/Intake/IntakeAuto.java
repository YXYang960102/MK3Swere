// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.AMPSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeAuto extends Command {
  private IntakeSubsystem intakeSubsystem;
  private AMPSubsystem AMPSubsystem;
  private boolean pass = false;
  private boolean delayed = false;
  private boolean reversed = false;
  Timer time = new Timer();
  Timer waitTimer = new Timer();

  /** Creates a new IntakeAuto. */
  public IntakeAuto(IntakeSubsystem intakeSubsystem, AMPSubsystem AMPSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intakeSubsystem = intakeSubsystem;
    this.AMPSubsystem = AMPSubsystem;

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intakeSubsystem.IntakeFwd();
    AMPSubsystem.AMPCrawlAutoFwd();
    time.reset();
    waitTimer.reset();
    pass = false;
    delayed = false;
    reversed = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (!pass && intakeSubsystem.isPass() && time.get() == 0) {
      time.start();
      waitTimer.start();

      pass = true;
    }

    if (pass && !delayed && waitTimer.get() >= 0.2   ) {
      intakeSubsystem.IntakeStop();
      AMPSubsystem.AMPCrawlStop();
      delayed = true;
      time.reset();
      time.start();
    }

    if (delayed && !reversed && time.get() >= 0.1) {
      AMPSubsystem.AMPCrawlRev();
      intakeSubsystem.IntakeAutoRev();
      reversed = true;
      time.reset();
      time.start();
    }

    if (reversed && time.get() >= 0.1) {
      AMPSubsystem.AMPCrawlStop();
    }
  }


  

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    waitTimer.stop();
    intakeSubsystem.IntakeStop();
    AMPSubsystem.AMPCrawlStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return reversed && time.get() > 0.2;
  }

}
