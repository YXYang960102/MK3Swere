// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.SwerveDrive;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;


import java.util.function.Supplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj2.command.Command;

public class SwerveDrive extends Command {

  // Create variables
  private final SwerveSubsystem swerveSubsystem;
  private final Supplier<Double> xSpdFunction, ySpdFunction, turningSpdFunction;
  private final SlewRateLimiter xLimiter, yLimiter;
  // private final SlewRateLimiter turnLimiter;

  // Command constructor
  public SwerveDrive(
      SwerveSubsystem swerveSubsystem,
      Supplier<Double> xSpdFunction,
      Supplier<Double> ySpdFunction,
      Supplier<Double> turningSpdFunction) {

    // Assign values passed from constructor
    this.swerveSubsystem = swerveSubsystem;
    this.xSpdFunction = xSpdFunction;
    this.ySpdFunction = ySpdFunction;
    this.turningSpdFunction = turningSpdFunction;

    // Slew rate limiter
    this.xLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    this.yLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAccelerationUnitsPerSecond);
    // this.turningLimiter = new SlewRateLimiter(DriveConstants.kTeleDriveMaxAngularAccelerationUnitsPerSecond);

    // Tell command that it needs swerveSubsystem
    addRequirements(swerveSubsystem);
  }

  @Override
  public void initialize() {
  }

  // Running loop of command
  @Override
  public void execute() {

    // Set joystick inputs to speed variables
    double xSpeed = xSpdFunction.get();
    double ySpeed = ySpdFunction.get();
    double turningAngle = turningSpdFunction.get();

    // Apply deadband to protect motors
    xSpeed = OIConstants.deadbandHandler(xSpeed, OIConstants.kDeadband);
    ySpeed = OIConstants.deadbandHandler(ySpeed, OIConstants.kDeadband);
    // turningAngle = Math.abs(turningAngle) > OIConstants.kDeadband ? turningAngle / (1 - OIConstants.kDeadband) : 0.0;

    // Apply slew rate to joystick input to make robot input smoother and mulitply
    // by max speed
    xSpeed = xLimiter.calculate(xSpeed);
    ySpeed = yLimiter.calculate(ySpeed);
    // turningAngle = turningLimiter.calculate(turningAngle);

    turningAngle *= 10; 

    swerveSubsystem.setChassisOutput(xSpeed, ySpeed, turningAngle);
  }

  // Stop all module motor movement when command ends
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stopModules();
  }

  @Override
  public boolean isFinished() {
    return false;
  }

}