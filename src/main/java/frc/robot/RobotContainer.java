// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.Constants.AMPConstants.ArmMode;
import frc.robot.Constants.AngleConstants.AngleMode;
import frc.robot.Constants.LimelightConstants.Limelight;
import frc.robot.Constants.AngleConstants.AngleMode;
import frc.robot.Constants.ShooterConstants.SpeedSet;
import frc.robot.commands.SwerveDrive.SwerveDrive;
import frc.robot.commands.SwerveDrive.SwerveLockHeading;
import frc.robot.commands.AMP.AMPCrawl;
// import frc.robot.commands.AMP.ArmAuto;
import frc.robot.commands.AMP.ArmNormal;
import frc.robot.commands.Angle.AngleAuto;
import frc.robot.commands.Angle.AngleNormal;
import frc.robot.commands.Climbing.ClimbingNormal;
import frc.robot.commands.Intake.IntakeAuto;
import frc.robot.commands.Intake.IntakeNormal;
// import frc.robot.commands.Shooter.ShooterAuto;
import frc.robot.commands.Shooter.ShooterCrawl;
import frc.robot.commands.Shooter.ShooterNormal;
import frc.robot.commands.Shooter.ShooterTogether;
import frc.robot.subsystems.AMPSubsystem;
import frc.robot.subsystems.AngleSubsystem;
import frc.robot.subsystems.ClimbingSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterCrawlSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);
  // private final Joystick DriverControllerNC  = new Joystick(OIConstants.kDriverControllerPort);
  private static XboxController operatorControllerNC = new XboxController(OIConstants.kOperatorControllerPort);

  // create subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();
  private final AMPSubsystem ampSubsystem = new AMPSubsystem();
  private final ShooterSubsystem shooterSubsystem = new ShooterSubsystem();
  private final ShooterCrawlSubsystem shooterCrawlSubsystem = new ShooterCrawlSubsystem();
  private final AngleSubsystem angleSubsystem = new AngleSubsystem();
  private final ClimbingSubsystem climbingSubsystem = new ClimbingSubsystem();


  // Create auto chooser
  private final SendableChooser<Command> autoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    configureNamedCommands();
    autoChooser = AutoBuilder.buildAutoChooser(); // Default auto will be `Commands.none()`
    SmartDashboard.putData("Auto Mode", autoChooser);


    // Configure the trigger bindings
    configureButtonBindings();
    setDefaultCommand();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureButtonBindings() {
    
    
    // Intake Rev
    // m_driverController.x().whileTrue(new IntakeNormal(intakeSubsystem, false));
  
   // Swerve Lock Heading AprilTag
   m_driverController.leftBumper().whileTrue(new SwerveLockHeading(swerveSubsystem,
   () -> -m_driverController.getLeftY(), // X-Axis
   () -> m_driverController.getLeftX(), // Y-Axis
   () -> -m_driverController.getRightX(), // R-Axis
   Limelight.kShooter));

    // Intake & AMP Crawl 
    m_driverController.x().toggleOnTrue(new IntakeAuto(intakeSubsystem, ampSubsystem));


    //Climbing
    m_driverController.pov(0).whileTrue( new ClimbingNormal(climbingSubsystem, true));
    m_driverController.pov(180).whileTrue(new ClimbingNormal(climbingSubsystem, false));


    // Arm Normal
    m_operatorController.pov(0).whileTrue(new ArmNormal(ampSubsystem, true));
    m_operatorController.pov(180).whileTrue(new ArmNormal(ampSubsystem, false));

    //AMP Crawl
    m_operatorController.rightBumper().whileTrue(new AMPCrawl(ampSubsystem, false));

    //Shooter Crawl Normal
    m_operatorController.b().whileTrue(new ShooterCrawl(shooterCrawlSubsystem, false));

    //Shooter Together
    m_operatorController.leftTrigger().whileTrue(new ShooterTogether(shooterCrawlSubsystem, ampSubsystem, intakeSubsystem, true));
  

    //AngleNormal
    m_operatorController.pov(90).whileTrue(new AngleNormal(angleSubsystem, AngleMode.kUP));
    m_operatorController.pov(270).whileTrue(new AngleNormal(angleSubsystem, AngleMode.kDown));

    // //AngleAuto
    // m_operatorController.a().onTrue(new AngleAuto(angleSubsystem, AngleMode.kSpeaker));
    // m_operatorController.x().onTrue(new AngleAuto(angleSubsystem, AngleMode.kDefult));

    m_operatorController.y().toggleOnTrue(new ShooterNormal(shooterSubsystem, true, false));
    m_operatorController.x().toggleOnTrue(new ShooterNormal(shooterSubsystem, false, true));
  }

  private void setDefaultCommand() {
    swerveSubsystem.setDefaultCommand(new SwerveDrive(swerveSubsystem,
    () -> -m_driverController.getLeftY(), // X-Axis
    () -> m_driverController.getLeftX(), // Y-Axis
    () -> -m_driverController.getRightX() // R-Axis
    ));

  }

  private void configureNamedCommands() {
    NamedCommands.registerCommand("Intake Auto", new IntakeAuto(intakeSubsystem, ampSubsystem));
    NamedCommands.registerCommand("Shooter Speaker", new ShooterNormal(shooterSubsystem, true, false));
    NamedCommands.registerCommand("Shooter Stage", new ShooterNormal(shooterSubsystem, false, true));
    NamedCommands.registerCommand("Shooter Together", new ShooterTogether(shooterCrawlSubsystem, ampSubsystem, intakeSubsystem, true));

    // NamedCommands.registerCommand("Angle Speaker", new AngleAuto(angleSubsystem, AngleMode.kSpeaker));
    // NamedCommands.registerCommand("Angle Stage", new AngleAuto(angleSubsystem, AngleMode.kStage));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}
