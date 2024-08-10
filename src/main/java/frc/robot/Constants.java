// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */

public final class Constants {

  // OIController
  public static class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final boolean kDriverFieldOriented = true;
    public static final double kDeadband = 0.1;

    public static double deadbandHandler(double value, double deadband) {
      if (Math.abs(value) < deadband) {
        return 0;
      } else if (value > 0) {
        return (value - OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      } else {
        return (value + OIConstants.kDeadband) / (1 - OIConstants.kDeadband);
      }
    }
  }

  // CAN ID
  public static class IDConstants {
    public static final int kFrontLeftDrivePort = 4;
    public static final int kFrontRightDrivePort = 3;
    public static final int kBackLeftDrivePort = 2;
    public static final int kBackRightDrivePort = 1;

    public static final int kFrontLeftTurnPort = 8;
    public static final int kFrontRightTurnPort = 7;
    public static final int kBackLeftTurnPort = 6;
    public static final int kBackRightTurnPort = 5;

    public static final int kIntakeMotorPort = 9;
    public static final int kShooterCrawlMotorPort = 10;
    public static final int kAngleMotorPort = 11;
    public static final int kTopShooterMotorPort = 12;
    public static final int kBottomShooterMotorPort = 13;
    public static final int kAMPArmMotorPort = 14;
    public static final int kAMPCrawlMotorPort = 15;
    public static final int kClimbingRMotorPort = 16;
    public static final int kClimbingLMotorPort = 17;

  }

  // SwerveModule
  public static class ModuleConstants {

    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);
    public static final double kDriveMotorGearRatio = 1.0 / 8.16; // 1.0 / 6.12;
    public static final double kTurningMotorGearRatio = 1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;

     // Used in working code currently
     public static final double kPTurning = 0.5;

     // These two used for simulation currently
     public static final double kITurning = 0.0;
     public static final double kDTurning = 0.005;
  }

  // SwerveDrive
  public static class DriveConstants {

    // Distance between right and left wheels
    public static final double kTrackWidth = 0.640;

    // Distance between front and back wheels
    public static final double kWheelBase = kTrackWidth;

    // Need to update to correct values, I dont remember the value we set last meet
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2), // FL
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2), // FR
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2), // BL
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2)); // BR

    public static final boolean kFrontLeftDriveMotorReversed = true;
    public static final boolean kFrontRightDriveMotorReversed = true;
    public static final boolean kBackLeftDriveMotorReversed = true;
    public static final boolean kBackRightDriveMotorReversed = true;

    public static final boolean kFrontLeftTurningMotorReversed = true;
    public static final boolean kFrontRightTurningMotorReversed = true;
    public static final boolean kBackLeftTurningMotorReversed = true;
    public static final boolean kBackRightTurningMotorReversed = true;

    // -------> ABE <-------- //
    public static final int kFrontLeftDriveAbsoluteEncoderPort = 4;
    public static final int kFrontRightDriveAbsoluteEncoderPort = 3;
    public static final int kBackLeftDriveAbsoluteEncoderPort = 2;
    public static final int kBackRightDriveAbsoluteEncoderPort = 1;

    // public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
    // public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
    // public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
    // public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

    public static final double kPhysicalMaxSpeedMetersPerSecond = 5.5;
    public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * Math.PI;

    public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond;
    public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond;
    public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 8;
    public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 2;

    public static final double kPTheta = 0.012;//0.012;
    public static final double kITheta = 0.01;//0.01;
    public static final double kDTheta = 0.00015;//0.00015;
    public static final double kIZTheta = 60.0;//60.0;

    public static final double kMaxDriveMotorTemp = 33.0;

    public static final double kMotorMaxOutput = 1;

  }

  // Auto
  public static final class AutoConstants {
    public static final double kAutoDriveMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond;

    public static final HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0), // Translation constants
        new PIDConstants(3, 0, 0), // Rotation constants
        kAutoDriveMaxSpeedMetersPerSecond,
        // Drive base radius (distance from center to furthest module)
        new Translation2d(DriveConstants.kWheelBase / 2, DriveConstants.kTrackWidth / 2).getNorm(),
        new ReplanningConfig());
  }

  // Intake
  public static class IntakeConstants {
    public static final double kIntakeMotorRate = 1;
    public static final double kIntakeAutoRate = 0.35;
    public static final double kIntakeGateValue = 100;
  }

  // Shooter
  public static class ShooterConstants {
    public static final double kShooterCrawlMotorRate = 0.35;

    public static final double kShooterMotorGearRatio = 1 / 1;

    // Shooter Motor Speed
    public static final int kShooterMaxRPM = (int) Math.floor(5700 * kShooterMotorGearRatio);
    public static final int kShooterMotorDefaultRPM = 3600;
    
    public static final double kShooteSpeakerTopRate = 1;
    public static final double kShooterSpeakerBottomRate = 1;
    public static final double kShooteStageTopRate = 0.9;
    public static final double kShooterStageBottomRate = 0.7;

    // PID Constants (if needed in the future)
    public static final double kP = 0.0;
    public static final double kI = 0.000000;// 0.0000005;
    public static final double kD = 0.00000;// 0.00001;
    public static final double kIZone = 0.0;// 200;
    public static final double kFF = 0.000;// 0.00018;
    public static final double kRamprate = 50000;

    public enum SpeedSet {
      kSpeak(5000, 3600),
      kStage(3600, 5000),
      kManual(3600, 3600);

      public final int topSpeed;
      public final int bottomSpeed;

      private SpeedSet(int topSpeed, int bottomSpeed) {
        this.topSpeed = topSpeed;
        this.bottomSpeed = bottomSpeed;
      }
    }
  }

  // Shooter Angle
  public static class AngleConstants {
    // Define the gear ratio for the chain sprocket and chainring
    public static final double AnglechainGearRatio = 28.0 / 10.0;
    // Define the gear ratio for the gearbox
    public static final double AnglegearboxRatio = 80.0;
    // Calculate the total gear ratio
    public static final double AngletotalGearRatio = AnglechainGearRatio * AnglegearboxRatio;

    // AnglePID
    public static final double kP = 0.005;
    public static final double kI = 0.000;
    public static final double kD = 0.000001;
    public static final double kIZone = 0.0;
    public static final double kFF = 0.0;
    public static final double kAngleMaxOutput = 0.35;  
    public static final double kAngleMinOutput = -0.35;

    public static final double kAngleMotorRate = 0.35;

    // AnglePosition
    // public static final double kInakePosition = 0.0;
    public static final double kSpeakerPosition = 36.0;
    public static final double kStagePosition = 5.00;
    public static final double kDefultPosition = 0.00;

    // AngleLimit
  public static final double kUPLimit = kSpeakerPosition + 1;
    public static final double kDownLimit = kDefultPosition - 1;

    public enum AngleMode {
      kSpeaker,
      kStage,
      kDefult,
      kUP,
      kDown
    }

  }

  // AMP
  public static class AMPConstants {
    // AMP Arm GearRatio
    public static final double AMPchainGearRatio = 46.0 / 14.0;
    public static final double AMPGearboxRatio = 80;
    public static final double AMPtotalGearRatio = AMPchainGearRatio * AMPGearboxRatio;

    // AMP Arm PID
    // public static final double kP = 0.1;
    // public static final double kI = 0.0;
    // public static final double kD = 0.0;
    // public static final double kIZone = 0.0;
    // public static final double kFF = 0.0;
    // public static final double kArmMaxOutput = 0.35;
    // public static final double kArmMinOutput = -0.35;

    public static final double kAMPArmMotorRate = 0.4;

    // // AMP Arm Position
    // public static final double kAMPPosition = 58.00;
    // public static final double kAMPDefultPosition = 328.00;

     // AMP Arm limit
    public static final double kUPLimit = 114.0;
    public static final double kDownLimit = -0.5;

    // AMP Crawl
    public static final double kAMPCrawlRateFwd = 0.35;
    public static final double kAMPCrawlRateAutoFwd = 0.1;
    public static final double kAMPCrawlRateRev = 0.1;

    public enum ArmMode {
      kAMP,
      kDefult
    }
  }

  // Climbing
  public static class ClimbingConstants {
    public static final double kClimbingLMotorRate = 0.5;
    // public static final double kUPLimit = 0.0;
    // public static final double kDownLimit = 0.0;
  }

  // Limelight
  public static final class LimelightConstants {

    // Limelight Name Mapping
    public enum Limelight {
      // kInatke("limelight-c", -1); // IP: 10.81.69.15
      kShooter("limelight-b", 1); // IP: 10.81.69.13
      

      public final String hostname;
      public final int approachingXSpeed;

      private Limelight(String hostname, int approachingXSpeed) {
        this.hostname = hostname;
        this.approachingXSpeed = approachingXSpeed;
      }
    }
  }
}
