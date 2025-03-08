// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  
  public static class OperatorConstants {
    
    /* ================ */
    /* CONTROLLER PORTS */
    /* ================ */
    
    public static final int kDriverControllerPort = 0;

    /* ========= */
    /* Deadbands */
    /* ========= */
    public static final double xDeadband = 0.2;
    public static final double yDeadband = 0.2;
    public static final double rDeadband = 0.3;

    /* ======= */
    /* BUTTONS */
    /* ======= */
    
  }
  
  public static class DrivetrainConstants {
    // Swerve Kinematics, X forward/backward and Y is left/right
    public static final double wheelBase = Units.inchesToMeters(23); // distance between front wheels
    public static final double trackWidth = Units.inchesToMeters(23); // distance between side wheels
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final SwerveDriveKinematics SwerveDriveKinematics = new SwerveDriveKinematics(
      new Translation2d(wheelBase / 2.0, trackWidth / 2.0), // front right (+,+)
      new Translation2d(wheelBase / 2.0, -trackWidth / 2.0), // back right (+,-)
      new Translation2d(-wheelBase / 2.0, trackWidth / 2.0), // front left (-,+)
      new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0) // back left (-,-)
    );

    /* ================== */
    /* CONVERSION FACTORS */
    /* ================== */
    
    // Given motor rotations, convert to meters traveled
    // (1 rev / Gear Ratio) * ((2 * PI * r) / (1 Rev)) = 
    // (2 * PI * r) / (Gear Ratio) = 
    public static final double drivePositionConversionFactor = 0.0472867872;

    // dx/dt
    // Given RPM, convert to m/s
    public static final double driveVelocityConversionFactor = drivePositionConversionFactor / 60.0;

    // Given Motor Rotations, convert to Radians travelled
    // (1 rev / Gear Ratio) * ((2 * PI) RAD / (1 Rev))
    // (2 * PI) RAD / (Gear Ratio)
    public static final double rotationPositionConversionFactor = 0.29321531433;

    // Given RPM, convert to radians/seconds
    public static final double rotationVelocityConversionFactor = rotationPositionConversionFactor / 60.0;

    /* ======== */
    /* MAXIMUMS */
    /* ======== */

    // maximillian
    public static final double maxVelocity = 5; // m/s
    public static final double maxAcceleration = 10; // m/s^2
    public static final double maxAngularVelocity = 2 * Math.PI; // rad/s
    public static final double maxAngularAcceleration = 4 * Math.PI; // rad/s^2
    // Teleop Max Speeds
    public static final double kTeleDriveMaxSpeed = 7.5 / 4.0; // meters/sec
    public static final double kTeleDriveMaxAngularSpeed = 3; // rad/sec

    
    /* ============== */
    /* SWERVE MODULES */
    /* ============== */
    // Front Left Module
    public static final int kFLDrive = 5;
    public static final int kFLRotate = 6;
    public static final int kFLCanCoder = 14;
    public static final double kFLOffsetRad = 0.046143 * 2 * Math.PI;
    public static final boolean fLIsInverted = false;

    // Front Right Module
    public static final int kFRDrive = 3;
    public static final int kFRRotate = 4;
    public static final int kFRCanCoder = 13;
    public static final double kFROffsetRad = 0.031006 * 2 * Math.PI;
    public static final boolean fRIsInverted = true;
    
    // Back Left Module
    public static final int kBLDrive = 7;
    public static final int kBLRotate = 8;
    public static final int kBLCanCoder = 12;
    public static final double kBLOffsetRad = 0.01709 * 2 * Math.PI;
    public static final boolean bLIsInverted = false;

    // Back Right Module
    public static final int kBRDrive = 1;
    public static final int kBRRotate = 2;
    public static final int kBRCanCoder = 11;
    public static final double kBROffsetRad = 0.170166 * 2 * Math.PI;
    public static final boolean bRIsInverted = true;

    /* =============================== */
    /* SWERVE MODULE CONTROL CONSTANTS */
    /* =============================== */

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD  = 0.0;
    public static final double kTolerance = 0.0;
    // public static final SimpleMotorFeedforward driveFF = new SimpleMotorFeedforward(0.2, 2.5, 0.0);
  }
}
