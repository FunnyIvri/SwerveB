// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public final class Constants {
  public static final class DriveConstants {
    // Driving Parameters
    public static final double kMaxSpeedMetersPerSecond = 4.5;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration (12.1387795276 inches * 2 = 24.2775590552 inches)
    public static final double kTrackWidth = Units.inchesToMeters(24.2775590552);
    public static final double kWheelBase = Units.inchesToMeters(24.2775590552);

    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians (from YAGSL JSONs)
    public static final double kFrontLeftChassisAngularOffset = Units.degreesToRadians(52.38);
    public static final double kFrontRightChassisAngularOffset = Units.degreesToRadians(103.271);
    public static final double kBackLeftChassisAngularOffset = Units.degreesToRadians(-60.9375);
    public static final double kBackRightChassisAngularOffset = Units.degreesToRadians(-7.44);

    // SPARK MAX Driving CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kFrontRightDrivingCanId = 4;
    public static final int kRearLeftDrivingCanId = 10;
    public static final int kRearRightDrivingCanId = 7;

    // SPARK MAX Turning CAN IDs
    public static final int kFrontLeftTurningCanId = 2;
    public static final int kFrontRightTurningCanId = 5;
    public static final int kRearLeftTurningCanId = 11;
    public static final int kRearRightTurningCanId = 8;

    // Talon SRX Turning Encoder CAN IDs
    public static final int kFrontLeftEncoderTalonCanId = 3;
    public static final int kFrontRightEncoderTalonCanId = 6;
    public static final int kRearLeftEncoderTalonCanId = 12;
    public static final int kRearRightEncoderTalonCanId = 9;

    // Pigeon 1 CAN ID & Inversion
    public static final int kPigeonCanId = 0;
    public static final boolean kGyroReversed = false;
  }

  public static final class ModuleConstants {
    // MK4i L1 Gear Ratios from physicalproperties.json
    public static final double kDrivingMotorReduction = 8.14;
    public static final double kTurningMotorReduction = 21.4285714286; // 150.0 / 7.0

    // 4-inch wheels from physicalproperties.json
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4.0);
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;

    // NEO free speed = 5676 RPM
    public static final double kDrivingMotorFreeSpeedRps = 5676.0 / 60.0;
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final double kDriveDeadband = 0.05;
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3.0;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3.0;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1.0;
    public static final double kPYController = 1.0;
    public static final double kPThetaController = 1.0;

    public static final TrapezoidProfile.Constraints kThetaControllerConstraints =
        new TrapezoidProfile.Constraints(
            kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }
}
