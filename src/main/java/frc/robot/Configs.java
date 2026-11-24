// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = (2 * Math.PI) / ModuleConstants.kTurningMotorReduction;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(false) // From module JSONs: "drive": false
                    .smartCurrentLimit(80) // From physicalproperties.json: "drive": 80
                    .closedLoopRampRate(0.25) // From physicalproperties.json: "drive": 0.25
                    .openLoopRampRate(0.25);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    .pid(0.020645, 0, 0) // From pidfproperties.json
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .inverted(true) // From module JSONs: "angle": true
                    .smartCurrentLimit(35) // From physicalproperties.json: "angle": 35
                    .closedLoopRampRate(0.25) // From physicalproperties.json: "angle": 0.25
                    .openLoopRampRate(0.25);
            turningConfig.encoder
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // Converted from YAGSL degree-based P (0.01) to radian-based P (~0.57)
                    .pid(0.57, 0, 0)
                    .outputRange(-1, 1)
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, 2 * Math.PI);
        }
    }
}
