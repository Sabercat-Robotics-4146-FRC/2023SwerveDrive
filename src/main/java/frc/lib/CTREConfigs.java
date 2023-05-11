package frc.lib;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;

import frc.robot.Constants.DriveConstants;

public final class CTREConfigs {
    public TalonFXConfiguration swerveAngleFXConfig;
    public TalonFXConfiguration swerveDriveFXConfig;
    public CANCoderConfiguration swerveCanCoderConfig;

    public CTREConfigs(){
        swerveAngleFXConfig = new TalonFXConfiguration();
        swerveDriveFXConfig = new TalonFXConfiguration();
        swerveCanCoderConfig = new CANCoderConfiguration();

        /* Swerve Angle Motor Configurations */
        SupplyCurrentLimitConfiguration angleSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveConstants.angleEnableCurrentLimit, 
            DriveConstants.angleContinuousCurrentLimit, 
            DriveConstants.anglePeakCurrentLimit, 
            DriveConstants.anglePeakCurrentDuration);

        swerveAngleFXConfig.slot0.kP = DriveConstants.anglePIDF[0];
        swerveAngleFXConfig.slot0.kI = DriveConstants.anglePIDF[1];
        swerveAngleFXConfig.slot0.kD = DriveConstants.anglePIDF[2];
        swerveAngleFXConfig.slot0.kF = DriveConstants.anglePIDF[3];
        swerveAngleFXConfig.supplyCurrLimit = angleSupplyLimit;

        /* Swerve Drive Motor Configuration */
        SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
            DriveConstants.driveEnableCurrentLimit, 
            DriveConstants.driveContinuousCurrentLimit, 
            DriveConstants.drivePeakCurrentLimit, 
            DriveConstants.drivePeakCurrentDuration);

        swerveDriveFXConfig.slot0.kP = DriveConstants.drivePIDF[0];
        swerveDriveFXConfig.slot0.kI = DriveConstants.drivePIDF[1];
        swerveDriveFXConfig.slot0.kD = DriveConstants.drivePIDF[2];
        swerveDriveFXConfig.slot0.kF = DriveConstants.drivePIDF[3];        
        swerveDriveFXConfig.supplyCurrLimit = driveSupplyLimit;
        swerveDriveFXConfig.openloopRamp = DriveConstants.openLoopRamp;
        swerveDriveFXConfig.closedloopRamp = DriveConstants.closedLoopRamp;
        
        /* Swerve CANCoder Configuration */
        swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
        swerveCanCoderConfig.sensorDirection = DriveConstants.canCoderInvert;
        swerveCanCoderConfig.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
        swerveCanCoderConfig.sensorTimeBase = SensorTimeBase.PerSecond;
    }
}