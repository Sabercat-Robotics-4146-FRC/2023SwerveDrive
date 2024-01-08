package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.util.COTSFalconSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.01;

    public static final class DriveConstants {
        public static final int pigeonID = 20;

        public static final COTSFalconSwerveConstants chosenModule = 
            COTSFalconSwerveConstants.SDSMK4(COTSFalconSwerveConstants.driveGearRatios.SDSMK4_L2);

        public static final double trackWidth = Units.inchesToMeters(20.0); 
        public static final double wheelBase = Units.inchesToMeters(21.0); 
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        public static final boolean canCoderInvert = chosenModule.canCoderInvert;

        public static final int angleContinuousCurrentLimit = 25;
        public static final int anglePeakCurrentLimit = 40;
        public static final double anglePeakCurrentDuration = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveContinuousCurrentLimit = 35;
        public static final int drivePeakCurrentLimit = 50;
        public static final double drivePeakCurrentDuration = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        public static final double openLoopRamp = 0.5;
        public static final double closedLoopRamp = 0.0;

        public static final double[] anglePIDF = 
            {chosenModule.angleKP,chosenModule.angleKI,chosenModule.angleKD,chosenModule.angleKF};

        public static final double[] drivePIDF = 
            {0.05, 0.0, 0.0, 0.0};
        
        public static final double[] driveSVA = 
            {(0.70067 / 12), (2.2741 / 12), (0.16779 / 12)};

        public static final double maxSpeed = 4.5; 
        public static final double maxAngularVelocity = 5.0; 

        public static final NeutralMode angleNeutralMode = NeutralMode.Coast;
        public static final NeutralMode driveNeutralMode = NeutralMode.Brake;

        public static final class FrontLeft {
            public static final int driveMotorID = 4;
            public static final int angleMotorID = 5;
            public static final int canCoderID = 14;
            public static final Rotation2d angleOffset = 
                Rotation2d.fromDegrees(58.2+2.62  +180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }

        public static final class FrontRight { 
            public static final int driveMotorID = 2;
            public static final int angleMotorID = 3;
            public static final int canCoderID = 12;
            public static final Rotation2d angleOffset = 
                Rotation2d.fromDegrees(104.4-0.68);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
        

        public static final class BackLeft { 
            public static final int driveMotorID = 6;
            public static final int angleMotorID = 7;
            public static final int canCoderID = 16;
            public static final Rotation2d angleOffset = 
                Rotation2d.fromDegrees(145.7+3.18  +180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }


        public static final class BackRight { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 18;
            public static final Rotation2d angleOffset = 
                Rotation2d.fromDegrees(152.58+2 + 180);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, angleOffset);
        }
    }
}
