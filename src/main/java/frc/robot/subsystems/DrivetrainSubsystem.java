package frc.robot.subsystems;

import frc.robot.Constants.DriveConstants;
import frc.lib.SwerveModule;
import frc.lib.util.HolonomicDriveSignal;


import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.Arrays;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class DrivetrainSubsystem extends SubsystemBase {
    public HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(), 0, false);

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public WPI_Pigeon2 gyroscope;

    public boolean driveFlag = true;
    public boolean fieldOriented = false;

    public PIDController driftCorrectionStat;
    public PIDController driftCorrectionRot;

    public double lastPigeonAngle;

    public DrivetrainSubsystem() {
        // init and config gyroscope
        gyroscope = new WPI_Pigeon2(DriveConstants.pigeonID);
        gyroscope.configFactoryDefault();
        gyroscope.reset();

        // init swerve modules
        swerveModules = new SwerveModule[] {
            new SwerveModule(0, DriveConstants.FrontLeft.constants),
            new SwerveModule(1, DriveConstants.FrontRight.constants),
            new SwerveModule(2, DriveConstants.BackLeft.constants),
            new SwerveModule(3, DriveConstants.BackRight.constants)
        };

        Timer.delay(1.0);
        resetModules();

        swerveOdometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, gyroscope.getRotation2d(), getModulePositions());

        // set drift correction PID
        driftCorrectionStat = new PIDController(0.04, 0.0, 0);
        driftCorrectionStat.enableContinuousInput(-180, 180);
        driftCorrectionRot = new PIDController(0.2, 0.0, 0);
        driftCorrectionRot.enableContinuousInput(-180, 180);

        lastPigeonAngle = gyroscope.getAngle();
    }

    /**
     * Scale input drive values according to max speed and angular velocity.
     * @param translationalVelocity translational velocity 
     * @param rotationalVelocity rotational velocity
     */
    public void driveRelative(Translation2d translationalVelocity, double rotationalVelocity) {
        drive(translationalVelocity.times(DriveConstants.maxSpeed), rotationalVelocity*DriveConstants.maxAngularVelocity);
    }

    // drive 
    public void drive(Translation2d translationalVelocity, double rotationalVelocity) {
        drive(translationalVelocity, rotationalVelocity, fieldOriented);
    }  
    
    // sets drive signal with the translational and rotational velocities.
    public void drive(Translation2d translationalVelocity, double rotationalVelocity, boolean fieldOriented) {
        driveSignal = new HolonomicDriveSignal(translationalVelocity, rotationalVelocity, fieldOriented);
    } 
    
    // drive correction code
    public SwerveModuleState[] driftCorrection(SwerveModuleState[] moduleStates) {
        ChassisSpeeds s = DriveConstants.swerveKinematics.toChassisSpeeds(moduleStates);
        if(Math.abs(s.vxMetersPerSecond) + Math.abs(s.vyMetersPerSecond) >= 0.1) {
            if (s.omegaRadiansPerSecond == 0) {
                s.omegaRadiansPerSecond = 
                    driftCorrectionStat.calculate(gyroscope.getAngle() % 360, lastPigeonAngle % 360) 
                    * DriveConstants.maxAngularVelocity;
            }
            if(Math.abs(s.omegaRadiansPerSecond) >= 0.01) {
                double expectedChange = s.omegaRadiansPerSecond * 0.02 * 180/Math.PI;
                s.omegaRadiansPerSecond -= 
                    driftCorrectionRot.calculate(gyroscope.getAngle() % 360, (lastPigeonAngle-expectedChange) % 360);
            }
        }
        lastPigeonAngle = gyroscope.getAngle();

        return DriveConstants.swerveKinematics.toSwerveModuleStates(s);  
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(gyroscope.getRotation2d(), getModulePositions(), pose);
    }

    public void resetModules(){
        for(SwerveModule mod : swerveModules){ mod.resetToAbsolute();}
    }

    public SwerveModuleState[] getModuleStates(){
        return Arrays.stream(swerveModules).map(
            m -> m.getState()).toArray(SwerveModuleState[]::new);
    }
    
    public SwerveModulePosition[] getModulePositions(){
        return Arrays.stream(swerveModules).map(
            m -> m.getPosition()).toArray(SwerveModulePosition[]::new);
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public WPI_Pigeon2 getGyro() {
        return gyroscope;
    }

    public void updateModules(SwerveModuleState[] desiredStates) {
        desiredStates = driftCorrection(desiredStates);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.maxSpeed);
        for(SwerveModule mod : swerveModules){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    private void update(HolonomicDriveSignal driveSignal) {
        if (driveFlag) {
            ChassisSpeeds speeds = driveSignal.isFieldOriented() ? 
                ChassisSpeeds.fromFieldRelativeSpeeds(
                    driveSignal.getTranslation().getX(), 
                    driveSignal.getTranslation().getY(), 
                    driveSignal.getRotation(),
                    gyroscope.getRotation2d()) : 
                new ChassisSpeeds(
                    driveSignal.getTranslation().getX(), 
                    driveSignal.getTranslation().getY(), 
                    driveSignal.getRotation());
      
          SwerveModuleState[] moduleStates = DriveConstants.swerveKinematics.toSwerveModuleStates(speeds);
          updateModules(moduleStates);
        }
    }

    @Override
    public void periodic(){
        swerveOdometry.update(gyroscope.getRotation2d(), getModulePositions()); 
        update(driveSignal);
    }

    public void toggleFieldOriented() {
        fieldOriented = !fieldOriented;
    }
}