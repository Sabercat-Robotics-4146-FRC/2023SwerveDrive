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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class DrivetrainSubsystem extends SubsystemBase {
    public HolonomicDriveSignal driveSignal = new HolonomicDriveSignal(new Translation2d(), 0, false);

    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] swerveModules;
    public WPI_Pigeon2 gyroscope;

    public boolean driveFlag = true;
    public boolean fieldOriented = true;
    public boolean rotationFlag = false;
    public boolean drivingFlag = false;

    public PIDController driftCorrectionStat;
    public PIDController driftCorrectionRot;

    public double lastPigeonAngle = 0;

    public DrivetrainSubsystem() {
        // init and config gyroscope
        gyroscope = new WPI_Pigeon2(DriveConstants.pigeonID);
        gyroscope.configFactoryDefault();
        gyroscope.reset();

        // init swerve modules
        swerveModules = new SwerveModule[] {
            new SwerveModule("FrontLeft", 0, DriveConstants.FrontLeft.constants),
            new SwerveModule("FrontRight", 1, DriveConstants.FrontRight.constants),
            new SwerveModule("BackLeft", 2, DriveConstants.BackLeft.constants),
            new SwerveModule("BackRight", 3, DriveConstants.BackRight.constants)
        };

        Timer.delay(1.0);
        resetModules();

        swerveOdometry = new SwerveDriveOdometry(DriveConstants.swerveKinematics, gyroscope.getRotation2d(), getModulePositions());

        // set drift correction PID
        driftCorrectionStat = new PIDController(0.05, 0.00, 0.00);
        driftCorrectionStat.enableContinuousInput(-180, 180);
        driftCorrectionRot = new PIDController(0.02, 0.0, 0);
        driftCorrectionRot.enableContinuousInput(-180, 180);

        lastPigeonAngle = gyroscope.getAngle();

        Shuffleboard.getTab("Drive").addNumber("Pose X", () -> getPose().getX());
        Shuffleboard.getTab("Drive").addNumber("Pose Y", () -> getPose().getY());
        Shuffleboard.getTab("Drive").addNumber("PIGEON", () -> gyroscope.getAngle());
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

        rotationFlag = Math.abs(rotationalVelocity) > 0.1;
        drivingFlag = Math.abs(translationalVelocity.getNorm()) > 0.5;
    } 

    public void zeroDrive() {
        drive(new Translation2d(0,0), 0, true);
    }
    
    // drive correction code
    public SwerveModuleState[] driftCorrection(SwerveModuleState[] moduleStates) {
        ChassisSpeeds s = DriveConstants.swerveKinematics.toChassisSpeeds(moduleStates);

        if(!rotationFlag && drivingFlag) {
            s.omegaRadiansPerSecond = 
                -driftCorrectionStat.calculate(gyroscope.getAngle(), lastPigeonAngle)
                * DriveConstants.maxAngularVelocity;
        } 
        // else if (rotationFlag && drivingFlag) {
        //     // double expectedChange = s.omegaRadiansPerSecond * 0.02 * 180/Math.PI;
        //     // s.omegaRadiansPerSecond += 
        //     //     driftCorrectionRot.calculate(gyroscope.getAngle(), (lastPigeonAngle-expectedChange));
        // }
         else {
            lastPigeonAngle = gyroscope.getAngle();
        }

        return DriveConstants.swerveKinematics.toSwerveModuleStates(s);  
    }

    public void resetOdometry(Pose2d pose) {
        swerveOdometry.resetPosition(gyroscope.getRotation2d(), getModulePositions(), pose);
    }

    public void resetModules(){
        for(SwerveModule mod : swerveModules) {mod.resetToAbsolute();}
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
            SmartDashboard.putNumber(mod.moduleName, mod.getState().angle.getDegrees());
            mod.setDesiredState(desiredStates[mod.moduleNumber], false);
        }
    }

    private void update(HolonomicDriveSignal driveSignal) {
        if (driveFlag) {
            Rotation2d rotOffset =
                (fieldOriented)
                    ? Rotation2d.fromDegrees(gyroscope.getAngle())
                    : Rotation2d.fromDegrees(0);

            Translation2d translation = driveSignal.getTranslation().rotateBy(rotOffset);

            ChassisSpeeds speeds = 
                new ChassisSpeeds(
                    translation.getX(), 
                    translation.getY(), 
                    driveSignal.getRotation());

            SmartDashboard.putNumber("x velocity", translation.getX());

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

    public void setBrakeMode() {
        for(SwerveModule module : swerveModules) {
            module.setBrakeMode();
        }
    }

    public void setCoastMode() {
        for(SwerveModule module : swerveModules) {
            module.setCoastMode();
        }
    }

    public void resetGyro180() {
        gyroscope.reset();
        gyroscope.addYaw(180);
    }
}
