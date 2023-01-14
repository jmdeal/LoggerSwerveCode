package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveDrivetrainConstants;
import frc.robot.util.LoggedTunableNumber;

public class SwerveDrivetrain extends SubsystemBase {
    public SwerveDriveOdometry m_swerveOdometry;
    public SwerveModule[] m_swerveModules;
    public Pigeon2 m_pigeonGyro;
    
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();

    private final SwerveModuleIO[] moduleIOs = new SwerveModuleIO[4]; // FL, FR, BL, BR
    private final ModuleIOInputsAutoLogged[] moduleInputs = new ModuleIOInputsAutoLogged[] {
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
            new ModuleIOInputsAutoLogged(),
    };

    private final LoggedTunableNumber driveKp = new LoggedTunableNumber("Drive/DriveKp");
    private final LoggedTunableNumber driveKd = new LoggedTunableNumber("Drive/DriveKd");
    private final LoggedTunableNumber driveKs = new LoggedTunableNumber("Drive/DriveKs");
    private final LoggedTunableNumber driveKv = new LoggedTunableNumber("Drive/DriveKv");
    
    private final LoggedTunableNumber turnKp = new LoggedTunableNumber("Drive/TurnKp");
    private final LoggedTunableNumber turnKd = new LoggedTunableNumber("Drive/TurnKd");


    
    public SwerveDrivetrain(GyroIO gyroIO, SwerveModuleIO flModuleIO, SwerveModuleIO frModuleIO,
                            SwerveModuleIO blModuleIO, SwerveModuleIO brModuleIO){
        m_pigeonGyro = new Pigeon2(SwerveDrivetrainConstants.PIGEON_ID);
        // m_pigeonGyro.setYaw(0);

        this.gyroIO = gyroIO;
        moduleIOs[0] = flModuleIO;
        moduleIOs[1] = frModuleIO;
        moduleIOs[2] = blModuleIO;
        moduleIOs[3] = brModuleIO;
        
        
        driveKp.initDefault(SwerveDrivetrainConstants.DRIVE_P);
        driveKd.initDefault(SwerveDrivetrainConstants.DRIVE_D);
        driveKs.initDefault(SwerveDrivetrainConstants.DRIVE_kS);
        driveKv.initDefault(SwerveDrivetrainConstants.DRIVE_kV);

        turnKp.initDefault(SwerveDrivetrainConstants.ANGLE_P);
        turnKd.initDefault(SwerveDrivetrainConstants.ANGLE_D);
    
        m_swerveModules = new SwerveModule[] {
            new SwerveModule(0, 
                            SwerveDrivetrainConstants.FRONT_LEFT_OFFSET,
                            SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_ID,
                            SwerveDrivetrainConstants.FRONT_LEFT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.FRONT_LEFT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.FRONT_LEFT_CANCODER_INVERT
            ),
            new SwerveModule(1, 
                            SwerveDrivetrainConstants.FRONT_RIGHT_OFFSET,
                            SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_ID,
                            SwerveDrivetrainConstants.FRONT_RIGHT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.FRONT_RIGHT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.FRONT_RIGHT_CANCODER_INVERT
            ),
            new SwerveModule(2, 
                            SwerveDrivetrainConstants.BACK_LEFT_OFFSET,
                            SwerveDrivetrainConstants.BACK_LEFT_ANGLE_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_DRIVE_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_CANCODER_ID,
                            SwerveDrivetrainConstants.BACK_LEFT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.BACK_LEFT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.BACK_LEFT_CANCODER_INVERT
            ),
            new SwerveModule(3, 
                            SwerveDrivetrainConstants.BACK_RIGHT_OFFSET,
                            SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_ID,
                            SwerveDrivetrainConstants.BACK_RIGHT_ANGLE_INVERT,
                            SwerveDrivetrainConstants.BACK_RIGHT_DRIVE_INVERT,
                            SwerveDrivetrainConstants.BACK_RIGHT_CANCODER_INVERT
            ),
        };

        m_swerveOdometry = new SwerveDriveOdometry(SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS, getYaw(), getModulePositions());
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        final SwerveModuleState[] swerveModuleStates;
        if (fieldRelative) {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
            );
        } else {
            swerveModuleStates = SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(
                    new ChassisSpeeds(translation.getX(), translation.getY(), rotation)
            );
        }

        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveDrivetrainConstants.MAX_SPEED);

        for(var module : m_swerveModules){
            module.setDesiredState(swerveModuleStates[module.m_moduleNumber], isOpenLoop);
        }
    }

    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveDrivetrainConstants.MAX_SPEED);
        
        for (var mod : m_swerveModules) {
            mod.setDesiredState(desiredStates[mod.m_moduleNumber], false);
        }
    }

    public void setChassisSpeeds(ChassisSpeeds targetSpeeds) {
        setModuleStates(SwerveDrivetrainConstants.SWERVE_DRIVE_KINEMATICS.toSwerveModuleStates(targetSpeeds));
    }

    public void stopSwerve() {
        drive(new Translation2d(0 ,0), 0, true, true);
    }

    public ChassisSpeeds getChassisSpeeds(double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond, Rotation2d robotAngle) {
        return ChassisSpeeds.fromFieldRelativeSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, robotAngle);
    }

    public Pose2d getPose() {
        return m_swerveOdometry.getPoseMeters();
    }

    public double getAngle() {
        return m_pigeonGyro.getYaw();
    }

    public double getNonContinuousGyro() {
        return getAngle() % 360;
    }

    public SwerveModuleState[] getStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (var mod : m_swerveModules) {
            states[mod.m_moduleNumber] = mod.getState();
        }
        return states;
    }

    public void goForward() {
        for(var mod : m_swerveModules) {
            mod.setDesiredState(new SwerveModuleState(0.1, Rotation2d.fromDegrees(0)), true);
        }
    }

    public void resetGyro() {
        m_pigeonGyro.setYaw(180);
    }

    public void zeroGyro(double reset) {
        m_pigeonGyro.setYaw(reset);
    }

    public void zeroModules() {
        for(SwerveModule mod: m_swerveModules) {
            mod.zeroModule();
        }
    }

    public SwerveModule[] getModules() {
        return m_swerveModules;
    }

    public Rotation2d getYaw() {
        if (SwerveDrivetrainConstants.PIGEON_INVERT) {
            return Rotation2d.fromDegrees(360 - m_pigeonGyro.getYaw());
        } else {
            return Rotation2d.fromDegrees(m_pigeonGyro.getYaw());
        }
    }

    public void resetOdometry(Pose2d pose) {
        m_swerveOdometry.resetPosition(getYaw(), getModulePositions(), pose);
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : m_swerveModules){
            positions[mod.m_moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    @Override
    public void periodic(){
        m_swerveOdometry.update(getYaw(), getModulePositions());

        // gyroIO.updateInputs(gyroInputs);
        // Logger.getInstance().processInputs("Drive/Gyro", gyroInputs);

        // for (int i = 0; i < 4; i++) {
        //     moduleIOs[i].updateInputs(moduleInputs[i]);
        //     Logger.getInstance().processInputs("Drive/Module" + Integer.toString(i),
        //         moduleInputs[i]);
        // }

        Logger.getInstance().recordOutput("Odometry/Robot", m_swerveOdometry.getPoseMeters());
        Logger.getInstance().recordOutput("Yaw/Robot", getAngle());
        Logger.getInstance().recordOutput("Drive/Module1", m_swerveModules[0].getPosition().distanceMeters);
         
        SmartDashboard.putNumber("Yaw", getAngle());

        // SmartDashboard.putNumber("pose x", getPose().getX());
        // SmartDashboard.putNumber("pose y", getPose().getY());
        // SmartDashboard.putNumber("pose rot", getPose().getRotation().getDegrees());
        

        // for(SwerveModule mod : m_swerveModules) {
        
        //     SmartDashboard.putNumber("drive motor velocity: " + mod.m_moduleNumber, mod.getDriveVelocity());
        //     SmartDashboard.putNumber("cancoder mod: " + mod.m_moduleNumber, mod.getDriveEncoderDegrees());
        // }

        // SmartDashboard.putNumber("gyro", m_pigeonGyro.getYaw());
    }
}
