package first.subsystems.drive;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class Drive extends SubsystemBase {

    private final ModuleIO[] modules;

    private final GyroIO gyro;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Drive(ModuleIO[] modules, GyroIO gyro) {

        this.modules = modules;
        this.gyro = gyro;

        this.kinematics = new SwerveDriveKinematics(DriveConstant.kModuleLocations);
        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.gyro.getYaw(),
                this.getSwerveModulePositions(),
                Pose2d.ZERO);
        
        this.configurePathPlanner();
    }

    public void resetHeading() {
        var alliance = MatchState.getAlliance();
        if (alliance.isPresent())
            this.poseEstimator.resetRotation(
                    alliance.get() == Alliance.RED ? Rotation2d.fromDegrees(180) : Rotation2d.fromDegrees(0));
        else
            this.poseEstimator.resetRotation(Rotation2d.fromDegrees(0));
    }

    public void resetPose(Pose2d pose) {
        this.poseEstimator.resetPose(pose);
    }

    public void configurePathPlanner() {
        RobotConfig config;
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            config = null;
            e.printStackTrace();
            return; // won't be able to use pathplanner if we can't get the config.
        }

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> setRobotRelativeVelocities(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config, // The robot configuration
                () -> {
                    var alliance = MatchState.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == Alliance.RED;
                    }
                    return false;
                },
                this);
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        var positions = new SwerveModulePosition[modules.length];
        for (int i = 0; i < modules.length; i++) {
            positions[i] = this.modules[i].getPosition();
        }
        return positions;
    }

    public SwerveModuleVelocity[] getSwerveModuleVelocities() {
        var velocities = new SwerveModuleVelocity[modules.length];
        for (int i = 0; i < modules.length; i++) {
            velocities[i] = this.modules[i].getVelocity();
        }
        return velocities;
    }

    public ChassisVelocities getRobotRelativeSpeeds() {
        return this.kinematics.toChassisVelocities(this.getSwerveModuleVelocities());
    }

    public Pose2d getPose() {
        return this.poseEstimator.getEstimatedPosition();
    }

    public Rotation2d getHeading() {
        return this.getPose().getRotation();
    }

    public void setRobotRelativeVelocities(ChassisVelocities velocities) {
        var descretizedVelocities = velocities.discretize(0.02);

        var moduleVelocities = this.kinematics.toSwerveModuleVelocities(descretizedVelocities);

        moduleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(moduleVelocities,
                DriveConstant.MAX_MODULE_SPEED);

        for (int i = 0; i < modules.length; i++) {
            this.modules[i].setVelocities(moduleVelocities[i]);
        }
    }

    public void setFieldOrientedVelocities(ChassisVelocities velocities) {
        var heading = getHeading();
        if (MatchState.getAlliance().orElse(Alliance.BLUE) == Alliance.RED) {
            heading = heading.plus(Rotation2d.fromDegrees(180));
        }
        setRobotRelativeVelocities(velocities.toRobotRelative(heading));
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(this.gyro.getYaw(), getSwerveModulePositions());
    }
}
