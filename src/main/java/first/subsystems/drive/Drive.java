package first.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Kilogram;
import static org.wpilib.units.Units.KilogramSquareMeters;

import org.littletonrobotics.junction.Logger;
import org.wpilib.command2.SubsystemBase;
import org.wpilib.driverstation.Alliance;
import org.wpilib.driverstation.MatchState;
import org.wpilib.driverstation.RobotState;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.math.system.DCMotor;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;


public class Drive extends SubsystemBase {

    private final ModuleIO[] modules;

    private final GyroIO gyro;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    private RobotConfig config;

    private final SwerveSetpointGenerator setpointGenerator;
    private SwerveSetpoint previousSetpoint;

    public Drive(ModuleIO[] modules, GyroIO gyro) {

        this.modules = modules;
        this.gyro = gyro;

        this.refreshAll();

        this.configureRobotConfig();

        this.kinematics = new SwerveDriveKinematics(DriveConstant.kModuleLocations);
        this.poseEstimator = new SwerveDrivePoseEstimator(
                this.kinematics,
                this.gyro.getYaw(),
                this.getSwerveModulePositions(),
                Pose2d.ZERO);

        this.setpointGenerator = new SwerveSetpointGenerator(
            this.config, 
            (DCMotor.getFalcon500(1).freeSpeed / ModuleConfigs.constants.steerGearRatio()) * 0.98);
        
        this.previousSetpoint = new SwerveSetpoint(getRobotRelativeSpeeds(), getSwerveModuleVelocities(),
                   DriveFeedforwards.zeros(config.numModules));
        
        
        this.configurePathPlanner();
    }

    public void refreshAll() {
        this.gyro.refreshAll();
        for (int i = 0; i < modules.length; i++) {
            this.modules[i].refreshAll();
        }
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

    public void configureRobotConfig() {
        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            var moduleConfig = new ModuleConfig(
                ModuleConfigs.constants.wheelDiameter().div(2), 
                DriveConstant.MAX_MODULE_SPEED, 
                1.2, 
                DCMotor.getFalcon500(1).withReduction(ModuleConfigs.constants.driveGearRatio()), 
                Amps.of(60.0), 
                1);
                    

            config = new RobotConfig(
                Kilogram.of(40), 
                KilogramSquareMeters.of(4.0), 
                moduleConfig, 
                DriveConstant.kModuleLocations);

            System.err.println("Failed to load robot config from PP_GUI, using default config");
        }
    }

    public void configurePathPlanner() {        

        AutoBuilder.configure(
                this::getPose,
                this::resetPose,
                this::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> setRobotRelativeVelocities(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0),
                        new PIDConstants(5.0, 0.0, 0.0)),
                config,
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
        this.previousSetpoint = setpointGenerator.generateSetpoint(previousSetpoint, velocities, 0.02);

        Logger.recordOutput("Drive/TargetStates", this.previousSetpoint.moduleStates());


        for (int i = 0; i < modules.length; i++) {
            this.modules[i].setVelocities(previousSetpoint.moduleStates()[i]);
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

        this.refreshAll();

        if (RobotState.isDisabled()) {
            this.previousSetpoint = new SwerveSetpoint(getRobotRelativeSpeeds(), getSwerveModuleVelocities(),
                   DriveFeedforwards.zeros(config.numModules));
        }
        
        this.poseEstimator.update(this.gyro.getYaw(), getSwerveModulePositions());

        Logger.recordOutput("Drive/Pose", this.getPose());
        Logger.recordOutput("Drive/ModuleVelocities", this.getSwerveModuleVelocities());
    }
}
