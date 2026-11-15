package first.subsystems.drive;

import org.wpilib.command2.SubsystemBase;
import org.wpilib.math.estimator.SwerveDrivePoseEstimator;
import org.wpilib.math.geometry.Pose2d;
import org.wpilib.math.kinematics.ChassisVelocities;
import org.wpilib.math.kinematics.SwerveDriveKinematics;
import org.wpilib.math.kinematics.SwerveModulePosition;


public class Drive extends SubsystemBase {
    
    private final ModuleIO FL, FR, BL, BR;

    private final GyroIO gyro;

    private final SwerveDriveKinematics kinematics;

    private final SwerveDrivePoseEstimator poseEstimator;

    public Drive() {
        this.FL = new ModuleIOTalon(ModuleConfigs.frontLeftIDs, ModuleConfigs.constants);
        this.FR = new ModuleIOTalon(ModuleConfigs.frontRightIDs, ModuleConfigs.constants);
        this.BL = new ModuleIOTalon(ModuleConfigs.backLeftIDs, ModuleConfigs.constants);
        this.BR = new ModuleIOTalon(ModuleConfigs.backRightIDs, ModuleConfigs.constants);

        this.gyro = new GyroIOPigeon();

        this.kinematics = new SwerveDriveKinematics(DriveConstant.kModuleLocations);
        this.poseEstimator = new SwerveDrivePoseEstimator(
            this.kinematics,
            this.gyro.getYaw(),
            this.getSwerveModulePositions(),
            Pose2d.ZERO
        );
    }

    public SwerveModulePosition[] getSwerveModulePositions() {
        return new SwerveModulePosition[] {
            FL.getPosition(),
            FR.getPosition(),
            BL.getPosition(),
            BR.getPosition()
        };
    }

    public void setVelocities(ChassisVelocities velocities) {
        var descretizedVelocities = velocities.discretize(0.02); //TODO I don't Know SystemCore's timestep.

        var moduleVelocities = this.kinematics.toSwerveModuleVelocities(descretizedVelocities);

        moduleVelocities = SwerveDriveKinematics.desaturateWheelVelocities(moduleVelocities, DriveConstant.MAX_MODULE_SPEED);

        this.FL.setStates(moduleVelocities[0]);
        this.FR.setStates(moduleVelocities[1]);
        this.BL.setStates(moduleVelocities[2]);
        this.BR.setStates(moduleVelocities[3]);
    }

    @Override
    public void periodic() {
        this.poseEstimator.update(this.gyro.getYaw(), getSwerveModulePositions());
    }
}
