package first.subsystems.drive;

import static org.wpilib.units.Units.Amps;
import static org.wpilib.units.Units.Meters;
import static org.wpilib.units.Units.MetersPerSecond;

import org.wpilib.math.geometry.Rotation2d;
import org.wpilib.math.kinematics.SwerveModulePosition;
import org.wpilib.math.kinematics.SwerveModuleVelocity;
import org.wpilib.units.measure.Angle;
import org.wpilib.units.measure.AngularVelocity;
import org.wpilib.units.measure.Distance;
import org.wpilib.units.measure.LinearVelocity;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import first.subsystems.drive.ModuleConfigs.ModuleConstants;
import first.subsystems.drive.ModuleConfigs.ModuleIDs;

public class ModuleIOTalon implements ModuleIO {

    private final TalonFX driveMotor, steerMotor;

    private final CANcoder absoluteEncoder;

    private final StatusSignal<Angle> drivePosition, steerPosition, absolutePosition;
    private final StatusSignal<AngularVelocity> driveVelocity, steerVelocity;

    private final VelocityVoltage driveOutput;
    private final PositionVoltage steerOutput;

    private final NeutralOut neutralOutput;

    private final ModuleIDs IDs;
    private final ModuleConstants constants;

    public ModuleIOTalon(ModuleIDs IDs, ModuleConstants constants) {

        this.IDs = IDs;
        this.constants = constants;

        this.driveMotor = new TalonFX(this.IDs.driveMotorId(), this.constants.canBus());
        this.steerMotor = new TalonFX(this.IDs.steerMotorId(), this.constants.canBus());

        this.absoluteEncoder = new CANcoder(IDs.absoluteEncoderId(), constants.canBus());

        this.drivePosition = this.driveMotor.getPosition();
        this.steerPosition = this.steerMotor.getPosition();
        this.absolutePosition = this.absoluteEncoder.getAbsolutePosition();

        this.driveVelocity = driveMotor.getVelocity();
        this.steerVelocity = steerMotor.getVelocity();

        this.driveOutput = new VelocityVoltage(0.0);
        this.steerOutput = new PositionVoltage(0.0);

        this.neutralOutput = new NeutralOut();

        configure();
    }

    private void configure() {
        var driveConfigs = new TalonFXConfiguration();
        var steerConfigs = new TalonFXConfiguration();

        driveConfigs.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);

        driveConfigs.CurrentLimits
                .withStatorCurrentLimit(Amps.of(60))
                .withSupplyCurrentLimit(Amps.of(40))
                .withSupplyCurrentLowerLimit(Amps.of(30))
                .withSupplyCurrentLowerTime(1.0)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        driveConfigs.Feedback
                .withSensorToMechanismRatio(this.constants.driveGearRatio());
        driveConfigs.Slot0
                .withKP(1.0)
                .withKI(0.0)
                .withKD(0.0)
                .withKS(0.1)
                .withKV(0.8)
                .withKA(0.0);

        steerConfigs.MotorOutput
                .withInverted(InvertedValue.Clockwise_Positive)
                .withNeutralMode(NeutralModeValue.Brake);
        steerConfigs.CurrentLimits
                .withStatorCurrentLimit(30)
                .withSupplyCurrentLimit(15)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimitEnable(true);
        steerConfigs.Feedback
                .withSensorToMechanismRatio(this.constants.steerGearRatio());
        steerConfigs.ClosedLoopGeneral
                .withContinuousWrap(true);
        steerConfigs.Slot0
                .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
                .withKP(100)
                .withKI(0.0)
                .withKD(0.5)
                .withKS(0.1)
                .withKV(0.0)
                .withKA(0.0);

        this.driveMotor.getConfigurator().apply(driveConfigs);
        this.steerMotor.getConfigurator().apply(steerConfigs);

        this.absolutePosition.waitForUpdate(0.25);

        if (!this.absolutePosition.getStatus().isOK()) {
            System.err.println("[ERROR] " + this.IDs.moduleName() + " CANcoder refresh failed!");
        } else {
            this.steerMotor.getConfigurator().setPosition(this.absolutePosition.getValue());
        }
    }

    @Override
    public Distance getDrivePosition() {
        return Meters.of(this.drivePosition.getValueAsDouble() * this.constants.wheelDiameter().in(Meters) * Math.PI);
    }

    @Override
    public LinearVelocity getDriveVelocity() {
        return MetersPerSecond
                .of(this.driveVelocity.getValueAsDouble() * this.constants.wheelDiameter().in(Meters) * Math.PI);
    }

    @Override
    public Rotation2d getSteerPosition() {
        return new Rotation2d(this.steerPosition.getValue());
    }

    @Override
    public AngularVelocity getSteerVelocity() {
        return this.steerVelocity.getValue();
    }

    @Override
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), getSteerPosition());
    }

    @Override
    public SwerveModuleVelocity getVelocity() {
        return new SwerveModuleVelocity(getDriveVelocity(), getSteerPosition());
    }

    @Override
    public void setVelocities(SwerveModuleVelocity velocities) {
        Rotation2d nowRotation2d = getSteerPosition();
        boolean isStopped = Math.abs(velocities.velocity) <= 0.01;
        velocities = velocities.optimize(nowRotation2d);
        velocities = velocities.cosineScale(nowRotation2d);

        if (isStopped) {
            steerMotor.setControl(neutralOutput);
            driveMotor.setControl(neutralOutput);
            return;
        }

        this.driveMotor.setControl(this.driveOutput
                .withVelocity(velocities.velocity / (this.constants.wheelDiameter().in(Meters) * Math.PI)));
        this.steerMotor.setControl(this.steerOutput
                .withPosition(velocities.angle.getRotations()));
    }

    @Override
    public void refreshAll() {
        BaseStatusSignal.refreshAll(
                drivePosition,
                driveVelocity,
                steerPosition,
                steerVelocity);
    }
}
