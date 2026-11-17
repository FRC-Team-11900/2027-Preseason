package first.subsystems.drive.module;

import static org.wpilib.units.Units.Inches;

import org.wpilib.hardware.bus.CANPort;
import org.wpilib.units.measure.Distance;

import com.ctre.phoenix6.CANBus;


public class ModuleConfigs {
    public static record ModuleIDs(
        String moduleName,
        int driveMotorId,
        int steerMotorId,
        int absoluteEncoderId
    ){}

    public static record ModuleConstants(
        Distance wheelDiameter,
        double driveGearRatio,
        double steerGearRatio,
        CANBus canBus
    ){}

    public static ModuleIDs frontLeftIDs = new ModuleIDs(
        "FrontLeft", 
        1, 
        5, 
        9
    );

    public static ModuleIDs frontRightIDs = new ModuleIDs(
        "FrontRight", 
        2, 
        6, 
        10
    );

    public static ModuleIDs backLeftIDs = new ModuleIDs(
        "BackLeft", 
        3, 
        7, 
        11
    );

    public static ModuleIDs backRightIDs = new ModuleIDs(
        "BackRight", 
        4, 
        8, 
        12
    );

    public static ModuleConstants constants = new ModuleConstants(
        Inches.of(4),
        6.75, 
        150.0 / 7.0,
        new CANBus(CANPort.CAN_D0)
    );

    public static ModuleIDs[] drivetrainIDs = {
        frontLeftIDs,
        frontRightIDs,
        backLeftIDs,
        backRightIDs
    };

    

    public ModuleConfigs() {}
}
