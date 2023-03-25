package frc.lib.swerve;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import frc.robot.Constants;

public final class CTREConfigs {
  public CANCoderConfiguration swerveCanCoderConfig;

  public CTREConfigs() {
    swerveCanCoderConfig = new CANCoderConfiguration();

    /* Swerve CANCoder Configuration */
    swerveCanCoderConfig.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
    swerveCanCoderConfig.sensorDirection = Constants.Swerve.canCoderInvert;
    swerveCanCoderConfig.initializationStrategy =
        SensorInitializationStrategy.BootToAbsolutePosition;
  }
}
