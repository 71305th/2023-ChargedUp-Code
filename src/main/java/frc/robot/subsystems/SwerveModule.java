package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.pid.PID;
import frc.lib.swerve.OnboardModuleState;
import frc.lib.swerve.SwerveModuleConstants;
import frc.lib.util.CANCoderUtil;
import frc.lib.util.CANCoderUtil.CCUsage;
import frc.lib.util.CANSparkMaxUtil;
import frc.lib.util.MathUtility;
import frc.lib.util.CANSparkMaxUtil.Usage;
import frc.robot.Constants;
import frc.robot.Robot;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private PID rotorPID;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private WPI_CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;

  private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(
      Constants.Swerve.driveKS, Constants.Swerve.driveKV, Constants.Swerve.driveKA);




  public SwerveModule(int moduleNumber, SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    /* Angle Encoder Config */
    angleEncoder = new WPI_CANCoder(moduleConstants.cancoderID, "7130");
    configAngleEncoder();
    angleEncoder.configMagnetOffset(angleOffset.getDegrees());

    /* Angle Motor Config */
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    configAngleMotor();

    /* Drive Motor Config */
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);

    Rotation2d angle = (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.Swerve.maxSpeed * 0.01))
        ? lastAngle
        : desiredState.angle;

    desiredState.angle = angle;

    double error = getState().angle.getDegrees() - desiredState.angle.getDegrees();
    double constrainedError = MathUtility.constrainAngleDegrees(error);
    double rotorOutput = rotorPID.calculate(constrainedError);
    rotorOutput = MathUtility.clamp(rotorOutput, -1, 1);
    angleMotor.set(rotorOutput);
    lastAngle = angle;


    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.Swerve.maxSpeed;
      percentOutput = MathUtility.clamp(percentOutput, -1, 1);
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }


  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kSensorDataOnly);
    angleEncoder.configAllSettings(Robot.ctreConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kVelocityOnly);
    angleMotor.setSmartCurrentLimit(Constants.Swerve.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.Swerve.angleInvert);
    angleMotor.setIdleMode(Constants.Swerve.angleNeutralMode);
    rotorPID = new PID(Constants.Swerve.angleKP, 0, Constants.Swerve.angleKD, 0, 0);
    angleMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.Swerve.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.Swerve.driveInvert);
    driveMotor.setIdleMode(Constants.Swerve.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.Swerve.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.Swerve.driveConversionPositionFactor);
    driveController.setP(Constants.Swerve.driveKP);
    driveController.setI(Constants.Swerve.driveKI);
    driveController.setD(Constants.Swerve.driveKD);
    driveController.setFF(Constants.Swerve.driveKFF);
    driveMotor.enableVoltageCompensation(Constants.Swerve.voltageComp);
    driveEncoder.setPosition(0.0);
  }

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModulePosition getPosition(){
    return new SwerveModulePosition(
      driveEncoder.getPosition(), 
      getAngle());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
}
