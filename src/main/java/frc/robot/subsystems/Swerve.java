package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.Swerve.Mod0;

public class Swerve extends SubsystemBase {
  private final WPI_Pigeon2 gyro1;
  private final WPI_Pigeon2 gyro2;
  private final WPI_Pigeon2 gyro3;
  private final WPI_Pigeon2 gyro4;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] mSwerveMods;

  private Field2d field;

  public Swerve() {
    gyro1 = new WPI_Pigeon2(Constants.Swerve.pigeon1, "7130");
    gyro2 = new WPI_Pigeon2(Constants.Swerve.pigeon2, "7130");
    gyro3 = new WPI_Pigeon2(Constants.Swerve.pigeon3, "7130");
    gyro4 = new WPI_Pigeon2(Constants.Swerve.pigeon4, "7130");
    gyro1.configFactoryDefault();
    gyro2.configFactoryDefault();
    gyro3.configFactoryDefault();
    gyro4.configFactoryDefault();
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getYaw(), pos);

    mSwerveMods =
        new SwerveModule[] {
          new SwerveModule(0, Constants.Swerve.Mod0.constants),
          new SwerveModule(1, Constants.Swerve.Mod1.constants),
          new SwerveModule(2, Constants.Swerve.Mod2.constants),
          new SwerveModule(3, Constants.Swerve.Mod3.constants)
        };

    field = new Field2d();
  }

  public static SwerveModulePosition[] pos = {
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
    new SwerveModulePosition(0, new Rotation2d(0)),
  };

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates = Constants.Swerve.swerveKinematics.toSwerveModuleStates(
      fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(translation.getX(), translation.getY(), rotation, getYaw())
                    : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  /* Used by SwerveControllerCommand in Auto */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);

    for (SwerveModule mod : mSwerveMods) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public void resetOdometry(Pose2d pose) {
    swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }


  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : mSwerveMods) {
      states[mod.moduleNumber] = mod.getState();
    }
    return states;
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition[] positions = new SwerveModulePosition[4];
    positions[0] = mSwerveMods[0].getPosition();
    positions[1] = mSwerveMods[1].getPosition();
    positions[2] = mSwerveMods[2].getPosition();
    positions[3] = mSwerveMods[3].getPosition();
    return positions;
  }

  public void zeroGyro() {
    gyro1.setYaw(180);
    gyro2.setYaw(180);
    gyro3.setYaw(180);
    gyro4.setYaw(180);
  }

  public Rotation2d getYaw() {
    double averageAngle = gyro1.getYaw() + gyro2.getYaw() + gyro3.getYaw() + gyro4.getYaw();
    averageAngle /= 4;
    return (Constants.Swerve.invertGyro) ? Rotation2d.fromDegrees(360 - averageAngle) : Rotation2d.fromDegrees(averageAngle);
  }

  public double getFrontRoll(){
    return (gyro1.getRoll() + gyro2.getRoll())/2;
  }

  public double getBackRoll(){
    return (gyro3.getRoll() + gyro4.getRoll())/2;
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());

    SmartDashboard.putNumber("gyro ", getYaw().getDegrees());

    for (SwerveModule mod : mSwerveMods) {
      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Cancoder", mod.getAngle().getDegrees());

      SmartDashboard.putNumber(
          "Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
  }

  SmartDashboard.putNumber("ROLL", getFrontRoll());

  System.out.println("Gyro: " + getYaw().getDegrees());
  System.out.println(swerveOdometry.getPoseMeters().getY() + ", " + swerveOdometry.getPoseMeters().getX() + ", " + swerveOdometry.getPoseMeters().getRotation().getDegrees());
}
}
