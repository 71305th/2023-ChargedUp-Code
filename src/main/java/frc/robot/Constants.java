package frc.robot;

import java.util.HashMap;
import java.util.Map;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.lib.swerve.SwerveModuleConstants;

public final class Constants {

  public static class JoystickConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 0;

    public static final int leftStick_X = 0;
    public static final int leftStick_Y = 1;
    public static final int rightStick_X = 4;
    public static final int rightStick_Y = 5;
    public static final int trigger_L = 2;
    public static final int trigger_R = 3;
    public static final int btn_A = 1;
    public static final int btn_B = 2;
    public static final int btn_X = 3;
    public static final int btn_Y = 4;
    public static final int btn_LB = 5;
    public static final int btn_RB = 6;
    public static final int btn_LS = 9;
    public static final int btn_RS = 10;
  }


  public static final class intake {
    public static final int leftMotorPort = 61;
    public static final int rightMotorPort = 62;
    public static final int tilterPort = 17; // motor "stringboi" on phoenix tuner 
    public static final int solenoidPort = 0;
  }

  public static final class Superstructure {
    public static final int talonLeftPort = 14;
    public static final int talonRightPort = 16;
    public static final int talonStringPort = 1; // needs update when using new motor
    public static final int elbowCanCoderPort = 5;
    public static final int stringCanCoderPort = 6;
    public static final int grabberLeft = 51;
    public static final int grabberRight = 52;
  }



  public static final class Swerve {
    public static final double stickDeadband = 0.05;
    public static final double rotDeadband = 0.1;
    public static final int pigeon1 = 1;
    public static final int pigeon2 = 2;
    public static final int pigeon3 = 3;
    public static final int pigeon4 = 4;
    public static final boolean invertGyro = false; // Always ensure Gyro is CCW+ CW-

    /* Drivetrain Constants */
    public static final double trackWidth = 0.576; //meters
    public static final double wheelBase = 0.576; //meters
    public static final double wheelDiameter = Units.inchesToMeters(4.0);
    public static final double wheelCircumference = wheelDiameter * Math.PI;

    public static final double openLoopRamp = 0.25;
    public static final double closedLoopRamp = 0.0;

    public static final double driveGearRatio = (6.7460317460317460317460317460317 / 1.0); // 6.75:1 (6.7460317460317460317460317460317)
    public static final double angleGearRatio = (150.0 / 7.0 / 1.0); // 150/7 : 1

    public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
        new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

    /* Swerve Voltage Compensation */
    public static final double voltageComp = 12.0;

    /* Swerve Current Limiting */
    public static final int angleContinuousCurrentLimit = 30; //20
    public static final int driveContinuousCurrentLimit = 40; //80

    /* Angle Motor PID Values */
    public static final double angleKP = 0.01;
    public static final double angleKD = 0.0;

    /* Drive Motor PID Values */
    public static final double driveKP = 0.12;
    public static final double driveKI = 0.0;
    public static final double driveKD = 0.0025;
    public static final double driveKFF = 0.0;

    /* Drive Motor Characterization Values */
    public static final double driveKS = 0.667;
    public static final double driveKV = 2.44;
    public static final double driveKA = 0.27;

    /* Drive Motor Conversion Factors */
    public static final double driveConversionPositionFactor = (wheelDiameter * Math.PI) / driveGearRatio;
    public static final double driveConversionVelocityFactor = driveConversionPositionFactor / 60.0;
    public static final double angleConversionFactor = 360.0 / angleGearRatio;

    /* Swerve Profiling Values */
    public static final double maxSpeed = 4.1; // meters per second
    public static final double maxAngularVelocity = 13.5; // meters per second

    /* Neutral Modes */
    public static final IdleMode angleNeutralMode = IdleMode.kBrake;
    public static final IdleMode driveNeutralMode = IdleMode.kBrake;

    /* Motor Inverts */
    public static final boolean driveInvert = false;
    public static final boolean angleInvert = false;

    /* Angle Encoder Invert */
    public static final boolean canCoderInvert = false;

    /* Module Specific Constants */
    /* Front Left Module - Module 0 */
    public static final class Mod0 {
      public static final int driveMotorID = 11;
      public static final int angleMotorID = 12;
      public static final int canCoderID = 1;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-134.0);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-131.572);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-0.79);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-132);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Front Right Module - Module 1 */
    public static final class Mod1 {
      public static final int driveMotorID = 21;
      public static final int angleMotorID = 22;
      public static final int canCoderID = 2;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.2);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(0);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-6.5);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Left Module - Module 2 */
    public static final class Mod2 {
      public static final int driveMotorID = 41;
      public static final int angleMotorID = 42;
      public static final int canCoderID = 4;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.21);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(15.21);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(1.5);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-343.3);

      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }

    /* Back Right Module - Module 3 */
    public static final class Mod3 {
      public static final int driveMotorID = 31;
      public static final int angleMotorID = 32;
      public static final int canCoderID = 3;
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-41.2);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-39.882);
      // public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-4);
      public static final Rotation2d angleOffset = Rotation2d.fromDegrees(-43.7);
      public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID, angleMotorID,
          canCoderID, angleOffset);
    }
  }

  public static final class AutoConstants {
    public static final double kMaxSpeedMetersPerSecond = 3;
    public static final double kMaxAccelerationMetersPerSecondSquared = 3;
    public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
    public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

    public static final double kPXController = 1;
    public static final double kPYController = 1;
    public static final double kPThetaController = 1;

    // Constraint for the motion profilied robot angle controller
    public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
        kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
  }

   /**
   * length: meters</p>
   * cone1-->lower one</p>
   * cone2--> upper one</p>
   */
  public static class ImageConstants{

    public static final double kXDis = 1;
    public static final double kYDis = 1;
    public static final double kZDis = 1;
    public static final double kApriltagHeight = 0.515;
    public static final double ConesHight_2 = 1.06600625;
    public static final double ConesHight_1 = 0.568325;
    public static final double DistenceBetweenCones = 0.4318;
    public static final Transform3d CameraToRobotCenter = new Transform3d(new Translation3d(), new Rotation3d(0,0,0));
  }

  
  public static final class FieldConstants {
    /**
     * length: meters </p>
     * </p>
     * ID position:</p>
     * |---blue--|----red----|</p>
     * |----5----------4-----|</p>
     * |----6----------3-----|</p>
     * |----7----------2-----|</p>
     * |----8----------1-----|</p>
     * @param targetID
     * @param position
     */
    public static Map<Integer, Translation3d> ApriltagMap = new HashMap<>(){{
        put(1, new Translation3d(15.69085, 1.07088214, 0.46355));
        put(2, new Translation3d(15.69085, 2.73911785, 0.46355));
        put(3, new Translation3d(15.69085, 4.40735356, 0.46355));
        put(4, new Translation3d(16.18615, 6.7262375, 0.695325));
        put(5, new Translation3d(0.3556, 6.7262375, 0.695325));
        put(6, new Translation3d(0.8509, 4.40735356, 0.46355));
        put(7, new Translation3d(0.8509, 2.73911785, 0.46355));
        put(8, new Translation3d(0.8509, 1.07088214, 0.46355));
    }};

    /**
     * length: meters </p>
     * </p>
     * Objects position:</p>
     * |---blue--|----red----|</p>
     * |----5----------4-----|</p>
     * |----6----------3-----|</p>
     * |------9----10--------|</p>
     * |----7----------2-----|</p>
     * |----8----------1-----|</p>
     * </p>
     * 9,10: charge station center
     * @param ObjectNumber
     * @param position
     */
    public static Map<Integer, Translation3d> ObjectMap = new HashMap<>(){{
      put(1, new Translation3d(9.4234, 0.92075, 0));
      put(2, new Translation3d(9.4234, 2.13995, 0));
      put(3, new Translation3d(9.4234, 3.35915, 0));
      put(4, new Translation3d(9.4234, 4.57835, 0));
      put(5, new Translation3d(7.96565, 4.57835, 0));
      put(6, new Translation3d(7.96565, 3.35915, 0));
      put(7, new Translation3d(7.96565, 2.13995, 0));
      put(8, new Translation3d(7.96565, 0.92075, 0));
      put(9, new Translation3d(2.3955375, 2.73911785, 0));
      put(10, new Translation3d(14.1462125, 0.92075, 0));
    }};

    /**
     * |----------blue---------------|----------------red-----------|</p>
     * | 609(19) 608(29) 607(39) ---------- 307(49) 308(59) 309(69) |</p>
     * | 606(18) 605(28) 604(38) ---------- 304(48) 305(58) 306(68) |</p>
     * | 603(17) 602(27) 601(37) ---------- 301(47) 302(57) 303(67) |</p>
     * | 709(16) 708(26) 707(36) ---------- 207(46) 208(56) 209(66) |</p>
     * | 706(15) 705(25) 704(35) ---------- 204(45) 205(55) 206(65) |</p>
     * | 703(14) 702(24) 701(34) ---------- 201(44) 202(54) 203(64) |</p>
     * | 809(13) 808(23) 807(33) ---------- 107(43) 108(53) 109(63) |</p>
     * | 806(12) 805(22) 804(32) ---------- 104(42) 105(52) 106(62) |</p>
     * | 803(11) 802(21) 801(31) ---------- 101(41) 102(51) 103(61) |</p>
     * @Unit meter
     * @param NodeNumber
     * @param position
     */
    public static Map<Integer, Translation3d> NodeMap = new HashMap<>(){{
      put(101, new Translation3d(15.3750, 0, 0));
      put(102, new Translation3d(15.7462, 0, 0.8636));
      put(103, new Translation3d(16.1735, 0, 1.169));
      put(104, new Translation3d(15.3750, 0, 0));
      put(105, new Translation3d(15.7462, 0, 0.5969));
      put(106, new Translation3d(16.1735, 0, 0.9017));
      put(107, new Translation3d(15.3750, 0, 0));
      put(108, new Translation3d(15.7462, 0, 0.8636));
      put(109, new Translation3d(16.1735, 0, 0.1169));
      put(201, new Translation3d(15.3750, 0, 0));
      put(202, new Translation3d(15.7462, 0, 0.8636));
      put(203, new Translation3d(16.1735, 0, 0.1169));
      put(204, new Translation3d(15.3750, 0, 0));
      put(205, new Translation3d(15.7462, 0, 0.5969));
      put(206, new Translation3d(16.1735, 0, 0.9017));
      put(207, new Translation3d(15.3750, 0, 0));
      put(208, new Translation3d(15.7462, 0, 0.8636));
      put(209, new Translation3d(16.1735, 0, 0.1169));
      put(301, new Translation3d(15.3750, 0, 0));
      put(302, new Translation3d(15.7462, 0, 0.8636));
      put(303, new Translation3d(16.1735, 0, 0.1169));
      put(304, new Translation3d(15.3750, 0, 0));
      put(305, new Translation3d(15.7462, 0, 0.8636));
      put(306, new Translation3d(16.1735, 0, 0.1169));
      put(307, new Translation3d(15.3750, 0, 0));
      put(308, new Translation3d(15.7462, 0, 0.5969));
      put(309, new Translation3d(16.1735, 0, 0.9017));
      put(601, new Translation3d(1.1747, 0, 0));
      put(602, new Translation3d(0.8001, 0, 0.8636));
      put(603, new Translation3d(0.3683, 0, 0.1169));
      put(604, new Translation3d(1.1747, 0, 0));
      put(605, new Translation3d(0.8001, 0, 0.8636));
      put(606, new Translation3d(0.3683, 0, 0.1169));
      put(607, new Translation3d(1.1747, 0, 0));
      put(608, new Translation3d(0.8001, 0, 0.5969));
      put(609, new Translation3d(0.3683, 0, 0.9017));
      put(701, new Translation3d(1.1747, 0, 0));
      put(702, new Translation3d(0.8001, 0, 0.8636));
      put(703, new Translation3d(0.3683, 0, 0.1169));
      put(704, new Translation3d(1.1747, 0, 0));
      put(705, new Translation3d(0.8001, 0, 0.5969));
      put(706, new Translation3d(0.3683, 0, 0.9017));
      put(707, new Translation3d(1.1747, 0, 0));
      put(708, new Translation3d(0.8001, 0, 0.8636));
      put(709, new Translation3d(0.3683, 0, 0.1169));
      put(801, new Translation3d(1.1747, 0, 0));
      put(802, new Translation3d(0.8001, 0, 0.8636));
      put(803, new Translation3d(0.3683, 0, 0.1169));
      put(804, new Translation3d(1.1747, 0, 0));
      put(805, new Translation3d(0.8001, 0, 0.5969));
      put(806, new Translation3d(0.3683, 0, 0.9017));
      put(807, new Translation3d(1.1747, 0, 0));
      put(808, new Translation3d(0.8001, 0, 0.8636));
      put(809, new Translation3d(0.3683, 0, 0.1169));

      put(41, new Translation3d(15.3750, 0, 0));
      put(51, new Translation3d(15.7462, 0, 0.8636));
      put(61, new Translation3d(16.1735, 0, 0.1169));
      put(42, new Translation3d(15.3750, 0, 0));
      put(52, new Translation3d(15.7462, 0, 0.5969));
      put(62, new Translation3d(16.1735, 0, 0.9017));
      put(43, new Translation3d(15.3750, 0, 0));
      put(53, new Translation3d(15.7462, 0, 0.8636));
      put(63, new Translation3d(16.1735, 0, 0.1169));
      put(44, new Translation3d(15.3750, 0, 0));
      put(54, new Translation3d(15.7462, 0, 0.8636));
      put(64, new Translation3d(16.1735, 0, 0.1169));
      put(45, new Translation3d(15.3750, 0, 0));
      put(55, new Translation3d(15.7462, 0, 0.5969));
      put(65, new Translation3d(16.1735, 0, 0.9017));
      put(46, new Translation3d(15.3750, 0, 0));
      put(56, new Translation3d(15.7462, 0, 0.8636));
      put(66, new Translation3d(16.1735, 0, 0.1169));
      put(47, new Translation3d(15.3750, 0, 0));
      put(57, new Translation3d(15.7462, 0, 0.8636));
      put(67, new Translation3d(16.1735, 0, 0.1169));
      put(48, new Translation3d(15.3750, 0, 0));
      put(58, new Translation3d(15.7462, 0, 0.8636));
      put(68, new Translation3d(16.1735, 0, 0.1169));
      put(49, new Translation3d(15.3750, 0, 0));
      put(59, new Translation3d(15.7462, 0, 0.5969));
      put(69, new Translation3d(16.1735, 0, 0.9017));
      put(37, new Translation3d(1.1747, 0, 0));
      put(27, new Translation3d(0.8001, 0, 0.8636));
      put(17, new Translation3d(0.3683, 0, 0.1169));
      put(38, new Translation3d(1.1747, 0, 0));
      put(28, new Translation3d(0.8001, 0, 0.8636));
      put(18, new Translation3d(0.3683, 0, 0.1169));
      put(39, new Translation3d(1.1747, 0, 0));
      put(29, new Translation3d(0.8001, 0, 0.5969));
      put(19, new Translation3d(0.3683, 0, 0.9017));
      put(34, new Translation3d(1.1747, 0, 0));
      put(24, new Translation3d(0.8001, 0, 0.8636));
      put(14, new Translation3d(0.3683, 0, 0.1169));
      put(35, new Translation3d(1.1747, 0, 0));
      put(25, new Translation3d(0.8001, 0, 0.5969));
      put(15, new Translation3d(0.3683, 0, 0.9017));
      put(36, new Translation3d(1.1747, 0, 0));
      put(26, new Translation3d(0.8001, 0, 0.8636));
      put(16, new Translation3d(0.3683, 0, 0.1169));
      put(31, new Translation3d(1.1747, 0, 0));
      put(21, new Translation3d(0.8001, 0, 0.8636));
      put(11, new Translation3d(0.3683, 0, 0.1169));
      put(32, new Translation3d(1.1747, 0, 0));
      put(22, new Translation3d(0.8001, 0, 0.5969));
      put(12, new Translation3d(0.3683, 0, 0.9017));
      put(33, new Translation3d(1.1747, 0, 0));
      put(23, new Translation3d(0.8001, 0, 0.8636));
      put(13, new Translation3d(0.3683, 0, 0.1169));
    }};
  }

}
