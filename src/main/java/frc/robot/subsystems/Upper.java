package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.pid.PID;
import frc.lib.util.MathUtility;
import frc.lib.util.SettledUtility;
import frc.robot.Constants;

public class Upper extends SubsystemBase {

    public static Object states;

    // Elbow
    private WPI_TalonFX elbowLeft = new WPI_TalonFX(Constants.Superstructure.talonLeftPort, "7130");
    private WPI_TalonFX elbowRight = new WPI_TalonFX(Constants.Superstructure.talonRightPort, "7130");;
    private CANCoder elbowEncoder = new CANCoder(Constants.Superstructure.elbowCanCoderPort, "7130");
    private SettledUtility elbowSU = new SettledUtility(100, 5, 2);

    // String
    private WPI_TalonFX stringboi = new WPI_TalonFX(Constants.Superstructure.talonStringPort, "7130");
    private CANCoder stringEncoder = new CANCoder(Constants.Superstructure.stringCanCoderPort, "7130");
    private SettledUtility stringSU = new SettledUtility(100, 50, 50);

    // Grabber
    private CANSparkMax grabberLeft = new CANSparkMax(Constants.Superstructure.grabberLeft, MotorType.kBrushless);
    private CANSparkMax grabberRight = new CANSparkMax(Constants.Superstructure.grabberRight, MotorType.kBrushless);

    private PID elbowPID = new PID(0.04, 0, 0.01, 0, 0);
    private PID stringPID = new PID(0.01, 0, 0, 0, 0);

    private double currentElbowTarget = elbowPos.down;
    private double currentStringTarget = stringPos.down;

    boolean stringSettled = true;
    boolean elbowSettled = true;

    boolean elbowIsOutside = false;
    boolean stringIsInside = true;

    // Tested Pos
    private static class elbowPos {
        static double coneMid = 71;
        static double down = 155;
        static double human = 60; //untested 
        static double placing = 75; //
        static double outside = 75;
    }

    // Tested Pos
    private static class stringPos {
        static double coneMid = 460;
        static double down = 0;
        static double human = 1900; //untested
    }

    public static enum States {
        placing,
        human,
        coneMid,
        down,
    }

    public static enum grabberStates{
        intake,
        standby,
        placing
    }

    static class limits {
        static double elbowback = 64;
        static double elbowforward = 350;
        static double stringup = 0;
        static double stringdown = 1500;
    }

    States state = States.down;
    States lastState = States.down;
    grabberStates grabberState = grabberStates.standby;

    double elapsedTime = 0;
    double lastTime = Timer.getFPGATimestamp();

    public Upper() {
        elbowLeft.setInverted(true);
        elbowRight.follow(elbowLeft);
        elbowRight.setInverted(TalonFXInvertType.OpposeMaster);

        elbowEncoder.configAbsoluteSensorRange(AbsoluteSensorRange.Unsigned_0_to_360);
        elbowEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToAbsolutePosition);
        elbowEncoder.configMagnetOffset(50);

        stringboi.setInverted(true);
        stringEncoder.configSensorInitializationStrategy(SensorInitializationStrategy.BootToZero);
        stringEncoder.setPosition(0);
        stringEncoder.configSensorDirection(true);

        grabberRight.follow(grabberLeft, true);
    }

    @Override
    public void periodic() {
        double stringError = currentStringTarget - stringEncoder.getPosition();
        double elbowError = currentElbowTarget - elbowEncoder.getAbsolutePosition();
        double grabberSpeed = 0;
        double dt = Timer.getFPGATimestamp() - lastTime;
        
        switch (state){
            case down:
                setStringTarget(stringPos.down);
                updateStates(stringError, elbowError);
                if (stringIsInside) setElbowTarget(elbowPos.down);;
                grabberState = grabberStates.standby;
                break;
            case coneMid:
                setElbowTarget(elbowPos.coneMid);
                updateStates(stringError, elbowError);
                if (elbowIsOutside) setStringTarget(stringPos.coneMid);
                if (grabberState != grabberStates.placing) grabberState = grabberStates.standby;
                break;
            case human:
                setElbowTarget(elbowPos.human);
                updateStates(stringError, elbowError);
                if (elbowIsOutside) setStringTarget(stringPos.human);
                grabberState = grabberStates.intake;
                break;
            case placing:
                setElbowTarget(elbowPos.placing); //
                grabberState = grabberStates.placing;
                elapsedTime += dt;
                if (elapsedTime > 1) {
                    state = States.coneMid;
                    elapsedTime = 0;
                    grabberState = grabberStates.standby;
                }
                break;
        }

        switch (grabberState){
            case intake:
                grabberSpeed = 0.1;
            case standby:
                grabberSpeed = 0;
            case placing:
                grabberSpeed = -0.4;
        }

        lastState = state;

        // updateStates(stringError, elbowError);
        SmartDashboard.putNumber("elbow abs pos", elbowEncoder.getAbsolutePosition());
        SmartDashboard.putNumber("string enc pos", stringEncoder.getPosition());
        SmartDashboard.putNumber("elbow error ", elbowError);
        SmartDashboard.putNumber("elbow out ", elbowPID.calculate(elbowError));
        SmartDashboard.putNumber("string error", stringError);
        SmartDashboard.putNumber("string out ", stringPID.calculate(stringError));
        SmartDashboard.putBoolean("string settled", stringSettled);
        SmartDashboard.putBoolean("elbow is outside ", elbowIsOutside);
        
        // stringSet(stringPID.calculate(stringError)); //
        // elbowSet(elbowPID.calculate(elbowError)); 
        // grabberSpeed = MathUtility.clamp(grabberSpeed, -1, 1);
        // setGrabberRollers(grabberSpeed);
    }

    public void setStates (States istate){
        state = istate;
    }

    void updateStates(double stringError, double elbowError) {
        stringSettled = stringSU.isSettled(stringError);
        elbowSettled = elbowSU.isSettled(elbowError);
        
        if (elbowEncoder.getAbsolutePosition() < elbowPos.outside) elbowIsOutside = true;
        else elbowIsOutside = false;
        
        if (stringEncoder.getPosition() < 50) stringIsInside = true;
        else stringIsInside = false;   
    }

    void setElbowTarget(double target) {
        currentElbowTarget = target;
    }

        public void elbowSet(double value) {
        // negative -> out
        // positive -> in
        // if (elbowEncoder.getAbsolutePosition() <= limits.elbowback && value > 0) 
        //     elbowLeft.set(0);
        // else if (elbowEncoder.getAbsolutePosition() >= limits.elbowforward && value < 0) 
        //     elbowLeft.set(0);
        // else 
        //     value = MathUtility.clamp(value, -1, 1);
            elbowLeft.set(value);
    }

    public void stringSet(double value) {
        // negative -> release
        // positive -> retract
        // value = MathUtility.clamp(value, -1, 1);
        // if (stringEncoder.getPosition() > 1500 && value < 0){
        //     stringboi.set(0);
        // }
        // else if (stringEncoder.getPosition() < 0 && value > 0){
        //     stringboi.set(0);
        // }
        // else {
            stringboi.set(value);
        
        
    }

    void setStringTarget(double target) {
        currentStringTarget = target;
    }

    public void setGrabberRollers(double speed) {
        grabberLeft.set(MathUtility.clamp(speed, -1, 1));
    }

}
