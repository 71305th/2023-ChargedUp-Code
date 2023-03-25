package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.pid.PID;
import frc.lib.util.MathUtility;
import frc.robot.Constants;

public class Intake extends SubsystemBase{
    private CANSparkMax leftIntake = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
    private CANSparkMax rightIntake = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
    private Solenoid intakeClamp = new Solenoid(PneumaticsModuleType.CTREPCM, Constants.intake.solenoidPort);

    private WPI_TalonFX tilter = new WPI_TalonFX(17, "7130");
    private PID tilterPID = new PID(0.00005, 0, 0.00005, 0, 0);

    private DigitalInput limitSwitch = new DigitalInput(1);


    private static class tilterPos{
        public static double up = 0;
        public static double down = -32000;
        public static double mid = -11000;
    }

    public static enum States{
        human,
        up,
        mid,
        down
    }

    public static enum TilterStates{
        up,
        mid,
        down,
        moving,
    }

    public static enum RollerStates{
        intake,
        shootHigh, 
        shootMid,
        shootFull
    }

    private double currentTilterTarget = tilterPos.up;
    private boolean isClampped = true;
    TilterStates tilterState = TilterStates.up;
    double intakeSpeed = 0;
    States state = States.up;
    RollerStates rollerState = RollerStates.intake;
    double elapsedTime = 0;
    double lastTime = Timer.getFPGATimestamp();

    public Intake(){
        rightIntake.setInverted(true);
        tilter.setSelectedSensorPosition(0);
    }

    @Override
    public void periodic() {
        double dt = Timer.getFPGATimestamp() - lastTime;
        updateStates();

        switch (state){
            //clamps then sets the target and give low power to intake to ensure the cube dont fly out and presses it against the backboard
            case up:
                isClampped = true;
                currentTilterTarget = tilterPos.up;
                intakeSpeed = 0.05;
                break;
            //clamps then sets the target and give low power to intake to ensure the cube dont fly out and presses it against the backboard
            case mid:
                isClampped = true;
                currentTilterTarget = tilterPos.mid;
                intakeSpeed = 0.05;
                break;
            //sets target to up and opens the clamp if its in the up position and then gives 0.3 speed to rollers
            case human:
                currentTilterTarget = tilterPos.up;
                if (tilterState == TilterStates.up) isClampped = false;
                else isClampped = true;
                intakeSpeed = 0.3;
                break;
            //sets the target to down and opens the clamp if its in the down position then gives 0.5 speed to rollers
            case down:
                currentTilterTarget = tilterPos.down;
                if (tilterState == TilterStates.down) isClampped = false;
                else isClampped = true;
                intakeSpeed = 0.5;
                break;
        }

        switch (rollerState){
            case intake: //intakeSpeed = intakeSpeed because in diff states the intake speed are diff
                break;
            case shootHigh:
                //checks if intake is mid then shoots for 1 second then goes back to intake mode
                if (tilterState == TilterStates.mid) {
                    intakeSpeed = -0.6;
                    elapsedTime += dt;    
                }
                if (elapsedTime > 1){
                    elapsedTime = 0;
                    rollerState = RollerStates.intake;
                }
                break;
            case shootMid:
                //checks if intake is mid then shoots for 1 second then goes back to intake mode
                if (tilterState == TilterStates.mid) {
                    intakeSpeed = -0.35;
                    elapsedTime += dt;
                }
                if (elapsedTime > 1){
                    elapsedTime = 0;
                    rollerState = RollerStates.intake;
                }
                break;
            case shootFull:
                //shoots for 1 second then goes back to intake mode
                intakeSpeed = -1;
                elapsedTime += dt;
                if (elapsedTime > 1){
                    elapsedTime = 0;
                    rollerState = RollerStates.intake;
                }
                break;
        }

        if (isClampped) intakeClamp.set(false);
        if (!isClampped) intakeClamp.set(true);
        
        if (limitSwitch.get()){
            tilter.setSelectedSensorPosition(0);
        }
        
        double tilterError = currentTilterTarget - tilter.getSelectedSensorPosition();
        double out = tilterPID.calculate(tilterError);
        out = MathUtility.clamp(out, -0.3, 0.3);
        //setTilter(out);


        setRollers(intakeSpeed);

        lastTime = Timer.getFPGATimestamp();

        // SmartDashboard.putBoolean("tileter limit", limitSwitch.get());
        // SmartDashboard.putNumber("tilterPos", tilter.getSelectedSensorPosition());
        // SmartDashboard.putNumber("tilterError", tilterError);
        // SmartDashboard.putNumber("tilter out ", out);
        // SmartDashboard.putBoolean("isClamped", isClampped);
        // SmartDashboard.putNumber("currentTarget", currentTilterTarget);
    }

    void updateStates(){
        if (tilter.getSelectedSensorPosition() > -1000) tilterState = TilterStates.up;
        if (MathUtility.isWithin(tilter.getSelectedSensorPosition(), -13000, -11000)) tilterState = TilterStates.mid;
        if (tilter.getSelectedSensorPosition() < -31000) tilterState = TilterStates.down;
        else tilterState = TilterStates.moving;
    }

    public void setStates(States istate, RollerStates irollerStates){
        state = istate;
        rollerState = irollerStates;
    }

    public RollerStates getRollerState(){
        return rollerState;
    }
    public States getState(){
        return state;
    }

    public void setTilter (double speed){
        speed = MathUtility.clamp(speed, -0.3, 0.3);
        if (limitSwitch.get() && speed > 0) tilter.set(0);
        //else if (its in down position and u want to set it to go down) set 0
        else tilter.set(speed);
    }

    public void setRollers(double speed){
        speed = MathUtility.clamp(speed, -1, 1);
        leftIntake.set(speed);
        rightIntake.set(speed);
    }

    public void clamp(){
        isClampped = true;
    }

    public void unClamp(){
        isClampped = false;
    }



}
