// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.ChenryLib.MathUtility;
import frc.ChenryLib.PID;
import frc.robot.Constants;
import frc.robot.Constants.intake;
import frc.robot.autos.betterDelay;

public class Index extends SubsystemBase {

  private CANSparkMax leftRollers = new CANSparkMax(Constants.intake.leftMotorPort, MotorType.kBrushless);
  private CANSparkMax rightRollers = new CANSparkMax(Constants.intake.rightMotorPort, MotorType.kBrushless);
  private Solenoid solenoid = new Solenoid(PneumaticsModuleType.CTREPCM, intake.solenoidPort);
  private WPI_TalonFX tilter = new WPI_TalonFX(intake.tilterPort, "7130");
  private DigitalInput limitSwitch = new DigitalInput(1);

  private PID tilterPID = new PID(0.00007, 0, 0.00005, 0, 0);
  private PID upPID = new PID(0.0003, 0, 0.00022, 0, 0);

  private boolean isClamped = true;
  private boolean shoot = false;
  private indexStates state = indexStates.Standby;
  private indexPos desirePos = indexPos.Up;
  private indexStates lastState;

  public enum indexStates {
    Indexing, // intake from floor
    IndexHuman, // intake from single station
    AimTop,
    AimMiddle,
    AimBottom,
    Standby
  }

  public enum indexPos {
    Up, // Up Limit
    Down, // Down Limit
    GridCom, // In Community  
    LowerGrid // lower
  }

  // Tested Pos
  public static class tilterPos {
    public static double upLimit = 0;
    public static double upSmoothPoint = -3564;
    public static double downlimit = -29000;
    public static double inCom = -6900;
    public static double lowerGrid = -20000;
  }

  private double currentTilterPos;

  /** Creates a new Index. */
  public Index() {
    rightRollers.follow(leftRollers, true);
    tilter.setSelectedSensorPosition(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (limitSwitch.get()) tilter.setSelectedSensorPosition(0);
    currentTilterPos = tilter.getSelectedSensorPosition();
    updateStates();
    if (shoot) shoot(-1);
    if (limitSwitch.get() && tilter.getSelectedSensorVelocity() > 0) tilter.set(0);
    if (!limitSwitch.get() && desirePos == indexPos.Up ) tilter.set(0.11);

    SmartDashboard.putBoolean("Intake isClamped", isClamped);
    SmartDashboard.putBoolean("Intake atUpLimit", limitSwitch.get());
    SmartDashboard.putString("Intake state", state.toString());
    SmartDashboard.putNumber("Intake Pos", tilter.getSelectedSensorPosition());
    SmartDashboard.putNumber("Intake Vel", tilter.getSelectedSensorVelocity());
  }

  public double getTilterPos(){
    return currentTilterPos;
  }

  void setTilter(double speed) {
    if (tilter.getSelectedSensorPosition() <= tilterPos.downlimit && speed < 0) tilter.set(0);
    else if (limitSwitch.get() && speed > 0) tilter.set(0);
    else tilter.set(speed);
  }

  public void setRollers(double speed) {
    leftRollers.set(MathUtility.clamp(speed, -1, 1));
  }

  public void shoot(double speed) {
    setRollers(MathUtility.clamp(speed, -1, 1));
    new betterDelay(1);
  }

  public void clamp() {
    solenoid.set(false);
    isClamped = true;
  }

  public void unclamp() {
    solenoid.set(true);
    isClamped = false;
  }

  public Command toggleClamp() {
    return new InstantCommand(() -> {
      if (isClamped == false) {
        clamp();
      } else if (isClamped == true) {
        unclamp();
      }
    }, this);
  }

  public void setTilterPosBySpd(double speed) {
    if (speed > 0 && limitSwitch.get()) 
      setTilter(0);
    else if (tilter.getSelectedSensorPosition() > tilterPos.upSmoothPoint) 
      setTilter(MathUtility.clamp(speed * 0.4, -1, 1));
    else if (tilter.getSelectedSensorPosition() < tilterPos.downlimit && speed < 0)
      setTilter(0);
    else 
      setTilter(MathUtility.clamp(speed, -1, 1));
  }

  // public void stick() {
  //   double setpoint = 0;
  //   double error = setpoint - tilter.getSelectedSensorPosition();
  //   double output = tilterPID.calculate(error);
  //   tilter.set(ControlMode.PercentOutput, MathUtility.clamp(output, -0.17, 0.17));
  // }

  public void setTilterPosAuto(indexPos pos) {
    switch (pos) {
      case Up:
        desirePos = indexPos.Up;
        if (lastState == indexStates.Indexing) setTilter(MathUtility.clamp(upPID.calculate(tilterPos.upLimit - currentTilterPos), -0.3, 0.3));
        else setTilter(MathUtility.clamp(tilterPID.calculate(tilterPos.upLimit - currentTilterPos), -0.3, 0.3));
        break;
      case Down:
        desirePos = indexPos.Down;
        setTilter(MathUtility.clamp(tilterPID.calculate(tilterPos.downlimit - currentTilterPos), -0.3, 0.3));
        break;
      case GridCom:
        desirePos = indexPos.GridCom;
        setTilter(MathUtility.clamp(tilterPID.calculate(tilterPos.inCom - currentTilterPos), -0.17, 0.17));
        break;
      case LowerGrid:
        desirePos = indexPos.LowerGrid;
        setTilter(MathUtility.clamp(tilterPID.calculate(tilterPos.lowerGrid - currentTilterPos), -0.2, 0.2));
        break;
      default:
        break;
    }
  }

  private double counter = 0;

  public void updateStates() {
    switch (state) {
      case Indexing:
        setTilterPosAuto(indexPos.Down);
        setRollers(0.28);
        if (MathUtility.isWithin(tilter.getSelectedSensorPosition(), tilterPos.downlimit - 1000, tilterPos.downlimit + 1000)) unclamp();
        lastState = indexStates.Indexing;
        break;
      case IndexHuman:
        clamp();
        setRollers(0.28);
        setTilterPosAuto(indexPos.Up);
        lastState = indexStates.IndexHuman;
        break;
      case AimTop:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.GridCom);
        lastState = indexStates.AimTop;
        break;
      case AimMiddle:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.GridCom);
        lastState = indexStates.AimMiddle;
        break;
      case AimBottom:
        clamp();
        setRollers(0);
        setTilterPosAuto(indexPos.LowerGrid);
        lastState = indexStates.AimBottom;
        break;
      case Standby:
        clamp();
        setRollers(0);
        counter++;
        if (counter > 10) {
          setTilterPosAuto(indexPos.Up);
          lastState = indexStates.Standby;
          counter = 0;
        }
        break;
      default:
        break;
    }
  }


  public void shootByState() {
    switch (state) {
        case AimTop:
          shoot(-0.93);
          break;
        case AimMiddle:
          shoot(-0.56);
          break;
        case AimBottom:
          shoot(-0.15);
          break;
        default:
          break;
    }
  }

  public void setState(indexStates state) {
    this.state = state;
  }

  public indexStates getState() {
    return state;
  }

}
