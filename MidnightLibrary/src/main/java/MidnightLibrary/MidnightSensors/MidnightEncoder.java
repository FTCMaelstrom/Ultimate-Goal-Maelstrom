package MidnightLibrary.MidnightSensors;

import MidnightLibrary.MidnightMovement.MidnightMotor;
import MidnightLibrary.MidnightMovement.MidnightMotorModel;

/**
 * Created by Archish on 3/14/18.
 */

public class MidnightEncoder {
  private MidnightMotorModel model;
  private MidnightMotor motor;
  private double wheelDiameter = 4, gearRatio = 1;
  private double currentPosition, zeroPos;
  public MidnightEncoder(MidnightMotor motor, MidnightMotorModel model) {
    this.model = model;
    this.motor = motor;
  }

  public double getRelativePosition() {
    currentPosition = (int) (getAbsolutePosition() - zeroPos);
    return currentPosition;
  }

  public double getInches () {
    return getRelativePosition() / getClicksPerInch();
  }

  public double getAbsolutePosition() {
    return motor.getAbsolutePosition();
  }

  public void resetEncoder() {
    zeroPos = (int) getAbsolutePosition();
    currentPosition = 0;
  }

  public double getWheelDiameter() {
    return wheelDiameter;
  }

  public double getClicksPerInch() {
    return (model.CPR() / (wheelDiameter * Math.PI)) * gearRatio;
  }

  public void setWheelDiameter(double wheelDiameter) {
    this.wheelDiameter = wheelDiameter;
  }

  public double getRPM () {
    return model.RPM();
  }
  public double getClicksPerRotation () {
    return model.CPR();
  }
  public double getGearRatio() {
    return gearRatio;
  }

  public void setGearRatio(double gearRatio) {
    this.gearRatio = gearRatio;
  }

  public void setModel(MidnightMotorModel model) {
    this.model = model;
  }

  public MidnightMotorModel getModel() {
    return model;
  }
}