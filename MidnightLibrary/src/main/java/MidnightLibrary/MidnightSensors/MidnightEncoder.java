package MidnightLibrary.MidnightSensors;

import MidnightLibrary.MidnightMovement.MidnightMotor;
import MidnightLibrary.MidnightMovement.MidnightMotorModel;

import static java.lang.Math.PI;

/**
 * Created by Archish on 3/14/18.
 */

public class MidnightEncoder {
  private MidnightMotorModel model;
  private MidnightMotor motor;
  private double wheelDiameter = 4, gearRatio = 1, zeroPos;

  public MidnightEncoder(MidnightMotor motor, MidnightMotorModel model) {
    this.model = model;
    this.motor = motor;
  }

  public double getRelativePosition() {return getAbsolutePosition() - zeroPos;}
  public double getInches () {return getRelativePosition() / getClicksPerInch();}
  public double getAbsolutePosition() {return motor.getAbsolutePosition();}
  public double getClicksPerInch() {return (model.CPR() / (wheelDiameter * PI)) * gearRatio;}
  public double getRPM () {return model.RPM();}
  public double getClicksPerRotation () {return model.CPR();}

  public void resetEncoder() {
    zeroPos = (int) getAbsolutePosition();
  }


  public void setWheelDiameter(double diameter) {wheelDiameter = diameter;}
  public void setGearRatio(double gearRatio) {this.gearRatio = gearRatio;}
  public void setModel(MidnightMotorModel model) {this.model = model;}


}