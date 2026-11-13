package frc.robot.controllers;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;

/** 8BitDo Ultimate in XInput mode, which presents as an Xbox controller. */
public class XboxDriverController implements DriverController {
  private final int port;
  private final XboxController controller;

  public XboxDriverController(int port) {
    this.port = port;
    controller = new XboxController(port);
  }

  private boolean connected() {
    return DriverStation.isJoystickConnected(port);
  }

  @Override
  public double forward() {
    return connected() ? -controller.getLeftY() : 0.0;
  }

  @Override
  public double strafe() {
    return connected() ? -controller.getLeftX() : 0.0;
  }

  @Override
  public double rotation() {
    return connected() ? -controller.getRightX() : 0.0;
  }

  @Override
  public double rotationForward() {
    return connected() ? -controller.getRightY() : 0.0;
  }

  @Override
  public double rotationRight() {
    return connected() ? controller.getRightX() : 0.0;
  }

  @Override
  public boolean slowMode() {
    return connected() && controller.getRightBumperButton();
  }

  @Override
  public boolean fieldOriented() {
    return connected() && controller.getPOV() == 0;
  }

  @Override
  public boolean robotOriented() {
    return connected() && controller.getPOV() == 180;
  }

  @Override
  public boolean cardinalDriving() {
    return connected() && controller.getPOV() == 90;
  }

  @Override
  public boolean xLock() {
    return connected() && controller.getAButton();
  }

  @Override
  public boolean zeroHeading() {
    return connected() && controller.getBackButton();
  }
}
