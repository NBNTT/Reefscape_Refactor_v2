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
  public boolean slowMode() {
    return connected() && controller.getRightBumperButton();
  }

  @Override
  public boolean cardinalLock() {
    return connected() && controller.getYButton();
  }

  @Override
  public boolean zeroHeading() {
    return connected() && controller.getBackButton();
  }
}
