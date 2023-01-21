package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
//import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.subsystems.management.Subsystem;
import frc.robot.subsystems.sensors.Sensors;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Drivetrain extends Subsystem {

  public GenericHID driver;
  public GenericHID aid;

  private CANSparkMax leftFrontMotor;
  private CANSparkMax leftBackMotor;
  //private MotorControllerGroup leftMotor;
  private CANSparkMax rightFrontMotor;
  private CANSparkMax rightBackMotor;
  //private MotorControllerGroup rightMotor;
  private CANSparkMax owensbadmotor;



  //math behind arcade drive
  private void mecanumDriveCustom(double y, double x, double rx, double k) {

    //y = forward/backwards movement, left stick y axis
    //x = turns, left stick x axis
    //rx = strafe, right stick x axis
    //k = speed variable
    
    double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
    double frontLeftPower = (y + x + rx) / denominator;
    double backLeftPower = (y - x + rx) / denominator;
    double frontRightPower = (y - x - rx) / denominator;
    double backRightPower = (y + x - rx) / denominator;

    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);

  }

  //math for tank drive
  private void TankDriveTeleOp(double ly, double ry, double lt, double rt, double k) {

    //ly = left motors forwards/backward, left stick y axis
    //ry = right motors forwards/backwards, right stick y axis
    //lt = left strafe, left trigger
    //rt = right strafe, right trigger
    //k = speed variable

    double denominator = Math.max(Math.abs(ly) + Math.abs(ry) + Math.abs(lt) + Math.abs(rt), 1);
    double frontLeftPower = (ly + lt - rt) / denominator;
    double backLeftPower = (ly - lt + rt) / denominator;
    double frontRightPower = (ry + lt - rt) / denominator;
    double backRightPower = (ry - lt + rt) / denominator;

    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);

  }

  private void FlightStickDriveCustom(double y, double z, double x, double k) {

    //y = forward/backwards movement, y axis
    //z = turns, z rotate axis
    //x = strafe, x axis
    //k = speed variable
    
    double denominator = Math.max(Math.abs(y) + Math.abs(z) + Math.abs(x), 1);
    double frontLeftPower = (y + z + x) / denominator;
    double backLeftPower = (y + z - x) / denominator;
    double frontRightPower = (y - z + x) / denominator;
    double backRightPower = (y - z - x) / denominator;

    leftFrontMotor.set(frontLeftPower * k);
    leftBackMotor.set(backLeftPower * k);
    rightFrontMotor.set(frontRightPower * k);
    rightBackMotor.set(backRightPower * k);

  }


  

  public void tankDriveCustom(double leftSidePower, double rightSidePower) {

    leftFrontMotor.set(leftSidePower);
    leftBackMotor.set(leftSidePower);

    rightFrontMotor.set(-rightSidePower);
    rightBackMotor.set(-rightSidePower);

  }

  public void strafeLeft(double power) {
    leftFrontMotor.set(-power);
    leftBackMotor.set(power);

    rightFrontMotor.set(-power);
    rightBackMotor.set(power);
  }

  public void strafeRight(double power) {

    leftFrontMotor.set(power);
    leftBackMotor.set(-power);

    rightFrontMotor.set(power);
    rightBackMotor.set(-power);

  }

  @Override
  public void onRobotInit() {


    leftBackMotor = new CANSparkMax(1, MotorType.kBrushless);
    leftFrontMotor = new CANSparkMax(7, MotorType.kBrushless);
    rightFrontMotor = new CANSparkMax(6, MotorType.kBrushless);
    rightBackMotor = new CANSparkMax(5, MotorType.kBrushless);
    owensbadmotor = new CANSparkMax(8, MotorType.kBrushed);


 
    
    // testFalcon = new TalonFX(2);
  }

  @Override
  public void onTeleopInit() {
    SmartDashboard.putString("Drivetrain", "RAN");
    //owensbadmotor.setVoltage(10);
  }

  @Override
  public void onTeleopPeriodic() {

    leftBackMotor.setOpenLoopRampRate(1);
    leftFrontMotor.setOpenLoopRampRate(1);
    rightBackMotor.setOpenLoopRampRate(1);
    rightFrontMotor.setOpenLoopRampRate(1);

    leftBackMotor.setInverted(true);
    leftFrontMotor.setInverted(true);


    //mecanum arcade drive
    //each line corresponds to a variable, e.g. y
   /*  mecanumDriveCustom(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0) * 1.1,
        (Math.abs(driver.getRawAxis(4)) < .1) ? 0 : -driver.getRawAxis(4),
        (driver.getRawAxis(3) > 0 ? .3 : 1));*/
    
    //mecanum tank drive
    //each line corresponds to a variable, e.g. rt
     /*TankDriveTeleOp(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(5)) < .1) ? 0 : driver.getRawAxis(5),
        (Math.abs(driver.getRawAxis(2)) < .1) ? 0 : driver.getRawAxis(2),
        (Math.abs(driver.getRawAxis(3)) < .1) ? 0 : driver.getRawAxis(3),
        (driver.getRawAxis(4) > 0 ? .3 : 1));*/


      //flight stick drive code
      //each line corresponds to a variable, e.g. z
      FlightStickDriveCustom(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(2)) < .1) ? 0 : -driver.getRawAxis(2) * 1.1,
        (Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0),
        (driver.getRawAxis(3) > 0 ? .3 : 1));



      
/*what am i doing, tank drive
leftBackMotor.set(1 * driver.getRawAxis(1));
leftFrontMotor.set(1 * driver.getRawAxis(1));
rightBackMotor.set(1 * driver.getRawAxis(5));
rightFrontMotor.set(1 * driver.getRawAxis(5));

//strafe bois left
leftBackMotor.set(-1 * driver.getRawAxis(2));
leftFrontMotor.set(1 * driver.getRawAxis(2));
rightBackMotor.set(-1 * driver.getRawAxis(2));
rightFrontMotor.set(1 * driver.getRawAxis(2));

//strafe gorls right
leftBackMotor.set(1 * driver.getRawAxis(3));
leftFrontMotor.set(-1 * driver.getRawAxis(3));
rightBackMotor.set(1 * driver.getRawAxis(3));
rightFrontMotor.set(-1 * driver.getRawAxis(3));*/



  
    SmartDashboard.putNumber("Drivetrain test", leftBackMotor.get());

  }

  @Override
  public void onRobotPeriodic() {
//if (driver.getRawButton(4)) {

  //owensbadmotor.setInverted(true);
//} else {
  //owensbadmotor.setInverted(false);
//}

  }

  @Override
  public void getSensors(Sensors sensors) {
    // TODO Auto-generated method stub

  }

  @Override
  public void getDrivetrain(Drivetrain drivetrain) {
    // TODO Auto-generated method stub

  }

  @Override
  public void getGamepad(GenericHID driver, GenericHID aid) {
    this.driver = driver;
    this.aid = aid;

  }

}