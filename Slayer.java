package frc.robot.subsystems;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.GenericHID;
//import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;



public class SLAYER extends SubsystemBase {
   
//this boy tells people that driver motor exist
public CANSparkMax leftFrontMotor;
public CANSparkMax leftBackMotor;
public CANSparkMax rightFrontMotor;
public CANSparkMax rightBackMotor;

//the game pad exist
public GenericHID driver;


void mecanumDriveCustom(double y, double x, double rx, double k) {
//each line corresponds to a variable, e.g. y
//y = forward/backwards movement, left stick y axis
//x = turns, left stick x axis
//rx = strafe, right stick x axis
//k = speed variable //mecanum arcade drive
    
 
// does math for arcade drive
double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
double frontLeftPower = ( y + x + rx) / denominator;
double backLeftPower = ( y - x + rx) / denominator;
double frontRightPower = ( y - x - rx) / denominator;
double backRightPower = ( y + x - rx) / denominator;


//set the man power for the motors
leftFrontMotor.set(frontLeftPower * k);
leftBackMotor.set(backLeftPower * k);
rightFrontMotor.set(frontRightPower * k);
rightBackMotor.set(backRightPower * k);
}

  

public void onTeleopInit(){
 

//tells where motor are plug into work

 leftBackMotor = new CANSparkMax(1, MotorType.kBrushless);
 leftFrontMotor = new CANSparkMax(7, MotorType.kBrushless);
 rightBackMotor = new CANSparkMax(6, MotorType.kBrushless);
 rightFrontMotor = new CANSparkMax(5, MotorType.kBrushless);
}


 public void onTeleopPeriodic(){


//how fast can we get fast
//set to (1)second

 leftBackMotor.setOpenLoopRampRate(1);
 leftFrontMotor.setOpenLoopRampRate(1);
 rightBackMotor.setOpenLoopRampRate(1);
 rightFrontMotor.setOpenLoopRampRate(1);


 //set left motors to inverted
 leftBackMotor.setInverted(true);
 leftFrontMotor.setInverted(true);


// this behemonth of knowledge tells the variables to the controller .
    mecanumDriveCustom(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0) * 1.1,
        (Math.abs(driver.getRawAxis(4)) < .1) ? 0 : -driver.getRawAxis(4),
        (driver.getRawAxis(3) > 0 ? .3 : 1));


        TankDriveTeleOp(
        (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
        (Math.abs(driver.getRawAxis(5)) < .1) ? 0 : driver.getRawAxis(5),
        (Math.abs(driver.getRawAxis(2)) < .1) ? 0 : driver.getRawAxis(2),
        (Math.abs(driver.getRawAxis(3)) < .1) ? 0 : driver.getRawAxis(3),
        (driver.getRawAxis(4) > 0 ? .3 : 1));


        FlightStickDriveCustom(  
            (Math.abs(driver.getRawAxis(1)) < .1) ? 0 : driver.getRawAxis(1),
          (Math.abs(driver.getRawAxis(2)) < .1) ? 0 : -driver.getRawAxis(2) * 1.1,    
            (Math.abs(driver.getRawAxis(0)) < .1) ? 0 : -driver.getRawAxis(0),
         (driver.getRawAxis(3) > 0 ? .3 : 1));
 }


//does the math for tankdrive.
 public void TankDriveTeleOp(double ly, double ry, double lt, double rt, double k) {

//ly = left joystick.
//ry = right joystick.
//lt = left trigger.
//rt = right trigger.
 double denominator = Math.max(Math.abs(ly) + Math.abs(ry) + Math.abs(lt) + Math.abs(rt), 1);
 double frontLeftPower = (ly + lt - rt) / denominator;
 double backLeftPower = (ly - lt + rt) / denominator;
 double frontRightPower = (ry + lt - rt) / denominator;
 double backRightPower = (ry - lt + rt) / denominator;


 //power for the motors
 leftFrontMotor.set(frontLeftPower * k);
 leftBackMotor.set(backLeftPower * k);
 rightFrontMotor.set(frontRightPower * k);
 rightBackMotor.set(backRightPower * k);

 }
 
 public void FlightStickDriveCustom(double y, double z, double x, double k) {
//y = forward/backwards movement, y axis
//z = turns, z rotate axis
//x = strafe, x axis
//k = speed variable
    
 double denominator = Math.max(Math.abs(y) + Math.abs(z) + Math.abs(x), 1);
 double frontLeftPower = (y + z + x) / denominator;
 double backLeftPower = (y + z - x) / denominator;
 double frontRightPower = (y + z - x) / denominator;
 double backRightPower = (y - z - x) / denominator;


//power for the motors
 leftFrontMotor.set(frontLeftPower * k);
 leftBackMotor.set(backLeftPower * k);
 rightFrontMotor.set(frontRightPower * k);
  rightBackMotor.set(backRightPower * k);
 }

