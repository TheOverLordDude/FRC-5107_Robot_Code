/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

#pragma once

#include <string>
//headers for fucking days
#include <cameraserver/CameraServer.h>
#include <frc/IterativeRobot.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <ctre/Phoenix.h>
#include <frc/Joystick.h>
#include <frc/XboxController.h>
#include <frc/Spark.h>
#include <frc/RobotDrive.h>
#include <frc/DigitalInput.h>
#include <frc/AnalogInput.h>
#include <frc/Solenoid.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableEntry.h>
#include <networktables/NetworkTableInstance.h>
#include <frc/Compressor.h>
#include <memory> ///smart pointers, google it, very helpful and useful 100/3 would recommend


class Robot : public frc::IterativeRobot {
 public:
  Robot();
  void RobotInit() override; //needed
  void RobotPeriodic() override; //needed
  void AutonomousInit() override; //needed
  void AutonomousPeriodic() override; //needed
  void TeleopInit() override; //needed
  void TeleopPeriodic() override; //needed
  void TestPeriodic() override; //needed
  void raiseLift();
  void driveCode();

 private:
 //all the variables necessary for drive and systems
  frc::SendableChooser<std::string> m_chooser;
  const std::string kAutoNameDefault = "Default";
  const std::string kAutoNameCustom = "My Auto";
  std::string m_autoSelected;
  std::unique_ptr<VictorSPX> snowBlowers;
  std::unique_ptr<VictorSPX> liftMotor;
  std::unique_ptr<TalonSRX> leftMotor;
  std::unique_ptr<TalonSRX> rightMotor;
  std::shared_ptr<frc::Spark> frontLeft;
  std::shared_ptr<frc::Spark> rearLeft;
  std::shared_ptr<frc::Spark> frontRight;
  std::shared_ptr<frc::Spark> rearRight;
  std::shared_ptr<frc::Joystick> leftConsole;
  std::shared_ptr<frc::XboxController> driveCont;
  std::unique_ptr<frc::RobotDrive> robotDrive;
  std::unique_ptr<frc::DigitalInput> cableStop;
  std::unique_ptr<frc::DigitalInput> liftMax;
  std::unique_ptr<frc::DigitalInput> liftMin;
  std::unique_ptr<frc::DigitalInput> discIn;
  std::unique_ptr<frc::DigitalInput> discDet;
  std::unique_ptr<frc::Solenoid> rearCylinder; 
  std::unique_ptr<frc::Solenoid> frontCylinder; 
  std::unique_ptr<frc::Solenoid> suckPos;
  std::unique_ptr<frc::Solenoid> suckEnable;
  std::unique_ptr<frc::Solenoid> releaseSuck;
  std::unique_ptr<frc::AnalogInput> pressureGauge;
  nt::NetworkTableInstance netTabInt;
  std::shared_ptr<nt::NetworkTable> netTab;
  std::unique_ptr<frc::Compressor> comp;
  cs::UsbCamera usbCam1;
  cs::UsbCamera usbCam2;
  //cs::AxisCamera netCam1; //this is the network camera, if you cna figure out why it stopped working, go for it
  bool alreadyThreaded = false; //I fORGOT WHAT THIS IS FOR
  void raiseHook();
  void lowerHook();
  bool discDetCheck = false;
  bool discDetChecked = false;
  void roboLift();
};
