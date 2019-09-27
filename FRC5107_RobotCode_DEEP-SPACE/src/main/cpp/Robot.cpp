/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/


#include "Robot.h"

#include <iostream>
#include <thread>
#include <chrono>
#include <frc/Watchdog.h>
#include <frc/smartdashboard/SmartDashboard.h>

bool testMode = false;
float speedMult = -0.75;
bool runLiftRaise = false;

Robot::Robot() 
{
    snowBlowers = std::unique_ptr<VictorSPX>(new VictorSPX(1));
    liftMotor = std::unique_ptr<VictorSPX>(new VictorSPX(0));
    leftMotor = std::unique_ptr<TalonSRX>(new TalonSRX(1));
    rightMotor = std::unique_ptr<TalonSRX>(new TalonSRX(0));
    driveCont = std::shared_ptr<frc::XboxController>(new frc::XboxController(0));
    leftConsole = std::shared_ptr<frc::Joystick>(new frc::Joystick(1));
    frontLeft = std::shared_ptr<frc::Spark>(new frc::Spark(0));
    rearLeft = std::shared_ptr<frc::Spark>(new frc::Spark(1));
    frontRight = std::shared_ptr<frc::Spark>(new frc::Spark(2));
    rearRight = std::shared_ptr<frc::Spark>(new frc::Spark(3));
    robotDrive = std::unique_ptr<frc::RobotDrive>(new frc::RobotDrive(frontLeft, rearLeft, frontRight, rearRight));
    cableStop = std::unique_ptr<frc::DigitalInput>(new frc::DigitalInput(0));
    rearCylinder = std::unique_ptr<frc::Solenoid>(new frc::Solenoid(0, 2));
    frontCylinder = std::unique_ptr<frc::Solenoid>(new frc::Solenoid(0, 1));
    suckPos = std::unique_ptr<frc::Solenoid>(new frc::Solenoid(0, 0));
    suckEnable = std::unique_ptr<frc::Solenoid>(new frc::Solenoid(1, 0));
    releaseSuck = std::unique_ptr<frc::Solenoid>(new frc::Solenoid(1, 1));
    pressureGauge = std::unique_ptr<frc::AnalogInput>(new frc::AnalogInput(0));
    liftMax = std::unique_ptr<frc::DigitalInput>(new frc::DigitalInput(2));
    liftMin = std::unique_ptr<frc::DigitalInput>(new frc::DigitalInput(1));
    discIn = std::unique_ptr<frc::DigitalInput>(new frc::DigitalInput(3));
    discDet = std::unique_ptr<frc::DigitalInput>(new frc::DigitalInput(4));

    netTabInt = nt::NetworkTableInstance::GetDefault();
    netTab = netTabInt.GetTable("pressureReader");
    comp = std::unique_ptr<frc::Compressor>(new frc::Compressor(0));
    usbCam1 = frc::CameraServer::GetInstance()->StartAutomaticCapture(0);
    usbCam2 = frc::CameraServer::GetInstance()->StartAutomaticCapture(1);
    alreadyThreaded = false;
}

void Robot::raiseHook()
{
    float liftSpeed = 0.75;
    
    if(liftMax->Get() == false)
    {
        liftMotor->Set(motorcontrol::ControlMode::PercentOutput, liftSpeed);
    }
    else
    {
        liftMotor->Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
}

void Robot::lowerHook()
{
    float liftSpeed = 0.75;
    if(leftConsole->GetRawButton(13))
    {
        liftSpeed = 0.5;
    }
    if(liftMin->Get() == false)
    {
        liftMotor->Set(motorcontrol::ControlMode::PercentOutput, -liftSpeed);
    }
    else
    {
        liftMotor->Set(motorcontrol::ControlMode::PercentOutput, 0);
    }
    
}

void Robot::driveCode()
{
    float leftSpeed = 0;
    float rightSpeed = 0;
    std::string errMsg = driveCont->GetError().GetMessage();
    if(errMsg != "")
    {
        std::cout<<"DrvieCont Error: "<<errMsg<<std::endl;
    }

    if(leftConsole->GetRawButton(14))
    {
        speedMult = -0.5;
    }
    else if(leftConsole->GetRawButton(13))
    {
        speedMult = -0.25;
    }
    else
    {
        speedMult = -0.75;
    }
    
    leftSpeed = driveCont->GetY(frc::GenericHID::JoystickHand::kLeftHand);
    rightSpeed = driveCont->GetY(frc::GenericHID::JoystickHand::kRightHand);
    leftSpeed *= speedMult;
    rightSpeed *= speedMult;
    robotDrive->TankDrive(leftSpeed, rightSpeed, false);
}
void Robot::RobotInit()
{
    m_chooser.SetDefaultOption(kAutoNameDefault, kAutoNameDefault);
    m_chooser.AddOption(kAutoNameCustom, kAutoNameCustom);
    frc::SmartDashboard::PutData("Auto Modes", &m_chooser);
}

void Robot::RobotPeriodic(){}
void Robot::AutonomousInit()
{
}

void Robot::AutonomousPeriodic()
{
    driveCode();
    if(driveCont->GetAButton())
    {
        raiseLift();
    }
    else
    {
        snowBlowers->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    }
    if(leftConsole->GetRawButton(15))
    {
        if(comp->Enabled() == true)
        {
            comp->Stop();
        }
    }
    else
    {
        if(comp->Enabled() == false)
        {
            comp->Start();
        }
    }
}

void Robot::raiseLift()
{
    
    if(cableStop->Get() != true)
    {
        snowBlowers->Set(motorcontrol::ControlMode::PercentOutput, 1.0);
        frc::Wait(0.001);
    }
    else
    {
        snowBlowers->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
        frc::SmartDashboard::PutBoolean("SnowblowersRunning", false);
    }
    
}

void Robot::TeleopInit() 
{
}

long map(long x, long in_min, long in_max, long out_min, long out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void Robot::TeleopPeriodic() 
{ 
    frc::SmartDashboard::PutBoolean("SnowBlower Stop", cableStop->Get());
    frc::SmartDashboard::PutString("pressure", std::to_string(map(pressureGauge->GetValue(), 0, 4095, 0, 120)));
    frc::SmartDashboard::PutNumber("pressureRead", map(pressureGauge->GetValue(), 0, 4095, 0, 120));
    std::cout<<"Pressure Direct Read: "<<std::to_string(pressureGauge->GetValue())<<std::endl;
    std::cout<<"Pressure: "<<std::to_string(map(pressureGauge->GetValue(), 0, 4095, 0, 120))<<std::endl;
    if(driveCont->GetAButton())
    {
        raiseLift();
    }
    else
    {
        snowBlowers->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
    }
    
    if(cableStop->Get())
    {
        snowBlowers->Set(motorcontrol::ControlMode::PercentOutput,0);
    }
    if(leftConsole->GetRawButton(5))
    {
        raiseHook();
    }
    else if(leftConsole->GetRawButton(4))
    {
        lowerHook();
    }
    else
    {
        if(leftConsole->GetRawButton(12) == true)
        {
            if(discDet->Get() && discDetCheck == false)
            {
                discDetCheck = true;
                lowerHook();
            }
            else
            {
                liftMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
                if(discDetCheck == true)
                {
                    discDetChecked = true;
                    discDetCheck = false;
                }
            }
        }
        else if(discDetChecked == true)
        {
            if(discIn->Get() == false)
            {
               suckPos->Set(true);
               raiseHook();
            }
            else
            {
                discDetChecked = true;
                suckPos->Set(false);
                liftMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
            } 
        }
        else
        {
            liftMotor->Set(motorcontrol::ControlMode::PercentOutput, 0.0);
        }
    }
    if(leftConsole->GetRawButton(15))
    {
        if(comp->Enabled() == true)
        {
            comp->Stop();
        }
    }
    else
    {
        if(comp->Enabled() == false)
        {
            comp->Start();
        }
    }
    roboLift();
    suckPos->Set(leftConsole->GetRawButton(3));
    driveCode();
    frc::SmartDashboard::PutBoolean("0", cableStop->Get());
    std::cout<<"0: "<<std::to_string(cableStop->Get())<<std::endl;
    frc::SmartDashboard::PutBoolean("1", liftMin->Get());
     std::cout<<"1: "<<std::to_string(liftMin->Get())<<std::endl;
    frc::SmartDashboard::PutBoolean("2", liftMax->Get());
     std::cout<<"2: "<<std::to_string(liftMax->Get())<<std::endl;
    frc::SmartDashboard::PutBoolean("3", discIn->Get());
     std::cout<<"3: "<<std::to_string(discIn->Get())<<std::endl;
    frc::SmartDashboard::PutBoolean("4", discDet->Get());
     std::cout<<"4: "<<std::to_string(discDet->Get())<<std::endl;
}

void Robot::roboLift()
{
    if(driveCont->GetTriggerAxis(frc::GenericHID::kLeftHand)>=1)
    {
        rearCylinder->Set(true);
    }
    else
    {
        rearCylinder->Set(false);
    }
    if(driveCont->GetTriggerAxis(frc::GenericHID::kRightHand)>=1)
    {
        frontCylinder->Set(true);
    }
    else
    {
        frontCylinder->Set(false);
    }
}

void Robot::TestPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() { return frc::StartRobot<Robot>(); }
#endif
