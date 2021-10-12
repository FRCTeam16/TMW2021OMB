#include <iostream>
#include <ctre/Phoenix.h>
#include "Commands/ZeroEncoders.h"
#include "Robot.h"



// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ZeroEncoders::ZeroEncoders(): Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    SetRunWhenDisabled(true);
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void ZeroEncoders::Initialize() {
	std::cout << "****** ZeroEncoders ******\n";
	auto wheels = Robot::driveBase->GetWheels();
    wheels.FL->ZeroDriveEncoder();
    wheels.FR->ZeroDriveEncoder();
    wheels.RL->ZeroDriveEncoder();
    wheels.RR->ZeroDriveEncoder();
	std::cout << "****** ZeroEncoders ******\n";
	SetTimeout(1);
}

// Called repeatedly when this Command is scheduled to run
void ZeroEncoders::Execute() {

}
// Make this return true when this Command no longer needs to run execute()
bool ZeroEncoders::IsFinished() {
    return IsTimedOut();
}

// Called once after isFinished returns true
void ZeroEncoders::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ZeroEncoders::Interrupted() {

}
