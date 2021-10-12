#include <iostream>
#include <ctre/Phoenix.h>
#include "Commands/ZeroFeederArm.h"
#include "Robot.h"



// BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

ZeroFeederArm::ZeroFeederArm(): Command() {
        // Use requires() here to declare subsystem dependencies
    // eg. requires(Robot::chassis.get());
    // BEGIN AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=REQUIRES
    SetRunWhenDisabled(true);
}

    // END AUTOGENERATED CODE, SOURCE=ROBOTBUILDER ID=CONSTRUCTOR

// Called just before this Command runs the first time
void ZeroFeederArm::Initialize() {
	std::cout << "****** ZERO FEEDER ARM ******\n";
	Robot::feederArm->ZeroArmPosition();
	std::cout << "****** ZERO FEEDER ARM ******\n";
	SetTimeout(1);
}

// Called repeatedly when this Command is scheduled to run
void ZeroFeederArm::Execute() {

}
// Make this return true when this Command no longer needs to run execute()
bool ZeroFeederArm::IsFinished() {
    return IsTimedOut();
}

// Called once after isFinished returns true
void ZeroFeederArm::End() {

}

// Called when another command which requires one or more of the same
// subsystems is scheduled to run
void ZeroFeederArm::Interrupted() {

}