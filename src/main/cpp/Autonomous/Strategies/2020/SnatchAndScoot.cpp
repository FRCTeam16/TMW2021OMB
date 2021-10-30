#include "Autonomous/Strategies/2020/SnatchAndScoot.h"

#include <iostream>
#include <units/units.h>

#include "Autonomous/Steps/ConcurrentStep.h"
#include "Autonomous/Steps/Delay.h"
#include "Autonomous/Steps/DriveToDistance.h"
#include "Autonomous/Steps/Rotate.h"
#include "Autonomous/Steps/SelectVisionPipeline.h"
#include "Autonomous/Steps/SetGyroOffset.h"
#include "Autonomous/Steps/SetVisionOffsetDegrees.h"
#include "Autonomous/Steps/TimedDrive.h"

#include "Autonomous/Steps/2020/SetTurretPosition.h"
#include "Autonomous/Steps/2020/EnableFeeder.h"
#include "Autonomous/Steps/2020/EnableIntake.h"
#include "Autonomous/Steps/2020/EnableShooter.h"
#include "Autonomous/Steps/2020/EnableVisionTracking.h"
#include "Autonomous/Steps/2020/SelectShootingProfile.h"
#include "Autonomous/Steps/2020/SetFeederArmPosition.h"
#include "Autonomous/Steps/2020/SetFeederArmOpenLoop.h"


SnatchAndScoot::SnatchAndScoot(std::shared_ptr<World> world)
{
    std::cout << "SnatchAndScoot::SnatchAndScoot\n";

    const double firstAngle = 180.0;
    steps.push_back(new ConcurrentStep({
		new SetGyroOffset(firstAngle),
        new SetFeederArmPosition(FeederArm::Position::kVertical, 0.25_s),
        new SetTurretPosition(-473, 0.2_s), // -451
		new Delay(0.75)
	}));

    // Run and grab balls
    auto grabOppoBalls = new DriveToDistance(firstAngle, 0.6, 0_in, -104_in);
    grabOppoBalls->SetRampDownDistance(6_in);
    steps.push_back(new ConcurrentStep({
        grabOppoBalls,
        new SetFeederArmPosition(FeederArm::Position::kZero, 0.25_s),
        new EnableIntake(true),
        new SelectVisionPipeline(0),
        new SetVisionOffsetDegrees(1),
        new SelectShootingProfile(ShootingProfile::kMedium)
    }));


    // Scoot back from the ball pickup
    auto scootBack = new DriveToDistance(firstAngle, 0.6, 0_in, 12_in);
    steps.push_back(new ConcurrentStep({
        scootBack,
        new SetFeederArmOpenLoop(0.0)
    }));


    auto crabOver = new DriveToDistance(firstAngle, 0.4, 80_in, -12_in);
    crabOver->SetRampDownDistance(3_in);
    steps.push_back(new ConcurrentStep({
        crabOver
    }));


    const double sweepAngle = 100.0;
    auto rotate = new Rotate(sweepAngle);
    rotate->SetContinueOnTimeout(rotate);
    steps.push_back(new ConcurrentStep({
        rotate,
        new SetFeederArmPosition(FeederArm::Position::kPlayerStation),
        new EnableShooter(true),
    }));

    steps.push_back(new EnableVisionTracking(true));
    steps.push_back(new ConcurrentStep({ new EnableIntake(true, true), new Delay(0.25) }));
    steps.push_back(new ConcurrentStep({ new EnableIntake(true, false), new Delay(0.25) }));
    steps.push_back(new ConcurrentStep({ new EnableIntake(true, true), new Delay(0.25) }));
    steps.push_back(new ConcurrentStep({ new EnableIntake(true, false), new Delay(0.25) }));
    
    // Empty magazine
    steps.push_back(new ConcurrentStep({
        new EnableFeeder(true),
        new Delay(1.5)
    }));

    //
    // Stop here to avoid bars
    //
    if (true) {
        return;
    }

    /*
    auto driveToBar = new DriveToDistance(sweepAngle, 0.15, 19.5_in, -21.12_in);
    steps.push_back(new ConcurrentStep({
        driveToBar,
        new EnableFeeder(false),
        new SetFeederArmOpenLoop(0.0)
    }));
    */

    //
    // Second group pickup
    //
    steps.push_back(new ConcurrentStep({
        new DriveToDistance(sweepAngle, 0.3, 12_in, -36_in),
        new EnableFeeder(false),
        new SetFeederArmOpenLoop(0.0)
    }));

    auto sweep = new DriveToDistance(sweepAngle, 0.3, 96_in, -26_in);
    sweep->SetTimeOut(2.5_s);
    steps.push_back(new ConcurrentStep({
        sweep,
        new EnableIntake(true)
    }));;

/*
    // sweep bar
    auto sweep = new DriveToDistance(sweepAngle, 0.15, 54_in, 27_in);
    sweep->SetTimeOut(2.5_s);
    steps.push_back(new ConcurrentStep({
        sweep
    }));;

    // coast atan2(xdist, ydist)
    auto coastSweep = new DriveToDistance(sweepAngle, 0.00001, 44_in, 22_in);
    coastSweep->SetTimeOut(0.25_s);
    steps.push_back(coastSweep);
*/

    // raise arm, then start shooting
    steps.push_back(new SetFeederArmPosition(FeederArm::Position::kPlayerStation));
    // Empty magazine
    steps.push_back(new ConcurrentStep({
        new EnableFeeder(true),
        new Delay(1.5)
    }));

    // Halt for debugging
    steps.push_back(new ConcurrentStep({
        new EnableFeeder(false),
        new EnableShooter(false),
        new SetFeederArmOpenLoop(0.0)
    }));
    
}