#include "planner.h"

Planner::Planner()
{
    // initialize OpenDE
    dInitODE2(0);

    // create the OpenDE environment
    oc::OpenDEEnvironmentPtr env(new RigidBodyEnvironment());

    // create the state space and the control space for planning
    RigidBodyStateSpace *stateSpace = new RigidBodyStateSpace(env);
    ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(stateSpace);

    ob::SpaceInformationPtr si(new ob::SpaceInformation(stateSpacePtr));

    si->setStateValidityChecker(ob::StateValidityCheckerPtr(new myStateValidityCheckerClass(si)));

    ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));


    // this will take care of setting a proper collision checker and the starting state for the planner as the initial OpenDE state
    oc::OpenDESimpleSetup ss(stateSpacePtr);

    // set the goal we would like to reach
    ss.setGoal(ob::GoalPtr(new RigidBodyGoal(ss.getSpaceInformation())));

//    ss.setGoalState();

//    ss.setStateValidityChecker();

//    ss.setOptimizationObjective();

//    ss.setStartAndGoalStates();

    dCloseODE();

    if (ss.solve(10))
        std::cout << ss.getSolutionPath().getControlDurations()[0] << std::endl;


}

Planner::~Planner()
{

}

