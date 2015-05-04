#include "ssplanner.h"
#include <QVector3D>
#include <QQuaternion>


SSPlanner::SSPlanner()
{

}

SSPlanner::~SSPlanner()
{

}



void SSPlanner::simulate()
{
    // initialize OpenDE
    dInitODE2(0);

    // create the OpenDE environment
    // here the OpenDE world, bodies, shapes, gravity, simulation properties, collision detection, etc, are defined
    oc::OpenDEEnvironmentPtr env( new RigidBodyEnvironment() );

    // create the state space
    // here we define the bouding volume of our space and the distance metric between two states
    RigidBodyStateSpace *stateSpace = new RigidBodyStateSpace(env);
    ob::StateSpacePtr stateSpacePtr = ob::StateSpacePtr(stateSpace);

    oc::OpenDESimpleSetup ss(stateSpacePtr);

    ss.setGoal(ob::GoalPtr(new RigidBodyGoal(ss.getSpaceInformation())));

    ss.getSpaceInformation()->setPropagationStepSize(1.0f/60.0f);


//    ob::PlannerPtr planner(new oc::RRT(ss.getSpaceInformation()));

//    planner->as<oc::RRT>()->setGoalBias(0.95);

//    ss.setPlanner(planner);
//    ss.getProblemDefinition()->setOptimizationObjective( ob::OptimizationObjectivePtr(new ob::PathLengthOptimizationObjective( ss.getSpaceInformation() ) ) );

    ss.setup();
    ss.print();

    if (ss.solve(5))
    {
//        ss.getSolutionPath().asGeometric().print(std::cout);

        Renderer::si->trajectoryPos.clear();
        Renderer::si->trajectoryRot.clear();

        const ob::StateSpace* space(ss.getStateSpace().get());
        std::vector<double> reals;
        for (unsigned int i = 0 ; i < ss.getSolutionPath().asGeometric().getStates().size() ; ++i)
        {
            space->copyToReals(reals, ss.getSolutionPath().asGeometric().getStates()[i]);
            std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(std::cout, " "));
            std::cout << std::endl;
            Renderer::si->trajectoryPos.push_back( QVector3D(reals[0], reals[1], reals[2]) );
            Renderer::si->trajectoryRot.push_back( QQuaternion() );
        }
        std::cout << std::endl;
    }


    // release OpenDE-related memory
    dCloseODE();
}

