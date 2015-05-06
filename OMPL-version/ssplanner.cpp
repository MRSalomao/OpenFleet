#include "ssplanner.h"
#include <QVector3D>
#include <QQuaternion>



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

    ss.setup();
    ss.print();

    // Try to solve for 15 seconds. If successful, visualize the output
    if (ss.solve(15))
    {
        Renderer::si->trajectoryPos.clear();
        Renderer::si->trajectoryRot.clear();

        Renderer::si->trajectoryPos.push_back(QVector<QVector3D>());
        Renderer::si->trajectoryRot.push_back(QVector<QQuaternion>());

        const ob::StateSpace* space(ss.getStateSpace().get());
        std::vector<double> reals;
        for (unsigned int i = 0 ; i < ss.getSolutionPath().asGeometric().getStates().size() ; ++i)
        {
            space->copyToReals(reals, ss.getSolutionPath().asGeometric().getStates()[i]);
            std::copy(reals.begin(), reals.end(), std::ostream_iterator<double>(std::cout, " "));
            std::cout << std::endl;
            Renderer::si->trajectoryPos.back().push_back( QVector3D(reals[0], reals[1], reals[2]) );
            Renderer::si->trajectoryRot.back().push_back( QQuaternion() );
        }
        std::cout << std::endl;
    }


    // release OpenDE-related memory
    dCloseODE();
}

