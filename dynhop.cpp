
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

int main()
{
    Model model("dynhop.osim");
    model.setUseVisualizer(true);

    auto* forces = new ForceReporter();
    forces->setName("forces");
    model.addAnalysis(forces);

    /*
    auto* pelviskin = new PointKinematics();
    pelviskin->setName("pelviskin");
    pelviskin->setBodyPoint("Pelvis", Vec3(0));
    model.addAnalysis(pelviskin);

    auto* kneekin = new PointKinematics();
    kneekin->setName("kneekin");
    kneekin->setBodyPoint("LThigh", Vec3(0, -0.25, 0));
    model.addAnalysis(kneekin);

    auto* footkin = new PointKinematics();
    footkin->setName("footkin");
    footkin->setBodyPoint("LShank", Vec3(0, -0.25, 0));
    model.addAnalysis(footkin);
    */

    model.print("dynhop_fuddled.osim");

    SimTK::State state = model.initSystem();

    model.updMatterSubsystem().setShowDefaultGeometry(true);

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(1.0);
    manager.integrate(state);

    manager.getStateStorage().print("dynhop_states.sto");
    model.updAnalysisSet().get("forces").printResults("dynhop");

}
