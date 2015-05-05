
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

    auto* pelviskin = new PointKinematics(&model);
    pelviskin->setName("pelviskin");
    pelviskin->setBodyPoint("Pelvis", Vec3(0));
    pelviskin->setPointName("pelvis");
    model.addAnalysis(pelviskin);

    auto* kneekin = new PointKinematics(&model);
    kneekin->setName("kneekin");
    kneekin->setBodyPoint("LeftThigh", Vec3(0, -0.25, 0));
    kneekin->setPointName("knee");
    model.addAnalysis(kneekin);

    auto* footkin = new PointKinematics(&model);
    footkin->setName("footkin");
    footkin->setBodyPoint("LeftShank", Vec3(0, -0.25, 0));
    footkin->setPointName("foot");
    model.addAnalysis(footkin);

    model.print("dynhop_fuddled.osim");

    SimTK::State state = model.initSystem();

    model.updMatterSubsystem().setShowDefaultGeometry(true);

    RungeKuttaMersonIntegrator integrator(model.getMultibodySystem());
    Manager manager(model, integrator);
    manager.setInitialTime(0); manager.setFinalTime(1.0);
    manager.integrate(state);

    manager.getStateStorage().print("dynhop_states.sto");
    model.updAnalysisSet().get("forces").printResults("dynhop");
    model.updAnalysisSet().get("pelviskin").printResults("dynhop");
    model.updAnalysisSet().get("kneekin").printResults("dynhop");
    model.updAnalysisSet().get("footkin").printResults("dynhop");

}
