
#include <OpenSim/Tools/DynamicsTool.h>
#include <OpenSim/Simulation/InverseDynamicsSolver.h>
#include <OpenSim/OpenSim.h>

using namespace OpenSim;
using namespace SimTK;

class AccelerationInvDyn : public DynamicsTool {
OpenSim_DECLARE_CONCRETE_OBJECT(AccelerationInvDyn, DynamicsTool);
public:
    OpenSim_DECLARE_PROPERTY(states_file, std::string, "");
    OpenSim_DECLARE_PROPERTY(dudt_file, std::string, "");
    AccelerationInvDyn() { constructProperties(); }
    AccelerationInvDyn(const std::string& filename) :
        DynamicsTool(filename, false)
    {
        constructProperties();
        updateFromXMLDocument();
    }

    bool run() override {
        Model m(_modelFileName); // TODO use _model.
        for (int iforce = 0; iforce < m.getForceSet().getSize(); ++iforce)
        { // TODO
            m.updForceSet().get(iforce).set_isDisabled(true);
        }
        bool externalLoads = createExternalLoads(
                _externalLoadsFileName, m, nullptr);
        auto& state = m.initSystem();
        // TODO externalloads? disableModelForces(m, state, _excludedForces);
        Storage states(get_states_file());
        Storage dudt(get_dudt_file());
        if (dudt.isInDegrees()) {
            m.getSimbodyEngine().convertDegreesToRadians(dudt);
        }
        Array<double> times;
        states.getTimeColumn(times);

        Array<std::string> stateNames = states.getColumnLabels();

        InverseDynamicsSolver idsolver(m);
        Storage generalizedForces(times.size());
        for (int i = 0; i < times.size(); ++i) {

            // Create state.
            const auto* statevec = states.getStateVector(i);
            const auto& statedata = states.getStateVector(i)->getData();
            state.setTime(statevec->getTime());
            for (int istate = 1; istate < stateNames.size(); ++istate) {
                m.setStateVariableValue(state, stateNames[istate],
                        statedata[istate - 1]);
            }

            // Create dudt.
            Vector udot(dudt.getColumnLabels().size() - 1, 0.0);
            // TODO consider leaving out.
            const auto& dudtdata = dudt.getStateVector(i)->getData();
            for (int iu = 0; iu < udot.size(); ++iu) {
                udot[iu] = dudtdata[iu + 1];
            }
            /*
            */

            // Solve.
            Vector solution = idsolver.solve(state, udot);
            StateVector solutionVec(statevec->getTime(), solution.size(),
                    &solution[0]);
            generalizedForces.append(solutionVec);
        }
        generalizedForces.setColumnLabels(dudt.getColumnLabels());
        generalizedForces.print("invdyn_solution.sto");
        return true;
    }
private:
    void constructProperties() {
        constructProperty_states_file("");
        constructProperty_dudt_file("");
    }
};

int main(int argc, char* argv[])
{
    Object::registerType(AccelerationInvDyn());
    if (argc == 1) {
        AccelerationInvDyn id;
        id.setSerializeAllDefaults(true);
        id.print("invdyn_default_setup.xml");
        id.setSerializeAllDefaults(false);
    }
    else {
        AccelerationInvDyn id(argv[1]);
        id.run();
    }
    return EXIT_SUCCESS;
}
