
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
        m.updForceSet().clearAndDestroy();
            /*
        for (int iforce = 0; iforce < m.getForceSet().getSize(); ++iforce)
        { // TODO
        }
        */
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
        Storage generalizedForces_f_v_a(times.size());
        Storage generalizedForces_f(times.size());
        Storage generalizedForces_v(times.size());
        Storage generalizedForces_a(times.size());
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
                udot[iu] = dudtdata[iu];
            }
            /*
            */

            // Solve.
            // f: grf
            // v: quadratic velocity
            // a: joint acceleration Mudot
            for (int iforce = 0; iforce < m.getForceSet().getSize(); ++iforce)
            {
                m.getForceSet().get(iforce).setDisabled(state, false);
            }
            Vector sol_f_v_a = idsolver.solve(state, udot);
            Vector sol_f_v = idsolver.solve(state);

            for (int iforce = 0; iforce < m.getForceSet().getSize(); ++iforce)
            {
                m.getForceSet().get(iforce).setDisabled(state, true);
            }
            Vector sol_v_a = idsolver.solve(state, udot);
            Vector sol_v = idsolver.solve(state);

            Vector sol_f = sol_f_v - sol_v;
            Vector sol_a = sol_v_a - sol_v;

            StateVector solVec_f_v_a(statevec->getTime(), sol_f_v_a.size(),
                    &sol_f_v_a[0]);
            generalizedForces_f_v_a.append(solVec_f_v_a);

            StateVector solVec_f(statevec->getTime(),
                    sol_f.size(), &sol_f[0]);
            generalizedForces_f.append(solVec_f);

            StateVector solVec_v(statevec->getTime(),
                    sol_v.size(), &sol_v[0]);
            generalizedForces_v.append(solVec_v);

            StateVector solVec_a(statevec->getTime(),
                    sol_a.size(), &sol_a[0]);
            generalizedForces_a.append(solVec_a);
        }
        generalizedForces_f_v_a.setColumnLabels(dudt.getColumnLabels());
        generalizedForces_f_v_a.print("invdyn_solution.sto");
        generalizedForces_f.setColumnLabels(dudt.getColumnLabels());
        generalizedForces_f.print("invdyn_solution_f.sto");
        generalizedForces_v.setColumnLabels(dudt.getColumnLabels());
        generalizedForces_v.print("invdyn_solution_v.sto");
        generalizedForces_a.setColumnLabels(dudt.getColumnLabels());
        generalizedForces_a.print("invdyn_solution_a.sto");
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
