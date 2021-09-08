#ifndef SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERMIP_HPP
#define SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERMIP_HPP

#ifdef CLOBER_RMF

#include <clober_ortools/ortools/base/logging.h>
#include <clober_ortools/ortools/linear_solver/linear_solver.h>
#include <yaml-cpp/yaml.h>


namespace rmf_traffic {
namespace agv {
namespace planning {

using namespace operations_research;


struct RobotVar{
    std::string id_;
    std::string start_;
    std::string goal_;
    int curpathIdx_;
    std::vector<std::string> path_;
};

struct FlowVar{
    std::vector<std::string> prev_;
    std::vector<std::string> now_;
};

class COrtools{
    public:
        COrtools();
        ~COrtools();

        void ClearRobotSet();

        void SetRobot(std::string id, std::string start, std::string goal);

        void SetTimePeriod(int t);

        void LoadData(std::string filename);

        void MakeInitVariables(std::string robot_name);

        void MakeStepVariables(std::string robot_name);

        void MakeCollisionCheck(std::string robot_name);

        void MakeArcCollisionCapacity();

        void MakeNodeCollisionCapacity();

        void MakeFlowConservation(std::string robot_name);

        void MakeOccupiedPath(std::string robot_name,  int idx, std::vector<std::string> path);


        MPVariable* GetVariableByName(std::string name);


        void SetObjective(std::string robot_name, std::string goal_name);

        void GetSolution(std::string robot_name, std::vector<std::string> &sol);


        void PrintVarConst();

        std::vector<std::string> Solve(std::string robot_name);

        std::vector<std::string> ConvertNodePath(std::vector<std::string> state_path);

    private:
        std::unique_ptr<MPSolver> solver_;

        std::map<std::string, RobotVar> t_robotset_;

        // time set //
        std::vector<int> t_timeset_;

        // node set //
        std::vector<std::string> t_nodeset_;

        // arc set //
        std::vector<std::string> t_arcset_;

        // node connection set //
        std::map<std::string, std::map<std::string, std::vector<std::string>>> t_connect_;

};

}
}
}

#endif

#endif //SRC__RMF_TRAFFIC__AGV__PLANNING__CLOBERMIP_HPP