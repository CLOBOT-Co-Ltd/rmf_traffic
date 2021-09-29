#ifdef CLOBER_RMF

#include "CloberMIP.hpp"

namespace rmf_traffic {
namespace agv {
namespace planning {

COrtools::COrtools()
{
    // Create the mip solver with the SCIP backend.
    std::unique_ptr<MPSolver> solver(MPSolver::CreateSolver("SCIP"));
    if (!solver)
    {
        LOG(WARNING) << "SCIP solver unavailable.";
        return;
    }

    solver_ = std::move(solver);
    SetTimePeriod(10);
    std::cout <<"Clober MIP planner 시작"<<std::endl;
}

COrtools::~COrtools()
{
}

void COrtools::ClearRobotSet(){
    solver_->Clear();
    t_robotset_.clear();
    // t_timeset_.clear();
    // t_nodeset_.clear();
    // t_arcset_.clear();
    // t_connect_.clear();
}

void COrtools::SetRobot(std::string id, std::string start, std::string goal)
{
    RobotVar robot;
    robot.id_ = id;
    robot.start_ = start;
    robot.goal_ = goal;

    t_robotset_.insert(std::make_pair(id, robot));

    // for(auto it=t_robotset_.begin(); it!=t_robotset_.end(); it++){
    //     std::cout <<"scheduler robot member : "<< it->first <<", start : "<<it->second.start_ <<", goal : "<<it->second.goal_<< std::endl;
    // }
}


void COrtools::SetTimePeriod(int t)
{
    for (int i = 0; i < t; i++)
    {
        t_timeset_.push_back(i);
    }
}

void COrtools::PrintMIPData(){
    std::cout <<"Print MIP Data " <<std::endl;
    std::cout <<" ==== nodeset ==== " << std::endl;
    for(int i=0; i<t_nodeset_.size(); i++){
        std::cout << t_nodeset_[i] << std::endl;
    }

    std::cout <<" ==== arcset ==== " << std::endl;
    for(int i=0; i<t_arcset_.size(); i++){
        std::cout << t_arcset_[i] << std::endl;
    }

    std::cout <<" ==== connectset ==== " << std::endl;
    for(auto it=t_connect_.begin(); it!=t_connect_.end(); it++){
        std::cout << it->first << std::endl;

        std::map<std::string, std::vector<std::string>> data = it->second;
        
        for(auto iter=data.begin(); iter!=data.end(); iter++){
            std::cout << iter->first << std::endl;
            std::vector<std::string> values = iter->second;

            for(int i=0; i<values.size(); i++){
                std::cout << values[i]<< ", ";
            }
            std::cout << std::endl;
        }
        std::cout <<" --------------------- " << std::endl;
    }
}


void COrtools::LoadData(std::vector<std::string> nodeset, std::vector<std::string> arcset, std::map<std::string, std::map<std::string, std::vector<std::string>>> connectset){
    t_nodeset_ = nodeset;
    t_arcset_ = arcset;
    t_connect_.insert(connectset.begin(), connectset.end());
}


void COrtools::LoadDataFrmFile(std::string filename)
{
    const YAML::Node mip_config = YAML::LoadFile(filename);
    if (!mip_config)
    {
        throw std::runtime_error("Failed to load solver file [" + filename + "]");
    }

    const YAML::Node nodes = mip_config["Node"];
    if(!nodes){
    throw std::runtime_error("node not exist");
    }

    for(const auto& node : nodes){
    const std::string& node_name = node.as<std::string>();
    std::cout << node_name << std::endl;
    t_nodeset_.push_back(node_name);
    }

    const YAML::Node arcs = mip_config["Arc"];
    if(!arcs){
    throw std::runtime_error("arc not exist");
    }

    for(const auto& arc : arcs){
    const std::string& arc_name = arc.as<std::string>();
    std::cout << arc_name << std::endl;
    t_arcset_.push_back(arc_name);
    }

    const YAML::Node connections = mip_config["Connection"];
    if(!connections){
    throw std::runtime_error("connection not exist");
    }

    if(!connections.IsMap()){
    throw std::runtime_error("connection is not map type structure");
    }

    for(const auto& connection : connections){
    
    std::map<std::string, std::vector<std::string>> inner_set;

    const std::string& con_key = connection.first.as<std::string>();
    std::cout << "con_key : "<<con_key << std::endl;

    // connection 'in' 
    const YAML::Node& ins = connection.second["in"];
    std::vector<std::string> vec_in;
    for(const auto& in : ins){
        const std::string& in_name = in.as<std::string>();
        std::cout << in_name << std::endl;
        vec_in.push_back(in_name);
    }

    inner_set.insert(std::make_pair("in", vec_in));

    // connection 'out' 
    const YAML::Node& outs = connection.second["out"];
    std::vector<std::string> vec_out;
    for(const auto& out : outs){
        const std::string& out_name = out.as<std::string>();
        std::cout << out_name << std::endl;
        vec_out.push_back(out_name);
    }

    inner_set.insert(std::make_pair("out", vec_out));

    // connection 'stay' 
    const YAML::Node& stays = connection.second["stay"];
    std::vector<std::string> vec_stay;
    for(const auto& stay : stays){
        const std::string& stay_name = stay.as<std::string>();
        std::cout << stay_name << std::endl;
        vec_stay.push_back(stay_name);
    }

    inner_set.insert(std::make_pair("stay", vec_stay));

    t_connect_.insert(std::make_pair(con_key, inner_set));
    }

    std::cout <<"solver meta file load successfully"<<std::endl;
}


void COrtools::MakeStepVariables(std::string robot_name)
{
    for (int t = 0; t < t_timeset_.size(); t++)
    {
        std::vector<MPVariable *> step_var;

        for (int a = 0; a < t_arcset_.size(); a++)
        {
            std::string name = robot_name + "_step_" + std::to_string(t) + "_" + t_arcset_[a];
            // std::cout << "step variable name : " << name << std::endl;

            MPVariable *const x_step = solver_->MakeBoolVar(name);
            // MPVariable *const x_step = solver_->MakeIntVar(0,1,name);
            step_var.push_back(x_step);
        }

        std::string const_name = robot_name + "_step_" + std::to_string(t) + "_const";
        MPConstraint *const step_const = solver_->MakeRowConstraint(1, 1, const_name);
        for (int i = 0; i < step_var.size(); i++)
        {
            step_const->SetCoefficient(step_var[i], 1);
        }

        // std::cout <<"step var const name : "<<const_name << std::endl;
    }
}

void COrtools::MakeInitVariables(std::string robot_name)
{
    std::string start_node = "";

    auto findIt = t_robotset_.find(robot_name);
    if (findIt != t_robotset_.end())
    {
        start_node = findIt->second.start_;
    }

    if (!start_node.empty())
    {
        std::vector<std::string> arc_list;
        if (t_connect_.find(start_node) != t_connect_.end())
        {
            for (int i = 0; i < t_connect_[start_node]["stay"].size(); i++)
            {
                arc_list.push_back(t_connect_[start_node]["stay"][i]);
            }
            for (int i = 0; i < t_connect_[start_node]["out"].size(); i++)
            {
                arc_list.push_back(t_connect_[start_node]["out"][i]);
            }
        }

        std::vector<MPVariable *> init_var;
        // variables
        const std::vector<MPVariable *> v = solver_->variables();
        for (int i = 0; i < arc_list.size(); i++)
        {
            std::string var_name = robot_name + "_step_0_" + arc_list[i];
            // std::cout << "init variable name : " << var_name << std::endl;

            for (int i = 0; i < v.size(); i++)
            {
                if (var_name == v[i]->name())
                {
                    // std::cout << "find var : " << var_name << std::endl;
                    init_var.push_back(v[i]);
                }
            }
        }

        // constraints
        std::string const_name = robot_name + "_init_const";
        MPConstraint *const init_const = solver_->MakeRowConstraint(1, 1, const_name);
        for (int i = 0; i < init_var.size(); i++)
        {
            init_const->SetCoefficient(init_var[i], 1);
        }
    }
}

void COrtools::MakeOccupiedPath(std::string robot_name, int idx, std::vector<std::string> path)
{
    for (int t = 0; t < t_timeset_.size(); t++)
    {
        std::string from; 
        std::string to;

        std::string var_name;
        std::string arc_name;

        if (idx  < path.size()-1)
        {
            from = path[idx];
            from.erase(from.begin());

            to = path[idx + 1];
            to.erase(to.begin());
        }
        else
        {
            idx = path.size();
            from = path[idx-1];
            from.erase(from.begin());

            to = path[idx-1];
            to.erase(to.begin());
        }

        
        arc_name.push_back('a');
        arc_name.insert(arc_name.end(), from.begin(), from.end());
        arc_name.push_back('-');
        arc_name.insert(arc_name.end(), to.begin(), to.end());

        var_name = robot_name + "_step_" + std::to_string(t) + "_" + arc_name;
        // std::cout <<"occupy path collision var name : "<<var_name<<std::endl;

        MPVariable* mpVar = NULL;
        mpVar = GetVariableByName(var_name);

        // constraints
        std::string const_name = robot_name + "_path_"+var_name+"const";
        MPConstraint *const path_const = solver_->MakeRowConstraint(1, 1, const_name);
        path_const->SetCoefficient(mpVar, 1);

        idx +=1;
    }
}

void COrtools::MakeCollisionCheck(std::string robot_name)
{
    std::vector<std::pair<MPVariable *, MPVariable *>> colliVar;

    for (int t = 0; t < t_timeset_.size(); t++)
    {
        for (int a = 0; a < t_arcset_.size(); a++)
        {
            std::string name = t_arcset_[a];

            std::string from = "";
            std::string to = "";

            // from to 이름 찾아서 바꾸기
            auto fIt = name.find('a');
            auto tIt = name.find('-');
            if (fIt != std::string::npos && tIt != std::string::npos)
            {
                for (int f = fIt + 1; f < tIt; f++)
                {
                    from.push_back(name[f]);
                }

                for (int t = tIt + 1; t < name.size(); t++)
                {
                    to.push_back(name[t]);
                }
            }

            if (from != to)
            {
                MPVariable *var_name = NULL;
                MPVariable *var_compname = NULL;

                std::string compname;
                compname.push_back('a');
                compname.insert(compname.end(), to.begin(), to.end());
                compname.push_back('-');
                compname.insert(compname.end(), from.begin(), from.end());

                // std::cout << "name : " << name << ", compname : " << compname << std::endl;

                var_name = GetVariableByName(robot_name + "_step_" + std::to_string(t) + "_" + name);
                var_compname = GetVariableByName(robot_name + "_step_" + std::to_string(t) + "_" + compname);

                if (var_name != nullptr)
                {
                    if (var_compname != nullptr)
                    {
                        colliVar.push_back(std::make_pair(var_name, var_compname));
                        // std::cout << var_name->name() << " , " << var_compname->name() << std::endl;
                    }
                }
            }
        }
    }

    // constraints
    const double infinity = solver_->infinity();
    // std::cout << "collivar size : " << colliVar.size() << std::endl;

    for (int i = 0; i < colliVar.size(); i++)
    {
        std::string const_name = robot_name + "_collision_" + std::to_string(i) + "_const";
        MPConstraint *const colli_const = solver_->MakeRowConstraint(0, 1, const_name);

        colli_const->SetCoefficient(colliVar[i].first, 1);
        colli_const->SetCoefficient(colliVar[i].second, 1);
    }
}

void COrtools::MakeArcCollisionCapacity()
{
    std::vector<std::pair<std::pair<std::string, std::string>, std::pair<MPVariable *, MPVariable *>>> colliVar;

    for (int t = 0; t < t_timeset_.size(); t++)
    {
        for (int a = 0; a < t_arcset_.size(); a++)
        {
            std::string name = t_arcset_[a];

            std::string from = "";
            std::string to = "";

            // from to 이름 찾아서 바꾸기
            auto fIt = name.find('a');
            auto tIt = name.find('-');
            if (fIt != std::string::npos && tIt != std::string::npos)
            {

                for (int f = fIt + 1; f < tIt; f++)
                {
                    from.push_back(name[f]);
                }

                for (int t = tIt + 1; t < name.size(); t++)
                {
                    to.push_back(name[t]);
                }
            }

            if (from != to)
            {
                std::string compname;
                compname.push_back('a');
                compname.insert(compname.end(), to.begin(), to.end());
                compname.push_back('-');
                compname.insert(compname.end(), from.begin(), from.end());

                // std::cout << "name : " << name << ", compname : " << compname << std::endl;

                for (auto r = t_robotset_.begin(); r != t_robotset_.end(); r++)
                {
                    for (auto k = t_robotset_.begin(); k != t_robotset_.end(); k++)
                    {

                        MPVariable *var_name = NULL;
                        MPVariable *var_compname = NULL;

                        var_name = GetVariableByName(r->first + "_step_" + std::to_string(t) + "_" + name);
                        var_compname = GetVariableByName(k->first + "_step_" + std::to_string(t) + "_" + compname);

                        if (var_name != nullptr)
                        {
                            if (var_compname != nullptr)
                            {
                                colliVar.push_back(std::make_pair(std::make_pair(r->first, k->first), std::make_pair(var_name, var_compname)));
                                // std::cout << var_name->name() << " , " << var_compname->name() << std::endl;
                            }
                        }
                    }
                }
            }
        }
    }

    // constraints
    const double infinity = solver_->infinity();
    // std::cout << "collivar size : " << colliVar.size() << std::endl;

    for (int i = 0; i < colliVar.size(); i++)
    {
        std::string const_name = colliVar[i].first.first + "_" + colliVar[i].first.second + "_collision_" + std::to_string(i) + "_const";
        MPConstraint *const colli_const = solver_->MakeRowConstraint(0, 1, const_name);

        colli_const->SetCoefficient(colliVar[i].second.first, 1);
        colli_const->SetCoefficient(colliVar[i].second.second, 1);
    }
}

void COrtools::MakeNodeCollisionCapacity()
{

    // 각 노드로 진입하는 arc (stay, in)의 모든 시간과 로봇에 대해서 합이 1보다 작거나 같아야 함.
    for (auto it = t_connect_.begin(); it != t_connect_.end(); it++)
    {

        std::vector<std::string> arclist;
        // map<std::string, map<std::string, std::vector<std::string>>> t_connect_;
        // map<std::string, RobotVar> t_robotset_;

        for (int i = 0; i < it->second["in"].size(); i++)
        {
            arclist.push_back(it->second["in"][i]);
        }

        for (int i = 0; i < it->second["stay"].size(); i++)
        {
            arclist.push_back(it->second["stay"][i]);
        }

        for (int t = 0; t < t_timeset_.size(); t++)
        {

            std::vector<MPVariable *> nodeCapVar;

            for (int a = 0; a < arclist.size(); a++)
            {
                for (auto rIt = t_robotset_.begin(); rIt != t_robotset_.end(); rIt++)
                {

                    MPVariable *var_name = NULL;
                    std::string name = rIt->first + "_step_" + std::to_string(t) + "_" + arclist[a];

                    var_name = GetVariableByName(name);
                    if (var_name != nullptr)
                    {
                        nodeCapVar.push_back(var_name);
                        // cout << "node capacity variable name : " << name << endl;
                    }
                }
            }

            // constraint
            std::string const_name = it->first + "_capacity_collision_" + std::to_string(t) + "_const";
            // cout << "node collision size : " << nodeCapVar.size() << endl;
            MPConstraint *const colli_const = solver_->MakeRowConstraint(0, 1, const_name);
            for (int i = 0; i < nodeCapVar.size(); i++)
            {
                colli_const->SetCoefficient(nodeCapVar[i], 1);
            }
        }
    }
}


void COrtools::MakeFlowConservation(std::string robot_name)
{
    std::vector<FlowVar> vec_flow;

    for (int t = 1; t < t_timeset_.size(); t++)
    {
        for (auto it = t_connect_.begin(); it != t_connect_.end(); it++)
        {
            std::string name;
            FlowVar flow;

            // std::cout << "flow time(t-1) : " << t - 1 << ", time(t) : " << t << "...  related " << it->first << std::endl;

            // std::cout << "(in) : ";
            for (int i = 0; i < it->second["in"].size(); i++)
            {
                name = robot_name + "_step_" + std::to_string(t - 1) + "_" + it->second["in"][i];
                flow.prev_.push_back(name);
                // std::cout << name << " , ";
            }
            // std::cout << std::endl;

            // std::cout << "(stay) : ";
            for (int i = 0; i < it->second["stay"].size(); i++)
            {
                name = robot_name + "_step_" + std::to_string(t - 1) + "_" + it->second["stay"][i];
                flow.prev_.push_back(name);
                // std::cout << name << " , ";
            }
            // std::cout << std::endl;

            // std::cout << "(out) : ";
            for (int i = 0; i < it->second["out"].size(); i++)
            {
                name = robot_name + "_step_" + std::to_string(t) + "_" + it->second["out"][i];
                flow.now_.push_back(name);
                // std::cout << name << " , ";
            }
            // std::cout << std::endl;

            // std::cout << "(stay) : ";
            for (int i = 0; i < it->second["stay"].size(); i++)
            {
                name = robot_name + "_step_" + std::to_string(t) + "_" + it->second["stay"][i];
                flow.now_.push_back(name);
                // std::cout << name << " , ";
            }
            // std::cout << std::endl;

            vec_flow.push_back(flow);
        }
        // std::cout << std::endl;
    }

    for (int i = 0; i < vec_flow.size(); i++)
    {
        std::vector<MPVariable *> p_vec;
        std::vector<MPVariable *> n_vec;

        for (int p = 0; p < vec_flow[i].prev_.size(); p++)
        {
            MPVariable *var_name = NULL;
            var_name = GetVariableByName(vec_flow[i].prev_[p]);

            if (var_name != nullptr)
            {
                p_vec.push_back(var_name);
            }
        }

        for (int n = 0; n < vec_flow[i].now_.size(); n++)
        {
            MPVariable *var_name = NULL;
            var_name = GetVariableByName(vec_flow[i].now_[n]);

            if (var_name != nullptr)
            {
                n_vec.push_back(var_name);
            }
        }

        std::string const_name = robot_name + "_flow_" + std::to_string(i) + "_const";
        MPConstraint *const colli_const = solver_->MakeRowConstraint(0, 0, const_name);

        for (int p = 0; p < p_vec.size(); p++)
        {
            colli_const->SetCoefficient(p_vec[p], 1);
        }

        for (int n = 0; n < n_vec.size(); n++)
        {
            colli_const->SetCoefficient(n_vec[n], -1);
        }
    }
}

MPVariable *COrtools::GetVariableByName(std::string name)
{
    const std::vector<MPVariable *> v = solver_->variables();

    for (int i = 0; i < v.size(); i++)
    {
        if (name == v[i]->name())
        {
            // std::cout << "find var : " << name << std::endl;
            return v[i];
        }
    }

    return NULL;
}

void COrtools::SetObjective(std::string robot_name, std::string goal_name)
{
    std::vector<std::string> arc_list;
    if (t_connect_.find(goal_name) != t_connect_.end())
    {
        for (int i = 0; i < t_connect_[goal_name]["stay"].size(); i++)
        {
            arc_list.push_back(t_connect_[goal_name]["stay"][i]);
        }

        // for( int i=0; i<t_connect_[goal_name]["in"].size(); i++){
        //     arc_list.push_back(t_connect_[goal_name]["in"][i]);
        // }
    }

    std::vector<MPVariable *> objVars;
    // get variables
    const std::vector<MPVariable *> v = solver_->variables();
    for (int i = 0; i < arc_list.size(); i++)
    {
        for (int t = 0; t < t_timeset_.size(); t++)
        {
            std::string name = robot_name + "_step_" + std::to_string(t) + "_" + arc_list[i];
            // std::cout << "objective variable name : " << name << std::endl;

            MPVariable *var_name = NULL;
            var_name = GetVariableByName(name);

            if (var_name != nullptr)
            {
                objVars.push_back(var_name);
            }
        }
    }

    MPObjective *const objective = solver_->MutableObjective();
    for (int c = 0; c < objVars.size(); c++)
    {
        objective->SetCoefficient(objVars[c], 1);
    }

    objective->SetMaximization();
}

void COrtools::GetSolution(std::string robot_name, std::vector<std::string> &sol)
{
    const std::vector<MPVariable *> v = solver_->variables();
    for (int i = 0; i < v.size(); i++)
    {
        if (v[i]->solution_value() == 1)
        {
            if (v[i]->name().find(robot_name) != std::string::npos)
            {
                sol.push_back(v[i]->name());
                // LOG(INFO) << v[i]->name() << " : " << v[i]->solution_value();
            }
        }
        // LOG(INFO) << v[i]->name() << " : " << v[i]->solution_value();
    }
}

void COrtools::PrintVarConst()
{
    LOG(INFO) << "Number of variables = " << solver_->NumVariables();
    LOG(INFO) << "Number of constraints = " << solver_->NumConstraints();
}

std::vector<std::string> COrtools::Solve(std::string robot_name)
{
    const MPSolver::ResultStatus result_status = solver_->Solve();
    // std::cout << "result status : " << result_status << std::endl;
    // LOG(INFO) << "Solution:";
    // LOG(INFO) << "Objective value = " << solver_->MutableObjective()->Value();

    std::vector<std::string> arcPath;
    GetSolution(robot_name, arcPath);

    std::vector<std::string> nodePath;
    nodePath = ConvertNodePath(arcPath);

    //   LOG(INFO) << "\nAdvanced usage:";
    //   LOG(INFO) << "Problem solved in " << solver->wall_time() << " milliseconds";
    //   LOG(INFO) << "Problem solved in " << solver->iterations() << " iterations";
    //   LOG(INFO) << "Problem solved in " << solver->nodes()
    //             << " branch-and-bound nodes";

    return nodePath;
}

std::vector<std::string> COrtools::ConvertNodePath(std::vector<std::string> state_path)
{
    std::vector<std::string> nodePath;
    
    bool start_check_flag = false;
    for (int i = 0; i < state_path.size(); i++)
    {
        // afrom-to 로 노드 경로 만들기
        std::string from = "";
        std::string to = "";

        auto fIt = state_path[i].find('a');
        auto tIt = state_path[i].find('-');
        if (fIt != std::string::npos && tIt != std::string::npos)
        {
            for (int f = fIt + 1; f < tIt; f++)
            {
                from.push_back(state_path[i][f]);
            }

            for (int t = tIt + 1; t < state_path[i].size(); t++)
            {
                to.push_back(state_path[i][t]);
            }
        }

        if (from == to)
        {
            continue;
        }
        
        if (!start_check_flag)
        {
            std::string name = "n" + from;
            nodePath.push_back(name);
            start_check_flag = true;
        }

        std::string name = "n" + to;
        nodePath.push_back(name);
    }
    return nodePath;
}

}  // namespace planning
}  // namespace agv
}  // namespace rmf_traffic

#endif