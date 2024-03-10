#include <iostream>
#include <chrono>
#include <string>
#include <vector>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

BT::NodeStatus BallFound(){
    std::cout << "Ball not fould\n";
    return BT::NodeStatus::FAILURE;
}


class FindBall: public BT::SyncActionNode{
    
public:
    explicit FindBall(const std::string& name,const BT::NodeConfig& config): BT::SyncActionNode(name, config)
    {
        
    }

    static BT::PortsList providedPorts(){
        return {BT::OutputPort<std::vector<int>>("ball_location")};
    }

    BT::NodeStatus tick() override
    {
        std::this_thread::sleep_for(3s);
        std::vector<int> ball_location{1, 2, 3};
        BT::TreeNode::setOutput("ball_location", ball_location);
        std::cout << "Ball Found\n";
        return BT::NodeStatus::SUCCESS;
    }
};

BT::NodeStatus ballClose(BT::TreeNode& self){
    auto msg = self.getInput<std::vector<int>>("ball_location");

    if(!msg){
        throw BT::RuntimeError("Missing required input [message]:", msg.error());
    }

    for(const auto position_coordinate: msg.value()){
        std::cout << position_coordinate << ' ';
    }

    std::cout << "That's far away\n";
    return BT::NodeStatus::FAILURE;
}

class ApproachBall: public BT::SyncActionNode{

public:
    explicit ApproachBall(const std::string& name, const BT::NodeConfig& config): BT::SyncActionNode(name, config){
        
    }

    static BT::PortsList providedPorts(){
        return {BT::InputPort<std::vector<int>>("ball_location")};
    }

    BT::NodeStatus tick() override
    {
        auto msg = getInput<std::vector<int>>("ball_location");

        if(!msg){
            throw BT::RuntimeError("Missing required input [message]:.", msg.error());
        }

        for(const auto position_coordinate: msg.value()){
            std::cout << position_coordinate << ' ';
        }

        std::this_thread::sleep_for(3s);
        std::cout << "Ball approached\n";
        return BT::NodeStatus::SUCCESS; 
    }
};

BT::NodeStatus ball_grasped()
{
    std::cout << "Ball not grasped\n";
    return BT::NodeStatus::FAILURE;
}

class GraspBall: public BT::SyncActionNode
{
    public:
        explicit GraspBall(const std::string& name): BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {
            std::this_thread::sleep_for(3s);
            std::cout << "Ball grasped\n";
            return BT::NodeStatus::SUCCESS;
        }
};

BT::NodeStatus bin_close()
{
    std::cout << "Bin not close\n";
    return BT::NodeStatus::FAILURE;
}

class ApproachBin: public BT::SyncActionNode
{
    public:
        explicit ApproachBin(const std::string& name): BT::SyncActionNode(name, {})
        {

        } 

        BT::NodeStatus tick() override
        {
            std::this_thread::sleep_for(3s);
            std::cout << "Bin approached\n";
            return BT::NodeStatus::SUCCESS;
        }
};

BT::NodeStatus ball_placed(){
    std::cout << "Ball not placed\n";
    return BT::NodeStatus::FAILURE;
}

class PlaceBall: public BT::SyncActionNode
{
    public: 
        explicit PlaceBall(const std::string& name): BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {
            std::this_thread::sleep_for(3s);
            std::cout << "Ball placed\n";
            return BT::NodeStatus::SUCCESS;
        }
};

class AskForHelp: public BT::SyncActionNode
{
    public: 
        explicit AskForHelp(const std::string& name): BT::SyncActionNode(name, {})
        {

        }

        BT::NodeStatus tick() override
        {
            std::cout << "Asking for help. Waiting for 10 seconds here\n";
            std::this_thread::sleep_for(3s);
            return BT::NodeStatus::SUCCESS;
        }
};


int main(){

    BT::BehaviorTreeFactory factory;
    
    factory.registerSimpleCondition("BallFound", std::bind(BallFound));
    factory.registerNodeType<FindBall>("FindBall");
    
    BT::PortsList say_something_ports = {BT::InputPort<std::vector<int>>("ball_location")};
    factory.registerSimpleCondition("BallClose", ballClose, say_something_ports);
    factory.registerNodeType<ApproachBall>("ApproachBall");
    
    factory.registerSimpleCondition("BallGrasped", std::bind(ball_grasped));
    factory.registerNodeType<GraspBall>("GraspBall");

    factory.registerSimpleCondition("BinClose", std::bind(bin_close));
    factory.registerNodeType<ApproachBin>("ApproachBin");

    factory.registerSimpleCondition("BallPlaced", std::bind(ball_placed));
    factory.registerNodeType<ApproachBin>("PlaceBall");

    factory.registerNodeType<AskForHelp>("AskForHelp");

    try{
        auto tree = factory.createTreeFromFile("/home/dan/learn/bt_learn/grasp_robot_bt_tree.xml");
        tree.tickWhileRunning();
    }catch(BT::BehaviorTreeException& e){
        std::cout << e.what() << "\n";
    }

    return 0;
}