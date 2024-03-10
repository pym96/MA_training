#include <iostream>
#include <chrono>
#include <string>
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/bt_factory.h"

using namespace std::chrono_literals;

class ApproachObject: public BT::SyncActionNode{

public:
    explicit ApproachObject(const std::string& name): BT::SyncActionNode(name, {}){

    }

    BT::NodeStatus tick() override{
        std::cout << "Approach Object: " << this->name() << std::endl;
        
        std::this_thread::sleep_for(5s);
        return BT::NodeStatus::SUCCESS;
    }
};

// Function
BT::NodeStatus CheckBattery(){
    std::cout << "Check battery\n";
    return BT::NodeStatus::SUCCESS;
}

// Custom class method
class GripperInterface{
public:
    GripperInterface(): open_(true){

    }

    BT::NodeStatus open(){
        this->open_ = true;
        std::cout << "Gripper open \n";
        return BT::NodeStatus::SUCCESS;
    }

    BT::NodeStatus close(){
        this->open_ = false;
        std::cout << "Gripper close\n";
        return BT::NodeStatus::SUCCESS;
    }

private:
    bool open_;
};


int main(){

    BT::BehaviorTreeFactory factory;
    
    factory.registerNodeType<ApproachObject>("ApproachObject");

    factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));

    GripperInterface gripper;
    factory.registerSimpleAction(
        "OpenGripper",
        std::bind(&GripperInterface::open, &gripper)
    );  

    factory.registerSimpleAction(
        "CloseGripper",
        std::bind(&GripperInterface::close, &gripper)
    );

    // Create tree
    try{
        auto tree = factory.createTreeFromFile("../bt_tree.xml");
        // Execute the tree
        tree.tickWhileRunning();
    }catch(BT::BehaviorTreeException& e){
        std::cout << e.what() << '\n';
    }

    return 0;
}