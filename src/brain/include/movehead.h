#pragma once

#include <behaviortree_cpp/behavior_tree.h>
#include <behaviortree_cpp/bt_factory.h>

class Brain; 
using namespace BT;

/*
    해당 파일은 MoveHead을 위한 노드와 함수만 모아놓은 헤더 파일입니다.
*/

void RegisterMoveHeadNodes(BT::BehaviorTreeFactory &factory, Brain* brain); // 노드 등록을 위한 함수

// 액션 노드들 정리
class MoveHead : public SyncActionNode{
public:
    MoveHead(const std::string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain){}

    NodeStatus tick() override;

    static BT::PortsList providedPorts(){
        return {
            InputPort<double>("pitch", 0, "target head pitch"),
            InputPort<double>("yaw", 0, "target head yaw"),
        };
    }

private:
    Brain *brain;
};

class CamFindBall : public SyncActionNode{
public:
    CamFindBall(const string &name, const NodeConfig &config, Brain *_brain);

    NodeStatus tick() override;

private:
    double _cmdSequence[6][2];    
    rclcpp::Time _timeLastCmd;   
    int _cmdIndex;                
    long _cmdIntervalMSec;        
    long _cmdRestartIntervalMSec; 

    Brain *brain;

};

class CamTrackBall : public SyncActionNode{
public:
    CamTrackBall(const string &name, const NodeConfig &config, Brain *_brain) : SyncActionNode(name, config), brain(_brain) {}

    static PortsList providedPorts(){ return {}; }
    NodeStatus tick() override;

private:
    Brain *brain;
};