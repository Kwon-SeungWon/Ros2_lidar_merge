#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdexcept>
#include <cstring>

class CInterFace : public rclcpp::Node {
public:
    CInterFace() : Node("CInterFace_node") {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CInterFace::checkInput, this));
    }

    ~CInterFace() {
        killAllProcesses();
    }

private:
    void checkInput() {
        int user_input;
        std::cout << "0: kill all \n1: teleop \n2: slam \n3: navigation \n4: autoslam" << std::endl;
        if (std::cin >> user_input) {
            switch (user_input) {
                case 0:
                    killAllProcesses();
                    break;
                case 1: // teleop
                    teleop();
                    break;
                case 2: // slam
                    slam();
                    break;
                case 3: // navigation
                    navigation();
                    break;
                case 4: // autoslam
                    autoslam();
                    break;
                default:
                    std::cout << "Invalid input. Please enter a number between 0 and 4." << std::endl;
                    break;
            }
        }
    }

    void openNewTerminal(const std::string& command) {
        pid_t pid = fork();
        if (pid == 0) { // Child process
            execlp("xterm", "xterm", "-hold", "-e", command.c_str(), (char*)NULL);
            std::cerr << "execlp failed: " << strerror(errno) << std::endl;
            _exit(EXIT_FAILURE); // If execlp fails, exit child process
        } else if (pid > 0) { // Parent process
            pids_.push_back(pid);
        } else {
            throw std::runtime_error("fork() failed!");
        }
    }

    void teleop() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        openNewTerminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
        openNewTerminal("ros2 launch turtlebot3_teleop teleop.launch.py");
    }

    void slam() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        openNewTerminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
    }

    void navigation() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        openNewTerminal("ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True");
    }

    void autoslam() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        openNewTerminal("ros2 launch nav2_bringup slam_launch.py");
        openNewTerminal("ros2 launch nav2_bringup navigation_launch.py");
        openNewTerminal("ros2 launch explore_ros2 explore_demo.py");
    }

    void killAllProcesses() {
        for (pid_t pid : pids_) {
            if (kill(pid, SIGTERM) == -1) {
                std::cerr << "Failed to kill process " << pid << ": " << strerror(errno) << std::endl;
            }
        }
        pids_.clear();
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::vector<pid_t> pids_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CInterFace>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}