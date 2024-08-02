#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <atomic>
#include <vector>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <stdexcept>
#include <cstring>

class CInterFace : public rclcpp::Node {
public:
    CInterFace() : Node("CInterFace_node"), stop_thread_(false) {
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&CInterFace::inputLoop, this));
    }

    ~CInterFace() {
        stop_thread_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
        killAllProcesses();
    }

private:
    void inputLoop() {
        while (!stop_thread_) {
            int user_input;
            if (std::cin >> user_input) {
                switch (user_input) {
                    case 0:
                        system("echo 'Command for 0 executed'");
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
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    void openNewTerminal(const std::string& command) {
        pid_t pid = fork();
        if (pid == 0) { // 자식 프로세스
            // 자식 프로세스가 xterm을 열어 명령어를 실행합니다.
            execlp("xterm", "xterm", "-hold", "-e", command.c_str(), (char*)NULL);
            std::cerr << "execlp failed: " << strerror(errno) << std::endl;
            _exit(EXIT_FAILURE); // execlp가 실패하면 자식 프로세스는 종료
        } else if (pid > 0) { // 부모 프로세스
            // 부모 프로세스는 자식 프로세스의 PID를 저장합니다.
            pids_.push_back(pid);
        } else {
            throw std::runtime_error("fork() failed!");
        }
    }

    void teleop() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch turtlebot3_teleop teleop.launch.py");
    }

    void slam() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
    }

    void navigation() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True");
    }

    void autoslam() {
        openNewTerminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch nav2_bringup slam_launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
        openNewTerminal("ros2 launch nav2_bringup navigation_launch.py");
        std::this_thread::sleep_for(std::chrono::milliseconds(2000));
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
    std::thread input_thread_;
    std::atomic<bool> stop_thread_;
    std::vector<pid_t> pids_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CInterFace>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
