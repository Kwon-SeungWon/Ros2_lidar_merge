#include <rclcpp/rclcpp.hpp>
#include <iostream>
#include <thread>
#include <atomic>

class HelloWorldNode : public rclcpp::Node {
public:
    HelloWorldNode() : Node("hello_world_node"), stop_thread_(false) {
        input_thread_ = std::thread([this]() { this->input_loop(); });
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(100), std::bind(&HelloWorldNode::check_input, this));
    }

    ~HelloWorldNode() {
        stop_thread_ = true;
        if (input_thread_.joinable()) {
            input_thread_.join();
        }
    }

private:
    void input_loop() {
        while (!stop_thread_) {
            int user_input;
            if (std::cin >> user_input) {
                switch (user_input) {
                    case 0:
                        system("echo 'Command for 0 executed'");
                        shutdown_other_nodes();
                        break;
                    case 1: //teleop
                        teleop();
                        break;
                    case 2: //slam
                        slam();
                        break;
                    case 3: //navigation
                        navigation();
                        break;
                    case 4: //autoslam
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

    void shutdown_other_nodes() {
        std::array<char, 128> buffer;
        std::string result;
        std::shared_ptr<FILE> pipe(popen("ros2 node list", "r"), pclose);
        if (!pipe) throw std::runtime_error("popen() failed!");

        while (fgets(buffer.data(), buffer.size(), pipe.get()) != nullptr) {
            result += buffer.data();
        }

        std::istringstream stream(result);
        std::string node_name;
        std::vector<std::string> nodes;

        while (std::getline(stream, node_name)) {
            nodes.push_back(node_name);
        }

        for (const auto& node : nodes) {
            if (node != this->get_name()) {
                std::string command = "ros2 node kill " + node;
                system(command.c_str());
            }
        }
    }

    void open_new_terminal(const std::string& command) 
    {
      std::string terminal_command = "xterm -hold -e \"" + command + "\" &";
      system(terminal_command.c_str());
    }
    void teleop()
    {
      open_new_terminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch turtlebot3_teleop teleop.launch.py");
    }

    void slam()
    {
      open_new_terminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch turtlebot3_cartographer cartographer.launch.py use_sim_time:=True");
    }

    void navigation()
    {
      open_new_terminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch turtlebot3_navigation2 navigation2.launch.py use_sim_time:=True");
    }

    void autoslam()
    {
      open_new_terminal("ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch ros2_laser_scan_merger merge_2_scan.launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch nav2_bringup slam_launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch nav2_bringup navigation_launch.py");
      std::this_thread::sleep_for(std::chrono::milliseconds(2000));
      open_new_terminal("ros2 launch explore_ros2 explore_demo.py");
    }
    void check_input() {
        // 다른 로직이 있다면 여기서 처리
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::thread input_thread_;
    std::atomic<bool> stop_thread_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<HelloWorldNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();
    return 0;
}
