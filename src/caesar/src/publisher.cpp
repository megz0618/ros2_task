// Importing libraries
#include "rclcpp/rclcpp.hpp" // source code for ROS in C++
#include "std_msgs/msg/string.hpp" // cos it was in the tutorials lol

// Including cipher_interfaces 
#include "cipher_interfaces/msg/cipher_message.hpp" // custom message
#include "cipher_interfaces/srv/cipher_answer.hpp" // to check answer

#include <string>
#include <memory>
#include <cctype>
using namespace std::chrono_literals;
using std::placeholders::_1; // bind request (what's sent back from subscriber)
using std::placeholders::_2; // bind response (whether decoded message is correct)

// Purpose:
// 1) Encode - using message and key from cipher
// 2) Publish - send out CipherMessage using a topic
// 3) Verify - after the subscriber has decoded, verify guess

// Publisher Class
class encodeNode : public rclcpp::Node {
    // New class called encodeNode that inherits 'node' structure from ros source code
    // Now we can make a publisher and a service server hooray, doo the same thing for 
    // decodeNode so we can use a subscriber and a service client

public:
    // Constructor for encodeNode
    // Sets originalMessage and key, creates publisher and service using inbuilt fns 
    encodeNode() : Node("encode_node")
    {
        // Sets originalMessage and key
        originalMessage = "abcde";
        key = 1;

        // Create publisher that uses cipher_interfaces message as type
        // publisher = create_publisher<TYPE>("name", queue size should be left as 10 even though there won't be a queue)
        publisher = this->create_publisher<cipher_interfaces::msg::CipherMessage>("encode_topic", 10); 

        // Create service that uses cipher_interfaces answer as type
        // std::bind(&nodeName::functionCalled, this, set arguments/placeholders where necessary)
        // Alternately, can use lambda functions but not many examples online
        service = this->create_service<cipher_interfaces::srv::CipherAnswer>("verify_message", 
                std::bind(&encodeNode::verifyResult, this, std::placeholders::_1, std::placeholders::_2));
    
        // Publish encoded message through topic using helper function
        publishMessage();

        // Timer is NOT necessary because only called once before shutdown
        // timer = this-> create_wall_timer(
        //     1s, std::bind(&encodeNode::publishMessage, this)
        // );

    }

private:
    // Member variables, originalMessage, key and encodedMessage
    std::string originalMessage;
    int key;
    std::string encodedMessage;

    // rclcpp::nameOfThing<Type>::SharedPtr nameOfThing
    rclcpp::Publisher<cipher_interfaces::msg::CipherMessage>::SharedPtr publisher;
    rclcpp::Service<cipher_interfaces::srv::CipherAnswer>::SharedPtr service;
    // Timer unnecessary but would be written as rclcpp::TimerBase::SharedPtr timer; 

    // Member functions

    // Encode function takes originalMessage and key as parameters.
    // Outputs an encodedMessage that is the originalMessage shifted 
    // according to the key.
    std::string encode(std::string originalMessage, int key) {
        std::string encodedMessage {};

        for (size_t i = 0; i < originalMessage.length(); i++) {

            if (std::isupper(originalMessage[i])) {
                encodedMessage += char(int(originalMessage[i] + key - 65) % 26 + 65);
            } else {
                encodedMessage += char(int(originalMessage[i] + key - 97) % 26 + 97);
            }
        }

        return encodedMessage;
    }

    // publishMessage (publisher) uses CipherMessage struct to publish encoded message.
    // decodeNode will subscribe to this topic.
    void publishMessage() {
        // Set the encoded message every time DURING publishing
        encodedMessage = encode(originalMessage, key);
    
        // Set all the data in CipherMessage so it can be published
        cipher_interfaces::msg::CipherMessage output;
        output.message = encodedMessage;
        output.key = key;
    
        // Show the message and key that will be published
        RCLCPP_INFO(this->get_logger(), "Publishing message: '%s', with key: %d", 
                        output.message.c_str(), output.key);
    
        // Publish to a topic that can be read by decodeNOde
        publisher->publish(output);
    }    

    // verifyResult takes CipherAnswer request (service server) and response and 
    // checks against originalMessage. Sends back whether answer is correct.
    // Shuts down immediately after.  
    void verifyResult(std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Request> request,
                    std::shared_ptr<cipher_interfaces::srv::CipherAnswer::Response> response) {
    
        std::string correct {"no"};

        // If incoming answer matches originalMessage, sets outgoing flag to correct
        if (request->answer == originalMessage) {
            response->result = true;
            correct = "yes";
        } else {
            response->result = false;
        }

        // Automatically sent back to other node so shutdown
        rclcpp::shutdown();
    }
};

int main(int argc, char **argv) {

    // Initialises the 'global context which is accessible via the fn'???
    rclcpp::init(argc, argv);

    // Initialise an encodeNode instance called node1
    std::shared_ptr<encodeNode> node1 = std::make_shared<encodeNode>();

    // Didn't include creating a service because handled elsewhere
    // Allows ros node to keep running and check for any events coming
    // to encodeNode (the request from decodeNode)
    rclcpp::spin(node1);

    // Shutdown the context
    rclcpp::shutdown();
    return 0;
}
