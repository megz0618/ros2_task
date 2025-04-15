// Importing libraries
#include "rclcpp/rclcpp.hpp" // source code for ROS in C++
#include "std_msgs/msg/string.hpp" // cos it was in the tutorials lol also useful for printing at the end

// Including cipher_interfaces 
#include "cipher_interfaces/msg/cipher_message.hpp" // custom message
#include "cipher_interfaces/srv/cipher_answer.hpp" // to check answer

#include <string>
#include <memory>
#include <cctype>
using namespace std::chrono_literals;
using std::placeholders::_1;

// Purpose:
// 1) Subscribe - get encoded message and key
// 2) Extract - decode message
// 3) Decode - send out CipherAnswer request after decoding

// Subscriber Class
class decodeNode : public rclcpp::Node {
// Now we can make a subscriber and a service client

public:
// Constructor for decodeNode, names the node
// Creates subscriber and server client using inbuilt fns
    decodeNode() : Node("decode_node")
    {
        // Create subscription so we can subscribe to what is published from encodeNode.
        // When something is published by encodeNode, calls findSendResult, which finds and 
        // sends result using service client
        subscription = this->create_subscription<cipher_interfaces::msg::CipherMessage>("encode_topic", 10,
                    std::bind(&decodeNode::findSendResult, this, std::placeholders::_1)); 

        // Create client so we can send a service request back to encodeNode to check our answer
        client = this->create_client<cipher_interfaces::srv::CipherAnswer>("verify_message");
    }

private:
    // Member variables, decodedMessage
    std::string decodedMessage;

    // rclcpp::nameOfThing<Type>::SharedPtr nameOfThing
    rclcpp::Subscription<cipher_interfaces::msg::CipherMessage>::SharedPtr subscription;
    rclcpp::Client<cipher_interfaces::srv::CipherAnswer>::SharedPtr client;

    // Member functions

    // decodeMessage function takes encodedMessage and key and returns 
    // the decoded message by 'undoing' the shift. 
    std::string decodeMessage(std::string encodedMessage, int key) {
        std::string decodedMessage {};

        for (size_t i = 0; i < encodedMessage.length(); i++) {
            if (std::isupper(encodedMessage[i])) {
                decodedMessage += char(int(encodedMessage[i]- key - 'A' + 26) % 26 + 'A');;
            } else {
                decodedMessage += char(int(encodedMessage[i] - key - 'a' + 26) % 26 + 'a');
            }
        }

        return decodedMessage;
    }

    // findSendResult (service client) takes in the 'given' message published to a topic by encodeNode.
    // Passes this info into decodeMessage fn, sends a request for encodeNode to check answer.
    // Waits for whether the result is correct, shuts down immediately after. 
    void findSendResult(cipher_interfaces::msg::CipherMessage::SharedPtr given) {
        decodedMessage = decodeMessage(given->message, given->key);
        RCLCPP_INFO(this->get_logger(), "Decoded message: %s", decodedMessage.c_str());

        // Create service request that will be sent back to server
        auto request = std::make_shared<cipher_interfaces::srv::CipherAnswer::Request>();
        // Assign decodedMessage as the 'answer' element of CipherAnswer struct
        request->answer = decodedMessage;

        // Wait for the service server in encodeNode 
        while (!client->wait_for_service(5s)) {
            RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
        }

        // When ok, send a 'request' for encodeNode to check answer
        client->async_send_request(request, std::bind(&decodeNode::getResult, this, std::placeholders::_1));
    }

    // Receives result from service server and prints whether correct
    // std::share_future makes it easier to handle async 
    void getResult(std::shared_future<cipher_interfaces::srv::CipherAnswer::Response::SharedPtr> result_shared) {
        auto response = result_shared.get();
        std::string correct = "no";

        if (response->result) {
            correct = "yes";
        }

        // Prints to terminal decoded message and whether correct, then shuts down
        RCLCPP_INFO(this->get_logger(), "Decoded message: '%s', correct: %s", decodedMessage.c_str(), correct.c_str());
        rclcpp::shutdown();
    }

};

int main(int argc, char **argv) {

    // Initialises the 'global context which is accessible via the fn'???
    rclcpp::init(argc, argv);

    // Initialise an encodeNode instance called node2
    std::shared_ptr<decodeNode> node2 = std::make_shared<decodeNode>();

    // Didn't include creating a service because handled elsewhere
    // Allows ros node to keep running and check for any events coming
    // to encodeNode (the request from decodeNode)
    rclcpp::spin(node2);

    // Shutdown the context
    rclcpp::shutdown();
    return 0;
}