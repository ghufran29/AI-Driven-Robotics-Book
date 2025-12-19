# ROS 2 API Contracts: Module 1: The Robotic Nervous System (ROS 2)

## Publisher-Subscriber Pattern Contracts

### Publisher Interface Contract
```
Interface: Publisher<T>
Method: publish(message: T) -> void
Precondition:
  - ROS 2 node is initialized
  - Topic is valid and properly named
  - Message type T matches topic type
Postcondition:
  - Message is sent to topic
  - All subscribers receive the message (subject to QoS)
```

### Subscriber Interface Contract
```
Interface: Subscription<T>
Method: callback(message: T) -> void
Precondition:
  - ROS 2 node is initialized
  - Subscription is properly configured to topic
  - Message type T matches topic type
Postcondition:
  - Callback function executes with received message
  - Message processing completed or error handled
```

## Service Client-Server Contracts

### Service Server Contract
```
Interface: ServiceServer<T>
Method: handle_request(request: T.Request) -> T.Response
Precondition:
  - Service is advertised and available
  - Request message is properly formatted
  - Server is in active state
Postcondition:
  - Response is sent back to client
  - Request processing completed
```

### Service Client Contract
```
Interface: ServiceClient<T>
Method: call(request: T.Request) -> Future<T.Response>
Precondition:
  - Service is available and discoverable
  - Request message is properly formatted
  - Client is connected to service
Postcondition:
  - Request sent to service
  - Future object returned for response
```

## Action Client-Server Contracts

### Action Server Contract
```
Interface: ActionServer<T>
Methods:
  - handle_goal(goal: T.Goal) -> GoalResponse
  - execute(goal_handle: GoalHandle) -> T.Result
Precondition:
  - Action is advertised and available
  - Goal message is properly formatted
  - Server is in active state
Postcondition:
  - Goal accepted/rejected
  - Execution feedback provided
  - Result returned when complete
```

### Action Client Contract
```
Interface: ActionClient<T>
Methods:
  - send_goal(goal: T.Goal) -> GoalHandle
  - get_result(goal_handle: GoalHandle) -> Future<T.Result>
Precondition:
  - Action server is available
  - Goal message is properly formatted
  - Client is connected to server
Postcondition:
  - Goal sent to server
  - Feedback received during execution
  - Result obtained when complete
```

## Code Example Contracts

### rclpy Node Contract
```
Class: Node
Constructor: Node(node_name: str, namespace: str = "")
Precondition:
  - ROS 2 context is initialized
  - Node name is unique in namespace
Postcondition:
  - Node is registered with ROS 2 graph
  - Node can create publishers/subscriptions/services/actions
```

### Topic Creation Contract
```
Method: Node.create_publisher(msg_type, topic_name, qos_profile)
Precondition:
  - Node is initialized
  - Message type is valid ROS 2 message
  - Topic name follows ROS naming conventions
Postcondition:
  - Publisher is created and ready to publish
  - Topic is registered in ROS 2 graph
```

### Service Creation Contract
```
Method: Node.create_service(srv_type, srv_name, callback)
Precondition:
  - Node is initialized
  - Service type is valid ROS 2 service
  - Service name follows ROS naming conventions
Postcondition:
  - Service server is created and ready to handle requests
  - Service is registered in ROS 2 graph
```

## Validation Requirements

### Code Validation Contract
- All Python code examples must be compatible with Python 3.10+
- All rclpy code must follow ROS 2 Humble API specifications
- All URDF files must be valid XML and follow URDF schema
- All examples must run successfully in Ubuntu 22.04 environment
- All visualization examples must work with RViz2

### Educational Contract
- Each example must include clear explanations
- All code must be properly commented
- Error handling must be demonstrated where appropriate
- Best practices for ROS 2 development must be followed
- Safety considerations must be noted for physical robot deployment