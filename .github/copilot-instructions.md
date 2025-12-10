# Project general coding guidelines

## Code Style
- Prefer modern c++ or python features like class, oop.
- Use consistent indentation (4 spaces)
- Limit line length to 120 characters
- Use spaces around operators and after commas
- Place opening braces on the same line as the control statement
- Use single quotes for strings in Python and double quotes in C++

## Project Structure
- follow ros2 package structure for robotics projects
- Organize files into logical directories (e.g., src/, include/, tests/, docs/)

## Error Handling
- Use exceptions for error handling in Python and C++
- Catch specific exceptions rather than using a generic catch-all
- Clean up resources (e.g., close files, release memory) in error cases

## abstraction and Modularity
- Design code with modularity in mind, breaking down functionality into smaller, reusable components
- Use interfaces or abstract classes to define contracts for components
- Encapsulate implementation details to reduce dependencies between modules
- Follow the Single Responsibility Principle, ensuring each module or class has one reason to change  

## best Practices
- Write code that is easy to read and understand
- Avoid premature optimization; focus on clarity and correctness first
- Use design patterns where appropriate to solve common problems
- Regularly review and refactor code to improve quality and maintainability 

## Ros2 Specific Guidelines
- Follow ROS 2 coding standards and conventions
- Use ROS 2 logging mechanisms for debug and error messages
- Log error messages with sufficient context for debugging
- Use ROS 2 parameter server for configurable settings
- Leverage ROS 2 launch files for starting nodes and managing configurations
- Utilize ROS 2 message types and services appropriately
- Ensure compatibility with ROS 2 middleware (e.g., DDS)
- Implement proper shutdown procedures for ROS 2 nodes
- Handle ROS 2 callbacks efficiently to avoid blocking the main thread
- Use ROS 2 timers for periodic tasks
- Follow ROS 2 security best practices when handling sensitive data
- Integrate with ROS 2 tools for simulation and visualization (e.g., RViz, Gazebo)
- Ensure proper use of ROS 2 lifecycle management for nodes
- Follow ROS 2 community guidelines for contributing to open-source projects
- Use ROS 2 services and actions for request-response and long-running tasks
- Implement proper message filtering and throttling to manage data flow
- Use ROS 2 bag files for data recording and playback during testing
- Ensure compatibility with different ROS 2 distributions
- Follow ROS 2 package structure and naming conventions
- Utilize ROS 2 tools for building, testing, and deploying packages
- Ensure proper use of ROS 2 communication mechanisms (topics, services, actions)
- Adhere to ROS 2 package structure and naming conventions

## ROS2 Python Specific Guidelines
- Use rospy for ROS 2 Python nodes
- Follow Python-specific best practices in ROS 2 context
- Use rospy logging functions for debug and error messages
- Leverage rospy parameters for configurable settings
- Utilize rospy timers for periodic tasks
- Implement proper shutdown procedures for rospy nodes
- Handle rospy callbacks efficiently to avoid blocking the main thread
- Use rospy services and actions for request-response and long-running tasks

## moveit2 Specific Guidelines
- Follow MoveIt 2 coding standards and conventions
- Use MoveIt 2 planning interfaces for motion planning tasks
- Leverage MoveIt 2 collision checking mechanisms
- Utilize MoveIt 2 kinematics solvers appropriately
- Ensure compatibility with MoveIt 2 plugins and extensions
- Implement proper error handling for MoveIt 2 operations
- Follow MoveIt 2 community guidelines for contributing to open-source projects
## Moveit2 task constructor Specific Guidelines 
- Use MTC for complex task planning and execution
- Define tasks using MTC's task and stage abstractions
- Utilize MTC's built-in stages for common operations (e.g., MoveTo, Pick, Place)
- Implement custom stages when necessary for specialized behavior
- Use MTC's error handling and recovery mechanisms
- Integrate MTC with MoveIt 2 planning and execution pipelines
- Follow MTC community guidelines for contributing to open-source projects

## Code Quality
- Use meaningful variable and function names that clearly describe their purpose
- Include helpful comments for complex logic
- Add error handling for calling low level interface, external api, h/w drivers.
- Avoid code duplication by creating reusable functions or classes
- Regularly refactor code to improve readability and maintainability
- Use version control effectively with clear commit messages
- Conduct code reviews to ensure adherence to coding standards and best practices
- Document public APIs and modules with clear docstrings or comments
- Apply clean code guidlines and SOLID principle 

## Documentation
- Maintain up-to-date documentation for all modules and functions
- Use docstrings in Python and Doxygen comments in C++
- Provide examples for complex functions or classes
- Keep README files current with installation and usage instructions
- Use markdown format for documentation files

## Version Control
- Use meaningful commit messages that describe the changes made
- Follow a branching strategy (e.g., Git Flow, feature branches)
- Regularly merge changes from the main branch to avoid conflicts
- Tag releases with version numbers following semantic versioning
- Review pull requests thoroughly before merging
## Collaboration
- Communicate effectively with team members about code changes and reviews
- Participate in code reviews to provide and receive constructive feedback
- Share knowledge and best practices with the team
- Document decisions made during development for future reference
- Foster a positive and inclusive team environment
## Performance
- Optimize code for performance where necessary, but prioritize readability and maintainability
- Profile code to identify bottlenecks before optimizing
- Use efficient algorithms and data structures appropriate for the task
- Avoid premature optimization; focus on clear and correct code first
- Test performance improvements to ensure they have the desired effect
## Security
- Follow best practices for secure coding to prevent vulnerabilities
- Validate and sanitize all user inputs
- Use secure libraries and frameworks
- Regularly update dependencies to patch known security issues
- Conduct security reviews and audits of the codebase
- Handle sensitive data with care, using encryption where appropriate
- Avoid hardcoding sensitive information such as passwords or API keys in the codebase
## Environment Configuration
- Use environment variables for configuration settings
- Avoid hardcoding environment-specific settings in the codebase
- Document required environment variables and their purposes
- Use configuration files (e.g., .env, config.json) for managing settings
- Ensure sensitive information is not included in version control
- Provide sample configuration files for development and testing
- Use consistent naming conventions for environment variables


