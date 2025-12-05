# Project general coding guidelines

## Code Style
- Prefer modern c++ or python features like class, oop.
- Use consistent indentation (4 spaces)
- Limit line length to 120 characters
- Use spaces around operators and after commas
- Place opening braces on the same line as the control statement
- Use single quotes for strings in Python and double quotes in C++

## Naming Conventions
- Use PEP 8 style guide for python and c++
- Prefix private class members with underscore (_)
- Use ALL_CAPS for constants

## Code Quality
- Use meaningful variable and function names that clearly describe their purpose
- Include helpful comments for complex logic
- Add error handling for calling low level interface, external api, h/w drivers.
- Write unit tests for new features and bug fixes
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


