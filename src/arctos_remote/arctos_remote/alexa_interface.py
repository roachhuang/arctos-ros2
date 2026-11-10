#!/usr/bin/env python3

from flask import Flask
from ask_sdk_core.skill_builder import SkillBuilder
from flask_ask_sdk.skill_adapter import SkillAdapter

from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard
from ask_sdk_core.dispatch_components import AbstractRequestHandler
from ask_sdk_core.dispatch_components import AbstractExceptionHandler
from ask_sdk_core.utils import is_request_type, is_intent_name
from ask_sdk_core.handler_input import HandlerInput
from ask_sdk_model import Response
from ask_sdk_model.ui import SimpleCard

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
# add import the action message, to intface btw alexa.action and ros2
from ros2_fndm_interface.action import Alexa
import threading

# This is a simple Alexa skill that interacts with a ROS2 action server.
# It uses Flask to create a web server and the ask-sdk to handle Alexa requests.
threading.Thread(target=lambda: rclpy.init(args=None)).start()

action_client = ActionClient(Node('alexa_interface'), Alexa, 'task_server')
app = Flask(__name__)

class GoHomeIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("GoHomeIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok. I'm moving to home position."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("GoHome", speech_text)).set_should_end_session(
            True)
        
        goal = Alexa.Goal()
        goal.task_number = 0
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response
    
class StandbyIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_intent_name("StandbyIntent")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        speech_text = "Ok. I'm moving to initial position."

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Standby", speech_text)).set_should_end_session(
            True)
        
        goal = Alexa.Goal()
        goal.task_number = 1
        action_client.send_goal_async(goal)
        return handler_input.response_builder.response

class SessionEndedRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("SessionEndedRequest")(handler_input)

    def handle(self, handler_input):
        # type: (HandlerInput) -> Response
        # Log the reason for session end if needed
        print(f"Session ended: {handler_input.request_envelope.request.reason}")
        return handler_input.response_builder.response  # Respond with empty response

class LaunchRequestHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # type: (HandlerInput) -> bool
        return is_request_type("LaunchRequest")(handler_input)

    def handle(self, handler_input):
        # speech_text = "Hello from basic test."
        # handler_input.response_builder.speak(speech_text).set_should_end_session(True)
        # return handler_input.response_builder.response
    
        # type: (HandlerInput) -> Response
        speech_text = "Hi Zer Zhang, how can i help you?"

        handler_input.response_builder.speak(speech_text).set_card(
            SimpleCard("Hello World", speech_text)).set_should_end_session(
            False)
        
        # goal = Alexa.Goal()
        # goal.task_number = 0
        # action_client.send_goal_async(goal)
        return handler_input.response_builder.response


class CatchAllExceptionHandler(AbstractExceptionHandler):
    def can_handle(self, handler_input, exception):
        return True

    def handle(self, handler_input, exception):
        print(exception)  # Log the exception
        speech = "Sorry, I had trouble doing what you asked. Please try again."
        return handler_input.response_builder.speak(speech).ask(speech).response

class CatchAllIntentHandler(AbstractRequestHandler):
    def can_handle(self, handler_input):
        # Catch all unhandled intents
        return True

    def handle(self, handler_input):
        # Provide a response for unhandled intents
        speech_text = "Sorry, I didn't understand that. Can you try again?"
        handler_input.response_builder.speak(speech_text).set_should_end_session(False)
        return handler_input.response_builder.response

# Ensure the ActionClient is ready before sending goals
if not action_client.wait_for_server(timeout_sec=5.0):
    print("Action server not available. Exiting.")
    exit(1)

# then also register it

skill_builder = SkillBuilder()
# Register your intent handlers to the skill_builder object
skill_builder.add_request_handler(SessionEndedRequestHandler())
skill_builder.add_exception_handler(CatchAllExceptionHandler())
skill_builder.add_request_handler(LaunchRequestHandler())
skill_builder.add_request_handler(GoHomeIntentHandler())
skill_builder.add_request_handler(StandbyIntentHandler())
skill_builder.add_request_handler(CatchAllIntentHandler())
skill_adapter = SkillAdapter(
    skill=skill_builder.create(), skill_id="amzn1.ask.skill.5ca425f2-b353-40da-9616-c6f013688a23", app=app)

@app.route("/", methods=['POST'])
def invoke_skill():
    return skill_adapter.dispatch_request()

skill_adapter.register(app=app, route="/")


if __name__ == "__main__":
    app.run()
