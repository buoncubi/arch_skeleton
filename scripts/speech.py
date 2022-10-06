#!/usr/bin/env python

import random
import rospy
# Import constant name defined to structure the architecture.
import architecture_name_mapper as anm
# Import the message type to be published.
from arch_skeleton.msg import Speech


# A tag for identifying logs producer.
LOG_TAG = anm.NODE_SPEECH


# Get speech-based commands keywords from ros parameters.
play_greeted_param = rospy.get_param(anm.PARAM_SPEECH_COMMANDS)
# Define the keyword that the user can say to start the interaction, e.g., "Hello".
PLAY_TAG = play_greeted_param[0]
# Define the keyword that the user can say to stop the interaction, e.g., "Bye".
GREETED_TAG = play_greeted_param[1]


# Initialise a new message that this node will publish.
# The published message is of type `Speech.msg`.
def init_msg():
    msg = Speech()
    msg.stamp = rospy.Time.now()
    return msg


# Publish a random message with either the `PLAY_TAG` or `GREETED_TAG` `command`.
# Each message generated by this method are published through the `publisher` 
# input parameter, and with a delay random chosen in the range
# [`speech_timing[0]`, `speech_timing[1]`) seconds. Somethimes an unknown  user'd
# command is produce to account possible detection errors.
def generate_random_speech(publisher, speech_timing):
    # Generate the message to be published.
    msg = init_msg()  
    # Choose what the user says.  Also publish unknown commands for testing purposes.
    if random.uniform(0,1) < 0.1:  # i.e., a probability of 0.1.
        msg.command = '???'  # Unknown user's command.
    else:
        msg.command = random.choice([PLAY_TAG, GREETED_TAG]) 
    # Publish the message.
    publisher.publish(msg)
    log_msg = 'Publishing random user command: `%s`' % msg.command
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    # Wait before the next random message is published.
    delay = random.uniform(speech_timing[0], speech_timing[1])
    rospy.sleep(delay)


# Allow keyboard interaction to emulate speech-based user's command.
def generate_manual_speech(publisher):
    # Wait until `Enter` is pressed and get the typed text.
    user_input = raw_input(' > ')
    user_input = user_input.lower()
    # Generate the message to be published.
    msg = init_msg()  
    # Understand the entered text.
    error = False
    if user_input == 'hello' or user_input == 'h':
        msg.command = PLAY_TAG
    elif user_input == 'bye' or user_input == 'b':
        msg.command = GREETED_TAG
    else:
        # Not understood user's command.
        error = True
        print('*** USER INPUT ERROR! Try again:')
    # Publish the message.
    if not error:
        publisher.publish(msg)   
        log_msg = 'Publishing entered user command: `%s`' % msg.command
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))


# Initialise and run a node that publishes speech-like commands.
if __name__ == '__main__':
    # Get parameter and initialise this node as well as its publisher.
    rospy.init_node(anm.NODE_SPEECH, log_level=rospy.INFO)
    randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE, True)
    publisher = rospy.Publisher(anm.TOPIC_SPEECH, Speech, queue_size=1, latch=True)
    # Log information.
    log_msg = 'Initialise node `%s` with topic `%s`.' % (anm.NODE_SPEECH, anm.TOPIC_SPEECH)
    rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))

    if randomness:
        # Configure node based on parameters to generate random speech-based data.
        speech_timing = rospy.get_param(anm.PARAM_SPEECH_TIME, [2.0, 30.0])
        log_msg = 'Random-based data generation active: a random command with a delay in the range of [%f, %f) seconds.' % (speech_timing[0], speech_timing[1])
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
    else:
        # Explain keyboard-based interaction.
        log_msg = 'Generate data through keyboard interaction.'
        rospy.loginfo(anm.tag_log(log_msg, LOG_TAG))
        print('  # Type `Hello` (`H`) to start interacting, or `Bye` (`B`) to finish interacting.')
        print('  # Type `cnt+C` and `Enter` to quit.')
        
    while not rospy.is_shutdown():
        if randomness:
            # Generate random speech-like data.
            generate_random_speech(publisher, speech_timing)
        else:
            # Allow keyboard interaction to generate speech-like data.
            generate_manual_speech(publisher)

