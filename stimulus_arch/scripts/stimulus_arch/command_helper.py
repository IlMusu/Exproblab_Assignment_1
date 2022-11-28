#!/usr/bin/env python

class CommandHelper(object):
    '''
    This class is a helper for memorizing commands and then parse them
    from strings. 
    '''

    def __init__(self):
        '''
        This is the constructor method for the CommandHelper class.
        It initialized the registered_commands to empty map and sets the
        default exit command to 'exit'.
        '''
        # The possible commands that the user can prompt
        self._registered_commands = {}
        self._exit_command = 'exit'
    
    
    def register_command(self, command, callback):
        '''
        Args:
            command (list) : The list of strings that compose the command.
            callback (function) : The callback to invoke for the command.

        This is a helper method which can be used by to register commands. 
        The commands are deconstructed by strings and stored in a tree 
        structure with the related callbacks.
        '''
        # This source will contain at each iteration all the args that can
        # be appended after the current argument. Initially, there are all
        # the possible commands.
        arg_source = self._registered_commands
        for arg in command :
            # Appending argument one next to each other
            if not arg in arg_source :
                arg_source[arg] = {}
            arg_source = arg_source[arg]
        # Appending the callback for later usage
        arg_source['_callback'] = callback
        
        
    def parse_command(self, command_str):
        '''
        Args:
            command (string) : The command that needs to be parsed.
        Returns:
            (bool) : If the user requested the exit command.

        This method parses the command from a string to the list of string
        composing the command and the actual arguments.
        If a valid command is received, the related callback is called.
        If the command is not valid, the user is informed.
        '''
        # Parsing the command from string
        command = list(filter(None, command_str.split(' ')))
        if len(command) == 1 and command[0] == self._exit_command:
            return True
        # Decoding the command to retrieve the callback
        arg_source = self._registered_commands
        parameters = []
        for i in range(len(command)) :
            current_arg = command[i]
            # If cannot find the next string composing the command, it
            # is supposed that it is an argument for the command. 
            # So, the command has been found.
            if not current_arg in arg_source :
                parameters = command[i:]
                break
            # Going down the tree until the command is complete
            arg_source = arg_source[current_arg]
        # Check if the callback actually exists
        if not '_callback' in arg_source:
            print('Incomplete or wrong command')
        else:
            arg_source['_callback'](parameters)
        return False


    def listen_for_user_commands(self, exit_command):
        '''
        Args:
            exit_command (string) : The exit command for stop listening.

        This methods starts to listen for user commands on the console
        until the exit_command is received.
        '''
        # Registering the special exit command
        self._exit_command = exit_command
        # Start listening for user inputs
        while True :
            if self.parse_command(input()) :
                break
